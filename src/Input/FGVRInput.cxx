// Handle user input from OpenXR devices (via osgXR)
//
// Copyright (C) 2021  James Hogan
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

#include "config.h"
#include "FGVRInput.hxx"

#include <simgear/debug/ErrorReportingCallback.hxx>
#include <simgear/props/props_io.hxx>

#include <Main/fg_props.hxx>
#include <Scenery/scenery.hxx>
#include <Scripting/NasalSys.hxx>
#include <Viewer/CameraGroup.hxx>
#include <Viewer/VRManager.hxx>

#include <algorithm>
#include <cassert>
#include <cmath>

#include "FGVRButton.hxx"
#include "FGVRPick.hxx"
#include "FGVRPoseEuler.hxx"

using flightgear::CameraGroup;
using flightgear::VRManager;

// FGVRInput::Subaction

FGVRInput::Subaction::Subaction(osgXR::Manager *manager,
                                const std::string &path) :
    osgXR::Subaction(manager, path)
{
}

void FGVRInput::Subaction::setup(SGPropertyNode *node)
{
    _modesNode = node->getNode("modes", false);
}

const char *FGVRInput::Subaction::getPresetMode(const char *id)
{
    if (!_modesNode)
        return nullptr;
    return _modesNode->getStringValue(id, nullptr);
}

void FGVRInput::Subaction::pushMode(FGVRInput::Mode *mode)
{
    _modeStack.push_back(mode);
    mode->activate(this);
}

unsigned int FGVRInput::Subaction::popModesFront(FGVRInput::Mode *keepMode)
{
    int index = -1;
    for (unsigned int i = 0; i < _modeStack.size(); ++i)
        if (_modeStack[i] == keepMode)
            index = i;
    // not found?
    if (index < 0)
        return 0;

    // pop off end leaving keepMode at the top
    unsigned int ret = _modeStack.size() - (index + 1);
    while ((int)_modeStack.size() > index + 1) {
        _modeStack[_modeStack.size() - 1]->deactivate(this);
        _modeStack.pop_back();
    }
    return ret;
}

void FGVRInput::Subaction::clearModes()
{
    while (!_modeStack.empty())
        _modeStack.back()->deactivate(this);
}

void FGVRInput::Subaction::update(double dt)
{
    // Allow active modes to poll for changes
    // Note this may change the mode stack
    for (unsigned int i = 0; i < _modeStack.size(); ++i)
        _modeStack[i]->update(this, dt);
}

// FGVRInput::ActionSet

FGVRInput::ActionSet::ActionSet(FGVRInput *input, osgXR::Manager *manager,
                                SGPropertyNode *node) :
    osgXR::ActionSet(manager),
    _node(node)
{
    // Set up action set information

    const char *name = node->getName();
    const char *desc = node->getStringValue("desc", name);
    int priority = node->getIntValue("priority", 0);

    setName(name, desc);
    if (priority >= 0)
        setPriority(priority);

    // Set up actions in the action set

    for (SGPropertyNode *actionNode: node->getChildren("action")) {
        const char *actionName = actionNode->getStringValue("name");
        const char *actionType = actionNode->getStringValue("type");

        // Create an action of the specified type
        osgXR::Action *action = nullptr;
        if (!strcmp(actionType, "boolean")) {
            action = new ActionBoolean(this, actionNode);
        } else if (!strcmp(actionType, "float")) {
            action = new ActionFloat(this, actionNode);
        } else if (!strcmp(actionType, "vector2f")) {
            action = new ActionVector2f(this, actionNode);
        } else if (!strcmp(actionType, "pose")) {
            action = new ActionPose(this, actionNode);
        } else {
            SG_LOG(SG_INPUT, SG_WARN,
                   "Unknown VR action type \"" << actionType << "\" for "
                                            << name << "." << actionName);
        }

        if (action) {
            std::ostringstream statusPath;
            statusPath << "action-sets/" << name << "/" << actionName;
            SGPropertyNode *statusNode = input->getStatusNode()->getNode(statusPath.str(), true);
            auto *actionCommon = dynamic_cast<ActionCommon *>(action);
            actionCommon->init(input, manager, action, statusNode);

            // keep a reference to it for later
            _actionMap[actionName] = action;

            // kick off an update to initialise status properties
            actionCommon->update(0);
        }
    }
}

const std::string &FGVRInput::ActionSet::getModule(Subaction *subaction)
{
    auto it = _modules.find(subaction);
    if (it != _modules.end())
        return (*it).second;

    std::ostringstream str;
    str << "vr/action-set/" << getName();
    if (subaction)
        str << subaction->getPath();

    const std::string &module = _modules[subaction] = str.str();

    // This function is called from postinit, so create module too
    FGNasalSys *nasalsys = (FGNasalSys *)globals->get_subsystem("nasal");
    nasalsys->createModule(module.c_str(), module.c_str(), "", 0);

    simgear::ErrorReportContext errCtx("input-device", module);
    for (SGPropertyNode *nasal: _node->getChildren("nasal")) {
        nasal->setStringValue("module", module.c_str());
        bool ok = nasalsys->handleCommand(nasal, nullptr);
        if (!ok) {
            // TODO: get the Nasal errors logged properly
            simgear::reportFailure(simgear::LoadFailure::BadData,
                                   simgear::ErrorCode::InputDeviceConfig,
                                   "Failed to parse VR action-set Nasal");
        }
    }

    return module;
}

osgXR::Action *FGVRInput::ActionSet::findAction(const char *name)
{
    auto it = _actionMap.find(name);
    if (it == _actionMap.end())
        return nullptr;
    return (*it).second;
}

void FGVRInput::ActionSet::cleanupActions()
{
    _actionMap.clear();
}

void FGVRInput::ActionSet::postinit()
{
    for (auto &actionPair: _actionMap)
        dynamic_cast<ActionCommon *>(actionPair.second.get())->postinit(this);
}

void FGVRInput::ActionSet::update(double dt)
{
    for (auto &actionPair: _actionMap)
        dynamic_cast<ActionCommon *>(actionPair.second.get())->update(dt);
}

void FGVRInput::ActionSet::fireBindings()
{
    for (auto &actionPair: _actionMap)
        dynamic_cast<ActionCommon *>(actionPair.second.get())->fireBindings();
}

// FGVRInput::ActionCommon

FGVRInput::ActionCommon::ActionCommon(osgXR::Action *action,
                                      SGPropertyNode *node) :
    _node(node)
{
    // Set up action information

    const char *name = node->getStringValue("name");
    const char *desc = node->getStringValue("desc", name);

    action->setName(name, desc);
}

FGVRInput::ActionCommon::~ActionCommon()
{
    for (auto &subactionPair: _subactions)
        delete subactionPair.second;
}

FGVRInput::SubactionInfo *FGVRInput::ActionCommon::getSubactionInfo(FGVRInput::Subaction *subaction)
{
    auto it = _subactions.find(subaction);
    if (it == _subactions.end())
        return nullptr;
    return (*it).second;
}

void FGVRInput::ActionCommon::init(FGVRInput *input, osgXR::Manager *manager,
                                   osgXR::Action *action,
                                   SGPropertyNode *statusNode)
{
    // Set up subactions
    auto subactions = _node->getChildren("subaction");
    for (unsigned int i = 0; i < subactions.size(); ++i) {
        SGPropertyNode *subactionNode = subactions[i];
        SGPropertyNode *pathNode = subactionNode->getNode("path", false);
        if (!pathNode)
            pathNode = subactionNode;
        std::string subactionPath = pathNode->getStringValue();
        if (!subactionPath.empty()) {
            Subaction *subaction = input->getSubaction(manager, subactionPath);
            if (_subactions.find(subaction) == _subactions.end()) {
                auto *info = createSubactionInfo(subaction, subactionNode,
                                                 statusNode->getChild("value", i, true));
                _subactions[subaction] = info;
                action->addSubaction(subaction);
            } else {
                SG_LOG(SG_INPUT, SG_WARN,
                       "Duplicate subaction \"" << subactionPath << "\" ignored in VR action " << action->getName() << " (" << action->getLocalizedName() << ")");
            }
        }
    }

    // Still have an entry for no subaction filter
    if (_subactions.empty()) {
        auto *info = createSubactionInfo(nullptr, nullptr,
                                         statusNode->getChild("value", 0, true));
        _subactions[nullptr] = info;
    }
}

void FGVRInput::ActionCommon::postinit(FGVRInput::ActionSet *actionSet)
{
    for (auto subactionPair: _subactions)
        subactionPair.second->postinit(actionSet, this);
}

void FGVRInput::ActionCommon::update(double dt)
{
    for (auto subactionPair: _subactions)
        subactionPair.second->update(this, dt);
}

void FGVRInput::ActionCommon::fireBindings()
{
    for (auto subactionPair: _subactions)
        subactionPair.second->fireBindings();
}

// FGVRInput::SubactionInfo

FGVRInput::SubactionInfo::SubactionInfo(osgXR::Action *action,
                                        FGVRInput::Subaction *subaction,
                                        SGPropertyNode *node) :
    _subaction(subaction),
    _node(node)
{
    std::ostringstream actionBuf;
    actionBuf << action->getName();
    if (subaction)
        actionBuf << subaction->getPath();
    _name = actionBuf.str();
}

void FGVRInput::SubactionInfo::postinit(FGVRInput::ActionSet *actionSet,
                                        FGVRInput::ActionCommon *action)
{
    auto &module = actionSet->getModule(_subaction);

    // Set up action wide bindings
    postinit(action->getNode(), module);
    // Set up subaction specific bindings in this action
    if (_node.valid())
        postinit(_node, module);
}

// FGVRInput::SubactionInfoBoolean

FGVRInput::SubactionInfoBoolean::SubactionInfoBoolean(osgXR::Action *action,
                                                      FGVRInput::Subaction *subaction,
                                                      SGPropertyNode *node,
                                                      SGPropertyNode *statusNode) :
    SubactionInfo(action, subaction, node),
    _curValue(false),
    _statusProp(statusNode)
{
}

void FGVRInput::SubactionInfoBoolean::postinit(SGPropertyNode *node,
                                               const std::string &module)
{
    _button.init(node, _name, module);
}

void FGVRInput::SubactionInfoBoolean::update(FGVRInput::ActionCommon *action,
                                             double dt)
{
    auto *actionBool = static_cast<FGVRInput::ActionBoolean*>(action);
    _curValue = actionBool->getValue(_subaction);
    _statusProp = _curValue;
}

void FGVRInput::SubactionInfoBoolean::fireBindings()
{
    int modifiers = fgGetKeyModifiers();
    _button.update(modifiers, _curValue);
}

// FGVRInput::SubactionInfoFloat

FGVRInput::SubactionInfoFloat::SubactionInfoFloat(osgXR::Action *action,
                                                  FGVRInput::Subaction *subaction,
                                                  SGPropertyNode *node,
                                                  SGPropertyNode *statusNode) :
    SubactionInfo(action, subaction, node),
    _curValue(0),
    _lastValue(0),
    _statusProp(statusNode)
{
}

void FGVRInput::SubactionInfoFloat::postinit(SGPropertyNode *node,
                                             const std::string &module)
{
    read_bindings(node, _bindings, KEYMOD_NONE, module);
}

void FGVRInput::SubactionInfoFloat::update(FGVRInput::ActionCommon *action,
                                           double dt)
{
    auto *actionFloat = static_cast<FGVRInput::ActionFloat*>(action);
    _curValue = actionFloat->getValue(_subaction);
    _statusProp = _curValue;
}

void FGVRInput::SubactionInfoFloat::fireBindings()
{
    if (_curValue != _lastValue) {
        if (!_bindings[KEYMOD_NONE].empty()) {
            _lastValue = _curValue;
            for (auto &binding: _bindings[KEYMOD_NONE])
                binding->fire(_curValue);
        }
    }
}

// FGVRInput::SubactionInfoVector2f

FGVRInput::SubactionInfoVector2f::SubactionInfoVector2f(osgXR::Action *action,
                                                        FGVRInput::Subaction *subaction,
                                                        SGPropertyNode *node,
                                                        SGPropertyNode *statusNode) :
    SubactionInfo(action, subaction, node),
    _curValue(0, 0),
    _lastValue(0, 0),
    _statusNode(statusNode),
    _statusProp {
        SGPropObjDouble(statusNode->getChild("x", 0, true)),
        SGPropObjDouble(statusNode->getChild("y", 0, true))
    }
{
}

void FGVRInput::SubactionInfoVector2f::postinit(SGPropertyNode *node,
                                                const std::string &module)
{
    read_bindings(node, _bindings, KEYMOD_NONE, module);
}

void FGVRInput::SubactionInfoVector2f::update(FGVRInput::ActionCommon *action,
                                           double dt)
{
    auto *actionVector2f = static_cast<FGVRInput::ActionVector2f*>(action);
    _curValue = actionVector2f->getValue(_subaction);
    _statusProp[0] = _curValue.x();
    _statusProp[1] = _curValue.y();
}

void FGVRInput::SubactionInfoVector2f::fireBindings()
{
    if (_curValue != _lastValue) {
        if (!_bindings[KEYMOD_NONE].empty()) {
            _lastValue = _curValue;
            for (auto &binding: _bindings[KEYMOD_NONE])
                binding->fire(_statusNode);
        }
    }
}

// FGVRInput::SubactionInfoPose

FGVRInput::SubactionInfoPose::SubactionInfoPose(osgXR::Action *action,
                                                FGVRInput::Subaction *subaction,
                                                SGPropertyNode *node,
                                                SGPropertyNode *statusNode) :
    SubactionInfo(action, subaction, node),
    _statusNode(statusNode),
    _statusPositionNode(statusNode->getChild("position", 0, true)),
    _statusPositionValidProp(SGPropObjBool(_statusPositionNode->getChild("valid", 0, true))),
    _statusPositionTrackedProp(SGPropObjBool(_statusPositionNode->getChild("tracked", 0, true))),
    _statusPositionProp {
        SGPropObjDouble(_statusPositionNode->getChild("x", 0, true)),
        SGPropObjDouble(_statusPositionNode->getChild("y", 0, true)),
        SGPropObjDouble(_statusPositionNode->getChild("z", 0, true))
    }
{
}

void FGVRInput::SubactionInfoPose::postinit(SGPropertyNode *node,
                                            const std::string &module)
{
    read_bindings(node, _bindings, KEYMOD_NONE, module);
}

void FGVRInput::SubactionInfoPose::update(FGVRInput::ActionCommon *action,
                                          double dt)
{
    auto *actionPose = static_cast<FGVRInput::ActionPose*>(action);
    _curValue = actionPose->getValue(_subaction);
    _statusPositionValidProp = _curValue.isPositionValid();
    _statusPositionTrackedProp = _curValue.isPositionTracked();
    _statusPositionProp[0] = _curValue.getPosition().x();
    _statusPositionProp[1] = _curValue.getPosition().y();
    _statusPositionProp[2] = _curValue.getPosition().z();
}

void FGVRInput::SubactionInfoPose::fireBindings()
{
    if (_curValue != _lastValue) {
        if (!_bindings[KEYMOD_NONE].empty()) {
            _lastValue = _curValue;
            for (auto &binding: _bindings[KEYMOD_NONE])
                binding->fire(_statusNode);
        }
    }
}

// FGVRInput::InteractionProfile

FGVRInput::InteractionProfile::InteractionProfile(FGVRInput *input,
                                                  osgXR::Manager *manager,
                                                  const char *vendor,
                                                  const char *type,
                                                  SGPropertyNode *node) :
    osgXR::InteractionProfile(manager, vendor, type)
{
    // Look for action bindings
    // The first level of child nodes are action set names
    for (int i = 0; i < node->nChildren(); ++i) {
        SGPropertyNode *actionSetNode = node->getChild(i);
        const char *actionSetName = actionSetNode->getName();
        auto it = input->_actionSets.find(actionSetName);
        if (it != input->_actionSets.end()) {
            ActionSet *actionSet = (*it).second;
            // The next level of child nodes are action names
            for (int j = 0; j < actionSetNode->nChildren(); ++j) {
                SGPropertyNode *actionNode = actionSetNode->getChild(j);
                const char *actionName = actionNode->getName();
                osgXR::Action *action = actionSet->findAction(actionName);
                if (action) {
                    // Finally binding child nodes contain suggested bindings
                    for (auto bindingNode: actionNode->getChildren("binding")) {
                        const char *binding = bindingNode->getStringValue();
                        suggestBinding(action, binding);
                    }
                }
            }
        }
    }
}

// FGVRInput::ModeProcessInputSourceAction

FGVRInput::ModeProcessInputSourceAction::ModeProcessInputSourceAction(
                                                    Subaction *subaction,
                                                    osgXR::Action *action) :
    _subaction(subaction),
    _action(action)
{
}

bool FGVRInput::ModeProcessInputSourceAction::getBoolValue(bool &outValue)
{
    if (_action.valid()) {
        SubactionInfo *info = getActionCommon()->getSubactionInfo(_subaction);
        if (info)
            return info->getBoolCurValue(outValue);
    }
    return false;
}

bool FGVRInput::ModeProcessInputSourceAction::getFloatValue(float &outValue)
{
    if (_action.valid()) {
        SubactionInfo *info = getActionCommon()->getSubactionInfo(_subaction);
        if (info)
            return info->getFloatCurValue(outValue);
    }
    return false;
}

bool FGVRInput::ModeProcessInputSourceAction::getVector2fValue(osg::Vec2f &outValue)
{
    if (_action.valid()) {
        SubactionInfo *info = getActionCommon()->getSubactionInfo(_subaction);
        if (info)
            return info->getVector2fCurValue(outValue);
    }
    return false;
}

bool FGVRInput::ModeProcessInputSourceAction::getPoseValue(osgXR::ActionPose::Location &outValue)
{
    if (_action.valid()) {
        SubactionInfo *info = getActionCommon()->getSubactionInfo(_subaction);
        if (info)
            return info->getPoseCurValue(outValue);
    }
    return false;
}

// FGVRInput::ModeProcessInput

FGVRInput::ModeProcessInput::ModeProcessInput(FGVRInput::Mode *mode,
                                              Subaction *subaction,
                                              SGPropertyNode *node) :
    _lastBool(false),
    _lastFloat(0.0f),
    _lastVec2f(0.0f, 0.0f)
{
    for (SGPropertyNode *actionNode: node->getChildren("action")) {
        const char *actionName = actionNode->getStringValue();
        osgXR::Action *action = mode->getActionSet()->findAction(actionName);
        if (action) {
            _sources.push_back(std::make_unique<ModeProcessInputSourceAction>(
                                                                subaction,
                                                                action));
        } else {
            SG_LOG(SG_INPUT, SG_WARN,
                   "No VR action \"" << actionName << "\" found in action-set \"" << mode->getActionSet()->getName() << "\" for VR mode " << mode->getPath() << " input " << node->getName());
        }
    }
}

void FGVRInput::ModeProcessInput::deactivate()
{
    _lastBool = false;
    _lastFloat = 0.0f;
    _lastVec2f = osg::Vec2f();
    _lastPose = osgXR::ActionPose::Location();
}

bool FGVRInput::ModeProcessInput::getBoolValue(bool &outValue, bool *outChanged)
{
    // Use logical OR of valid values from sources (like OpenXR)
    bool ret = false;
    for (auto &source: _sources) {
        if (source->getBoolValue(outValue)) {
            if (outChanged)
                *outChanged = (outValue != _lastBool);
            _lastBool = outValue;
            ret = true;
            if (outValue)
                return true;
        }
    }
    return ret;
}

bool FGVRInput::ModeProcessInput::getFloatValue(float &outValue,
                                                bool *outChanged)
{
    // Use largest absolute valid value from sources (like OpenXR)
    bool ret = false;
    float maxVal;
    for (auto &source: _sources) {
        float val;
        if (source->getFloatValue(val)) {
            if (!ret || fabs(val) > fabs(maxVal))
                maxVal = val;
            ret = true;
        }
    }
    if (ret) {
        outValue = maxVal;
        if (outChanged)
            *outChanged = (outValue != _lastFloat);
        _lastFloat = outValue;
    }
    return false;
}

bool FGVRInput::ModeProcessInput::getVector2fValue(osg::Vec2f &outValue,
                                                   bool *outChanged)
{
    // Use longest valid value from sources (like OpenXR)
    bool ret = false;
    float maxLength2;
    osg::Vec2f maxVal;
    for (auto &source: _sources) {
        osg::Vec2f val;
        if (source->getVector2fValue(val)) {
            float length2 = val.length2();
            if (!ret || length2 > maxLength2) {
                maxLength2 = length2;
                maxVal = val;
            }
            ret = true;
        }
    }
    if (ret) {
        outValue = maxVal;
        if (outChanged)
            *outChanged = (outValue != _lastVec2f);
        _lastVec2f = outValue;
    }
    return false;
}

bool FGVRInput::ModeProcessInput::getPoseValue(osgXR::ActionPose::Location &outValue,
                                               bool *outChanged)
{
    // Use first valid value from sources
    for (auto &source: _sources) {
        if (source->getPoseValue(outValue)) {
            if (outChanged)
                *outChanged = (outValue != _lastPose);
            _lastPose = outValue;
            return true;
        }
    }
    return false;
}

// FGVRInput::ModeProcess

FGVRInput::ModeProcess::ModeProcess(FGVRInput::Mode *mode,
				    Subaction *subaction,
				    SGPropertyNode *node,
				    SGPropertyNode *statusNode) :
    _node(node),
    _statusNode(statusNode),
    _name(node->getStringValue("name")),
    _subaction(subaction)
{
}

void FGVRInput::ModeProcess::postinit(const std::string &module)
{
    postinit(_node, module);
}

SGPropertyNode *FGVRInput::ModeProcess::getInputNode(const std::string &name)
{
    return _node->getNode("inputs", true)->getNode(name, true);
}

// FGVRInput::Mode::SubactionInfo

FGVRInput::Mode::SubactionInfo::SubactionInfo(Subaction *subaction,
                                              SGPropertyNode *modeNodeCopy,
                                              SGPropertyNode *node) :
    _subaction(subaction),
    _modeNodeCopy(modeNodeCopy),
    _node(node)
{
}

FGVRInput::Mode::SubactionInfo::~SubactionInfo()
{
    for (auto *process: _processes)
        delete process;
}

void FGVRInput::Mode::SubactionInfo::readProcesses(FGVRInput *input,
                                                   Mode *mode,
                                                   SGPropertyNode *node,
                                                   SGPropertyNode *statusNode)
{
    // Read input processing nodes
    for (SGPropertyNode *processNode: node->getChildren("process")) {
        const char *processName = processNode->getStringValue("name");
        const char *processType = processNode->getStringValue("type");

        SGPropertyNode *processStatusNode = statusNode->getNode(processName,
                                                                true);

        // Create a process of the specified type
        ModeProcess *process = nullptr;
        if (!strcmp(processType, "button")) {
            process = new FGVRButton(mode, _subaction, processNode,
                                     processStatusNode);
        } else if (!strcmp(processType, "pick")) {
            process = new FGVRPick(input, mode, _subaction, processNode,
                                   processStatusNode);
        } else if (!strcmp(processType, "pose_euler")) {
            process = new FGVRPoseEuler(mode, _subaction, processNode,
                                        processStatusNode);
        } else {
            SG_LOG(SG_INPUT, SG_WARN,
                   "Unknown VR mode process type \"" << processType << "\" VR mode " << mode->getPath() << " process " << processName);
        }
        if (process)
            _processes.push_back(process);
    }
}

void FGVRInput::Mode::SubactionInfo::initNasal()
{
    initNasal(_modeNodeCopy);
    initNasal(_node);
}

void FGVRInput::Mode::SubactionInfo::initNasal(SGPropertyNode *node)
{
    simgear::ErrorReportContext errCtx("input-device", _module);
    for (SGPropertyNode *nasal: node->getChildren("nasal")) {
        nasal->setStringValue("module", _module.c_str());
        FGNasalSys *nasalsys = (FGNasalSys *)globals->get_subsystem("nasal");
        bool ok = nasalsys->handleCommand(nasal, nullptr);
        if (!ok) {
            // TODO: get the Nasal errors logged properly
            simgear::reportFailure(simgear::LoadFailure::BadData,
                                   simgear::ErrorCode::InputDeviceConfig,
                                   "Failed to parse VR action-set Nasal");
        }
    }
}

void FGVRInput::Mode::SubactionInfo::postinit(const std::string &modulePfx)
{
    std::ostringstream str;
    str << modulePfx << _subaction->getPath();
    _module = str.str();

    FGNasalSys *nasalsys = (FGNasalSys *)globals->get_subsystem("nasal");
    nasalsys->createModule(_module.c_str(), _module.c_str(), "", 0);

    for (auto *process: _processes)
        process->postinit(_module);
}

void FGVRInput::Mode::SubactionInfo::update(double dt)
{
    for (auto *process: _processes)
        process->update(dt);
}

void FGVRInput::Mode::SubactionInfo::activate()
{
    for (auto *process: _processes)
        process->activate();
}

void FGVRInput::Mode::SubactionInfo::deactivate()
{
    for (auto *process: _processes)
        process->deactivate();
}

// FGVRInput::Mode

FGVRInput::Mode::Mode(FGVRInput *input,
                      osgXR::Manager *manager,
                      const char *type,
                      const char *mode,
                      SGPropertyNode *node,
                      SGPropertyNode *dataNode)
{
    std::ostringstream modePath;
    modePath << type << "/" << mode;
    _path = modePath.str();
    node->setStringValue("mode-path", _path);

    // Get the mode's default action-set
    const char *actionSet = node->getStringValue("action-set");
    auto it = input->_actionSets.find(actionSet);
    if (it != input->_actionSets.end()) {
        _actionSet = (*it).second;
    } else {
        SG_LOG(SG_INPUT, SG_WARN,
               "Unknown action-set \"" << actionSet << "\" required by VR mode " << type << "." << mode);
        return;
    }

    // Get subactions this mode can be attached to
    auto subactions = node->getChildren("subaction");
    for (unsigned int i = 0; i < subactions.size(); ++i) {
        SGPropertyNode *subactionNode = subactions[i];
        SGPropertyNode *pathNode = subactionNode->getNode("path", false);
        if (!pathNode)
            pathNode = subactionNode;
        std::string subactionPath = pathNode->getStringValue();
        if (!subactionPath.empty()) {
            Subaction *subaction = input->getSubaction(manager, subactionPath);
            assert(_subactions.find(subaction) == _subactions.end());
            // Copy the whole mode node so bindings etc can be per-subaction
            SGPropertyNode *nodeCopy = dataNode->getChild("subaction", i, true);
            copyProperties(node, nodeCopy);
            // for fgcommands to be able to distinguish subactions, put the
            // subaction path in a property.
            nodeCopy->setStringValue("subaction-path", subactionPath);

            SGPropertyNode *subactionCopy = nodeCopy->getChild("subaction", i);

            SubactionInfo *info = new SubactionInfo(subaction, nodeCopy, subactionCopy);
            _subactions[subaction] = info;

            std::ostringstream statusPath;
            statusPath << "modes/" << type << "/" << mode << "[" << i << "]";
            SGPropertyNode *statusNode = input->getStatusNode()->getNode(statusPath.str(), true);
            // Read mode wide process objects
            info->readProcesses(input, this, nodeCopy, statusNode);

            // And subaction specific process objects
            info->readProcesses(input, this, subactionCopy, statusNode);
        }
    }
}

FGVRInput::Mode::~Mode()
{
    for (auto &subactionPair: _subactions)
        delete subactionPair.second;
}

void FGVRInput::Mode::postinit()
{
    std::ostringstream str;
    str << "vr/mode/" << _path;
    std::string module = str.str();

    for (auto &subactionPair: _subactions)
    {
        auto *info = subactionPair.second;
        info->postinit(module);
    }
}

void FGVRInput::Mode::activate(Subaction *subaction)
{
    if (_actionSet.valid())
        _actionSet->activate(subaction);

    auto it = _subactions.find(subaction);
    if (it != _subactions.end()) {
        auto *info = (*it).second;
        // Call mode init code with each activation
        info->initNasal();
        info->activate();
    }
}

void FGVRInput::Mode::deactivate(Subaction *subaction)
{
    // FIXME hmm, need per subaction ref counting in actionset activation
    if (_actionSet.valid())
        _actionSet->deactivate(subaction);

    auto it = _subactions.find(subaction);
    if (it != _subactions.end())
        (*it).second->deactivate();
}

void FGVRInput::Mode::update(Subaction *subaction, double dt)
{
    auto it = _subactions.find(subaction);
    if (it != _subactions.end())
        (*it).second->update(dt);
}

namespace {

/// Update callback for MatrixTransform to sync it to master camera.
class LocalSpaceUpdateCallback : public osg::NodeCallback
{
    public:

        LocalSpaceUpdateCallback(osg::MatrixTransform *transform) :
            _transform(transform)
        {
        }

        void operator()(osg::Node* node, osg::NodeVisitor* nv) override
        {
            CameraGroup *cgroup = CameraGroup::getDefault();
            osg::Camera *masterCam = cgroup->getView()->getCamera();

            // Update the transform object to match the master camera view
            _transform->setMatrix(masterCam->getInverseViewMatrix());

            traverse(node, nv);
        }

    protected:

        /// Transform node to adjust.
        osg::ref_ptr<osg::MatrixTransform> _transform;
};

}

// FGVRInput

FGVRInput::FGVRInput() :
    _running(false),
    _localSpaceUpdater(new osg::Group),
    _localSpace(new osg::MatrixTransform)
{
    // Only transform nodes in range are updated, so the update callback must be
    // higher up the scene graph than _localSpace.
    _localSpace->setName("LocalSpace");
    _localSpaceUpdater->setName("LocalSpaceUpdater");
    _localSpaceUpdater->addChild(_localSpace);
    _localSpaceUpdater->setUpdateCallback(new LocalSpaceUpdateCallback(_localSpace));
}

FGVRInput::~FGVRInput()
{
    _remove(true);
}

void FGVRInput::_remove(bool all)
{
    // Clean up references to actions so action sets can be destroyed
    for (auto &actionSetPair: _actionSets)
        actionSetPair.second->cleanupActions();
    _actionSets.clear();
    _profiles.clear();

    // Clean up interaction modes
    for (auto &subactionPair: _subactions)
        subactionPair.second->clearModes();
    for (auto &modePair: _modes)
        delete modePair.second;
}

void FGVRInput::init()
{
    SG_LOG(SG_INPUT, SG_DEBUG, "Initializing VR bindings");
    SGPropertyNode_ptr vrNode = fgGetNode("/input/vr", true);
    _statusNode = fgGetNode("/devices/status/vr", true);

    VRManager *manager = VRManager::instance();

    // Set up local space updating
    osg::Group* sceneGroup = globals->get_scenery()->get_scene_graph();
    sceneGroup->addChild(_localSpaceUpdater);

    // Set up action sets from property tree
    SGPropertyNode_ptr actionSetsNode = vrNode->getNode("action-sets", true);
    for (int i = 0; i < actionSetsNode->nChildren(); ++i) {
        SGPropertyNode *actionSetNode = actionSetsNode->getChild(i);
        const char *name = actionSetNode->getName();
        _actionSets[name] = new ActionSet(this, manager, actionSetNode);
    }

    // Set up interaction profiles from property tree
    SGPropertyNode_ptr interactionProfilesNode = vrNode->getNode("interaction-profiles", true);
    for (int i = 0; i < interactionProfilesNode->nChildren(); ++i) {
        SGPropertyNode *vendorNode = interactionProfilesNode->getChild(i);
        const char *vendor = vendorNode->getName();
        for (int j = 0; j < vendorNode->nChildren(); ++j) {
            SGPropertyNode *profileNode = vendorNode->getChild(j);
            const char *type = profileNode->getName();
            _profiles.push_back(new InteractionProfile(this, manager, vendor,
                                                       type, profileNode));
        }
    }

    // Sync action setup changes
    manager->syncActionSetup();

    // Set up interaction modes from property tree
    SGPropertyNode *modesNode = vrNode->getNode("modes", true);
    SGPropertyNode *modeCopy = vrNode->getNode("mode-data", true);
    for (int i = 0; i < modesNode->nChildren(); ++i) {
        SGPropertyNode *typeNode = modesNode->getChild(i);
        const char *type = typeNode->getName();
        SGPropertyNode *typeCopy = modeCopy->getNode(type, true);
        for (int j = 0; j < typeNode->nChildren(); ++j) {
            SGPropertyNode *modeNode = typeNode->getChild(j);
            const char *modeName = modeNode->getName();
            SGPropertyNode *modeCopy = typeCopy->getNode(modeName, true);
            auto *mode = new Mode(this, manager, type, modeName, modeNode,
                                  modeCopy);
            const std::string &modePath = mode->getPath();
            assert(_modes.find(modePath) == _modes.end());
            _modes[modePath] = mode;
        }
    }

    // Set up subactions
    for (SGPropertyNode *subactionNode: vrNode->getChildren("subaction")) {
        const char *subactionPath = subactionNode->getStringValue("path");
        Subaction *subaction = getSubaction(manager, subactionPath);
        subaction->setup(subactionNode);

        // Set default mode
        const char *defaultMode = subaction->getPresetMode("default");
        auto it = _modes.find(defaultMode);
        if (it != _modes.end())
            subaction->pushMode((*it).second);
    }
}

void FGVRInput::postinit()
{
    SGPropertyNode_ptr js_nodes = fgGetNode("/input/vr");

    // Init ActionSet nasal modules and Action bindings
    for (auto &actionSetPair: _actionSets)
        actionSetPair.second->postinit();
    // Init interaction mode nasal modules and input bindings
    for (auto &modePair: _modes)
        modePair.second->postinit();
}

void FGVRInput::reinit()
{
    SG_LOG(SG_INPUT, SG_DEBUG, "Re-Initializing VR bindings");
    _remove(false);

    FGVRInput::init();
    FGVRInput::postinit();
}

void FGVRInput::update(double dt)
{
    // Only bother updating while a session is running (and once after to reset
    // the input values)
    VRManager *manager = VRManager::instance();
    bool running = manager->isRunning();
    if (!running && !_running)
        return;
    _running = running;

    // Find action values
    for (auto &actionSetPair: _actionSets)
        actionSetPair.second->update(dt);
    // Fire bindings for changes
    for (auto &actionSetPair: _actionSets)
        actionSetPair.second->fireBindings();
    // Handle subactions (and current interaction modes)
    for (auto &subactionPair: _subactions)
        subactionPair.second->update(dt);
}

FGVRInput::Mode *FGVRInput::getTranslatedMode(FGVRInput::Subaction *subaction,
                                              const char *modePath)
{
    if (!modePath)
        return nullptr;

    // Try translating mode via the subaction presets
    const char *translatedModePath = subaction->getPresetMode(modePath);
    if (translatedModePath)
        modePath = translatedModePath;

    // Find the mode object
    auto it = _modes.find(modePath);
    if (it != _modes.end())
        return (*it).second;

    return nullptr;
}

void FGVRInput::findModeSubaction(const SGPropertyNode *node,
                                  FGVRInput::Mode **outMode,
                                  FGVRInput::Subaction **outSubaction)
{
    const char *modePath = nullptr;
    const char *subactionPath = nullptr;
    if (outSubaction)
        subactionPath = node->getStringValue("subaction-path", nullptr);

    if (outMode || (outSubaction && !subactionPath)) {
        // Search upward for a parent with mode-path
        const SGPropertyNode *parent;
        for (parent = node->getParent(); parent; parent = parent->getParent()) {
            modePath = parent->getStringValue("mode-path", nullptr);
            if (modePath)
                break;
        }
        if (parent && outSubaction && !subactionPath)
            subactionPath = parent->getStringValue("subaction-path", nullptr);
    }

    if (outMode && modePath) {
        // Find the mode
        auto it = _modes.find(modePath);
        if (it != _modes.end())
            *outMode = (*it).second;
    }

    if (outSubaction && subactionPath) {
        // Find the subaction
        auto it = _subactions.find(subactionPath);
        if (it != _subactions.end())
            *outSubaction = (*it).second;
    }
}

bool FGVRInput::handleModePushCommand(const SGPropertyNode *arg,
                                      SGPropertyNode *root)
{
    // Find the subaction and optionally current mode
    Mode *curMode = nullptr;
    Subaction *subaction = nullptr;
    findModeSubaction(arg, &curMode, &subaction);
    if (!subaction) {
        SG_LOG(SG_GENERAL, SG_ALERT,
               "do_vr_mode_push command: Could not find subaction");
        return false;
    }

    // Find the destination mode
    Mode *mode = getTranslatedMode(subaction,
                                   arg->getStringValue("mode", nullptr));
    if (!mode) {
        SG_LOG(SG_GENERAL, SG_ALERT,
               "do_vr_mode_push command: Mode not found");
        return false;
    }

    // Drop modes in front of current mode
    if (curMode)
        subaction->popModesFront(curMode);
    // And push new mode
    subaction->pushMode(mode);
    return true;
}

bool FGVRInput::handleModeToggleCommand(const SGPropertyNode *arg,
                                        SGPropertyNode *root)
{
    // Find the subaction and optionally current mode
    Mode *curMode = nullptr;
    Subaction *subaction = nullptr;
    findModeSubaction(arg, &curMode, &subaction);
    if (!subaction) {
        SG_LOG(SG_GENERAL, SG_ALERT,
               "do_vr_mode_toggle command: Could not find subaction");
        return false;
    }

    // Find the destination mode
    Mode *mode = getTranslatedMode(subaction,
                                   arg->getStringValue("mode", nullptr));
    if (!mode) {
        SG_LOG(SG_GENERAL, SG_ALERT,
               "do_vr_mode_toggle command: Mode not found");
        return false;
    }

    // Drop modes in front of current mode
    bool doPush = false;
    if (curMode)
        doPush = !subaction->popModesFront(curMode);
    // And push new mode
    if (doPush)
        subaction->pushMode(mode);
    return true;
}

FGVRInput::Subaction *FGVRInput::getSubaction(osgXR::Manager *manager,
                                              const std::string &path)
{
    auto it = _subactions.find(path);
    if (it != _subactions.end())
        return (*it).second.get();

    Subaction *subaction = new Subaction(manager, path);
    _subactions[path] = subaction;
    return subaction;
}


// Register the subsystem.
SGSubsystemMgr::Registrant<FGVRInput> registrantFGVRInput(
    SGSubsystemMgr::GENERAL,
    {{"nasal", SGSubsystemMgr::Dependency::HARD}});
