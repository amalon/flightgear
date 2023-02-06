// Handle a VR hand interaction
//
// Copyright (C) 2023  James Hogan
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

#include "FGVRHandInteraction.hxx"
#include "FGVRHand.hxx"

#include <GUI/Highlight.hxx>
#include <Main/fg_props.hxx>
#include <Viewer/VRManager.hxx>

#include <simgear/scene/util/RenderConstants.hxx>
#include <simgear/scene/util/SGSceneUserData.hxx>

#include <osg/ref_ptr>

using namespace flightgear;

// FGVRHandInteraction::Private

class FGVRHandInteraction::Private
{
public:
    FGVRInput* _input;

    /// Tracked hand pose.
    std::shared_ptr<osgXR::HandPoseTracked> _handTracking;

    /// FG interacting hand pose.
    std::shared_ptr<FGVRHand> _handPose;

    /// Visible hand node.
    osg::ref_ptr<osgXR::Hand> _hand;

    struct Contact {
        struct SGSceneryPick pick;
        //std::shared_ptr<SGSpringPickContact> contact;
    };

    /// Contact points (5 fingers + palm).
    Contact _contacts[6];
};


// FGVRHandInteraction

FGVRHandInteraction::FGVRHandInteraction(FGVRInput* input,
                                         FGVRInput::Mode* mode,
                                         FGVRInput::Subaction* subaction,
                                         SGPropertyNode* node,
                                         SGPropertyNode* statusNode)
    : ModeProcess(mode, subaction, node, statusNode),
      _grabPalm(mode, subaction, getInputNode("grab-palm")),
      _grabFingers{
          FGVRInput::ModeProcessInput(mode, subaction, getInputNode("grab-thumb")),
          FGVRInput::ModeProcessInput(mode, subaction, getInputNode("grab-index")),
          FGVRInput::ModeProcessInput(mode, subaction, getInputNode("grab-middle")),
          FGVRInput::ModeProcessInput(mode, subaction, getInputNode("grab-ring")),
          FGVRInput::ModeProcessInput(mode, subaction, getInputNode("grab-little")),
      },
      _private(std::make_unique<Private>())
{
    auto& subactionPath = subaction->getPath();
    std::string namePrefix = mode->getPath() + " " + _name + " " + subactionPath + " ";

    _private->_input = input;

    osgXR::HandPose::Hand hand;
    if (subactionPath == "/user/hand/left") {
        hand = osgXR::HandPose::HAND_LEFT;
    } else if (subactionPath == "/user/hand/right") {
        hand = osgXR::HandPose::HAND_RIGHT;
    } else {
        SG_LOG(SG_INPUT, SG_WARN,
               "Unknown VR hand interaction subaction path \"" << subactionPath << "\" for "
               << mode->getPath() + " " + _name + " " + subactionPath);
        return;
    }

    VRManager* manager = VRManager::instance();
    auto* localSpaceGroup = input->getLocalSpaceGroup();

    _private->_handTracking = std::make_shared<osgXR::HandPoseTracked>(manager, hand);

    _private->_handPose = std::make_shared<FGVRHand>(localSpaceGroup, _private->_handTracking);

    _private->_hand = new osgXR::Hand(_private->_handPose);
    _private->_hand->setNodeMask(~simgear::PICK_BIT);
    _private->_hand->setName(namePrefix + "hand");

    localSpaceGroup->addChild(_private->_hand);
}

FGVRHandInteraction::~FGVRHandInteraction()
{
    // Remove the hand from the local space group
    _private->_input->getLocalSpaceGroup()->removeChild(_private->_hand);
}

void FGVRHandInteraction::postinit(SGPropertyNode* node,
                                   const std::string& module)
{
}

void FGVRHandInteraction::update(double dt)
{
    bool grabs[6] = {};
    bool grabsChanged[6] = {};

    for (unsigned int i = 0; i < 5; ++i) {
        _grabFingers[i].getBoolValue(grabs[i], &grabsChanged[i]);
        _private->_handPose->setFingerHoverDistance(i, grabs[i] ? 0.0f : 0.1f);
    }
    _grabPalm.getBoolValue(grabs[5], &grabsChanged[5]);

    _private->_handPose->advance(dt);

    const osg::NodePath* grabNodes[6];
    for (unsigned int i = 0; i < 5; ++i)
        grabNodes[i] = _private->_handPose->getFingerTouchingNodePath(i);
    grabNodes[5] = _private->_handPose->getPalmTouchingNodePath();

    auto highlight = globals->get_subsystem<Highlight>();
    int higlight_num_props = 0;

    for (unsigned int grab = 0; grab < 6; ++grab) {
        if (!grabs[grab] || !grabNodes[grab])
            continue;

        // Construct a chain of pick callbacks, tip to root
        bool pickFound = false;
        for (auto it = grabNodes[grab]->begin(); it != grabNodes[grab]->end(); ++it) {
            SGSceneUserData* ud = SGSceneUserData::getSceneUserData(*it);
            if (!ud || (ud->getNumPickCallbacks() == 0)) {
                continue;
            }

            for (unsigned i = 0; i < ud->getNumPickCallbacks(); ++i) {
                SGPickCallback* pickCallback = ud->getPickCallback(i);
                if (!pickCallback)
                    continue;

#if 0
                auto* contactConfig = pickCallback->getContactConfig();
                if (!contactConfig)
                    continue;
                if (!contactConfig->isEnabled())
                    continue;

                // If top pick callback is different to last time, clear contact and
                // update
                if (pickCallback != _private->_contacts[grab].pick.callback) {
                    if (_private->_contacts[grab].contact)
                        _private->_contacts[grab].contact->setStale();
                    _private->_contacts[grab].contact = nullptr;
                    _private->_contacts[grab].pick.callback = pickCallback;
                }
                // Create a new spring contact
                if (!_private->_contacts[grab].contact) {
                    _private->_contacts[grab].contact = std::make_shared<SGSpringPickContact>();
                    pickCallback->addContact(_private->_contacts[grab].contact, *grabNodes[grab]);
                    pickFound = true;
                    break;
                }
#endif
            }
            if (pickFound)
                break;
        }
    }


    // Events to pick objects
    // - interract
    // - alternate interact
    // - start grab
    // - move grab
    // - stop grab
#if 0
        // How about some interaction?
        for (unsigned int i = 0; i < 5; ++i) {
            if (!_fingersRange[i].touchingNodes.empty()) {
                std::cout << "Finger " << i << " touching";
                for (auto* node: _fingersRange[i].touchingNodes)
                    std::cout << " '" << node->getName() << "'";
                std::cout << std::endl;
            }
        }
        if (!_wristRange.touchingNodes.empty()) {
            std::cout << "Wrist touching";
                for (auto* node: _wristRange.touchingNodes)
                    std::cout << " '" << node->getName() << "'";
                std::cout << std::endl;
        }
#endif

}

void FGVRHandInteraction::deactivate()
{
    _grabPalm.deactivate();
    for (auto& grabFinger: _grabFingers)
        grabFinger.deactivate();

    // FIXME deactivate hand tracking stuff
}
