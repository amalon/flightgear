// Handle picking using VR controller
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

#include "FGVRPick.hxx"

#include <Main/fg_props.hxx>
#include <Viewer/renderer.hxx>

#include <simgear/scene/util/OsgMath.hxx>

#include <osg/Geode>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/ref_ptr>

using namespace flightgear;

// FGVRPick::Private

class FGVRPick::Private
{
public:
    void hover(PickList& pickList);
    void buttonDown(unsigned int button, PickList& pickList);
    void buttonUp(unsigned int button);
    void update(double dt);

    FGVRInput* _input;
    osg::ref_ptr<osg::Switch> _pickSwitch;
    osg::ref_ptr<osg::Geode> _pickGeode;
    osg::ref_ptr<osg::Geometry> _pickGeom;
    osg::ref_ptr<osg::Vec3Array> _pickVerts;
    osg::ref_ptr<osg::StateSet> _stateSetMiss;
    osg::ref_ptr<osg::StateSet> _stateSetHit;

protected:
    SGSceneryPick _hoverPick;
    SGSceneryPick _buttonPicks[2];
};

void FGVRPick::Private::hover(PickList& pickList)
{
    osg::Vec2d dummyPos(0, 0);
    for (auto& pick : pickList) {
        // Unchanged hover, no change
        if (pick.callback == _hoverPick.callback)
            return;
        // New hover, leave previously hovered pick
        if (pick.callback.valid() && pick.callback->hover(dummyPos, pick.info)) {
            if (_hoverPick.callback.valid())
                _hoverPick.callback->mouseLeave(dummyPos);
            _hoverPick = pick;
            return;
        }
    }
    // No hover, leave previously hovered pick
    if (_hoverPick.callback.valid())
        _hoverPick.callback->mouseLeave(dummyPos);
    _hoverPick.callback.reset();
}

void FGVRPick::Private::buttonDown(unsigned int button, PickList& pickList)
{
    // FIXME broken, ea.getGraphicsContext()==nullptr breaks
    // simgear/scene/model/SGPickAnimation.cxx eventToWindowCoords()
    osgGA::GUIEventAdapter* ea = osgGA::GUIEventAdapter::getAccumulatedEventState().get();
    for (auto& pick : pickList) {
        if (pick.callback.valid() && pick.callback->buttonPressed(button, *ea, pick.info)) {
            _buttonPicks[button] = pick;
            return;
        }
    }
}

void FGVRPick::Private::buttonUp(unsigned int button)
{
    // FIXME broken, ea.getGraphicsContext()==nullptr breaks
    // simgear/scene/model/SGPickAnimation.cxx eventToWindowCoords()
    osgGA::GUIEventAdapter* ea = osgGA::GUIEventAdapter::getAccumulatedEventState().get();
    if (_buttonPicks[button].callback.valid())
        _buttonPicks[button].callback->buttonReleased(button, *ea, &_buttonPicks[button].info);
    _buttonPicks[button].callback.reset();
}

void FGVRPick::Private::update(double dt)
{
    int keyModState = fgGetKeyModifiers();
    for (auto& pick : _buttonPicks)
        if (pick.callback.valid())
            pick.callback->update(dt, keyModState);
}


// FGVRPick

static osg::StateSet* createPickStateSet(const osg::Vec4f& color, float width)
{
    int forceOff = osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED;
    osg::StateSet* ss = new osg::StateSet;

    // It should look like some sort of laser
    ss->setMode(GL_LIGHTING, forceOff);
    if (width != 1.0f)
        ss->setAttribute(new osg::LineWidth(width));

    // Material setup
    osg::Material* material = new osg::Material;
    material->setColorMode(osg::Material::OFF);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
    ss->setAttribute(material);

    return ss;
}

FGVRPick::FGVRPick(FGVRInput* input,
                   FGVRInput::Mode* mode,
                   FGVRInput::Subaction* subaction,
                   SGPropertyNode* node,
                   SGPropertyNode* statusNode)
    : ModeProcess(mode, subaction, node, statusNode),
      _pose(mode, subaction, getInputNode("pose")),
      _mouseLeft(mode, subaction, getInputNode("mouse-left-click")),
      _mouseMiddle(mode, subaction, getInputNode("mouse-middle-click")),
      _reach(node->getDoubleValue("reach", 5.0f)),
      _private(std::make_unique<Private>())
{
    // Create buffers for a simple line
    osg::Vec3Array* vertices = _private->_pickVerts = new osg::Vec3Array(2);
    osg::DrawArrays* prim = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, 2);

    // Create a geometry for the line
    osg::Geometry* pickGeom = _private->_pickGeom = new osg::Geometry;
    pickGeom->setVertexArray(vertices);
    pickGeom->addPrimitiveSet(prim);
    pickGeom->setUseDisplayList(false);

    // Create a geode for the line
    osg::Geode* pickGeode = _private->_pickGeode = new osg::Geode;
    pickGeode->addDrawable(pickGeom);

    // Hard code hit & miss style for now
    _private->_stateSetMiss = createPickStateSet(osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f),
                                                 2);
    _private->_stateSetHit = createPickStateSet(osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f),
                                                3);
    pickGeode->setStateSet(_private->_stateSetMiss);

    // Switch on and off
    osg::Switch* sw = _private->_pickSwitch = new osg::Switch();
    sw->addChild(pickGeode);

    // Add it to the local space group
    _private->_input = input;
    input->getLocalSpaceGroup()->addChild(sw);
}

FGVRPick::~FGVRPick()
{
    // Remove the line from the local space group
    _private->_input->getLocalSpaceGroup()->removeChild(_private->_pickSwitch);
}

void FGVRPick::postinit(SGPropertyNode* node,
                        const std::string& module)
{
}

void FGVRPick::update(double dt)
{
    PickList pickList;
    osgXR::ActionPose::Location location;

    bool active = _pose.getPoseValue(location) &&
                  location.isPositionValid() && location.isOrientationValid();
    _private->_pickSwitch->setValue(0, active);
    if (active) {
        // get line segment in scene space
        osg::Vec3d start, end;
        start = location.getPosition();
        end = start + location.getOrientation() * osg::Vec3d(0, 0, -_reach);
        // Keep a copy of local ray
        (&_private->_pickVerts->front())[0] = start;
        (&_private->_pickVerts->front())[1] = end;
        auto& invMatrix = _private->_input->getLocalSpaceGroup()->getMatrix();
        start = start * invMatrix;
        end = end * invMatrix;

        // perform the pick
        pickList = globals->get_renderer()->pick(start, end);

        if (pickList.empty()) {
            _private->_pickGeode->setStateSet(_private->_stateSetMiss);
        } else {
            _private->_pickGeode->setStateSet(_private->_stateSetHit);

            // Find vector to first item
            end = toOsg(pickList.front().info.wgs84);
            auto& matrix = _private->_input->getLocalSpaceGroup()->getInverseMatrix();
            (&_private->_pickVerts->front())[1] = end * matrix;
        }
        _private->_pickGeom->setVertexArray(_private->_pickVerts);

        // Handle hovering
        _private->hover(pickList);

        // Handle inputs
        bool value, changed;
        if (_mouseLeft.getBoolValue(value, &changed) && changed) {
            if (value)
                _private->buttonDown(0, pickList);
            else
                _private->buttonUp(0);
        }
        if (_mouseMiddle.getBoolValue(value, &changed) && changed) {
            if (value)
                _private->buttonDown(1, pickList);
            else
                _private->buttonUp(1);
        }

        // Update active pick callbacks
        _private->update(dt);
    }
}

void FGVRPick::deactivate()
{
    _pose.deactivate();
    _private->_pickSwitch->setValue(0, false);

    if (_mouseLeft.getLastBoolValue()) {
        _mouseLeft.deactivate();
        _private->buttonUp(0);
    }
    if (_mouseMiddle.getLastBoolValue()) {
        _mouseMiddle.deactivate();
        _private->buttonUp(1);
    }
}
