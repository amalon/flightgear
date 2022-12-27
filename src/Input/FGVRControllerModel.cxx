// Handle a VR controller model
//
// Copyright (C) 2022  James Hogan
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

#include "FGVRControllerModel.hxx"

#include <Main/fg_props.hxx>
#include <Viewer/renderer.hxx>

#include <simgear/scene/util/RenderConstants.hxx>

#include <osg/Geode>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/ref_ptr>

using namespace flightgear;

// FGVRControllerModel::Private

class FGVRControllerModel::Private
{
public:
    FGVRInput* _input;

    /// Wrapper around pose transform node for hiding controller.
    osg::ref_ptr<osg::Switch> _transformSwitch;

    /// Pose transform scene graph node.
    osg::ref_ptr<osg::MatrixTransform> _transform;
};


// FGVRControllerModel

FGVRControllerModel::FGVRControllerModel(FGVRInput* input,
                                         FGVRInput::Mode* mode,
                                         FGVRInput::Subaction* subaction,
                                         SGPropertyNode* node,
                                         SGPropertyNode* statusNode)
    : ModeProcess(mode, subaction, node, statusNode),
      _pose(mode, subaction, getInputNode("pose")),
      _private(std::make_unique<Private>())
{
    std::string namePrefix = mode->getPath() + " " + _name + " " + subaction->getPath() + " ";

    // Create buffers for a simple cuboid
    const float w = 0.02f;
    const float h = 0.09f;
#define VERT_COUNT (4 * 6)
    const osg::Vec3 verticesRaw[VERT_COUNT] = {
        /* bottom */
        { -w, -w, -h }, { -w,  w, -h }, {  w,  w, -h }, {  w, -w, -h },
        /* top */
        { -w, -w,  h }, { -w,  w,  h }, {  w,  w,  h }, {  w, -w,  h },
        /* sides */
        { -w, -w, -h }, { -w, -w,  h }, { -w,  w,  h }, { -w,  w, -h },
        { -w,  w, -h }, { -w,  w,  h }, {  w,  w,  h }, {  w,  w, -h },
        {  w,  w, -h }, {  w,  w,  h }, {  w, -w,  h }, {  w, -w, -h },
        {  w, -w, -h }, {  w, -w,  h }, { -w, -w,  h }, { -w, -w, -h },
    };
    const osg::Vec3 normalsRaw[VERT_COUNT] = {
        /* bottom */
        {  0,  0, -1 }, {  0,  0, -1 }, {  0,  0, -1 }, {  0,  0, -1 },
        /* top */
        {  0,  0,  1 }, {  0,  0,  1 }, {  0,  0,  1 }, {  0,  0,  1 },
        /* sides */
        { -1,  0,  0 }, { -1,  0,  0 }, { -1,  0,  0 }, { -1,  0,  0 },
        {  0,  1,  0 }, {  0,  1,  0 }, {  0,  1,  0 }, {  0,  1,  0 },
        {  1,  0,  0 }, {  1,  0,  0 }, {  1,  0,  0 }, {  1,  0,  0 },
        {  0, -1,  0 }, {  0, -1,  0 }, {  0, -1,  0 }, {  0, -1,  0 },
    };
    osg::Vec3Array* vertices = new osg::Vec3Array(VERT_COUNT, verticesRaw);
    osg::Vec3Array* normals = new osg::Vec3Array(VERT_COUNT, normalsRaw);
    osg::DrawArrays* prim = new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, VERT_COUNT);

    // Create a geometry for the controller
    osg::Geometry* geom = new osg::Geometry;
    geom->setVertexArray(vertices);
    geom->setNormalArray(normals, osg::Array::BIND_PER_VERTEX);
    geom->addPrimitiveSet(prim);

    // Create a geode for the controller
    osg::Geode* geode = new osg::Geode;
    geode->setName(namePrefix + "Controller");
    geode->addDrawable(geom);

    // And a state set
    osg::ref_ptr<osg::StateSet> state = geode->getOrCreateStateSet();

    // Material setup
    osg::Material* material = new osg::Material;
    material->setColorMode(osg::Material::OFF);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4f(0.5f, 0.5f, 0.5f, 1.0f));
    state->setAttribute(material);

    // Transform node to move around within local space
    osg::MatrixTransform* transform = _private->_transform = new osg::MatrixTransform;
    transform->setName(namePrefix + "Pose");
    transform->addChild(geode);

    // Switch on and off
    osg::Switch* sw = _private->_transformSwitch = new osg::Switch();
    sw->setName(namePrefix + "Switch");
    sw->setNodeMask(~simgear::PICK_BIT);
    sw->addChild(transform);

    // Add it to the local space group
    _private->_input = input;
    input->getLocalSpaceGroup()->addChild(sw);
}

FGVRControllerModel::~FGVRControllerModel()
{
    // Remove the line from the local space group
    _private->_input->getLocalSpaceGroup()->removeChild(_private->_transformSwitch);
}

void FGVRControllerModel::postinit(SGPropertyNode* node,
                                   const std::string& module)
{
}

void FGVRControllerModel::update(double dt)
{
    osgXR::ActionPose::Location location;

    bool active = _pose.getPoseValue(location) &&
                  location.isPositionValid() && location.isOrientationValid();
    _private->_transformSwitch->setValue(0, active);
    if (active) {
        osg::Matrix mat(location.getOrientation());
        mat.setTrans(location.getPosition());
        _private->_transform->setMatrix(mat);
    }
}

void FGVRControllerModel::deactivate()
{
    _pose.deactivate();
    _private->_transformSwitch->setValue(0, false);
}
