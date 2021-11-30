// Handle VR pose actions as euler angles
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

#include "FGVRPoseEuler.hxx"

#include <simgear/math/SGMisc.hxx>

#include <Main/fg_props.hxx>

// FGVRPoseEuler

FGVRPoseEuler::FGVRPoseEuler(FGVRInput::Mode* mode,
                             FGVRInput::Subaction* subaction,
                             SGPropertyNode* node,
                             SGPropertyNode* statusNode)
    : ModeProcess(mode, subaction, node, statusNode),
      _pose(mode, subaction, getInputNode("pose")),
      _transform(SGQuatd::unit()),
      _statusPropEuler{SGPropObjDouble(statusNode->getChild("euler", 0, true)),
                       SGPropObjDouble(statusNode->getChild("euler", 1, true)),
                       SGPropObjDouble(statusNode->getChild("euler", 2, true))},
      _lastValue{0.0, 0.0, 0.0}
{
    // Get the transform
    SGPropertyNode* transform = node->getChild("transform", 0, false);
    if (transform) {
        double yaw = transform->getDoubleValue("yaw-deg", 0.0);
        double pitch = transform->getDoubleValue("pitch-deg", 0.0);
        double roll = transform->getDoubleValue("roll-deg", 0.0);
        _transform = SGQuatd::fromEulerDeg(yaw, pitch, roll);
    }
}

void FGVRPoseEuler::postinit(SGPropertyNode* node,
                             const std::string& module)
{
    read_bindings(node, _bindings, KEYMOD_NONE, module);
}

void FGVRPoseEuler::update(double dt)
{
    osgXR::ActionPose::Location location;
    if (_pose.getPoseValue(location)) {
        if (location.isOrientationValid()) {
            // Convert orientation to SGQuatd
            const auto& xrQuat = location.getOrientation();
            SGQuatd quat(xrQuat._v);

            /*
             * Transform into sensible euler friendly coordinate space
             * OpenXR space: x=right, y=up, z=behind
             * Desired space: x=right, y=forward, z=up
             * So thats yaw left 90, pitch up 90
             */
            static const SGQuatd fixedTransform = SGQuatd::fromEulerDeg(-90.0, 90.0, 0.0);
            quat = conj(fixedTransform) * (quat * fixedTransform);

            // Transform by the custom input transformation
            quat = quat * _transform;

            // Calculate euler angles
            double value[3];
            quat.getEulerRad(value[2], value[1], value[0]);

            // Fire bindings
            bool changed = false;
            for (unsigned int i = 0; i < 3; ++i) {
                if (value[i] != _lastValue[i]) {
                    changed = true;
                    _statusPropEuler[i] = SGMiscd::rad2deg(value[i]);
                }
            }
            if (changed) {
                if (!_bindings[KEYMOD_NONE].empty()) {
                    for (unsigned int i = 0; i < 3; ++i)
                        _lastValue[i] = value[i];
                    for (auto& binding : _bindings[KEYMOD_NONE])
                        binding->fire(_statusNode);
                }
            }
        }
    }
}
