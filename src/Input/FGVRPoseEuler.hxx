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

#ifndef _FGVRPOSEEULER_HXX
#define _FGVRPOSEEULER_HXX

#include "FGVRInput.hxx"
#include "FGVRPoseEuler.hxx"

/**
 * A mode process object which extracts euler angles from a pose action.
 * Pose actions provide a position vector and orientation quaternion. In
 * order to use the VR controllers as direct input devices which feel like
 * real flight controls, these need converting into euler axis angles as a
 * joystick would normally do.
 */
class FGVRPoseEuler : public FGVRInput::ModeProcess
{
    public:

        /**
         * Construct from a property node.
         * @param mode       Interaction mode object.
         * @param subaction  Subaction the mode is tied to.
         * @param node       Property node describing the process object.
         * @param statusNode Property node for describing the process object status.
         */
        FGVRPoseEuler(FGVRInput::Mode *mode, FGVRInput::Subaction *subaction,
                      SGPropertyNode *node, SGPropertyNode *statusNode);

        // Implement ModeProcess virtual functions
        void postinit(SGPropertyNode *node,
                      const std::string &module) override;
        void update(double dt) override;

    protected:

        /// The pose input.
        FGVRInput::ModeProcessInput _pose;
        /// The quaternion transformation to apply first.
        SGQuatd _transform;
        /// The property object for describing the pose euler angles.
        SGPropObjDouble _statusPropEuler[3];
        /// Bindings.
        binding_list_t _bindings[KEYMOD_MAX];
        /// The previous euler angle values.
        double _lastValue[3];
};

#endif
