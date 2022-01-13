// Handle VR boolean actions as buttons
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

#ifndef _FGVRBUTTON_HXX
#define _FGVRBUTTON_HXX

#include "FGButton.hxx"
#include "FGVRInput.hxx"

/**
 * A mode process object to treat a boolean input as a button.
 * This is a straightforward pass through mode process object for boolean
 * actions.
 */
class FGVRButton : public FGVRInput::ModeProcess
{
public:
    /**
     * Construct from a property node.
     * @param mode       Interaction mode object.
     * @param subaction  Subaction the mode is tied to.
     * @param node       Property node describing the process object.
     * @param statusNode Property node for describing the process object status.
     */
    FGVRButton(FGVRInput::Mode* mode, FGVRInput::Subaction* subaction,
               SGPropertyNode* node, SGPropertyNode* statusNode);

    // Implement ModeProcess virtual functions
    void postinit(SGPropertyNode* node,
                  const std::string& module) override;
    void update(double dt) override;
    void deactivate() override;

protected:
    /// The boolean input.
    FGVRInput::ModeProcessInput _input;
    /// The property object for describing the process object status.
    SGPropObjBool _statusProp;
    /// Generic button object hands most of the specifics.
    FGButton _button;
};

#endif
