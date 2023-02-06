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

#ifndef _FGVRHAND_INTERACTION_HXX
#define _FGVRHAND_INTERACTION_HXX

#include "FGVRInput.hxx"

#include <memory>

/**
 * A mode process object to handle hand interaction.
 */
class FGVRHandInteraction : public FGVRInput::ModeProcess
{
public:
    /**
     * Construct from a property node.
     * @param input      VR input subsystem object.
     * @param mode       Interaction mode object.
     * @param subaction  Subaction the mode is tied to.
     * @param node       Property node describing the process object.
     * @param statusNode Property node for describing the process object status.
     */
    FGVRHandInteraction(FGVRInput* input, FGVRInput::Mode* mode,
                        FGVRInput::Subaction* subaction, SGPropertyNode* node,
                        SGPropertyNode* statusNode);
    ~FGVRHandInteraction();

    // Implement ModeProcess virtual functions
    void postinit(SGPropertyNode* node,
                  const std::string& module) override;
    void update(double dt) override;
    void deactivate() override;

protected:
    /// The boolean input for palm grab.
    FGVRInput::ModeProcessInput _grabPalm;
    /// The boolean inputs for finger grabs.
    FGVRInput::ModeProcessInput _grabFingers[5];

    class Private;
    std::unique_ptr<Private> _private;
};

#endif
