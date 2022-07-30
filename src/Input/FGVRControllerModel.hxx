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

#ifndef _FGVRCONTROLLER_MODEL_HXX
#define _FGVRCONTROLLER_MODEL_HXX

#include "FGButton.hxx"
#include "FGVRInput.hxx"

#include <memory>

/**
 * A mode process object to handle rendering a 3D model of a VR controller.
 */
class FGVRControllerModel : public FGVRInput::ModeProcess
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
    FGVRControllerModel(FGVRInput* input, FGVRInput::Mode* mode,
                        FGVRInput::Subaction* subaction, SGPropertyNode* node,
                        SGPropertyNode* statusNode);
    ~FGVRControllerModel();

    // Implement ModeProcess virtual functions
    void postinit(SGPropertyNode* node,
                  const std::string& module) override;
    void update(double dt) override;
    void deactivate() override;

protected:
    /// The pose input to position the controller model.
    FGVRInput::ModeProcessInput _pose;

    class Private;
    std::unique_ptr<Private> _private;
};

#endif
