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

#include "config.h"

#include "FGVRButton.hxx"

#include <Main/fg_props.hxx>

// FGVRButton

FGVRButton::FGVRButton(FGVRInput::Mode* mode,
                       FGVRInput::Subaction* subaction,
                       SGPropertyNode* node,
                       SGPropertyNode* statusNode)
    : ModeProcess(mode, subaction, node, statusNode),
      _input(mode, subaction, getInputNode("input")),
      _statusProp(statusNode)
{
}

void FGVRButton::postinit(SGPropertyNode* node,
                          const std::string& module)
{
    _button.init(node, _name, module);
}

void FGVRButton::update(double dt)
{
    int modifiers = fgGetKeyModifiers();
    bool val;
    if (_input.getBoolValue(val)) {
        _button.update(modifiers, val);
        _statusProp = val;
    }
}

void FGVRButton::deactivate()
{
    if (_input.getLastBoolValue()) {
        _input.deactivate();
        _statusProp = false;
        _button.update(fgGetKeyModifiers(), false);
    }
}
