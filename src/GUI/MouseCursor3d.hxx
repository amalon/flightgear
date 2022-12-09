// Cursor3d.hxx -- handle user input from mouse devices
//
// Copyright (C) 2022 James Hogan <james@albanarts.com>
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
//
// $Id$


#ifndef FG_GUI_MOUSE_CURSOR_3D_HXX
#define FG_GUI_MOUSE_CURSOR_3D_HXX

#include <osg/MatrixTransform>

/**
 * Represents a 3D mouse cursor visible in the scene.
 */
class FGMouseCursor3d : public osg::MatrixTransform
{
public:
    FGMouseCursor3d();
    virtual ~FGMouseCursor3d() = default;

private:
};

#endif
