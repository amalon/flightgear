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

#include "VRManager.hxx"

#include <simgear/scene/viewer/CompositorPass.hxx>

namespace flightgear
{
using namespace simgear;
using namespace compositor;

struct VRMirrorPassBuilder : public PassBuilder {
public:
    virtual Pass *build(Compositor *compositor, const SGPropertyNode *root,
                        const SGReaderWriterOptions *options) {
        osg::ref_ptr<Pass> pass = PassBuilder::build(compositor, root, options);
        pass->useMastersSceneData = false;

        osg::Camera *camera = pass->camera;
        VRManager::instance()->setupMirrorCamera(camera);

        return pass.release();
    }
};
RegisterPassBuilder<VRMirrorPassBuilder> registerVRMirrorPass("vr-mirror");

}
