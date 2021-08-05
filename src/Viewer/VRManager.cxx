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
#include "WindowBuilder.hxx"
#include "renderer.hxx"

#include <osgXR/Settings>

#include <simgear/scene/viewer/CompositorPass.hxx>

#include <Main/fg_props.hxx>
#include <Main/globals.hxx>

namespace flightgear
{

VRManager::VRManager() :
    _enabled(false),
    _reloadCompositorCallback(new ReloadCompositorCallback(this))
{
    uint32_t fgVersion = (FLIGHTGEAR_MAJOR_VERSION << 16 |
                          FLIGHTGEAR_MINOR_VERSION << 8 |
                          FLIGHTGEAR_PATCH_VERSION);
    _settings->setApp("FlightGear", fgVersion);
    _settings->preferEnvBlendMode(osgXR::Settings::OPAQUE);
}

VRManager *VRManager::instance()
{
    static osg::ref_ptr<VRManager> single = new VRManager;
    return single;
}

void VRManager::resetProperties()
{
    fgTie("/sim/vr/present",
          _settings.get(), &osgXR::Settings::present );
    fgTie("/sim/vr/openxr/layers/validation",
          _settings.get(), &osgXR::Settings::hasValidationLayer );
    fgTie("/sim/vr/openxr/extensions/depth-info",
          _settings.get(), &osgXR::Settings::hasDepthInfoExtension );
    fgTie("/sim/vr/openxr/runtime/name",
          _settings.get(), &osgXR::Settings::runtimeName );

    fgTie("/sim/vr/openxr/system/name",
          static_cast<osgXR::Manager*>(this), &osgXR::Manager::getSystemName );
    fgTie("/sim/vr/enabled",
          this, &VRManager::getEnabled, &VRManager::setEnabled );
    fgTie("/sim/vr/mirror-mode",
          this, &VRManager::getMirrorMode, &VRManager::setMirrorMode );
    fgTie("/sim/vr/mode",
          this, &VRManager::getVRMode, &VRManager::setVRMode );
    fgTie("/sim/vr/swapchain-mode",
          this, &VRManager::getSwapchainMode, &VRManager::setSwapchainMode );
}

bool VRManager::getEnabled() const
{
    return _enabled;
}

void VRManager::setEnabled(bool enabled)
{
    if (_enabled == enabled)
        return;
    if (enabled) {
        osgViewer::View *view = globals->get_renderer()->getView();
        if (view) {
            setViewer(globals->get_renderer()->getViewerBase());
            view->apply(this);
            if (_settings->present())
                _enabled = true;
        }
    } else {
        // FIXME uninit
        //_enabled = false;
    }
}

const char * VRManager::getMirrorMode() const
{
    osgXR::MirrorSettings &settings = _settings->getMirrorSettings();
    osgXR::MirrorSettings::MirrorMode mirrorMode = settings.getMirrorMode();

    switch (mirrorMode) {
    case osgXR::MirrorSettings::MIRROR_AUTOMATIC:
        return "AUTOMATIC";
    case osgXR::MirrorSettings::MIRROR_NONE:
        return "NONE";
    case osgXR::MirrorSettings::MIRROR_SINGLE:
        switch (settings.getMirrorViewIndex())
        {
        case 0:
            return "LEFT";
        case 1:
            return "RIGHT";
        }
        break;
    case osgXR::MirrorSettings::MIRROR_LEFT_RIGHT:
        return "LEFT_RIGHT";
    }
    return "UNKNOWN";
}

void VRManager::setMirrorMode(const char * mode)
{
    osgXR::MirrorSettings::MirrorMode mirrorMode = osgXR::MirrorSettings::MIRROR_AUTOMATIC;
    int viewIndex = -1;

    if (strcmp(mode, "AUTOMATIC") == 0) {
        mirrorMode = osgXR::MirrorSettings::MIRROR_AUTOMATIC;
    } else if (strcmp(mode, "NONE") == 0) {
        mirrorMode = osgXR::MirrorSettings::MIRROR_NONE;
    } else if (strcmp(mode, "LEFT") == 0) {
        mirrorMode = osgXR::MirrorSettings::MIRROR_SINGLE;
        viewIndex = 0;
    } else if (strcmp(mode, "RIGHT") == 0) {
        mirrorMode = osgXR::MirrorSettings::MIRROR_SINGLE;
        viewIndex = 1;
    } else if (strcmp(mode, "LEFT_RIGHT") == 0) {
        mirrorMode = osgXR::MirrorSettings::MIRROR_LEFT_RIGHT;
    }

    _settings->getMirrorSettings().setMirror(mirrorMode, viewIndex);
}

const char * VRManager::getVRMode() const
{
    osgXR::Settings::VRMode vrMode = _settings->getVRMode();

    switch (vrMode) {
    case osgXR::Settings::VRMODE_AUTOMATIC:
        return "AUTOMATIC";
    case osgXR::Settings::VRMODE_SLAVE_CAMERAS:
        return "SLAVE_CAMERAS";
    case osgXR::Settings::VRMODE_SCENE_VIEW:
        return "SCENE_VIEW";
    }
    return "NONE";
}

void VRManager::setVRMode(const char * mode)
{
    osgXR::Settings::VRMode vrMode = osgXR::Settings::VRMODE_AUTOMATIC;

    if (strcmp(mode, "AUTOMATIC") == 0) {
        vrMode = osgXR::Settings::VRMODE_AUTOMATIC;
    } else if (strcmp(mode, "SLAVE_CAMERAS") == 0) {
        vrMode = osgXR::Settings::VRMODE_SLAVE_CAMERAS;
    } else if (strcmp(mode, "SCENE_VIEW") == 0) {
        vrMode = osgXR::Settings::VRMODE_SCENE_VIEW;
    }

    _settings->setVRMode(vrMode);
}

const char * VRManager::getSwapchainMode() const
{
    osgXR::Settings::SwapchainMode swapchainMode = _settings->getSwapchainMode();

    switch (swapchainMode) {
    case osgXR::Settings::SWAPCHAIN_AUTOMATIC:
        return "AUTOMATIC";
    case osgXR::Settings::SWAPCHAIN_MULTIPLE:
        return "MULTIPLE";
    case osgXR::Settings::SWAPCHAIN_SINGLE:
        return "SINGLE";
    }
    return "NONE";
}

void VRManager::setSwapchainMode(const char * mode)
{
    osgXR::Settings::SwapchainMode swapchainMode = osgXR::Settings::SWAPCHAIN_AUTOMATIC;

    if (strcmp(mode, "AUTOMATIC") == 0) {
        swapchainMode = osgXR::Settings::SWAPCHAIN_AUTOMATIC;
    } else if (strcmp(mode,"MULTIPLE") == 0) {
        swapchainMode = osgXR::Settings::SWAPCHAIN_MULTIPLE;
    } else if (strcmp(mode,"SINGLE") == 0) {
        swapchainMode = osgXR::Settings::SWAPCHAIN_SINGLE;
    }

    _settings->setSwapchainMode(swapchainMode);
}

void VRManager::doCreateView(osgXR::View *xrView)
{
    // Construct a property tree for the camera
    SGPropertyNode_ptr camNode = new SGPropertyNode;
    WindowBuilder *windowBuilder = WindowBuilder::getWindowBuilder();
    setValue(camNode->getNode("window/name", true),
             windowBuilder->getDefaultWindowName());

    // Build a camera
    CameraGroup *cgroup = CameraGroup::getDefault();
    CameraInfo *info = cgroup->buildCamera(camNode);

    // Notify osgXR about the new compositor's scene slave cameras
    if (info) {
        _camInfos[xrView] = info;
        _xrViews[info] = xrView;
        info->reloadCompositorCallback = _reloadCompositorCallback;

        postReloadCompositor(cgroup, info);
    }
}

void VRManager::doDestroyView(osgXR::View *xrView)
{
    auto it = _camInfos.find(xrView);
    if (it != _camInfos.end()) {
        osg::ref_ptr<CameraInfo> info = (*it).second;
        _camInfos.erase(it);

        CameraGroup *cgroup = CameraGroup::getDefault();
        cgroup->removeCamera(info.get());
    }
}

void VRManager::preReloadCompositor(CameraGroup *cgroup, CameraInfo *info)
{
    osgXR::View *xrView = _xrViews[info];

    auto& passes = info->compositor->getPassList();
    for (auto& pass: passes)
        if (pass->type == "scene")
            xrView->removeSlave(pass->camera);
}

void VRManager::postReloadCompositor(CameraGroup *cgroup, CameraInfo *info)
{
    osgXR::View *xrView = _xrViews[info];

    auto& passes = info->compositor->getPassList();
    for (auto& pass: passes)
        if (pass->type == "scene")
            xrView->addSlave(pass->camera);
}

}
