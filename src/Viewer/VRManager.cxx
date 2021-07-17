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
    _reloadCompositorCallback(new ReloadCompositorCallback(this))
{
    uint32_t fgVersion = (FLIGHTGEAR_MAJOR_VERSION << 16 |
                          FLIGHTGEAR_MINOR_VERSION << 8 |
                          FLIGHTGEAR_PATCH_VERSION);
    _settings->setApp("FlightGear", fgVersion);
    _settings->preferEnvBlendMode(osgXR::Settings::OPAQUE);

    // Hook into viewer, but don't enable VR just yet
    osgViewer::View *view = globals->get_renderer()->getView();
    if (view) {
        setViewer(globals->get_renderer()->getViewerBase());
        view->apply(this);
    }
}

VRManager *VRManager::instance()
{
    static osg::ref_ptr<VRManager> single = new VRManager;
    return single;
}

void VRManager::resetProperties()
{
    fgTie("/sim/vr/openxr/layers/validation",
          static_cast<osgXR::Manager*>(this), &osgXR::Manager::hasValidationLayer );
    fgTie("/sim/vr/openxr/extensions/depth-info",
          static_cast<osgXR::Manager*>(this), &osgXR::Manager::hasDepthInfoExtension );
    fgTie("/sim/vr/openxr/runtime/name",
          static_cast<osgXR::Manager*>(this), &osgXR::Manager::getRuntimeName );
    fgTie("/sim/vr/openxr/system/name",
          static_cast<osgXR::Manager*>(this), &osgXR::Manager::getSystemName );

    fgTie("/sim/vr/state-string",
          static_cast<osgXR::Manager*>(this), &osgXR::Manager::getStateString );
    fgTie("/sim/vr/present",
          static_cast<osgXR::Manager*>(this), &osgXR::Manager::getPresent );
    fgTie("/sim/vr/running",
          static_cast<osgXR::Manager*>(this), &osgXR::Manager::isRunning );
    fgTie("/sim/vr/enabled",
          static_cast<osgXR::Manager*>(this), &osgXR::Manager::getEnabled,
                                              &osgXR::Manager::setEnabled );

    fgTie("/sim/vr/depth-info",        this,  &VRManager::getDepthInfo,
                                              &VRManager::setDepthInfo );
    fgTie("/sim/vr/mode",              this,  &VRManager::getVRMode,
                                              &VRManager::setVRMode );
    fgTie("/sim/vr/swapchain-mode",    this,  &VRManager::getSwapchainMode,
                                              &VRManager::setSwapchainMode );
    fgTie("/sim/vr/validation-layer",  this,  &VRManager::getValidationLayer,
                                              &VRManager::setValidationLayer );
}

bool VRManager::getValidationLayer() const
{
    return _settings->getValidationLayer();
}

void VRManager::setValidationLayer(bool validationLayer)
{
    _settings->setValidationLayer(validationLayer);
    syncSettings();
}

bool VRManager::getDepthInfo() const
{
    return _settings->getDepthInfo();
}

void VRManager::setDepthInfo(bool depthInfo)
{
    _settings->setDepthInfo(depthInfo);
    syncSettings();
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
    syncSettings();
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
    syncSettings();
}

void VRManager::doCreateView(osgXR::View *xrView)
{
    // Restarted in osgXR::Manager::update()
    _viewer->stopThreading();

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
    // Restarted in osgXR::Manager::update()
    _viewer->stopThreading();

    CameraGroup *cgroup = CameraGroup::getDefault();
    auto it = _camInfos.find(xrView);
    if (it != _camInfos.end()) {
        osg::ref_ptr<CameraInfo> info = (*it).second;
        _camInfos.erase(it);

        auto it2 = _xrViews.find(info.get());
        if (it2 != _xrViews.end())
            _xrViews.erase(it2);

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
