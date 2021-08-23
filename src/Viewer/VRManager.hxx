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

#ifndef VRMANAGER_HXX
#define VRMANAGER_HXX 1

#include <config.h>

#ifdef ENABLE_OSGXR

#include <osg/ref_ptr>
#include <osg/observer_ptr>

#include <osgXR/Manager>

#include <simgear/scene/viewer/CompositorPass.hxx>

#include "CameraGroup.hxx"

#include <map>

namespace flightgear
{

class VRManager : public osgXR::Manager
{
    public:

        class ReloadCompositorCallback : public CameraInfo::ReloadCompositorCallback
        {
            public:

                ReloadCompositorCallback(VRManager *manager) :
                    _manager(manager)
                {
                };

                virtual void preReloadCompositor(CameraGroup *cgroup, CameraInfo *info)
                {
                    _manager->preReloadCompositor(cgroup, info);
                }

                virtual void postReloadCompositor(CameraGroup *cgroup, CameraInfo *info)
                {
                    _manager->postReloadCompositor(cgroup, info);
                }

            protected:

                osg::observer_ptr<VRManager> _manager;
        };

        VRManager();

        static VRManager *instance();

        void resetProperties();

        // Settings

        bool getValidationLayer() const;
        void setValidationLayer(bool validationLayer);

        bool getDepthInfo() const;
        void setDepthInfo(bool depthInfo);

        const char * getMirrorMode() const;
        void setMirrorMode(const char * mode);

        const char * getVRMode() const;
        void setVRMode(const char * mode);

        const char * getSwapchainMode() const;
        void setSwapchainMode(const char * mode);

        // osgXR::Manager overrides

        void doCreateView(osgXR::View *xrView) override;
        void doDestroyView(osgXR::View *xrView) override;

        void onRunning() override;
        void onStopped() override;

        void preReloadCompositor(CameraGroup *cgroup, CameraInfo *info);
        void postReloadCompositor(CameraGroup *cgroup, CameraInfo *info);

    protected:

        typedef std::map<osgXR::View *, osg::ref_ptr<CameraInfo>> XRViewToCamInfo;
        XRViewToCamInfo _camInfos;

        typedef std::map<CameraInfo *, osg::ref_ptr<osgXR::View>> CamInfoToXRView;
        CamInfoToXRView _xrViews;

        osg::ref_ptr<ReloadCompositorCallback> _reloadCompositorCallback;
};

}

#endif // ENABLE_OSGXR

#endif
