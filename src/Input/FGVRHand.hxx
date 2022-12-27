// Interacting VR Hand Pose
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

#ifndef _FGVRHANDS_HXX
#define _FGVRHANDS_HXX

#include <osgXR/HandPose>

#include <osg/MatrixTransform>

#include <memory>

class FGVRInput;

class FGVRHand : public osgXR::HandPose
{
    public:

        /// Constructor.
        FGVRHand(osg::MatrixTransform* localSpaceGroup,
                 const std::shared_ptr<osgXR::HandPose>& parent);
        /// Destructor.
        virtual ~FGVRHand();

        // Reimplemented from osgXR::HandPose
        void advance(float dt) override;

    protected:

        /// Parent hand pose to base this one on.
        std::shared_ptr<osgXR::HandPose> _parent;
        /// Parent joint motion ranges.
        osgXR::HandPose::JointMotionRanges _ranges;
        /// Current squeeze values.
        osgXR::HandPose::SqueezeValues _squeeze;
        /// Whether each finger is currently overriding the parent.
        bool _fingersOverriding[5];

        // Debugging

        /// Local space group.
        osg::ref_ptr<osg::MatrixTransform> _localSpaceGroup;
        /// Geode containing debug graphics
        osg::ref_ptr<osg::Geode> _debugGeode;
};

#endif
