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
                 const std::shared_ptr<HandPose>& parent);
        /// Destructor.
        virtual ~FGVRHand();

        // Reimplemented from osgXR::HandPose
        void advance(float dt) override;

        void setFingerHoverDistance(unsigned int finger, float hoverDistance)
        {
            _fingersRange[finger].hoverDistance = hoverDistance;
        }

        void setFingerFreeze(unsigned int finger, bool freeze)
        {
            _fingersRange[finger].freeze = freeze;
        }

        bool isPalmTouching() const
        {
            return !_wristRange.touchNodes.empty();
        }

        bool isFingerTouching(unsigned int finger) const
        {
            return !_fingersRange[finger].touchNodes.empty();
        }

        const osg::NodePath* getPalmTouchNodePath() const
        {
            if (!isPalmTouching())
                return nullptr;
            return &_wristRange.touchNodes;
        }

        const osg::NodePath* getFingerTouchNodePath(unsigned int finger) const
        {
            if (!isFingerTouching(finger))
                return nullptr;
            return &_fingersRange[finger].touchNodes;
        }

        const osg::Vec3f* getPalmTouchPosition() const
        {
            if (!isPalmTouching() || !_wristRange.hasPosition)
                return nullptr;
            return &_wristRange.touchPosition;
        }

        const osg::Vec3f* getFingerTouchPosition(unsigned int finger) const
        {
            if (!isFingerTouching(finger) || !_fingersRange[finger].hasPosition)
                return nullptr;
            return &_fingersRange[finger].touchPosition;
        }

        const osg::Vec3f* getPalmTouchNormal() const
        {
            if (!isPalmTouching() || !_wristRange.hasPosition)
                return nullptr;
            return &_wristRange.touchNormal;
        }

        const osg::Vec3f* getFingerTouchNormal(unsigned int finger) const
        {
            if (!isFingerTouching(finger) || !_fingersRange[finger].hasPosition)
                return nullptr;
            return &_fingersRange[finger].touchNormal;
        }

        typedef struct {
            /// Hover distance when autosqueezing.
            float hoverDistance = 0.0f;
            /// Freeze these joints.
            bool freeze = false;

            /// Whether currently overriding the parent pose.
            bool overriding = false;
            /// Whether pushed to which limit.
            int atLimit = 0;
            /// Clearance ratio each way.
            float clearance[2] = {0.0f, 1.0f};

            float curValue = 0;

            osg::NodePath touchNodes;
            bool hasPosition;
            osg::Vec3f touchPosition;
            osg::Vec3f touchNormal;
        } RangeState;

    protected:

        osg::Geometry* initDebugGeom();

        /// Parent hand pose to base this one on.
        std::shared_ptr<osgXR::HandPose> _parent;
        /// Parent joint motion ranges.
        JointMotionRanges _ranges;
        /// Current squeeze values.
        SqueezeValues _squeeze;
        /// Range state for each finger.
        RangeState _fingersRange[5];
        /// Range state for wrist.
        RangeState _wristRange;

        // Debugging

        /// Local space group.
        osg::ref_ptr<osg::MatrixTransform> _localSpaceGroup;
        /// Geode containing debug graphics
        osg::ref_ptr<osg::Geode> _debugGeode;
};

#endif