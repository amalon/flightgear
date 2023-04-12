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

#include "FGVRHand.hxx"

#include "FGVRCollision.hxx"

#include <Viewer/CameraGroup.hxx>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Point>

#define NUM_INTERSECTION_SEGMENTS 8

// Debug
#include <iostream>

// Shows finger-plane vs scene-geometry intersections
//#define DRAW_PLANE_INTERSECTIONS (finger == 1)

// Shows hand range bounding box vs scene-geometry intersections
//#define DRAW_BOUNDINGBOX_INTERSECTIONS

// Shows wrist vs scene mesh intersection ranges
//#define DRAW_WRIST_INTERSECTIONS

// Shows finger vs scene mesh intersection ranges
//#define DRAW_SQUEEZE_INTERSECTIONS (finger == 1)
//#define DRAW_SQUEEZE_INTERSECTIONS true

// Log finger movement decisions
//#define DEBUG_FINGER_MOVEMENT (finger == 1)
//#define DEBUG_FINGER_MOVEMENT true

// Debug defaults
#ifndef DEBUG_FINGER_MOVEMENT
#define DEBUG_FINGER_MOVEMENT false
#endif

#if defined(DRAW_PLANE_INTERSECTIONS) || \
    defined(DRAW_BOUNDINGBOX_INTERSECTIONS) || \
    defined(DRAW_WRIST_INTERSECTIONS) || \
    defined(DRAW_SQUEEZE_INTERSECTIONS)
#define USE_DEBUG_GEOM true
#endif

using flightgear::CameraGroup;

FGVRHand::FGVRHand(osg::MatrixTransform* localSpaceGroup,
                   const std::shared_ptr<osgXR::HandPose>& parent) :
    _parent(parent),
    _localSpaceGroup(localSpaceGroup)
{
    // Allow fingers to stretch back at the proximal joint
    _ranges.extendX(JOINT_THUMB_PROXIMAL, M_PI * 30 / 180);
    _ranges.extendX(JOINT_INDEX_PROXIMAL, M_PI * 30 / 180);
    _ranges.extendX(JOINT_MIDDLE_PROXIMAL, M_PI * 30 / 180);
    _ranges.extendX(JOINT_RING_PROXIMAL, M_PI * 30 / 180);
    _ranges.extendX(JOINT_LITTLE_PROXIMAL, M_PI * 30 / 180);

    // Allow wrist to bend both ways to avoid collisions
    _ranges.extendX(JOINT_WRIST, M_PI * -80 / 180);
    _ranges.extendX(JOINT_WRIST, M_PI * 80 / 180);
}

FGVRHand::~FGVRHand()
{
    if (_debugGeode.valid())
        _localSpaceGroup->removeChild(_debugGeode);
}

typedef FGVRCollision::Mesh CollisionMesh;
// FIXME how about a reference to node path, but compare by dereference
typedef std::pair<osg::NodePath, osg::Drawable*> MeshesKey;
typedef std::map<MeshesKey, CollisionMesh> CollisionMeshes;

static void intersectionsToMeshes(const osgUtil::PolytopeIntersector::Intersections& intersections,
                                  CollisionMeshes& meshes,
                                  const osg::Matrix& toLocalMatrix)
{
    for (auto &hit: intersections) {
        // Find/create a mesh
        MeshesKey key(hit.nodePath, hit.drawable);
        auto it = meshes.find(key);
        if (it == meshes.end())
            it = meshes.emplace(key, CollisionMesh()).first;
        CollisionMesh *mesh = &(*it).second;

        // Add collision polygon to mesh
        unsigned int pointCount = hit.numIntersectionPoints;
        if (pointCount > 1)
            --pointCount;
        FGVRCollision::Polygon poly(pointCount);
        for (unsigned int i = 0; i < pointCount; ++i)
            poly.setVertex(i, hit.intersectionPoints[i] * *hit.matrix * toLocalMatrix);
        mesh->addPolygon(poly);
    }
}

static void meshesDebugGeom(const CollisionMeshes& meshes,
                            osg::Geometry* debugGeom)
{
    if (!debugGeom)
        return;

    auto* debugVertices = dynamic_cast<osg::Vec3Array*>(debugGeom->getVertexArray());
    for (auto& pair: meshes) {
        auto& mesh = pair.second;

        unsigned int pointsStart = debugVertices->size();
        for (unsigned int i = 0; i < mesh.sweep.points.size(); ++i)
            debugVertices->push_back(mesh.sweep.points[i].position);
        if (debugVertices->size() > pointsStart) {
            osg::DrawArrays* prim = new osg::DrawArrays(osg::PrimitiveSet::POINTS,
                                                        pointsStart,
                                                        debugVertices->size() - pointsStart);
            debugGeom->addPrimitiveSet(prim);
        }

        unsigned int edgesStart = debugVertices->size();
        for (unsigned int i = 0; i < mesh.sweep.edges.size(); ++i) {
            debugVertices->push_back(mesh.sweep.edges[i].start.position);
            debugVertices->push_back(mesh.sweep.edges[i].end.position);
        }
        if (debugVertices->size() > edgesStart) {
            osg::DrawArrays* prim = new osg::DrawArrays(osg::PrimitiveSet::LINES,
                                                        edgesStart,
                                                        debugVertices->size() - edgesStart);
            debugGeom->addPrimitiveSet(prim);
        }

        for (unsigned int i = 0; i < mesh.fixed.polygons.size(); ++i) {
            unsigned int numVerts = mesh.fixed.polygons[i].numVertices;
            if (numVerts < 3)
                continue;
            osg::Vec3f meanPos;
            osg::DrawArrays* prim = new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,
                                                        debugVertices->size(),
                                                        numVerts);
            for (unsigned int j = 0; j < numVerts; ++j) {
                meanPos += mesh.sweep.polygons[i]->vertices[j];
                debugVertices->push_back(mesh.sweep.polygons[i]->vertices[j]);
            }
            debugGeom->addPrimitiveSet(prim);

            meanPos /= numVerts;
            prim = new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,
                                       debugVertices->size(),
                                       numVerts);
            for (unsigned int j = 0; j < numVerts; ++j)
                debugVertices->push_back((meanPos + mesh.sweep.polygons[i]->vertices[j])*0.5f);
            debugGeom->addPrimitiveSet(prim);
        }
    }
}

static void expandBoundingBox(osg::BoundingBoxd& bb,
                              osgXR::HandPose& pose,
                              float fingerSqueeze,
                              float wristBend,
                              const osgXR::HandPose::JointMotionRanges& ranges,
                              const osgXR::HandPose::HandDimentions* dim,
                              const osg::Matrix& invMatrix)
{
    osgXR::HandPose::SqueezeValues squeeze(fingerSqueeze, fingerSqueeze,
                                           fingerSqueeze, fingerSqueeze,
                                           fingerSqueeze, wristBend);
    pose.setPose(squeeze, ranges, dim);
    pose.expandBoundingBox(bb, invMatrix);
}

static void growBoundingBox(osg::BoundingBoxd& bb, float grow)
{
    bb.expandBy(bb.xMin()-grow, bb.yMin()-grow, bb.zMin()-grow);
    bb.expandBy(bb.xMax()+grow, bb.yMax()+grow, bb.zMax()+grow);
}

static void boundingBoxToPolytope(const osg::BoundingBoxd& bb,
                                  osg::Polytope& polytope)
{
        // FIXME The use of OSG_USE_FLOAT_BOUNDINGBOX is dumb
        //polytope.setToBoundingBox(bbHand);
        osg::Polytope::PlaneList planes;
        planes.push_back(osg::Plane(1.0,0.0,0.0,-bb.xMin())); // left plane.
        planes.push_back(osg::Plane(-1.0,0.0,0.0,bb.xMax())); // right plane.
        planes.push_back(osg::Plane(0.0,1.0,0.0,-bb.yMin())); // bottom plane.
        planes.push_back(osg::Plane(0.0,-1.0,0.0,bb.yMax())); // top plane.
        planes.push_back(osg::Plane(0.0,0.0,1.0,-bb.zMin())); // near plane
        planes.push_back(osg::Plane(0.0,0.0,-1.0,bb.zMax())); // far plane
        polytope.set(planes);
}

class FGVRMeshInstant : public FGVRCollision::Strip::Instant
{
    public:
        typedef FGVRCollision::Strip::Instant Super;

        osg::NodePath nodePath;
        osg::Drawable* drawable;

        FGVRMeshInstant() :
            Super(0.0f, 0)
        {
        }

        FGVRMeshInstant(const Super& super) :
            Super(super)
        {
        }

        FGVRMeshInstant(const Super& super, const MeshesKey& key) :
            Super(super),
            nodePath(key.first),
            drawable(key.second)
        {
        }
};
typedef FGVRCollision::TIntersection<FGVRMeshInstant> FGVRMeshIntersection;
typedef FGVRCollision::TIntersections<FGVRMeshIntersection> FGVRMeshIntersections;

class JointRangeState
{
    public:
        JointRangeState() :
            _autoSqueeze(false),
            _speedNormal(1.0f),
            _speedOverriding(1.0f),
            _targetValue(0),
            _clearanceHits{},
            _state(nullptr)
        {
        }

        JointRangeState(bool autoSqueeze,
                        float speedNormal,
                        float speedOverriding,
                        FGVRHand::RangeState* state) :
            _autoSqueeze(autoSqueeze),
            _speedNormal(speedNormal),
            _speedOverriding(speedOverriding),
            _targetValue(0),
            _clearanceHits{},
            _state(state)
        {
        }

        typedef FGVRCollision::OneEndCapsuleGroupStrip BonesStrip;

        void setTargetValue(float targetValue)
        {
            _targetValue = targetValue;
        }

        float handleIntersections(const CollisionMeshes& meshes,
                                  const BonesStrip& bonesStrip,
                                  bool debugLog = false,
                                  osg::Geometry* debugGeom = nullptr,
                                  int debugBone = -1);

        float advance(const BonesStrip& bonesStrip,
                      float dt);

    protected:
        // General behaviour
        bool _autoSqueeze;
        float _speedNormal;
        float _speedOverriding;

        // Dynamic variables
        float _targetValue;
        FGVRMeshInstant _clearanceHits[2];

        FGVRHand::RangeState *_state;
};

template <typename SWEEP>
unsigned int intersect(const CollisionMeshes& meshes,
                       const SWEEP& sweep,
                       FGVRMeshIntersections& intersections)
{
    unsigned int ret = 0;
    for (auto& pair: meshes) {
        FGVRCollision::StripIntersections tempIntersections(intersections);
        ret += FGVRCollision::intersect(pair.second, sweep, tempIntersections);
        for (auto hit: tempIntersections) {
            FGVRMeshInstant entry(hit.entry, pair.first);
            FGVRMeshInstant exit(hit.exit, pair.first);
            FGVRMeshIntersection modHit(entry, exit);
            intersections.insertIntersection(modHit);
        }
    }
    return ret;
}

float JointRangeState::handleIntersections(const CollisionMeshes& meshes,
                                           const BonesStrip& bonesStrip,
                                           bool debugLog,
                                           osg::Geometry* debugGeom,
                                           int debugBone)
{
    // Perform the intersection tests
    FGVRMeshIntersections intersections;
    intersect(meshes, bonesStrip, intersections);

    auto* debugVertices = (debugGeom && debugBone >= 0)
                    ? dynamic_cast<osg::Vec3Array*>(debugGeom->getVertexArray())
                    : nullptr;
    unsigned int primStart = debugVertices ? debugVertices->size() : 0;

    // Extent of current clearance
    // Ratios are reversed if inside intersection
    const float minRatio = bonesStrip.getMinRatio();
    const float maxRatio = bonesStrip.getMaxRatio();
    _state->clearance[0] = minRatio;
    _state->clearance[1] = maxRatio;
    _clearanceHits[0].hasPosition = false;
    _clearanceHits[1].hasPosition = false;
    _clearanceHits[1].nodePath.clear();
    _state->atLimit = false;
    for (auto& hit: intersections) {
        float intersectionEntry = bonesStrip.getRatio(hit.entry);
        float intersectionExit = bonesStrip.getRatio(hit.exit);
        if (debugVertices) {
            const osg::Vec3f& entrySegmentPos1 = bonesStrip.sweep->strip[hit.entry.segment].items[debugBone].position[1];
            const osg::Vec3f& entrySegmentPos2 = bonesStrip.sweep->strip[hit.entry.segment+1].items[debugBone].position[1];
            const osg::Vec3f& exitSegmentPos1 = bonesStrip.sweep->strip[hit.exit.segment].items[debugBone].position[1];
            const osg::Vec3f& exitSegmentPos2 = bonesStrip.sweep->strip[hit.exit.segment+1].items[debugBone].position[1];
            unsigned int stripStart = debugVertices->size();
            debugVertices->push_back(entrySegmentPos1 * (1.0f - hit.entry.ratio) +
                                entrySegmentPos2 * hit.entry.ratio);
            for (unsigned int i = hit.entry.segment+1; i <= hit.exit.segment; ++i) {
                debugVertices->push_back(bonesStrip.sweep->strip[i].items[debugBone].position[1]);
            }
            debugVertices->push_back(exitSegmentPos1 * (1.0f - hit.exit.ratio) +
                                exitSegmentPos2 * hit.exit.ratio);
            osg::DrawArrays* prim = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,
                                                        stripStart,
                                                        debugVertices->size() - stripStart);
            debugGeom->addPrimitiveSet(prim);
        }
        if (debugLog) {
            std::cout << " [" << intersectionEntry << " " << intersectionExit << "]" << std::endl;
            std::cout << " " << hit.entry.source << std::endl;
        }

        if (intersectionExit <= _state->curValue) {
            // _state->curValue after intersection
            _state->clearance[0] = intersectionExit;
            _clearanceHits[0] = hit.exit;
        } else if (intersectionEntry < _state->curValue) {
            // _state->curValue inside intersection
            float downward = intersectionExit - _state->curValue;
            float upward = _state->curValue - intersectionEntry;
            static const float maxFingerJump = 0.2f;
            if (intersectionExit < maxRatio && downward <= upward) {
                _state->clearance[0] = intersectionExit;
                _clearanceHits[0] = hit.exit;
                // Don't jump too far
                if (downward < maxFingerJump) {
                    // push down
                    _state->curValue = intersectionExit;
                    if (debugLog)
                        std::cout << "  down out of intersection" << std::endl;
                } else {
                    // Stuck!
                    _state->clearance[1] = intersectionEntry;
                    _clearanceHits[1] = hit.entry;
                    if (debugLog)
                        std::cout << "  stuck in intersection (down)" << std::endl;
                    break;
                }
            } else if (intersectionEntry > minRatio && upward <= downward) {
                _state->clearance[1] = intersectionEntry;
                _clearanceHits[1] = hit.entry;
                // Don't jump too far
                if (upward < maxFingerJump) {
                    // push up
                    _state->curValue = intersectionEntry;
                    if (debugLog)
                        std::cout << "  up out of intersection" << std::endl;
                    // no further intersections are relevant
                    break;
                } else {
                    // Stuck!
                    _state->clearance[0] = intersectionExit;
                    _clearanceHits[0] = hit.exit;
                    if (debugLog)
                        std::cout << "  stuck in intersection (up)" << std::endl;
                    break;
                }
            } else {
                _state->clearance[0] = intersectionExit;
                _state->clearance[1] = intersectionEntry;
                _clearanceHits[0] = hit.exit;
                _clearanceHits[1] = hit.entry;
                _state->atLimit = (downward < upward) ? 1 : -1;;
                if (debugLog)
                    std::cout << "  limited " << _state->atLimit << std::endl;
            }
        } else if (_state->clearance[1] >= maxRatio) {
            _state->clearance[1] = intersectionEntry;
            _clearanceHits[1] = hit.entry;
        }
    }
    if (debugVertices) {
        for (unsigned int i = 0; i < 2; ++i)
            if (_clearanceHits[i].hasPosition)
                debugVertices->push_back(_clearanceHits[i].position);
        if (debugVertices->size() > primStart)  {
            auto* primPoints = new osg::DrawArrays(osg::PrimitiveSet::POINTS,
                                                   primStart,
                                                   debugVertices->size() - primStart);
            debugGeom->addPrimitiveSet(primPoints);
        }
        for (unsigned int i = 0; i < 2; ++i) {
            if (_clearanceHits[i].hasPosition) {
                osg::Vec3f norm = _clearanceHits[i].normal;
                norm.normalize();
                auto* primNormal = new osg::DrawArrays(osg::PrimitiveSet::LINES,
                                                       debugVertices->size(),
                                                       2);
                debugVertices->push_back(_clearanceHits[i].position);
                debugVertices->push_back(_clearanceHits[i].position + norm*0.02f);
                debugGeom->addPrimitiveSet(primNormal);

#if 0
                if (!_clearanceHits[i].linestrip.empty()) {
                    auto* primStrip = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,
                                                          debugVertices->size(),
                                                          _clearanceHits[i].linestrip.size());
                    for (auto& vert: _clearanceHits[i].linestrip)
                        debugVertices->push_back(vert);
                    debugGeom->addPrimitiveSet(primStrip);
                }
#endif
            }
        }
    }

    return _state->curValue;
}

float JointRangeState::advance(const BonesStrip& bonesStrip,
                               float dt)
{
    const float maxRatio = bonesStrip.getMaxRatio();
    bool shouldOverride = false;
    bool limitedDownwards = false;
    _state->touchNodes.clear();
    if (_state->freeze) {
        return _state->curValue;
    }

    if (_state->clearance[0] > _state->clearance[1]) {
        // If stuck, target current squeeze (don't change it)
        _targetValue = _state->curValue;
        shouldOverride = true;
    } else if (_autoSqueeze && _state->clearance[1] < maxRatio) {
        // push down if there's anything to touch
        _targetValue = _state->clearance[1] - _state->hoverDistance;
        if (_targetValue < _state->clearance[0])
            _targetValue = _state->clearance[0];
        shouldOverride = true;
        limitedDownwards = true;
    } else if (_targetValue < _state->clearance[0]) {
        // don't target upwards beyond clearance
        _targetValue = _state->clearance[0];
        shouldOverride = true;
    } else if (_targetValue >= _state->clearance[1]) {
        // don't target downwards beyond clearance
        _targetValue = _state->clearance[1];
        shouldOverride = true;
        limitedDownwards = true;
    }

    if (shouldOverride)
        _state->overriding = true;
    float speed = _state->overriding ? _speedOverriding : _speedNormal;
    bool touching = false;
    if (_state->curValue < _targetValue) {
        // Move downwards
        _state->curValue += dt*speed;
        if (_state->curValue > _targetValue) {
            _state->curValue = _targetValue;
            if (!shouldOverride)
                _state->overriding = false;
            // If pushing beyond downward limit, then touching
            touching = limitedDownwards;
        }
    } else if (_state->curValue > _targetValue) {
        // Move upwards
        _state->curValue -= dt*speed;
        if (_state->curValue < _targetValue) {
            _state->curValue = _targetValue;
            if (!shouldOverride)
                _state->overriding = false;
        }
    } else {
        // If already at downward limit, then touching
        touching = limitedDownwards;
    }

    if (touching) {
        _state->touchNodes = _clearanceHits[1].nodePath;
        _state->hasPosition = _clearanceHits[1].hasPosition;
        if (_state->hasPosition) {
            _state->touchPosition = _clearanceHits[1].position;
            if (_state->touchPosition.length2() < 0.001) {
                std::cout << "ZERO TOUCH POS " << _state->touchPosition.x() << "," << _state->touchPosition.y() << "," << _state->touchPosition.z() << " from " << _clearanceHits[1].source << std::endl;
            }
            _state->touchNormal = _clearanceHits[1].normal;
        } else {
#if 0
            std::cout << "NO TOUCH POS from ";
            if (_clearanceHits[1].source)
                std::cout << _clearanceHits[1].source << std::endl;
            else
                std::cout << "null" << std::endl;
#endif
        }
    }

    return _state->curValue;
}

class SqueezeGenFingers
{
    public:
        SqueezeGenFingers(float wristBend = 0.0f) :
            SqueezeGenFingers(wristBend, 0.0f, 1.0f)
        {
        }

        SqueezeGenFingers(float wristBend,
                          float fingerSqueeze0,
                          float fingerSqueeze1) :
            _wristBend(wristBend),
            _fingerSqueeze0(fingerSqueeze0),
            _fingerSqueezeRange(fingerSqueeze1 - fingerSqueeze0)
        {
        }

        void initSqueeze(osgXR::HandPose::SqueezeValues& outSqueeze) const
        {
            outSqueeze.setWristBend(_wristBend);
        }

        void updateSqueeze(osgXR::HandPose::SqueezeValues& outSqueeze,
                           float param) const
        {
            outSqueeze.setFingersSqueeze(osgXR::HandPose::FINGER_ALL_BITS,
                                _fingerSqueeze0 + param*_fingerSqueezeRange);
        }

    protected:
        float _wristBend;
        float _fingerSqueeze0;
        float _fingerSqueezeRange;
};

class SqueezeGenWrist
{
    public:
        SqueezeGenWrist(float fingerSqueeze = 0.0f) :
            SqueezeGenWrist(fingerSqueeze, 0.0f, 1.0f)
        {
        }

        SqueezeGenWrist(float fingerSqueeze,
                        float wristBend0,
                        float wristBend1) :
            _fingerSqueeze(fingerSqueeze),
            _wristBend0(wristBend0),
            _wristBendRange(wristBend1 - wristBend0)
        {
        }

        void initSqueeze(osgXR::HandPose::SqueezeValues& outSqueeze) const
        {
            outSqueeze.setFingersSqueeze(osgXR::HandPose::FINGER_ALL_BITS,
                                         _fingerSqueeze);
        }

        void updateSqueeze(osgXR::HandPose::SqueezeValues& outSqueeze,
                           float param) const
        {
            outSqueeze.setWristBend(_wristBend0 + param*_wristBendRange);
        }

    protected:
        float _fingerSqueeze;
        float _wristBend0;
        float _wristBendRange;
};

static int getDebugBone(unsigned int jointMask,
                        unsigned int debugJoint)
{
    // MUST match logic in buildCollisionBonesStrip
    int ret = 0;
    for (unsigned int joint = 0; joint < osgXR::HandPose::JOINT_COUNT; ++joint) {
        if (!((1 << joint) & jointMask))
            continue;
        int parentJoint = osgXR::HandPose::getJointParent((osgXR::HandPose::Joint)joint);
        if (parentJoint < 0)
            continue;

        if (joint == debugJoint)
            return ret;
        ++ret;
    }
    return -1;
}

// appends to strips if not empty
template <typename SQUEEZE_GEN>
static void buildCollisionBonesStrip(unsigned int numStripNodes,
                                     const SQUEEZE_GEN& squeezeGen,
                                     float param0, float param1,
                                     osgXR::HandPose& tempPose,
                                     const osgXR::HandPose::JointMotionRanges& ranges,
                                     const osgXR::HandPose::HandDimentions* dim,
                                     unsigned int numStrips,
                                     FGVRCollision::OneEndCapsuleGroupStrip* strips,
                                     const unsigned int* jointMasks)
{
    // Go through each node in the strip
    const float paramRange = param1 - param0;
    osgXR::HandPose::SqueezeValues squeeze;
    squeezeGen.initSqueeze(squeeze);
    // Skip 0 if strips already has entries
    for (unsigned int i = strips[0].empty() ? 0 : 1; i < numStripNodes; ++i) {
        float ratio = (float)i / (numStripNodes - 1);

        // Update the squeeze values using the generator
        float param = param0 + ratio*paramRange;
        squeezeGen.updateSqueeze(squeeze, param);

        // Update the pose using the squeeze values
        tempPose.setPose(squeeze, ranges, dim);

        for (unsigned int j = 0; j < numStrips; ++j) {
            // Generate a group of bones for each joint in jointMask at this
            // node in the strip
            FGVRCollision::OneEndCapsuleGroup bones;
            for (unsigned int joint = 0; joint < osgXR::HandPose::JOINT_COUNT; ++joint) {
                if (!((1 << joint) & jointMasks[j]))
                    continue;
                int parentJoint = osgXR::HandPose::getJointParent((osgXR::HandPose::Joint)joint);
                if (parentJoint < 0)
                    continue;

                auto &loc1 = tempPose.getJointLocation((osgXR::HandPose::Joint)parentJoint);
                auto &loc2 = tempPose.getJointLocation((osgXR::HandPose::Joint)joint);

                FGVRCollision::Sphere joints[2];
                joints[0].set(loc1.getRadius(), loc1.getPosition());
                joints[1].set(loc2.getRadius(), loc2.getPosition());

                FGVRCollision::OneEndCapsule bone(joints[0], joints[1]);
                // Treat bones as fixed radius, to match osgXR::Hand rendering.
                bone.setRadii(bone.minRadius());
                bones.addItem(bone);
            }

            // Add the bones as a new node to the strip
            strips[j].addNode(param, bones);
        }
    }
}

osg::Geometry* FGVRHand::initDebugGeom()
{
#ifdef USE_DEBUG_GEOM
    if (!USE_DEBUG_GEOM)
        return nullptr;
#endif

    // Create debug geometry

    osg::Vec3Array* vertices = new osg::Vec3Array();
    osg::Geometry* debugGeom = new osg::Geometry;
    debugGeom->setVertexArray(vertices);
    debugGeom->setUseDisplayList(false);
    if (_debugGeode.valid())
        _localSpaceGroup->removeChild(_debugGeode);
    _debugGeode = new osg::Geode;
    _debugGeode->addDrawable(debugGeom);

    // Hard code hit & miss style for now
    int forceOff = osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED;
    osg::StateSet* ss = new osg::StateSet;
    // It should look like some sort of laser
    ss->setMode(GL_LIGHTING, forceOff);
    ss->setMode(GL_DEPTH_TEST, forceOff);
    ss->setAttribute(new osg::LineWidth(2.0f));
    // Material setup
    osg::Material* material = new osg::Material;
    material->setColorMode(osg::Material::OFF);
    material->setDiffuse(osg::Material::FRONT_AND_BACK,
                         osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
    ss->setAttribute(material);
    ss->setAttribute(new osg::Point(4.0f));
    _debugGeode->setStateSet(ss);

    _localSpaceGroup->addChild(_debugGeode);

    return debugGeom;
}

void FGVRHand::advance(float dt)
{
    // still move fingers if paused
    if (dt == 0.0f)
        dt = 0.01f;

    // Ensure parent pose tracking is up to date
    _parent->update();

    auto& invMatrix = _localSpaceGroup->getMatrix();
    osg::Matrix mat;
    mat.invert(invMatrix);

#if 0
    if (_parent->isActive() && isActive()) {
        // How far have we moved?
        auto& root0loc = getJointLocation(JOINT_ROOT);
        auto& root1loc = _parent->getJointLocation(JOINT_ROOT);
        auto rootVec = root1loc.getPosition() - root0loc.getPosition();
        float rootDist2 = rootVec.length2();

        // Construct polytope from bounding box
        osg::BoundingBoxd bbHandSweep;
        osg::Polytope polytope;
        expandBoundingBox(bbHandSweep, invMatrix);
        _parent->expandBoundingBox(bbHandSweep, invMatrix);
        growBoundingBox(bbHandSweep, 0.1f);
        boundingBoxToPolytope(bbHandSweep, polytope);

        // Intersect hand sweep with nearby objects
        osgUtil::PolytopeIntersector::Intersections polyIntersections;
        CollisionMeshes meshes;
        if (computeSceneIntersections(CameraGroup::getDefault(), polytope,
                                      polyIntersections)) {
            intersectionsToMeshes(polyIntersections, meshes, mat);

            // Make a sweep of non-finger bones
            FGVRCollision::OneEndCapsuleGroup handBones[2];
            // Hand joints except finger metacarpals
            unsigned int fingerMask = JOINT_HAND_BITS &
                ~(JOINT_PALM_BIT | JOINT_METACARPAL_BITS |
                  (JOINT_FINGERS_BITS & JOINT_PROXIMAL_BITS));
            for (unsigned int joint = 0; joint < JOINT_COUNT; ++joint) {
                // skip finger joints that are free to move
                int finger = -1;
                if ((1 << joint) & fingerMask) {
#if 0
                    finger = getJointFinger((Joint)joint);
                    if (!_fingerAtLimit[finger])
#endif
                        continue;
                }

                int parentJoint = getJointParent((Joint)joint);
                if (parentJoint < 0)
                    continue;

                auto &locT0A = getJointLocation((Joint)parentJoint);
                auto &locT0B = getJointLocation((Joint)joint);
                auto &locT1A = _parent->getJointLocation((Joint)parentJoint);
                auto &locT1B = _parent->getJointLocation((Joint)joint);

#if 0
                // skip finger joints that aren't free to move in this direction
                if (finger >= 0) {
                    osg::Vec3f movement = locT1B.getPosition() - locT0B.getPosition();
                    const osg::Quat& quat = locT0B.getOrientation();
                    osg::Vec3f localUp = quat * osg::Vec3f(0.0f, 1.0f, 0.0f);
                    // Allow movement away from limit
                    if ((movement * localUp) * _fingerAtLimit[finger] >= 0.0f)
                        continue;
                }
#endif


                // [time][end]
                FGVRCollision::Sphere joints[2][2];
                joints[0][0].set(locT0A.getRadius(), locT0A.getPosition());
                joints[0][1].set(locT0B.getRadius(), locT0B.getPosition());
                joints[1][0].set(locT1A.getRadius(), locT1A.getPosition());
                joints[1][1].set(locT1B.getRadius(), locT1B.getPosition());

                for (unsigned int t = 0; t < 2; ++t) {
                    FGVRCollision::OneEndCapsule bone(joints[t][0], joints[t][1]);
                    // Treat bones as fixed radius, to match osgXR::Hand
                    // rendering.
                    bone.setRadii(bone.minRadius());
                    handBones[t].addItem(bone);
                }
            }
            FGVRCollision::OneEndCapsuleGroupSweep handSweep(handBones[0], handBones[1]);

            // Do accurate collision detection
            FGVRCollision::SweepIntersections intersections;
            for (auto& pair: meshes)
                FGVRCollision::intersect(pair.second, handSweep, intersections);

            // Interpret results
            if (!intersections.empty()) {
                // Slerp to first intersection and stop there
                bool slerped = false;
                for (auto& hit: intersections) {
#if 0
                    std::cout << " [" << hit.entry.ratio << " " << hit.exit.ratio << "]" << std::endl;
#endif
                    // go straight through this intersection if pulled > 30cm
                    // beyond it
                    if (hit.exit.ratio < 1.0f) {
                        float remaining = (1.0f - hit.exit.ratio);
                        if (rootDist2*remaining*remaining > 0.3f*0.3f)
                            continue;
                    }
                    this->slerp(*_parent, hit.entry.ratio);
                    if (hit.entry.hasPosition) {
                        // Now try moving tangentially a bit
                        // FIXME
                    }
                    slerped = true;
                    break;
                }
                if (!slerped) {
                    // Stretched through intersection, yay
                    // Copy the pose from the parent pose
                    *(osgXR::HandPose*)this = *_parent;
                }
            } else {
                // No intersections, yay
                // Copy the pose from the parent pose
                *(osgXR::HandPose*)this = *_parent;
            }
        } else {
            // No nearby objects, yay
            // Copy the pose from the parent pose
            *(osgXR::HandPose*)this = *_parent;
        }
    } else
#endif
    {
        // One of poses are invalid
        // Copy the pose from the parent pose
        *(osgXR::HandPose*)this = *_parent;
        // FIXME duplication
    }

    if (_parent->isActive()) {
        // Create debug geometry
#ifdef USE_DEBUG_GEOM
        auto* debugGeom = FGVRHand::initDebugGeom();
#endif

        HandDimentions dim(*this);
        // Extend the possible ranges of the joints based on the current pose
        _ranges.extend(*this);
#if 0
        std::cout << "thumb: [" << _ranges.getMinJointAngle(osgXR::HandPose::JOINT_THUMB_PROXIMAL).x()
                        << ".." << _ranges.getMaxJointAngle(osgXR::HandPose::JOINT_THUMB_PROXIMAL).x()
                         << " " << _ranges.getMinJointAngle(osgXR::HandPose::JOINT_THUMB_PROXIMAL).y()
                        << ".." << _ranges.getMaxJointAngle(osgXR::HandPose::JOINT_THUMB_PROXIMAL).y()
                         << " " << _ranges.getMinJointAngle(osgXR::HandPose::JOINT_THUMB_PROXIMAL).z()
                        << ".." << _ranges.getMaxJointAngle(osgXR::HandPose::JOINT_THUMB_PROXIMAL).z()
                        << "]" << std::endl;
#endif

        // Calculate the squeeze values of the tracked pose
        SqueezeValues trackedSqueeze(*_parent, _ranges);
        osgXR::HandPose tempPose(*this);
        unsigned int jointMask = 0;


        // Create collision sweep of non finger bones over wrist range

        FGVRCollision::OneEndCapsuleGroupStrip wristStrip;
#if 1
        unsigned int handJointMask = JOINT_METACARPAL_BITS |
                                    (JOINT_FINGERS_BITS & JOINT_PROXIMAL_BITS);
#else
        unsigned int handJointMask = JOINT_MIDDLE_PROXIMAL_BIT;
#endif
        buildCollisionBonesStrip(NUM_INTERSECTION_SEGMENTS,
                                 SqueezeGenWrist(), 0.0f, 1.0f,
                                 tempPose, _ranges, &dim,
                                 1, &wristStrip, &handJointMask);
        int wristDebugBone = getDebugBone(handJointMask, JOINT_MIDDLE_PROXIMAL);


        // Create a bounding box based on a range of hand poses
        osg::BoundingBoxd bbHand;
        ::expandBoundingBox(bbHand, tempPose, 0.0f, 0.0f,  _ranges, &dim, invMatrix);
        ::expandBoundingBox(bbHand, tempPose, 0.0f, 0.25f, _ranges, &dim, invMatrix);
        ::expandBoundingBox(bbHand, tempPose, 0.0f, 0.5f,  _ranges, &dim, invMatrix);
        ::expandBoundingBox(bbHand, tempPose, 0.0f, 0.75f, _ranges, &dim, invMatrix);
        ::expandBoundingBox(bbHand, tempPose, 0.0f, 1.0f,  _ranges, &dim, invMatrix);
        ::expandBoundingBox(bbHand, tempPose, 0.5f, 1.0f,  _ranges, &dim, invMatrix);
        ::expandBoundingBox(bbHand, tempPose, 1.0f, 1.0f,  _ranges, &dim, invMatrix);
        // Grow a bit
        growBoundingBox(bbHand, 0.03f);
        // FIXME ugly, why are bounds sometimes huge if controller not
        // detected!?
        if (!bbHand.valid() ||
            bbHand.xMax() - bbHand.xMin() > 1.0f ||
            bbHand.yMax() - bbHand.yMin() > 1.0f ||
            bbHand.zMax() - bbHand.zMin() > 1.0f) {
            return;
        }

        // Create polytope from bounding box
        osg::Polytope polytope;
        boundingBoxToPolytope(bbHand, polytope);
        // Find intersecting scene geometry, and create intersection meshes
        osgUtil::PolytopeIntersector::Intersections polyIntersections;
        CollisionMeshes meshes;
#if 1
        if (computeSceneIntersections(CameraGroup::getDefault(), polytope,
                                      polyIntersections))
            intersectionsToMeshes(polyIntersections, meshes, mat);
#else
        FGVRCollision::Polygon poly(2);
        poly.setVertex(0, osg::Vec3f(0.1f, 0.0f, -0.3f));
        poly.setVertex(1, osg::Vec3f(-0.1f, 0.0f, -0.3f));
        CollisionMesh mesh;
        mesh.addPolygon(poly);
        meshes[MeshesKey(nullptr, nullptr)] = mesh;
#endif

#ifdef DRAW_BOUNDINGBOX_INTERSECTIONS
        meshesDebugGeom(meshes, debugGeom);
#endif

        // Update wrist bend angle based on non-finger bones

        float oldWristBend = _wristRange.curValue;
        JointRangeState wristState(false, 2.0f, 1.0f, &_wristRange);
        wristState.setTargetValue(trackedSqueeze.getWristBend());
#ifdef DRAW_WRIST_INTERSECTIONS
        osg::Geometry *wristDebugGeom = debugGeom;
#else
        osg::Geometry *wristDebugGeom = nullptr;
#endif
        wristState.handleIntersections(meshes, wristStrip,
                                       false, wristDebugGeom, wristDebugBone);
        float wristBend = wristState.advance(wristStrip, dt);
        _squeeze.setWristBend(wristBend);
        jointMask |= JOINT_WRIST_BIT;

        // Adjust finger squeeze values opposite to wrist bend value

        float wristDelta = wristBend - oldWristBend;
        for (unsigned int finger = 1; finger < 5; ++finger)
            _fingersRange[finger].curValue -= wristDelta;

        // Now for the fingers

        static const unsigned int fingerJointMasks[5] = {
            JOINT_THUMB_BITS  & JOINT_PROXIMAL_UP_BITS,
            JOINT_INDEX_BITS  & JOINT_INTERMEDIATE_UP_BITS,
            JOINT_MIDDLE_BITS & JOINT_INTERMEDIATE_UP_BITS,
            JOINT_RING_BITS   & JOINT_INTERMEDIATE_UP_BITS,
            JOINT_LITTLE_BITS & JOINT_INTERMEDIATE_UP_BITS,
        };
        for (unsigned int attempt = 0; attempt < 2; ++attempt) {
            // Create collision sweeps of finger bones as wrist moves from max
            // (uncolliding) backwards bend to current wrist position (0).
            FGVRCollision::OneEndCapsuleGroupStrip fingerSqueezeStrips[5];
            float wristRange = _squeeze.getWristBend() - _wristRange.clearance[0];
#if 0
            std::cout << "wrist range: " << wristRange << " (bend: " << _squeeze.getWristBend() << ", clearance:[" << _wristRange.clearance[0] << " " << _wristRange.clearance[1] << "]" << std::endl;
#endif
            if (wristRange > 0.0f) {
                unsigned int stripNodes = 2 + std::floor(wristRange * NUM_INTERSECTION_SEGMENTS);
                buildCollisionBonesStrip(stripNodes,
                                         SqueezeGenWrist(0.0f, _squeeze.getWristBend(),
                                                         _squeeze.getWristBend() + 1.0f), -wristRange, 0.0f,
                                         tempPose, _ranges, &dim,
                                         4, fingerSqueezeStrips+1, fingerJointMasks+1);
            }
            // Append collision sweeps of finger bones over squeeze range
            // (0 to 1), with wrist bend unchanged.
            buildCollisionBonesStrip(NUM_INTERSECTION_SEGMENTS,
                                     SqueezeGenFingers(_squeeze.getWristBend()), 0.0f, 1.0f,
                                     tempPose, _ranges, &dim,
                                     5, fingerSqueezeStrips, fingerJointMasks);

            JointRangeState fingerRangeStates[5];
            float maxWristPushback = 0.0f;
            for (unsigned int finger = 0; finger < 5; ++finger) {
                auto& rangeState = fingerRangeStates[finger];
                rangeState = JointRangeState(true, 10.0f, 2.0f, &_fingersRange[finger]);

                // Find finger squeeze values
                rangeState.setTargetValue(trackedSqueeze.getFingerSqueeze((Finger)finger));
#ifdef DRAW_SQUEEZE_INTERSECTIONS
                osg::Geometry *fingerDebugGeom = (DRAW_SQUEEZE_INTERSECTIONS ? debugGeom : nullptr);
#else
                osg::Geometry *fingerDebugGeom = nullptr;
#endif
                float fingerSqueeze = rangeState.handleIntersections(meshes, fingerSqueezeStrips[finger],
                                                                     DEBUG_FINGER_MOVEMENT, fingerDebugGeom, 2);
#if 0
                std::cout << "#" << attempt << " finger " << finger << " squeeze: " << fingerSqueeze << std::endl;
#endif
                // If finger is pushing wrist back, we're going to have to
                // recalculate finger sweeps with the new wrist position.
                // However another finger may push it back further, so continue
                // for now until we know.
                if (fingerSqueeze < maxWristPushback)
                    maxWristPushback = fingerSqueeze;
            }
            // Is one or more fingers pushing the wrist back?
            // We only allow a retry once.
            if (attempt < 1 && maxWristPushback < 0.0f) {
                // Adjust wrist
                _wristRange.curValue += maxWristPushback;
                _squeeze.setWristBend(_squeeze.getWristBend() + maxWristPushback);
                // Adjust fingers in the opposite direction so they don't move
                // too far.
                for (unsigned int finger = 1; finger < 5; ++finger)
                    _fingersRange[finger].curValue -= maxWristPushback;
                // Attempt the whole finger calculation with the new wrist
                // value.
#if 0
                std::cout << "wrist -> " << _squeeze.getWristBend() << std::endl;
#endif
                continue;
            }
#if 0
            if (maxWristPushback < 0.0f)
                std::cout << "wrist -> " << (_squeeze.getWristBend() + maxWristPushback * wristRange) << " out of tries" << std::endl;
#endif
            // Advance fingers
            for (unsigned int finger = 0; finger < 5; ++finger) {
                auto& rangeState = fingerRangeStates[finger];
                float fingerSqueeze = rangeState.advance(fingerSqueezeStrips[finger], dt);
#if 0
                std::cout << "!" << attempt << " finger " << finger << " squeeze: " << fingerSqueeze << std::endl;
#endif
                _squeeze.setFingerSqueeze((Finger)finger, std::max(0.0f, fingerSqueeze));
                jointMask |= fingerJointMasks[finger];
            }
            // And we're done
            break;
        }
        setPose(_squeeze, _ranges, &dim);
#if 0
        unsigned int totPolys = 0;
        unsigned int totEdgesUniq = 0;
        unsigned int totEdges = 0;
        unsigned int totVertsUniq = 0;
        unsigned int totVerts = 0;
        for (auto& pair: meshes) {
            auto& mesh = pair.second;
            totPolys += mesh.numPolygons();
            totEdgesUniq += mesh.numEdgesUnique();
            totEdges += mesh.numEdgesAdded();
            totVertsUniq += mesh.numVertsUnique();
            totVerts += mesh.numVertsAdded();
        }
        std::cout << "meshes:\t" << meshes.size()
                  << "\tpolys: " << totPolys
                  << "\tedges: " << totEdgesUniq << "/" << totEdges
                  << "\tverts: " << totVertsUniq << "/" << totVerts
                  << std::endl;
        std::cout << "collision bounds stats:"
                     "\tno: " << FGVRCollision::boundsCheck<1>::stats[0]
                  << "\tyes: " << FGVRCollision::boundsCheck<1>::stats[1]
                  << std::endl;
        FGVRCollision::boundsCheck<1>::stats[0] = 0;
        FGVRCollision::boundsCheck<1>::stats[1] = 0;
#endif

        // THE END



#if 0
        static const unsigned int fingerMasks[5] = {
            JOINT_THUMB_BITS,
            JOINT_INDEX_BITS  & ~JOINT_METACARPAL_BITS,
            JOINT_MIDDLE_BITS & ~JOINT_METACARPAL_BITS,
            JOINT_RING_BITS   & ~JOINT_METACARPAL_BITS,
            JOINT_LITTLE_BITS & ~JOINT_METACARPAL_BITS,
        };
        unsigned int jointMask = 0;
        for (unsigned int finger = 0; finger < 5; ++finger) {
#if 0
            // and get plane of finger motion
            auto& loc1 = _parent->getJointLocation(fingerJoints[finger][0]);
            auto& loc4 = _parent->getJointLocation(fingerJoints[finger][3]);
            auto pos1 = (osg::Vec3d)loc1.getPosition() * invMatrix;
            //osg::Matrixd ori1(loc1.getOrientation());
            //auto vec3_4 = osg::Vec4d(0.0f, -1.0f, 0.0f, 0.0f) * ori1;
            //osg::Vec3d vec3(vec3_4.x(), vec3_4.y(), vec3_4.z());
            //auto pos3 = (osg::Vec3d)(loc2.getPosition() + ori1*osg::Vec3d(0.0f, -1.0f, 0.0f)) * invMatrix;
#endif

#if 0
            if (finger == 4) {
                osg::DrawArrays* prim = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,
                                                            vertices->size(),
                                                            NUM_INTERSECTION_SEGMENTS);
                for (unsigned int i = 0; i < NUM_INTERSECTION_SEGMENTS; ++i) {
                    auto vertTransformed = pos[i][finger][1];
                    vertices->push_back(vertTransformed);
                }
                debugGeom->addPrimitiveSet(prim);
            }
#endif

#if 0
            osg::Polytope polytope;
            osg::BoundingBoxd bb;// = _parent->getBoundingBox(invMatrix);
            const double grow = dim.getJointLength(fingerJoints[finger][1])
                              + dim.getJointLength(fingerJoints[finger][2])
                              + dim.getJointLength(fingerJoints[finger][3])
                              + loc4.getRadius();
            bb.expandBy(pos1);
            growBoundingBox(bb, grow);


            osg::Polytope polytope;
            boundingBoxToPolytope(bb, polytope);
#endif

            bool overriding = false;
            float curSqueeze = _squeeze.getFingerSqueeze((Finger)finger);
            float targetSqueeze = trackedSqueeze.getFingerSqueeze((Finger)finger);

            // FIXME fall back to plane intersection collision detection?
#if 0
            auto planePos1 = (osg::Vec3d)pos[0][finger][3] * invMatrix;
            auto planePos2 = (osg::Vec3d)pos[NUM_INTERSECTION_SEGMENTS>>1][finger][3] * invMatrix;
            auto planePos3 = (osg::Vec3d)pos[NUM_INTERSECTION_SEGMENTS-1][finger][3] * invMatrix;
            osg::Plane plane(planePos1, planePos2, planePos3);
            osgUtil::PlaneIntersector::Intersections planeIntersections;
            if (computeSceneIntersections(CameraGroup::getDefault(), plane, polytope,
                                          planeIntersections))
#endif
            {
#if 0
                unsigned int verts = 0;
                for (auto &hit: planeIntersections)
                    verts += hit.polyline.size();
                if (verts)
#endif
                {
                    FGVRCollision::StripIntersections intersections;
                    for (auto& pair: meshes) {
                        FGVRCollision::intersect(pair.second, fingersStrip[finger], intersections);
                    }
#if 0
                    for (auto &hit: planeIntersections) {
                        FGVRCollision::LineStrip linestrip;
#ifdef DRAW_PLANE_INTERSECTIONS
                        osg::ref_ptr<osg::DrawArrays> prim;
                        if (DRAW_PLANE_INTERSECTIONS)
                            prim = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,
                                                       vertices->size(),
                                                       hit.polyline.size());
#endif
                        for (auto &vert: hit.polyline) {
                            auto vertTransformed = vert * *hit.matrix * mat;
#ifdef DRAW_PLANE_INTERSECTIONS
                            if (prim.valid())
                                vertices->push_back(vertTransformed);
#endif
                            linestrip.addNode(FGVRCollision::Point(vertTransformed));
                        }
#ifdef DRAW_PLANE_INTERSECTIONS
                        if (prim.valid())
                            debugGeom->addPrimitiveSet(prim);
#endif

                        //FGVRCollision::intersect(linestrip, fingersStrip[finger], intersections);
                    }
#endif

#ifdef DRAW_SQUEEZE_INTERSECTIONS
                    unsigned int primStart = vertices->size();
#endif
                    if (DEBUG_FINGER_MOVEMENT)
                        std::cout << "finger " << finger << " cur " << curSqueeze << std::endl;
                    // Extent of current clearance
                    // Ratios are reversed if inside intersection
                    float clearanceRatio[2] = { 0.0f, 1.0f };
                    for (auto& hit: intersections) {
                        float intersectionEntry = fingersStrip[finger].getRatio(hit.entry);
                        float intersectionExit = fingersStrip[finger].getRatio(hit.exit);
#ifdef DRAW_SQUEEZE_INTERSECTIONS
                        if (DRAW_SQUEEZE_INTERSECTIONS) {
                            unsigned int stripStart = vertices->size();
                            for (unsigned int j = 3; j < 4; ++j) {
                                vertices->push_back(pos[hit.entry.segment][finger][j] * (1.0f - hit.entry.ratio) +
                                                    pos[hit.entry.segment + 1][finger][j] * hit.entry.ratio);
                                for (unsigned int i = hit.entry.segment+1; i <= hit.exit.segment; ++i)
                                    vertices->push_back(pos[i][finger][j]);
                                vertices->push_back(pos[hit.exit.segment][finger][j] * (1.0f - hit.exit.ratio) +
                                                    pos[hit.exit.segment + 1][finger][j] * hit.exit.ratio);
                            }
                            osg::DrawArrays* prim = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,
                                                                        stripStart,
                                                                        vertices->size() - stripStart);
                            debugGeom->addPrimitiveSet(prim);
                        }
#endif
                        if (DEBUG_FINGER_MOVEMENT)
                            std::cout << " [" << intersectionEntry << "-" << intersectionExit << "]" << std::endl;

                        if (intersectionExit <= curSqueeze) {
                            // curSqueeze after intersection
                            clearanceRatio[0] = intersectionExit;
                        } else if (intersectionEntry < curSqueeze) {
                            // curSqueeze inside intersection
                            float downward = intersectionExit - curSqueeze;
                            float upward = curSqueeze - intersectionEntry;
                            static const float maxFingerJump = 0.2f;
                            if (intersectionExit < 1.0f && downward <= upward) {
                                clearanceRatio[0] = intersectionExit;
                                // Don't jump too far
                                if (downward < maxFingerJump) {
                                    // push down
                                    curSqueeze = intersectionExit;
                                    if (DEBUG_FINGER_MOVEMENT)
                                        std::cout << "  down out of intersection" << std::endl;
                                } else {
                                    // Stuck!
                                    clearanceRatio[1] = intersectionEntry;
                                    if (DEBUG_FINGER_MOVEMENT)
                                        std::cout << "  stuck in intersection (down)" << std::endl;
                                    break;
                                }
                            } else if (intersectionEntry > 0.0f && upward <= downward) {
                                clearanceRatio[1] = intersectionEntry;
                                // Don't jump too far
                                if (upward < maxFingerJump) {
                                    // push up
                                    curSqueeze = intersectionEntry;
                                    if (DEBUG_FINGER_MOVEMENT)
                                        std::cout << "  up out of intersection" << std::endl;
                                    // no further intersections are relevant
                                    break;
                                } else {
                                    // Stuck!
                                    clearanceRatio[0] = intersectionExit;
                                    if (DEBUG_FINGER_MOVEMENT)
                                        std::cout << "  stuck in intersection (up)" << std::endl;
                                    break;
                                }
                            } else {
                                clearanceRatio[0] = intersectionExit;
                                clearanceRatio[1] = intersectionEntry;
                                _fingerAtLimit[finger] = (downward < upward) ? 1 : -1;
                                if (DEBUG_FINGER_MOVEMENT)
                                    std::cout << "  limited " << _fingerAtLimit[finger] << std::endl;
                            }
                        } else if (clearanceRatio[1] >= 1.0f) {
                            clearanceRatio[1] = intersectionEntry;
                        }
                    }
                    if (clearanceRatio[0] > clearanceRatio[1]) {
                        // If stuck, target current squeeze (don't change it)
                        targetSqueeze = curSqueeze;
                        overriding = true;
#if 0
                    } else if (clearanceRatio[1] < 1.0f) {
                        // push down if there's anything to touch
                        targetSqueeze = clearanceRatio[1];
                        overriding = true;
#endif
                    } else if (targetSqueeze < clearanceRatio[0]) {
                        // don't target upwards beyond clearance
                        targetSqueeze = clearanceRatio[0];
                        overriding = true;
                    } else if (targetSqueeze > clearanceRatio[1]) {
                        // don't target downwards beyond clearance
                        targetSqueeze = clearanceRatio[1];
                        overriding = true;
                    }
#ifdef DRAW_SQUEEZE_INTERSECTIONS
                    if (vertices->size() > primStart)  {
                        osg::DrawArrays* primPoints = new osg::DrawArrays(osg::PrimitiveSet::POINTS,
                                                                    primStart,
                                                                    vertices->size() - primStart);
                        debugGeom->addPrimitiveSet(primPoints);
                    }
#endif
                }
            }

            float fingerSpeed = 2.0f;
            // still move fingers if paused
            if (dt == 0.0f)
                dt = 0.01f;
            if (overriding)
                _fingersOverriding[finger] = true;
            if (!_fingersOverriding[finger])
                fingerSpeed = 10.0f;
            if (curSqueeze < targetSqueeze) {
                curSqueeze += dt*fingerSpeed;
                if (curSqueeze > targetSqueeze) {
                    curSqueeze = targetSqueeze;
                    if (!overriding)
                        _fingersOverriding[finger] = false;
                }
            } else if (curSqueeze > targetSqueeze) {
                curSqueeze -= dt*fingerSpeed;
                if (curSqueeze < targetSqueeze) {
                    curSqueeze = targetSqueeze;
                    if (!overriding)
                        _fingersOverriding[finger] = false;
                }
            }
            _squeeze.setFingerSqueeze((Finger)finger, curSqueeze);
            jointMask |= fingerMasks[finger];
        }
        // TODO
        // Collision detect the rest of the hand, swept from previous position
        // Also include any fingers which are pushed to their maximum extent
        // If it collides, move the hand back and leave it there
        // Collect normals, allow motion without intersecting normals?
        if (jointMask)
            setPose(_squeeze, _ranges, &dim, jointMask);
#endif
    }
}
