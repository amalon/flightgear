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
#define DRAW_SQUEEZE_INTERSECTIONS (finger == 1)
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

// Attach meshes key to each mesh
typedef std::pair<osg::NodePath, osg::Drawable*> MeshesKey;
typedef FGVRCollision::Mesh<> RawCollisionMesh;
typedef FGVRCollision::TMetadata<RawCollisionMesh, MeshesKey> CollisionMesh;
// FIXME how about a reference to node path, but compare by dereference

class CollisionMeshes : public FGVRCollision::TGroup<CollisionMesh>
{
public:
    typedef std::pair<osg::NodePath, osg::Drawable*> MeshIndexKey;
    typedef std::map<MeshesKey, unsigned int> MeshIndex;

    MeshIndex index;
};

// Attach joint number to each bone capsule
typedef FGVRCollision::RawOneEndCapsule BoneCapsule;
typedef FGVRCollision::TMetadata<BoneCapsule, unsigned int> Bone;
typedef FGVRCollision::TGroup<Bone> Bones;
typedef FGVRCollision::TStrip<Bones> BonesStrip;

static void intersectionsToMeshes(const osgUtil::PolytopeIntersector::Intersections& intersections,
                                  CollisionMeshes& meshes,
                                  const osg::Matrix& toLocalMatrix)
{
    for (auto &hit: intersections) {
        // Find/create a mesh
        MeshesKey key(hit.nodePath, hit.drawable);
        auto it = meshes.index.find(key);
        if (it == meshes.index.end()) {
            unsigned int index = meshes.addItem(CollisionMesh(key));
            it = meshes.index.emplace(key, index).first;
        }
        auto mesh = meshes.getItem(it->second).getItem();

        // Add collision polygon to mesh
        unsigned int pointCount = hit.numIntersectionPoints;
        if (pointCount > 1)
            --pointCount;
        FGVRCollision::Polygon poly(pointCount);
        for (unsigned int i = 0; i < pointCount; ++i)
            poly.setVertex(i, hit.intersectionPoints[i] * *hit.matrix * toLocalMatrix);
        mesh.addPolygon(poly);
    }

    // We've added polygons to meshes that are already inserted into a group, so
    // the bounds need updating.
    meshes.updateBounds();
}

static void meshesDebugGeom(const CollisionMeshes& meshes,
                            osg::Geometry* debugGeom)
{
    if (!debugGeom)
        return;

    auto* debugVertices = dynamic_cast<osg::Vec3Array*>(debugGeom->getVertexArray());
    for (unsigned int i = 0; i < meshes.numItems(); ++i) {
        auto mesh = meshes.getItem(i).getItem();

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
        unsigned int joint;

        FGVRMeshInstant() :
            Super(0.0f, 0)
        {
        }

        FGVRMeshInstant(const Super& super) :
            Super(super)
        {
        }

        FGVRMeshInstant(const Super& super, const MeshesKey& key,
                        unsigned int joint) :
            Super(super),
            nodePath(key.first),
            drawable(key.second),
            joint(joint)
        {
        }

        FGVRMeshInstant(const Super& super,
                        const CollisionMeshes& meshes,
                        const BonesStrip& bonesStrip) :
            Super(super)
        {
            auto* meshId = super.staticId.as<CollisionMeshes>();
            auto* boneId = super.sweepId.as<BonesStrip>();
            auto key = meshes.getItem(meshId->index).getMetadata();
            nodePath = key.first;
            drawable = key.second;
            joint = bonesStrip.getNode(boneId->index).getItem(boneId->node.index).getMetadata();
        }
};
#if 0
typedef FGVRCollision::TIntersection<FGVRMeshInstant> FGVRMeshIntersection;
typedef FGVRCollision::TIntersections<FGVRMeshIntersection> FGVRMeshIntersections;
#endif

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

#if 0
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
#endif

float JointRangeState::handleIntersections(const CollisionMeshes& meshes,
                                           const BonesStrip& bonesStrip,
                                           bool debugLog,
                                           osg::Geometry* debugGeom,
                                           int debugBone)
{
    // If frozen, revert to frozen value
    if (_state->freeze)
        return _state->curValue = _state->frozenValue;

    // Perform the intersection tests
    //FGVRMeshIntersections intersections;
    auto intersections = intersect(meshes, bonesStrip);

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

#if 0
        auto* entryMeshId = hit.entry.staticId.as<CollisionMeshes>();
        auto* entryBoneId = hit.entry.sweepId.as<BonesStrip>();
        auto* exitMeshId = hit.exit.staticId.as<CollisionMeshes>();
        auto* exitBoneId = hit.exit.sweepId.as<BonesStrip>();
        std::cout << "meshes::id sizeof: " << sizeof(CollisionMeshes::Id) << std::endl;
        if (entryMeshId)
            std::cout << "Entry Mesh Id: " << *entryMeshId << std::endl;
        if (entryBoneId)
            std::cout << "Entry Bone Id: " << *entryBoneId << std::endl;
        if (exitMeshId)
            std::cout << "Exit Mesh Id: " << *exitMeshId << std::endl;
        if (exitBoneId)
            std::cout << "Exit Bone Id: " << *exitBoneId << std::endl;
#endif

        if (intersectionExit <= _state->curValue) {
            // _state->curValue after intersection
            _state->clearance[0] = intersectionExit;
            _clearanceHits[0] = FGVRMeshInstant(hit.exit, meshes, bonesStrip);
        } else if (intersectionEntry < _state->curValue) {
            // _state->curValue inside intersection
            float downward = intersectionExit - _state->curValue;
            float upward = _state->curValue - intersectionEntry;
            static const float maxFingerJump = 0.2f;
            if (intersectionExit < maxRatio && downward <= upward) {
                _state->clearance[0] = intersectionExit;
                _clearanceHits[0] = FGVRMeshInstant(hit.exit, meshes, bonesStrip);
                // Don't jump too far
                if (downward < maxFingerJump) {
                    // push down
                    _state->curValue = intersectionExit;
                    if (debugLog)
                        std::cout << "  down out of intersection" << std::endl;
                } else {
                    // Stuck!
                    _state->clearance[1] = intersectionEntry;
                    _clearanceHits[1] = FGVRMeshInstant(hit.entry, meshes, bonesStrip);
                    if (debugLog)
                        std::cout << "  stuck in intersection (down)" << std::endl;
                    break;
                }
            } else if (intersectionEntry > minRatio && upward <= downward) {
                _state->clearance[1] = intersectionEntry;
                _clearanceHits[1] = FGVRMeshInstant(hit.entry, meshes, bonesStrip);
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
                    _clearanceHits[0] = FGVRMeshInstant(hit.exit, meshes, bonesStrip);
                    if (debugLog)
                        std::cout << "  stuck in intersection (up)" << std::endl;
                    break;
                }
            } else {
                _state->clearance[0] = intersectionExit;
                _state->clearance[1] = intersectionEntry;
                _clearanceHits[0] = FGVRMeshInstant(hit.exit, meshes, bonesStrip);
                _clearanceHits[1] = FGVRMeshInstant(hit.entry, meshes, bonesStrip);
                _state->atLimit = (downward < upward) ? 1 : -1;;
                if (debugLog)
                    std::cout << "  limited " << _state->atLimit << std::endl;
            }
        } else if (_state->clearance[1] >= maxRatio) {
            _state->clearance[1] = intersectionEntry;
            _clearanceHits[1] = FGVRMeshInstant(hit.entry, meshes, bonesStrip);
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
        _state->touchJoint = _clearanceHits[1].joint;
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
        SqueezeGenFingers(float wristBend = 0.0f,
                          std::optional<float> thumbX = std::nullopt,
                          std::optional<float> thumbY = std::nullopt) :
            SqueezeGenFingers(wristBend, thumbX, thumbY, 0.0f, 1.0f)
        {
        }

        SqueezeGenFingers(float wristBend,
                          std::optional<float> thumbX,
                          std::optional<float> thumbY,
                          float fingerSqueeze0,
                          float fingerSqueeze1) :
            _wristBend(wristBend),
            _thumbX(thumbX),
            _thumbY(thumbY),
            _fingerSqueeze0(fingerSqueeze0),
            _fingerSqueezeRange(fingerSqueeze1 - fingerSqueeze0)
        {
        }

        void initSqueeze(osgXR::HandPose::SqueezeValues& outSqueeze) const
        {
            outSqueeze.setWristBend(_wristBend);
            outSqueeze.setThumbX(_thumbX);
            outSqueeze.setThumbY(_thumbY);
        }

        void updateSqueeze(osgXR::HandPose::SqueezeValues& outSqueeze,
                           float param) const
        {
            outSqueeze.setFingersSqueeze(osgXR::HandPose::FINGER_ALL_BITS,
                                _fingerSqueeze0 + param*_fingerSqueezeRange);
        }

    protected:
        float _wristBend;
        std::optional<float> _thumbX;
        std::optional<float> _thumbY;
        float _fingerSqueeze0;
        float _fingerSqueezeRange;
};

class SqueezeGenWrist
{
    public:
        SqueezeGenWrist(float fingerSqueeze = 0.0f,
                        std::optional<float> thumbX = std::nullopt,
                        std::optional<float> thumbY = std::nullopt) :
            SqueezeGenWrist(fingerSqueeze, thumbX, thumbY, 0.0f, 1.0f)
        {
        }

        SqueezeGenWrist(float fingerSqueeze,
                        std::optional<float> thumbX,
                        std::optional<float> thumbY,
                        float wristBend0,
                        float wristBend1) :
            _fingerSqueeze(fingerSqueeze),
            _thumbX(thumbX),
            _thumbY(thumbY),
            _wristBend0(wristBend0),
            _wristBendRange(wristBend1 - wristBend0)
        {
        }

        void initSqueeze(osgXR::HandPose::SqueezeValues& outSqueeze) const
        {
            outSqueeze.setFingersSqueeze(osgXR::HandPose::FINGER_ALL_BITS,
                                         _fingerSqueeze);
            outSqueeze.setThumbX(_thumbX);
            outSqueeze.setThumbY(_thumbY);
        }

        void updateSqueeze(osgXR::HandPose::SqueezeValues& outSqueeze,
                           float param) const
        {
            outSqueeze.setWristBend(_wristBend0 + param*_wristBendRange);
        }

    protected:
        float _fingerSqueeze;
        std::optional<float> _thumbX;
        std::optional<float> _thumbY;
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
                                     BonesStrip* strips,
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
            Bones bones;
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

                FGVRCollision::OneEndCapsule capsule(joints[0], joints[1]);
                // Treat bones as fixed radius, to match osgXR::Hand rendering.
                capsule.setRadii(capsule.minRadius());
                Bone bone(capsule, joint);
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

        BonesStrip wristStrip;
#if 1
        unsigned int handJointMask = JOINT_METACARPAL_BITS |
                                    (JOINT_FINGERS_BITS & JOINT_PROXIMAL_BITS);
#else
        unsigned int handJointMask = JOINT_MIDDLE_PROXIMAL_BIT;
#endif
        buildCollisionBonesStrip(NUM_INTERSECTION_SEGMENTS,
                                 SqueezeGenWrist(0.0f, _squeeze.getThumbX(), _squeeze.getThumbY()),
                                 0.0f, 1.0f, tempPose, _ranges, &dim, 1,
                                 &wristStrip, &handJointMask);
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
        if (!_wristRange.freeze) {
            _squeeze.setWristBend(wristBend);
            jointMask |= JOINT_WRIST_BIT;
        }

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
            BonesStrip fingerSqueezeStrips[5];
            float wristRange = _squeeze.getWristBend() - _wristRange.clearance[0];
#if 0
            std::cout << "wrist range: " << wristRange << " (bend: " << _squeeze.getWristBend() << ", clearance:[" << _wristRange.clearance[0] << " " << _wristRange.clearance[1] << "]" << std::endl;
#endif
            if (!_wristRange.freeze && wristRange > 0.0f) {
                unsigned int stripNodes = 2 + std::floor(wristRange * NUM_INTERSECTION_SEGMENTS);
                buildCollisionBonesStrip(stripNodes,
                                         SqueezeGenWrist(0.0f,
                                                         _squeeze.getThumbX(),
                                                         _squeeze.getThumbY(),
                                                         _squeeze.getWristBend(),
                                                         _squeeze.getWristBend() + 1.0f), -wristRange, 0.0f,
                                         tempPose, _ranges, &dim,
                                         4, fingerSqueezeStrips+1, fingerJointMasks+1);
            }
            // Append collision sweeps of finger bones over squeeze range
            // (0 to 1), with wrist bend unchanged.
            buildCollisionBonesStrip(NUM_INTERSECTION_SEGMENTS,
                                     SqueezeGenFingers(_squeeze.getWristBend(),
                                                       _squeeze.getThumbX(),
                                                       _squeeze.getThumbY()),
                                     0.0f, 1.0f,
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
            if (attempt < 1 && maxWristPushback < 0.0f && !_wristRange.freeze) {
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
    }
}

void FGVRHand::setFingerFreeze(unsigned int finger, bool freeze)
{
    _fingersRange[finger].freeze = freeze;
    _fingersRange[finger].frozenValue = _fingersRange[finger].curValue;
    // Freeze wrist if any fingers are frozen
    if (freeze) {
        if (!_wristRange.freeze) {
            _wristRange.frozenValue = _wristRange.curValue;
            _wristRange.freeze = true;
        }
    } else {
        if (_wristRange.freeze) {
            for(auto& range: _fingersRange)
                if (range.freeze)
                    return;
            _wristRange.freeze = false;
        }
    }
}
