// Handle a VR hand interaction
//
// Copyright (C) 2023  James Hogan
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

#include "FGVRHandInteraction.hxx"
#include "FGVRHand.hxx"

#include <GUI/Highlight.hxx>
#include <Main/fg_props.hxx>
#include <Viewer/VRManager.hxx>

#include <simgear/scene/model/SGIKLink.hxx>
#include <simgear/scene/util/OsgMath.hxx>
#include <simgear/scene/util/RenderConstants.hxx>
#include <simgear/scene/util/SGSceneUserData.hxx>

#include <osg/ref_ptr>

using namespace flightgear;

typedef std::vector<SGSceneryPick> SGSceneryPicks;
typedef SGSharedPtr<SGPickCallback> SGPickCallbackPtr;
typedef std::list<SGPickCallbackPtr> SGPickCallbackList;

// FGVRHandInteraction::Private

class FGVRHandInteraction::Private
{
public:
    FGVRInput* _input;

    /// Tracked hand pose.
    std::shared_ptr<osgXR::HandPoseTracked> _handTracking;

    /// FG interacting hand pose.
    std::shared_ptr<FGVRHand> _handPose;

    /// Visible hand node.
    osg::ref_ptr<osgXR::Hand> _hand;

    struct Contact {
        //struct SGSceneryPick pick;
        osg::observer_ptr<osg::Node> rootNode;
        // FIXME save pointing...
        SGIKLink* ikLink;
        std::shared_ptr<SGIKContactSpringStatic> contact;
        // Joint contact is made with
        unsigned int joint;
        // Contact point relative to joint
        osg::Vec3f jointContactPos;

        // Normal pick callbacks
        std::map<int, SGPickCallbackList> activeCallbacks;
    };

    /// Contact points (5 fingers + palm).
    Contact _contacts[6];

    SGSceneryPicks handlePickIntersections(const osg::NodePath& np,
                                           const osg::Vec3f& grabPos)
    {
        SGSceneryPicks result;
        SGSceneryPick sceneryPick;
        bool pickCalculated = false;

        for (auto npi = np.rbegin(); npi != np.rend(); ++npi) {
            SGSceneUserData* ud = SGSceneUserData::getSceneUserData(*npi);
            if (!ud || (ud->getNumPickCallbacks() == 0)) {
                continue;
            }

            for (unsigned int i = 0; i < ud->getNumPickCallbacks(); ++i) {
                SGPickCallback* pickCallback = ud->getPickCallback(i);
                if (!pickCallback)
                    continue;

                if (!pickCalculated) {
                    pickCalculated = true;

                    auto hitPosLocal  = (osg::Vec3d)grabPos;
                    auto& localSpaceMatrix = _input->getLocalSpaceGroup()->getMatrix();
                    auto invLocalNodeMatrix = computeWorldToLocal(np);
                    auto hitPosGlobal = hitPosLocal * localSpaceMatrix;
                    auto hitPosNode = hitPosGlobal * invLocalNodeMatrix;

                    // Find position relative to world, and relative to top node
                    sceneryPick.info.local = toSG(hitPosNode);
                    sceneryPick.info.wgs84 = toSG(hitPosGlobal);

                    // FIXME this needs implementing from FGVRCollision data
                    if(pickCallback->needsUV())
                        //sceneryPick.info.uv = uvFromIntersection(*hit);
                        sceneryPick.info.uv = SGVec2d(-9999,-9999);
                }

                sceneryPick.callback = pickCallback;
                result.push_back(sceneryPick);
            }
        }

        return result;
    }

    void initPick(unsigned int contact,
                  int button,
                  const osg::NodePath& np,
                  const osg::Vec3f& contactPos)
    {
        // Get the list of hit callbacks. Take the first callback that
        // accepts the mouse button press and ignore the rest of them
        SGSceneryPicks pickList = handlePickIntersections(np, contactPos);
        if (pickList.empty())
            return;

        for (auto& pick: pickList) {
            if (!pick.callback)
                continue;

            osgGA::GUIEventAdapter* hackTmp = nullptr;
            if (pick.callback->buttonPressed(button, *hackTmp, pick.info)) {
                _contacts[contact].activeCallbacks[button].push_back(pick.callback);
                return;
            }
        }
    }

    void updatePicks(unsigned int contact, double dt, unsigned int keyModState)
    {
        // handle repeatable mouse press events
        for (auto& callbacks: _contacts[contact].activeCallbacks)
            for (auto& callback: callbacks.second)
                callback->update(dt, keyModState);
    }

    static const SGSceneryPick* getPick(const SGSceneryPicks& pickList,
                                        const SGPickCallback* cb)
    {
        for (auto& pick: pickList)
            if (pick.callback == cb)
                return &pick;

        return nullptr;
    }


    void finishPick(unsigned int contact,
                    int button,
                    const osg::NodePath& np,
                    const osg::Vec3f& contactPos)
    {
        SGSceneryPicks pickList = handlePickIntersections(np, contactPos);

        auto& callbacks = _contacts[contact].activeCallbacks[button];
        while (!callbacks.empty()) {
            auto& cb = callbacks.front();
            const SGSceneryPick* pick = getPick(pickList, cb);
            osgGA::GUIEventAdapter* hackTmp = nullptr;
            cb->buttonReleased(button, *hackTmp, pick ? &pick->info : nullptr);

            callbacks.pop_front();
        }
    }
};


// FGVRHandInteraction

FGVRHandInteraction::FGVRHandInteraction(FGVRInput* input,
                                         FGVRInput::Mode* mode,
                                         FGVRInput::Subaction* subaction,
                                         SGPropertyNode* node,
                                         SGPropertyNode* statusNode)
    : ModeProcess(mode, subaction, node, statusNode),
      _grabPalm(mode, subaction, getInputNode("grab-palm")),
      _grabFingers{
          FGVRInput::ModeProcessInput(mode, subaction, getInputNode("grab-thumb")),
          FGVRInput::ModeProcessInput(mode, subaction, getInputNode("grab-index")),
          FGVRInput::ModeProcessInput(mode, subaction, getInputNode("grab-middle")),
          FGVRInput::ModeProcessInput(mode, subaction, getInputNode("grab-ring")),
          FGVRInput::ModeProcessInput(mode, subaction, getInputNode("grab-little")),
      },
      _thumbDown(mode, subaction, getInputNode("thumb-down")),
      _thumbPosition(mode, subaction, getInputNode("thumb-position")),
      _private(std::make_unique<Private>())
{
    auto& subactionPath = subaction->getPath();
    std::string namePrefix = mode->getPath() + " " + _name + " " + subactionPath + " ";

    _private->_input = input;

    osgXR::HandPose::Hand hand;
    if (subactionPath == "/user/hand/left") {
        hand = osgXR::HandPose::HAND_LEFT;
    } else if (subactionPath == "/user/hand/right") {
        hand = osgXR::HandPose::HAND_RIGHT;
    } else {
        SG_LOG(SG_INPUT, SG_WARN,
               "Unknown VR hand interaction subaction path \"" << subactionPath << "\" for "
               << mode->getPath() + " " + _name + " " + subactionPath);
        return;
    }

    VRManager* manager = VRManager::instance();
    auto* localSpaceGroup = input->getLocalSpaceGroup();

    _private->_handTracking = std::make_shared<osgXR::HandPoseTracked>(manager, hand);

    _private->_handPose = std::make_shared<FGVRHand>(localSpaceGroup, _private->_handTracking);

    _private->_hand = new osgXR::Hand(_private->_handPose);
    _private->_hand->setNodeMask(~simgear::PICK_BIT);
    _private->_hand->setName(namePrefix + "hand");

    localSpaceGroup->addChild(_private->_hand);
}

FGVRHandInteraction::~FGVRHandInteraction()
{
    // Remove the hand from the local space group
    _private->_input->getLocalSpaceGroup()->removeChild(_private->_hand);
}

void FGVRHandInteraction::postinit(SGPropertyNode* node,
                                   const std::string& module)
{
}

void FGVRHandInteraction::update(double dt)
{
    bool grabs[6] = {};
    bool grabsChanged[6] = {};
    bool thumbDown = false;
    bool thumbDownChanged = false;
    osg::Vec2f thumbPos;
    bool hasThumbPos = false;
    bool thumbPosChanged = false;

    for (unsigned int i = 0; i < 5; ++i) {
        _grabFingers[i].getBoolValue(grabs[i], &grabsChanged[i]);
        //_private->_handPose->setFingerHoverDistance(i, grabs[i] ? 0.0f : 0.01f);
    }
    _grabPalm.getBoolValue(grabs[5], &grabsChanged[5]);
    _thumbDown.getBoolValue(thumbDown, &thumbDownChanged);
    if (thumbDown)
        hasThumbPos = _thumbPosition.getVector2fValue(thumbPos, &thumbPosChanged);

    if (hasThumbPos) {
        _private->_handPose->setThumbPosition(thumbPos);
    } else {
        _private->_handPose->clearThumbPosition();
    }

    _private->_handPose->advance(dt);

    const osg::NodePath* grabNodes[6];
    int grabJoints[6];
    const osg::Vec3f* grabPositions[6];
    const osg::Vec3f* grabNormals[6];
    for (unsigned int i = 0; i < 5; ++i) {
        grabNodes[i] = _private->_handPose->getFingerTouchNodePath(i);
        grabJoints[i] = _private->_handPose->getFingerTouchJoint(i);
        grabPositions[i] = _private->_handPose->getFingerTouchPosition(i);
        grabNormals[i] = _private->_handPose->getFingerTouchNormal(i);
    }
    grabNodes[5] = _private->_handPose->getPalmTouchNodePath();
    grabJoints[5] = _private->_handPose->getPalmTouchJoint();
    grabPositions[5] = _private->_handPose->getPalmTouchPosition();
    grabNormals[5] = _private->_handPose->getPalmTouchNormal();

    //auto highlight = globals->get_subsystem<Highlight>();
    //int higlight_num_props = 0;

    auto& localMatrix = _private->_input->getLocalSpaceGroup()->getMatrix();

    for (unsigned int grab = 0; grab < 6; ++grab) {
        SGIKLink::LinkPath ikLinks;
        int rootIndex = -1;
        osg::Matrix rootMatrix, tipMatrix;
#if 0
        if (grabsChanged[grab])
            std::cout << "Grab " << grab << " = " << grabs[grab] << " (changed: " << grabsChanged[grab] << ")" << std::endl;
#endif

        if (grabs[grab] && !grabsChanged[grab]) {
            // Grab in progress, so we should just update contact point
            if (_private->_contacts[grab].contact && _private->_contacts[grab].rootNode.valid()) {
                //std::cout << "Updating contact " << grab << " " << _private->_contacts[grab].contact << std::endl;

                auto nodePaths = _private->_contacts[grab].rootNode->getParentalNodePaths();
                if (nodePaths.empty())
                    continue;
                nodePaths.front().pop_back();
                rootMatrix = computeWorldToLocal(nodePaths.front());

                auto jointLoc = _private->_handPose->getJointLocation((osgXR::HandPose::Joint)_private->_contacts[grab].joint);
                osg::Matrix jointMat(jointLoc.getOrientation());
                jointMat.setTrans(jointLoc.getPosition());
                osg::Matrix invJointMat(jointLoc.getOrientation().inverse());
                invJointMat.preMultTranslate(-jointLoc.getPosition());

                // Update spring destination relative to IK root, using new hand
                // position
                auto springPosJoint = (osg::Vec3d)_private->_contacts[grab].jointContactPos;
                auto springPosLocal = springPosJoint * jointMat;
                auto springPosGlobal = springPosLocal * localMatrix;
                auto springPosRoot = springPosGlobal * rootMatrix;
                /*
                std::cout << "  springPosPalm " << springPosPalm.x() << "," << springPosPalm.y() << "," << springPosPalm.z() << std::endl;
                std::cout << "  springPosLocal " << springPosLocal.x() << "," << springPosLocal.y() << "," << springPosLocal.z() << std::endl;
                std::cout << "  springPosRoot " << springPosRoot.x() << "," << springPosRoot.y() << "," << springPosRoot.z() << std::endl;
                */
                _private->_contacts[grab].contact->setSpringPositionRoot(springPosRoot);
            }
            continue;
        }
        if (grabNodes[grab])
            SGIKLink::nodePathToLinks(*grabNodes[grab], ikLinks, rootIndex, rootMatrix, tipMatrix);
        if (!ikLinks.empty() && grabPositions[grab]) {
            auto* ik = ikLinks.back();
            // If top link is different to last time, clear contact and
            // update
            if (ik != _private->_contacts[grab].ikLink) {
                if (_private->_contacts[grab].contact)
                    _private->_contacts[grab].contact->setStale();
                _private->_contacts[grab].contact = nullptr;
                _private->_contacts[grab].ikLink = ik;
                if (grab < 5)
                    _private->_handPose->setFingerFreeze(grab, false);
            }
            // Create a new spring contact
            if (!_private->_contacts[grab].contact) {
                _private->_contacts[grab].contact = std::make_shared<SGIKContactSpringStatic>();
                //std::cout << "Creating contact " << grab << " " << _private->_contacts[grab].contact << std::endl;
                ik->addContact(_private->_contacts[grab].contact, ikLinks);
            } else {
                //std::cout << "Updating contact " << grab << " " << _private->_contacts[grab].contact << std::endl;
            }

            auto jointLoc = _private->_handPose->getJointLocation((osgXR::HandPose::Joint)grabJoints[grab]);
            osg::Matrix jointMat(jointLoc.getOrientation());
            jointMat.setTrans(jointLoc.getPosition());
            osg::Matrix invJointMat(jointLoc.getOrientation().inverse());
            invJointMat.preMultTranslate(-jointLoc.getPosition());

            auto contactPosLocal  = (osg::Vec3d)*grabPositions[grab];
            auto contactPosJoint  = contactPosLocal * invJointMat;
            auto contactPosGlobal = contactPosLocal * localMatrix;
            auto contactPosTip    = contactPosGlobal * tipMatrix;
            auto contactPosRoot   = contactPosGlobal * rootMatrix;
#if 0
            std::cout << "  contactPosLocal " << contactPosLocal.x() << "," << contactPosLocal.y() << "," << contactPosLocal.z() << std::endl;
            std::cout << "  contactPosPalm " << contactPosPalm.x() << "," << contactPosPalm.y() << "," << contactPosPalm.z() << std::endl;
            std::cout << "  contactPosRoot " << contactPosRoot.x() << "," << contactPosRoot.y() << "," << contactPosRoot.z() << std::endl;
            std::cout << "  contactPosTip " << contactPosTip.x() << "," << contactPosTip.y() << "," << contactPosTip.z() << std::endl;
#endif

            // Update contact position relative to hand (for when hand moves)
            _private->_contacts[grab].joint = grabJoints[grab];
            _private->_contacts[grab].jointContactPos = contactPosJoint;

            // Update contact position relative to IK tip (for IK calculations)
            _private->_contacts[grab].contact->setContactPositionTip(contactPosTip);

            // Update spring destination to match
            _private->_contacts[grab].contact->setSpringPositionRoot(contactPosRoot);
            _private->_contacts[grab].contact->setForce(10.0f, grabs[grab] ? 1.0f : 0.0f);
            if (grabs[grab] && grab < 5)
                _private->_handPose->setFingerFreeze(grab, true);

            // Update root node pointer
            if (rootIndex >= 0)
                _private->_contacts[grab].rootNode = (*grabNodes[grab])[rootIndex];
            else
                _private->_contacts[grab].rootNode = nullptr;

            continue;
        }

        // No contact, make existing contact stale
        if (_private->_contacts[grab].contact) {
            //std::cout << "Dropping contact " << grab << " " << _private->_contacts[grab].contact << std::endl;
            _private->_contacts[grab].contact->setStale();
            _private->_contacts[grab].contact = nullptr;
            _private->_contacts[grab].ikLink = nullptr;
            if (grab < 5)
                _private->_handPose->setFingerFreeze(grab, false);
        } else if (!grabs[grab] && grabsChanged[grab] &&
                   grabNodes[grab] && grabPositions[grab]) {
            // If grab finished, execute the mouse up event
            _private->finishPick(grab, 0, *grabNodes[grab], *grabPositions[grab]);
        }


        // If grab started
        if (grabs[grab] && grabsChanged[grab] &&
            grabNodes[grab] && grabPositions[grab]) {
            // Fall back to normal mouse clicks
            _private->initPick(grab, 0, *grabNodes[grab], *grabPositions[grab]);
        }

        // Update callbacks
        _private->updatePicks(grab, dt, 0);
    }


    // Events to pick objects
    // - interract
    // - alternate interact
    // - start grab
    // - move grab
    // - stop grab
#if 0
        // How about some interaction?
        for (unsigned int i = 0; i < 5; ++i) {
            if (!_fingersRange[i].touchingNodes.empty()) {
                std::cout << "Finger " << i << " touching";
                for (auto* node: _fingersRange[i].touchingNodes)
                    std::cout << " '" << node->getName() << "'";
                std::cout << std::endl;
            }
        }
        if (!_wristRange.touchingNodes.empty()) {
            std::cout << "Wrist touching";
                for (auto* node: _wristRange.touchingNodes)
                    std::cout << " '" << node->getName() << "'";
                std::cout << std::endl;
        }
#endif

}

void FGVRHandInteraction::deactivate()
{
    _grabPalm.deactivate();
    for (auto& grabFinger: _grabFingers)
        grabFinger.deactivate();
    _thumbDown.deactivate();
    _thumbPosition.deactivate();

    // FIXME deactivate hand tracking stuff
}
