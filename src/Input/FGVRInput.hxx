// Handle user input from OpenXR devices (via osgXR)
//
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

#ifndef _FGVRINPUT_HXX
#define _FGVRINPUT_HXX

#include "FGCommonInput.hxx"
#include "FGButton.hxx"

#include <simgear/math/SGMath.hxx>
#include <simgear/props/propertyObject.hxx>
#include <simgear/structure/subsystem_mgr.hxx>

#include <osgXR/Action>
#include <osgXR/ActionSet>
#include <osgXR/InteractionProfile>
#include <osgXR/Subaction>

#include <osg/MatrixTransform>
#include <osg/Vec2f>

#include <list>
#include <map>
#include <memory>
#include <string>

////////////////////////////////////////////////////////////////////////
// The VR Input Class
////////////////////////////////////////////////////////////////////////
class FGVRInput : public SGSubsystem,
                  FGCommonInput
{
public:
    FGVRInput();
    virtual ~FGVRInput();

    // Subsystem API.
    void init() override;
    void postinit() override;
    void reinit() override;
    void update(double dt) override;

    // Subsystem identification.
    static const char* staticSubsystemClassId() { return "input-vr"; }

    // Accessors

    /// Get a group representing the OpenXR local space.
    osg::MatrixTransform *getLocalSpaceGroup() { return _localSpace.get(); }

    // Forward class declarations
    class Mode;

private:

    void _remove(bool all);

    // Forward class declarations
    class SubactionInfo;

public:

    /**
     * Represents a part of the user they can interact in VR using.
     * This Subaction class extends osgXR's Subaction class with Flightgear
     * specifics.
     */
    class Subaction : public osgXR::Subaction
    {
        public:

            /**
             * Main constructor.
             * @param manager VR manager object.
             * @param path    OpenXR subaction / top-level-user path of this
             *                subaction, e.g. "/user/hand/left".
             */
            Subaction(osgXR::Manager *manager, const std::string &path);

            /// Set the current mode this subaction should be using.
            void setMode(Mode *mode);

            /// Get the current mode this subaction should be using.
            Mode *getMode(Mode *mode)
            {
                return _curMode;
            }

            /// Regular update of subaction.
            void update(double dt);

        protected:

            /// Current mode this subaction should be using
            Mode *_curMode;
    };

private:

    /**
     * Represents a set of osgXR actions.
     * This ActionSet class extends osgXR's ActionSet class with Flightgear
     * specifics.
     */
    class ActionSet : public osgXR::ActionSet
    {
        public:

            /**
             * Construct an action set based on a property node.
             * @param input   VR input subsystem object.
             * @param manager VR manager object.
             * @param node    Property node describing the action set.
             */
            ActionSet(FGVRInput *input, osgXR::Manager *manager,
                      SGPropertyNode *node);

            /**
             * Get the Nasal module name for a @a subaction.
             * This may also create the nasal module, so should be called at
             * postinit stage.
             * @param subaction Subaction to get module name for, or nullptr for
             *                  all subactions.
             */
            const std::string &getModule(Subaction *subaction);

            /// Find an action in this set by name.
            osgXR::Action *findAction(const char *name);

            /**
             * Clean up child action references.
             * This is used during VR input subsystem shut down to ensure that
             * actions get cleaned up, since they refer back to this ActionSet
             * object.
             */
            void cleanupActions();
            /// Initialise anything depending on nasal.
            void postinit();
            /// Poll the actions for input changes.
            void update(double dt);
            /// Fire bindings for changed actions.
            void fireBindings();

        protected:

            /// The property node for this action set.
            SGPropertyNode_ptr _node;

            /// Subaction specific module names.
            std::map<Subaction *, std::string> _modules;

            /**
             * Map of actions accessible by name.
             * This is used for keeping track of actions in the action set,
             * however the actions refer back to this ActionSet in a loop so
             * cleanup must be explicit via cleanupActions().
             */
            std::map<std::string, osg::ref_ptr<osgXR::Action>> _actionMap;
    };

    /**
     * Additional base class for FlightGear actions.
     * This base class is tied into osgXR's various action classes via multiple
     * inheritance to extend them with Flightgear specifics.
     */
    class ActionCommon
    {
        public:

            /**
             * Construct an action based on a property node.
             * @param action The osgXR part of this action object.
             * @param node   Property node describing the action.
             */
            ActionCommon(osgXR::Action *action,
                         SGPropertyNode *node);
            /// Destructor
            virtual ~ActionCommon();

            /// Get the property node corresponding to this action.
            SGPropertyNode *getNode()
            {
                return _node;
            }

            /**
             * Get subaction specific info for this action.
             * @param subaction Subaction to get info for, or nullptr for all
             *                  subactions.
             * @return The SubactionInfo object for @a subaction, or nullptr if
             *         it doesn't exist (because it hasn't been defined as a
             *         possible subaction to filter actions by).
             */
            SubactionInfo *getSubactionInfo(Subaction *subaction);

            /**
             * Initialise the common action data based on a property node.
             * This has to be separate from the constructor as it calls the pure
             * virtual createSubactionInfo().
             * @param input      VR input subsystem object.
             * @param manager    VR manager object.
             * @param action     The osgXR part of this action object.
             * @param statusNode Property node for describing the action status.
             */
            void init(FGVRInput *input, osgXR::Manager *manager,
                      osgXR::Action *action, SGPropertyNode *statusNode);
            /**
             * Initialise anything depending on nasal.
             * @param actionSet The ActionSet object this action is part of.
             */
            void postinit(ActionSet *actionSet);
            /// Poll the action for input changes.
            void update(double dt);
            /// Fire bindings if the value has changed.
            void fireBindings();

            /**
             * Create a subaction info object for this action.
             * This pure virtual function allows derived type specific action
             * classes to choose what SubactionInfo derived classes to create
             * for the action.
             * @param subaction  Subaction to create info for, or nullptr for all
             *                   subactions.
             * @param node       Property node describing the action.
             * @param statusNode Property node for describing the subaction
             *                   status.
             * @return A new SubactionInfo derived object for this action and @a
             *         subaction.
             */
            virtual SubactionInfo *createSubactionInfo(Subaction *subaction,
                                                       SGPropertyNode *node,
                                                       SGPropertyNode *statusNode) = 0;

        protected:

            /// Subaction info objects for each selected subaction.
            std::map<Subaction *, SubactionInfo *> _subactions;

            /// The property node for this action.
            SGPropertyNode_ptr _node;
    };

    /**
     * Base class represents state specific to an action and subaction.
     * Actions can be filtered by subaction, so allow the same control on
     * controllers in different hands to be distinguished. This is the base
     * class for action type specific derived classes representing state that is
     * specific to an action and subaction, such as current values of the input
     * action and nasal bindings.
     *
     * Not to be confused with FGVRInput::Mode::SubactionInfo.
     */
    class SubactionInfo
    {
        public:

            /**
             * Construct subaction info for an action subaction property node.
             * @param action    The osgXR part of the action object.
             * @param subaction Subaction this object relates to, or nullptr for
             *                  all subactions.
             * @param node      Property node describing the subaction, or
             *                  nullptr for all subactions.
             */
            SubactionInfo(osgXR::Action *action, Subaction *subaction,
                          SGPropertyNode *node);
            /// Destructor.
            virtual ~SubactionInfo() = default;

            /**
             * Initialise anything depending on nasal.
             * @param actionSet The ActionSet object @a action is part of.
             * @param action    The common part of the action object.
             */
            void postinit(ActionSet *actionSet, ActionCommon *action);

            /**
             * Initialise nasal bindings for the specified property @a node.
             * @param node Property node to find bindings in.
             * @param module Nasal module name.
             */
            virtual void postinit(SGPropertyNode *node,
                                  const std::string &module) = 0;

            /**
             * Poll the action for input changes in the corresponding subaction.
             * @param action The common part of the action object.
             * @param dt     Time passed in seconds.
             */
            virtual void update(ActionCommon *action, double dt) = 0;

            /// Fire bindings if the value has changed.
            virtual void fireBindings() = 0;

            /**
             * Get the value of the action as a boolean.
             * @param outValue[out] Output value.
             * @return true on success, false on failure.
             */
            virtual bool getBoolCurValue(bool &outValue) { return false; }

            /**
             * Get the value of the action as a float.
             * @param outValue[out] Output value.
             * @return true on success, false on failure.
             */
            virtual bool getFloatCurValue(float &outValue) { return false; }

            /**
             * Get the value of the action as a 2D vector.
             * @param outValue[out] Output value.
             * @return true on success, false on failure.
             */
            virtual bool getVector2fCurValue(osg::Vec2f &outValue) { return false; }

            /**
             * Get the value of the action as a pose.
             * @param outValue[out] Output value.
             * @return true on success, false on failure.
             */
            virtual bool getPoseCurValue(osgXR::ActionPose::Location &outValue) { return false; }

        protected:

            /// The subaction or nullptr for all.
            osg::ref_ptr<Subaction> _subaction;
            /// The property node for the subaction in the action.
            SGPropertyNode_ptr _node;
            /// A string describing both the action and subaction.
            std::string _name;
    };

    /// Represents boolean action state specific to a subaction.
    class SubactionInfoBoolean : public SubactionInfo
    {
        public:

            /**
             * Construct subaction info for a boolean subaction property node.
             * @param action     The osgXR part of the action object.
             * @param subaction  Subaction this object relates to, or nullptr
             *                   for all subactions.
             * @param node       Property node describing the subaction, or
             *                   nullptr for all subactions.
             * @param statusNode Property node for describing the subaction
             *                   status.
             */
            SubactionInfoBoolean(osgXR::Action *action, Subaction *subaction,
                                 SGPropertyNode *node,
                                 SGPropertyNode *statusNode);

            // Implement SubactionInfo virtual functions
            void postinit(SGPropertyNode *node,
                          const std::string &module) override;
            void update(ActionCommon *action, double dt) override;
            void fireBindings() override;

            /// Get the last polled value of the action/subaction.
            const auto &getCurValue() const
            {
                return _curValue;
            }

            bool getBoolCurValue(bool &outValue) override
            {
                outValue = _curValue;
                return true;
            }

            bool getFloatCurValue(float &outValue) override
            {
                outValue = _curValue ? 1.0f : 0.0f;
                return true;
            }

        protected:

            /// The last polled value of the action/subaction.
            bool _curValue;
            /// The property object for describing the subaction status.
            SGPropObjBool _statusProp;
            /// Generic button object hands most of the specifics.
            FGButton _button;
    };

    /// Represents float/axis action state specific to a subaction.
    class SubactionInfoFloat : public SubactionInfo
    {
        public:

            /**
             * Construct subaction info for a float/axis subaction property
             * node.
             * @param action     The osgXR part of the action object.
             * @param subaction  Subaction this object relates to, or nullptr
             *                   for all subactions.
             * @param node       Property node describing the subaction, or
             *                   nullptr for all subactions.
             * @param statusNode Property node for describing the subaction
             *                   status.
             */
            SubactionInfoFloat(osgXR::Action *action, Subaction *subaction,
                               SGPropertyNode *node,
                               SGPropertyNode *statusNode);

            // Implement SubactionInfo virtual functions
            void postinit(SGPropertyNode *node,
                          const std::string &module) override;
            void update(ActionCommon *action, double dt) override;
            void fireBindings() override;

            /// Get the last polled value of the action/subaction.
            const auto &getCurValue() const
            {
                return _curValue;
            }

            bool getBoolCurValue(bool &outValue) override
            {
                outValue = _curValue > 0.5f;
                return true;
            }

            bool getFloatCurValue(float &outValue) override
            {
                outValue = _curValue;
                return true;
            }

        protected:

            /// The last polled value of the action/subaction.
            float _curValue;
            /// The previous polled value of the action/subaction.
            float _lastValue;
            /// The property object for describing the subaction status.
            SGPropObjDouble _statusProp;
            /// Bindings for this axis.
            binding_list_t _bindings[KEYMOD_MAX];
    };

    /// Represents 2D float/axis vector action state specific to a subaction.
    class SubactionInfoVector2f : public SubactionInfo
    {
        public:

            /**
             * Construct subaction info for a 2D float/axis vector subaction
             * property node.
             * @param action     The osgXR part of the action object.
             * @param subaction  Subaction this object relates to, or nullptr
             *                   for all subactions.
             * @param node       Property node describing the subaction, or
             *                   nullptr for all subactions.
             * @param statusNode Property node for describing the subaction
             *                   status.
             */
            SubactionInfoVector2f(osgXR::Action *action, Subaction *subaction,
                                  SGPropertyNode *node,
                                  SGPropertyNode *statusNode);

            // Implement SubactionInfo virtual functions
            void postinit(SGPropertyNode *node,
                          const std::string &module) override;
            void update(ActionCommon *action, double dt) override;
            void fireBindings() override;

            /// Get the last polled value of the action/subaction.
            const auto &getCurValue() const
            {
                return _curValue;
            }

            bool getVector2fCurValue(osg::Vec2f &outValue) override
            {
                outValue = _curValue;
                return true;
            }

        protected:

            /// The last polled value of the action/subaction.
            osg::Vec2f _curValue;
            /// The previous polled value of the action/subaction.
            osg::Vec2f _lastValue;
            /// The property node for describing the subaction status.
            SGPropertyNode_ptr _statusNode;
            /// The property objects for each dimention of the subaction status.
            SGPropObjDouble _statusProp[2];
            /// Bindings for these axes.
            binding_list_t _bindings[KEYMOD_MAX];
    };

    /// Represents 3D pose action state specific to a subaction.
    class SubactionInfoPose : public SubactionInfo
    {
        public:

            /**
             * Construct subaction info for a 3D pose subaction property node.
             * @param action     The osgXR part of the action object.
             * @param subaction  Subaction this object relates to, or nullptr
             *                   for all subactions.
             * @param node       Property node describing the subaction, or
             *                   nullptr for all subactions.
             * @param statusNode Property node for describing the subaction
             *                   status.
             */
            SubactionInfoPose(osgXR::Action *action, Subaction *subaction,
                              SGPropertyNode *node,
                              SGPropertyNode *statusNode);

            // Implement SubactionInfo virtual functions
            void postinit(SGPropertyNode *node,
                          const std::string &module) override;
            void update(ActionCommon *action, double dt) override;
            void fireBindings() override;

            /// Get the last polled location value of the pose action/subaction.
            const osgXR::ActionPose::Location &getCurValue() const
            {
                return _curValue;
            }

            bool getPoseCurValue(osgXR::ActionPose::Location &outValue) override
            {
                outValue = _curValue;
                return true;
            }

        protected:

            /// The last polled location of the action/subaction.
            osgXR::ActionPose::Location _curValue;
            /// The previous polled location of the action/subaction.
            osgXR::ActionPose::Location _lastValue;
            /// The property node for describing the subaction status.
            SGPropertyNode_ptr _statusNode;
            /// The property node for the subaction position.
            SGPropertyNode_ptr _statusPositionNode;
            /// The property node for whether the position is valid.
            SGPropObjBool _statusPositionValidProp;
            /// The property node for whether the position is being tracked.
            SGPropObjBool _statusPositionTrackedProp;
            /// The property node for the subaction position coordinate values.
            SGPropObjDouble _statusPositionProp[3];
            /// Bindings for the pose.
            binding_list_t _bindings[KEYMOD_MAX];
    };

    /**
     * Represents a typed osgXR action.
     * This template class brings osgXR's typed Action* classes together with
     * the Flightgear specifics in ActionCommon via basic multiple inheritance.
     * @param XRAction      osgXR's action type to inherit from.
     * @param SubactionInfo The subaction information class.
     */
    template <typename XRAction, typename SubactionInfo>
    class ActionTyped : public XRAction, public ActionCommon
    {
        public:

            /**
             * Construct typed action from a property node.
             * @param actionSet  The action set it belongs to.
             * @param node       Property node describing the action.
             */
            ActionTyped(ActionSet *actionSet, SGPropertyNode *node) :
                XRAction(actionSet),
                ActionCommon(this, node)
            {
            }

            /**
             * Get correctly typed subaction specific info for this action.
             * @param subaction Subaction to get info for, or nullptr for all
             *                  subactions.
             * @return The SubactionInfo object for @a subaction of the more
             *         specific type, or nullptr if it doesn't exist (because it
             *         hasn't been defined as a possible subaction to filter
             *         actions by).
             */
            SubactionInfo *getSubactionInfo(Subaction *subaction)
            {
                return static_cast<SubactionInfo *>(ActionCommon::getSubactionInfo(subaction));
            }

            // Virtual function implementations from ActionCommon
            SubactionInfo *createSubactionInfo(Subaction *subaction,
                                               SGPropertyNode *node,
                                               SGPropertyNode *statusNode)
            {
                return new SubactionInfo(this, subaction, node, statusNode);
            }
    };

    typedef ActionTyped<osgXR::ActionBoolean, SubactionInfoBoolean> ActionBoolean;
    typedef ActionTyped<osgXR::ActionFloat, SubactionInfoFloat> ActionFloat;
    typedef ActionTyped<osgXR::ActionVector2f, SubactionInfoVector2f> ActionVector2f;
    typedef ActionTyped<osgXR::ActionPose, SubactionInfoPose> ActionPose;

    /**
     * Represents a set of suggested bindings for a VR controller.
     * This InteractionProfile class extends osgXR's InteractionProfile class
     * with Flightgear specifics.
     */
    class InteractionProfile : public osgXR::InteractionProfile
    {
        public:

            /**
             * Construct interaction profile from a property node.
             * @param input   VR input subsystem object.
             * @param manager VR manager object.
             * @param vendor  Vendor string forming first part of OpenXR
             *                interaction profile path.
             * @param type    Type string forming second part of OpenXR
             *                interaction profile path.
             * @param node    Property node describing the interaction profile.
             */
            InteractionProfile(FGVRInput *input, osgXR::Manager *manager,
                               const char *vendor, const char *type,
                               SGPropertyNode *node);
    };

    // FG specific concepts

    class ModeProcessInputSource
    {
        public:

            /// Destructor.
            virtual ~ModeProcessInputSource() = default;

            // Interface for derived classes to implement

            /**
             * Get the value of the input source as a boolean.
             * @param outValue[out] Output value.
             * @return true on success, false on failure.
             */
            virtual bool getBoolValue(bool &outValue) { return false; }

            /**
             * Get the value of the input source as a float.
             * @param outValue[out] Output value.
             * @return true on success, false on failure.
             */
            virtual bool getFloatValue(float &outValue) { return false; }

            /**
             * Get the value of the input source as a 2D vector.
             * @param outValue[out] Output value.
             * @return true on success, false on failure.
             */
            virtual bool getVector2fValue(osg::Vec2f &outValue) { return false; }

            /**
             * Get the value of the input source as a pose.
             * @param outValue[out] Output value.
             * @return true on success, false on failure.
             */
            virtual bool getPoseValue(osgXR::ActionPose::Location &outValue) { return false; }
    };

    class ModeProcessInputSourceAction : public ModeProcessInputSource
    {
        public:

            /**
             * Construct from a property node.
             * @param subaction Subaction the mode is tied to.
             * @param action    Action object.
             */
            ModeProcessInputSourceAction(Subaction *subaction,
                                         osgXR::Action *action);

            // Implement ModeProcessInputSource virtual functions
            bool getBoolValue(bool &outValue) override;
            bool getFloatValue(float &outValue) override;
            bool getVector2fValue(osg::Vec2f &outValue) override;
            bool getPoseValue(osgXR::ActionPose::Location &outValue) override;

        protected:

            ActionCommon *getActionCommon()
            {
                return dynamic_cast<ActionCommon*>(_action.get());
            }

            osg::ref_ptr<Subaction> _subaction;
            osg::ref_ptr<osgXR::Action> _action;
    };

public:

    class ModeProcessInput
    {
        public:

            /**
             * Construct from a property node.
             * @param mode      Interaction mode object.
             * @param subaction Subaction the mode is tied to.
             * @param node      Property node describing the input.
             */
            ModeProcessInput(FGVRInput::Mode *mode,
                             Subaction *subaction,
                             SGPropertyNode *node);

            bool getBoolValue(bool &outValue,
                              bool *outChanged = nullptr);
            bool getFloatValue(float &outValue,
                               bool *outChanged = nullptr);
            bool getVector2fValue(osg::Vec2f &outValue,
                                  bool *outChanged = nullptr);
            bool getPoseValue(osgXR::ActionPose::Location &outValue,
                              bool *outChanged = nullptr);

        protected:

            std::list<std::unique_ptr<ModeProcessInputSource>> _sources;
            bool _lastBool;
            float _lastFloat;
            osg::Vec2f _lastVec2f;
            osgXR::ActionPose::Location _lastPose;
    };

    /**
     * Base class for input processing objects used by a VR interaction mode.
     * VR interaction modes are made up of process objects derived from this
     * base class, which refer to VR actions and other process objects, may have
     * bindings, and which interpret them in an interaction specific way.
     * Instances of this class are specific to a single subaction, so process
     * objects defined outside of a subaction XML tag will instantiate multiple
     * times.
     */
    class ModeProcess : public FGCommonInput
    {
        public:

            /**
             * Construct from a property node.
             * @param mode       Interaction mode object.
             * @param subaction  Subaction the mode is tied to.
             * @param node       Property node describing the process object.
             * @param statusNode Property node for describing the process object
             *                   status.
             */
            ModeProcess(Mode *mode, Subaction *subaction, SGPropertyNode *node,
			SGPropertyNode *statusNode);
            /// Destructor.
            virtual ~ModeProcess() = default;

            /**
             * Initialise nasal bindings in the provided @a module.
             * @param module Nasal module name.
             */
            void postinit(const std::string &module);

            /**
             * Initialise nasal bindings for the specified property @a node.
             * This pure virtual function must be implemented by derived
             * classes.
             * @param node   Mode process object property node to get bindings
             *               from.
             * @param module Nasal module name.
             */
            virtual void postinit(SGPropertyNode *node,
                                  const std::string &module) = 0;

            /**
             * Update values and fire bindings.
             * This pure virtual function must be implemented by derived
             * classes.
             * @param dt Time passed in seconds.
             */
            virtual void update(double dt) = 0;

        protected:

            SGPropertyNode *getInputNode(const std::string &name);

            /// Property node for the process object.
            SGPropertyNode_ptr _node;
            /// Main property node for process object status.
            SGPropertyNode_ptr _statusNode;
            /// Identifying name.
            std::string _name;
            /// Which subaction this process object belongs to.
            osg::ref_ptr<Subaction> _subaction;
    };

    /**
     * VR interaction mode.
     * VR interaction modes are a FlightGear mechanism to describe how the
     * inputs from an osgXR ActionSet should be interpreted. They can be swapped
     * in and out for each controller without requiring multiple action sets and
     * the corresponding interaction profiles bindings to be defined.
     */
    class Mode
    {
        public:

            /**
             * Construct from a property node.
             * @param input      VR input subsystem object.
             * @param manager    VR manager object.
             * @param type       Type of mode the mode belongs in.
             * @param mode       Identifying mode name within @a type.
             * @param node       Property node describing the mode.
             */
            Mode(FGVRInput *input, osgXR::Manager *manager, const char *type,
                 const char *mode, SGPropertyNode *node);
            /// Destructor.
            ~Mode();

            // Accessors

            /// Get the path of the mode, in the form "type/mode".
            const std::string &getPath() const
            {
                return _path;
            }

            /// Get the default action set this mode attaches to.
            ActionSet *getActionSet()
            {
                return _actionSet;
            }

            /// Initialise nasal bindings for the interaction mode.
            void postinit();

            /**
             * Activate the mode on a subaction.
             * @param subaction Subaction to activate the mode for.
             */
            void activate(Subaction *subaction);

            /**
             * Deactivate the mode on a subaction.
             * @param subaction Subaction to activate the mode for.
             */
            void deactivate(Subaction *subaction);

            /**
             * Update an active mode on a subaction.
             * Poll inputs from @a subaction and fire bindings as appropriate.
             * @param subaction Subaction to use to read inputs.
             * @param dt        Time passed in seconds.
             */
            void update(Subaction *subaction, double dt);

        protected:

            /**
             * Contains subaction specific interaction mode data.
             * Not to be confused with FGVRInput::SubactionInfo.
             */
            class SubactionInfo
            {
                public:

                    /// Constructor.
                    SubactionInfo(Subaction *subaction,
                                  SGPropertyNode *node);
                    /// Destructor.
                    ~SubactionInfo();

                    /// Get property node for subaction info.
                    SGPropertyNode *getNode()
                    {
                        return _node;
                    }

                    /// Read process objects from a given property node.
                    void readProcesses(Mode *mode, SGPropertyNode *node,
                                       SGPropertyNode *statusNode);
                    /// Run initial nasal commands in a node.
                    void initNasal(SGPropertyNode *node);
                    /// Initialise nasal bindings.
                    void postinit(const std::string &modulePfx);
                    /// Update inpus and fire bindings.
                    void update(double dt);

                protected:

                    /// The subaction object.
                    osg::ref_ptr<Subaction> _subaction;
                    /// Property node for subaction info.
                    SGPropertyNode_ptr _node;
                    /// Nasal model name.
                    std::string _module;
                    /// The list of process objects for the subaction.
                    std::list<ModeProcess *> _processes;
            };

            /// Path string, in the form "type/mode".
            std::string _path;
            /// Property node for mode.
            SGPropertyNode_ptr _node;
            /// Primary action set used by this mode.
            osg::ref_ptr<ActionSet> _actionSet;
            /// Subaction specific data indexed by subaction object.
            std::map<Subaction *, SubactionInfo *> _subactions;
    };

private:

    /// Get the main property node for VR input status nodes.
    SGPropertyNode *getStatusNode()
    {
        return _statusNode;
    }

    /**
     * Get or create a subaction object for a given subaction @a path.
     * Get an FGVRInput::subaction object or create a new one if it doesn't
     * already exist.
     * @param manager VR manager object.
     * @param path    OpenXR Subaction / top-level-user path.
     * @return An FGVRInput::Subaction object for the given @a path.
     */
    Subaction *getSubaction(osgXR::Manager *manager, const std::string &path);

    /// Main property node for VR input status nodes.
    SGPropertyNode_ptr _statusNode;
    /// Subaction objects, indexed by their OpenXR path.
    std::map<std::string, osg::ref_ptr<Subaction>> _subactions;
    /// ActionSet objects, indexed by their names.
    std::map<std::string, osg::ref_ptr<ActionSet>> _actionSets;
    /// InteractionProfile objects.
    std::list<osg::ref_ptr<InteractionProfile>> _profiles;
    /// Interaction mode objects, indexed by their paths.
    std::map<std::string, Mode *> _modes;
    /**
     * Saved value of osgXR::Manager::isRunning().
     * This is so bindings can be fired once after VR stops to clear values.
     */
    bool _running;
    /// Wrapper around local space node for updating.
    osg::ref_ptr<osg::Group> _localSpaceUpdater;
    /// Local space scene graph node.
    osg::ref_ptr<osg::MatrixTransform> _localSpace;
};

#endif
