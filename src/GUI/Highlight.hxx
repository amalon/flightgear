#pragma once

#include <simgear/structure/subsystem_mgr.hxx>
#include <osg/Node>

#include <string>
#include <set>

/* Represents a menu item. */
struct HighlightMenu
{
    HighlightMenu(int menu, int item) : menu(menu), item(item)
    {}
    int menu;   /* Number of menu in the menubar, e.g. 0 is 'File'. */
    int item;   /* Number of item in the menu. */
    std::string description() const;    /* E.g. "0/6: File/Joystick Configuration". */
};

/* Information about OSG nodes, dialogs etc that are associated with a
particular property. */
struct HighlightInfo
{
    /* OSG Nodes that are animated by the property. */
    std::set<osg::ref_ptr<osg::Node>> nodes;
    
    /* Dialogues that modify the property.*/
    std::set<std::string> dialogs;
    
    /* Keypresses that modify the property. */
    std::set<std::string> keypresses;
    
    /* Menu items that modify the property. */
    std::set<HighlightMenu> menus;
};

struct Highlight : SGSubsystem
{
    Highlight();
    ~Highlight();
    
    void bind() override;
    void init() override;
    void reinit() override;
    void shutdown() override;
    void unbind() override;
    void update(double dt) override;

    static const char* staticSubsystemClassId() { return "reflect"; }

    /* If specified node is animated, highlights it and other nodes
    that are animated by the same or related properties. Also updates
    /sim/highlighting/current to contain information about associated dialogs,
    menus, and keypresses.

    Returns the number of properties found. Returns -1 if highlighting is not
    currently enabled. */
    int highlightNodes(osg::Node* node);

    /* Returns information about nodes and UI elements that are associated with a
    specific property. */
    const HighlightInfo& findPropertyInfo(const std::string& property);


    /* Below are individual functions that return properties that are associated
    with a particular node or UI element. */

    /* Returns list of properties that are used to animate the specified OSG node.
    */
    const std::set<std::string>& findNodeProperties(osg::Node* node);

    /* Returns list of properties affected by specified dialog. */
    const std::set<std::string>& findDialogProperties(const std::string& dialog);

    /* Returns list of properties affected by specified keypress. */
    const std::set<std::string>& findKeypressProperties(const std::string& keypress);

    /* Returns list of properties affected by specified menu. */
    const std::set<std::string>& findMenuProperties(const HighlightMenu& menu);

    /* Returns list of properties that are influenced by the specified property,
    /e.g. if <property> is controls/flight/rudder, the returned set could contain
    /surface-positions/rudder-pos-norm. */
    const std::set<std::string>& findPropertyToProperties(const std::string& property);

    /* Returns list of properties that influence the specified property, e.g.
    if <property> is /surface-positions/rudder-pos-norm, the returned set could
    contain /controls/flight/rudder. */
    const std::set<std::string>& findPropertyFromProperties(const std::string& property);

    /* Returns list of menus that open the specified dialog. */
    const std::set<HighlightMenu>& findMenuFromDialog(const std::string& dialog);


    /* Below are functions that are used to set up associations. */

    /* Should be called if <node> is animated using <property>. */
    void addPropertyNode(const std::string& property, osg::ref_ptr<osg::Node> node);

    /* Should be called if <dialog> affects <property>. */
    void addPropertyDialog(const std::string& property, const std::string& dialog);

    /* Should be called if <keypress> affects <property>. */
    void addPropertyKeypress(const std::string& property, const std::string& keypress);

    /* Should be called if <menu> affects <property>. */
    void addPropertyMenu(HighlightMenu menu, const std::string& property);

    /* Should be called if <menu> opens <dialog>. */
    void addMenuDialog(HighlightMenu menu, const std::string& dialog);

    /* Should be called if two properties are associated, for example YASim
    associates /controls/flight/flaps with /surface-positions/flap-pos-norm. */
    void addPropertyProperty(const std::string& property1, const std::string& property2);
};
