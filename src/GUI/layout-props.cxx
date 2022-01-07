#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/props/props.hxx>

#include "FlightGear_pu.h"
#include "layout.hxx"

// This file contains the code implementing the LayoutWidget class in
// terms of a PropertyNode (plus a tiny bit of PUI glue).  See
// layout.cxx for the actual layout engine.

puFont LayoutWidget::FONT;

void LayoutWidget::setDefaultFont(puFont* font, int pixels)
{
    UNIT = (int)(pixels * (1/3.) + 0.999);
    FONT = *font;
}

int LayoutWidget::stringLength(const char* s)
{
    return (int)(FONT.getFloatStringWidth(s) + 0.999);
}

std::string LayoutWidget::type()
{
    string t = _prop->getNameString();
    return t.empty() ? "dialog" : t;
}

bool LayoutWidget::hasParent()
{
    return _prop->getParent() ? true : false;
}

LayoutWidget LayoutWidget::parent()
{
    return LayoutWidget(_prop->getParent());
}

int LayoutWidget::nChildren()
{
    // Hack: assume that any non-leaf nodes but "hrule" and "vrule"
    // are widgets...
    int n = 0;
    for(int i=0; i<_prop->nChildren(); i++) {
        SGPropertyNode* p = _prop->getChild(i);
        string name = p->getNameString();
        if (p->nChildren() != 0 || name == "hrule" || name == "vrule")
            n++;
    }
    return n;
}

LayoutWidget LayoutWidget::getChild(int idx)
{
    // Same hack.  Note that access is linear time in the number of
    // children...
    int n = 0;
    for(int i=0; i<_prop->nChildren(); i++) {
        SGPropertyNode* p = _prop->getChild(i);
        string name = p->getNameString();
        if (p->nChildren() != 0 || name == "hrule" || name == "vrule") {
            if(idx == n) return LayoutWidget(p);
            n++;
        }
    }
    return LayoutWidget(0);
}

bool LayoutWidget::hasField(const char* f)
{
    return _prop->hasChild(f);
}

int LayoutWidget::getNum(const char* f)
{
    return _prop->getIntValue(f);
}

bool LayoutWidget::getBool(const char* f, bool dflt)
{
    return _prop->getBoolValue(f, dflt);
}

std::string LayoutWidget::getStr(const char* f)
{
    return _prop->getStringValue(f);
}

void LayoutWidget::setNum(const char* f, int num)
{
    _prop->setIntValue(f, num);
}

