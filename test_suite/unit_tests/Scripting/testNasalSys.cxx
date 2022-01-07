/*
 * Copyright (C) 2016 Edward d'Auvergne
 *
 * This file is part of the program FlightGear.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "testNasalSys.hxx"

#include "test_suite/FGTestApi/testGlobals.hxx"
#include "test_suite/FGTestApi/NavDataCache.hxx"

#include <simgear/structure/commands.hxx>

#include <Main/fg_props.hxx>
#include <Main/globals.hxx>
#include <Main/util.hxx>

#include <Scripting/NasalSys.hxx>

#include <Main/FGInterpolator.hxx>

// Set up function for each test.
void NasalSysTests::setUp()
{
    FGTestApi::setUp::initTestGlobals("NasalSys");
    FGTestApi::setUp::initNavDataCache();

    fgInitAllowedPaths();
    globals->get_props()->getNode("nasal", true);

    globals->add_subsystem("prop-interpolator", new FGInterpolator, SGSubsystemMgr::INIT);

    globals->get_subsystem_mgr()->bind();
    globals->get_subsystem_mgr()->init();

    globals->add_new_subsystem<FGNasalSys>(SGSubsystemMgr::INIT);

    globals->get_subsystem_mgr()->postinit();
}


// Clean up after each test.
void NasalSysTests::tearDown()
{
    FGTestApi::tearDown::shutdownTestGlobals();
}


// Test test
void NasalSysTests::testStructEquality()
{
    bool ok = FGTestApi::executeNasal(R"(
     var foo = {
      "name": "Bob",
          "size": [512, 512],
          "mipmapping": 1.9
        };
                                      
    var bar = {
      "name": "Bob",
          "size": [512, 512],
          "mipmapping": 1.9
        };       

    unitTest.assert_equal(foo, bar);
                                      
    append(bar.size, "Wowow");
    unitTest.assert(unitTest.equal(foo, bar) == 0);
                                      
    append(foo.size, "Wowow");
    unitTest.assert_equal(foo, bar);
                                      
    foo.wibble = 99.1;
    unitTest.assert(unitTest.equal(foo, bar) == 0);
                                      
    bar.wibble = 99;
    unitTest.assert(unitTest.equal(foo, bar) == 0);
    bar.wibble = 99.1;
    unitTest.assert_equal(foo, bar);

    )");
    CPPUNIT_ASSERT(ok);
}

void NasalSysTests::testCommands()
{
    auto nasalSys = globals->get_subsystem<FGNasalSys>();
    nasalSys->getAndClearErrorList();

    fgSetInt("/foo/test", 7);
    bool ok = FGTestApi::executeNasal(R"(
     var f = func { 
         var i = getprop('/foo/test');
         setprop('foo/test', i + 4);
     };
                                      
      addcommand('do-foo', f);
      var ok = fgcommand('do-foo');
      unitTest.assert(ok);
    )");
    CPPUNIT_ASSERT(ok);

    CPPUNIT_ASSERT_EQUAL(11, fgGetInt("/foo/test"));

    SGPropertyNode_ptr args(new SGPropertyNode);
    ok = globals->get_commands()->execute("do-foo", args);
    CPPUNIT_ASSERT(ok);
    CPPUNIT_ASSERT_EQUAL(15, fgGetInt("/foo/test"));

    ok = FGTestApi::executeNasal(R"(
       var g = func { print('fail'); };
       addcommand('do-foo', g);
    )");
    CPPUNIT_ASSERT(ok);

    auto errors = nasalSys->getAndClearErrorList();
    CPPUNIT_ASSERT_EQUAL(1UL, errors.size());

    // old command should still be registered and work
    ok = globals->get_commands()->execute("do-foo", args);
    CPPUNIT_ASSERT(ok);
    CPPUNIT_ASSERT_EQUAL(19, fgGetInt("/foo/test"));

    ok = FGTestApi::executeNasal(R"(
      removecommand('do-foo');
   )");
    CPPUNIT_ASSERT(ok);

    ok = FGTestApi::executeNasal(R"(
     var ok = fgcommand('do-foo');
     unitTest.assert(!ok);
  )");
    CPPUNIT_ASSERT(ok);

    errors = nasalSys->getAndClearErrorList();
    CPPUNIT_ASSERT_EQUAL(0UL, errors.size());

    // should fail, command is removed
    ok = globals->get_commands()->execute("do-foo", args);
    CPPUNIT_ASSERT(!ok);
    CPPUNIT_ASSERT_EQUAL(19, fgGetInt("/foo/test"));
}

void NasalSysTests::testAirportGhost()
{
    auto nasalSys = globals->get_subsystem<FGNasalSys>();
    nasalSys->getAndClearErrorList();

    bool ok = FGTestApi::executeNasal(R"(
        var apt = airportinfo('LFBD');
        var taxiways = apt.taxiways;    
        unitTest.assert_equal(size(taxiways), 0);
    )");
    CPPUNIT_ASSERT(ok);

}

// https://sourceforge.net/p/flightgear/codetickets/2246/

void NasalSysTests::testCompileLarge()
{
    auto nasalSys = globals->get_subsystem<FGNasalSys>();
    nasalSys->getAndClearErrorList();

    
    string code = "var foo = 0;\n";
    for (int i=0; i<14; ++i) {
        code = code + code;
    }
    
    nasalSys->parseAndRun(code);
    
//    bool ok = FGTestApi::executeNasal(R"(
//var try_compile = func(code) {
//    call(compile, [code], nil,nil,var err=[]);
//    return size(err);
//}
//
//var expression = "var foo = 0;\n";
//var code = "";
//
//for(var i=0;i<=10000;i+=1) {
//    code ~= expression;
//    if (try_compile(code) == 1) {
//        print("Error compiling, LOC count is:", i+1);
//        break;
//    }
//}
//    )");
//    CPPUNIT_ASSERT(ok);
}


void NasalSysTests::testRoundFloor()
{
    auto nasalSys = globals->get_subsystem<FGNasalSys>();
    nasalSys->getAndClearErrorList();

    bool ok = FGTestApi::executeNasal(R"(
        unitTest.assert_equal(math.round(121266, 1000), 121000);
        unitTest.assert_equal(math.round(121.1234, 0.01), 121.12);
        unitTest.assert_equal(math.round(121266, 10), 121270);
    
        unitTest.assert_equal(math.floor(121766, 1000), 121000);
        unitTest.assert_equal(math.floor(121.1299, 0.01), 121.12);
    
        # floor towards lower value
        unitTest.assert_equal(math.floor(-121.1229, 0.01), -121.13);
    
        # truncate towards zero
        unitTest.assert_equal(math.trunc(-121.1229, 0.01), -121.12);
        unitTest.assert_equal(math.trunc(-121.1299, 0.01), -121.12);
    )");
    CPPUNIT_ASSERT(ok);
}

void NasalSysTests::testRange()
{
    auto nasalSys = globals->get_subsystem<FGNasalSys>();
    nasalSys->getAndClearErrorList();

    bool ok = FGTestApi::executeNasal(R"(
        unitTest.assert_equal(range(5), [0, 1, 2, 3, 4]);
        unitTest.assert_equal(range(2, 8), [2, 3, 4, 5, 6, 7]);
        unitTest.assert_equal(range(2, 10, 3), [2, 5, 8]);

    )");
    CPPUNIT_ASSERT(ok);
}
