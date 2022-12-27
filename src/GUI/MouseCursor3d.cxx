#include "MouseCursor3d.hxx"

#include <simgear/scene/util/RenderConstants.hxx>

#include <osg/Geometry>
#include <osg/Geode>

FGMouseCursor3d::FGMouseCursor3d()
{
    // Create buffers for a simple cuboid
    const float w1 = 0.03f;
    const float h1 = 0.05f;
    const float w2 = 0.02f;
    const float h2 = 0.10f;
#define VERT_COUNT (3 * 12)
    const osg::Vec3 verticesRaw[VERT_COUNT] = {
        /* point edges */
        { 0, 0, 0 }, { -w1, -h1,   0 }, {   0, -h1, -w1 },
        { 0, 0, 0 }, {   0, -h1, -w1 }, {  w1, -h1,   0 },
        { 0, 0, 0 }, {  w1, -h1,   0 }, {   0, -h1,  w1 },
        { 0, 0, 0 }, {   0, -h1,  w1 }, { -w1, -h1,   0 },
        /* bottom */
        { -w1, -h1, 0 }, { 0, -h1, -w1 }, {  w1, -h1, 0 },
        { -w1, -h1, 0 }, { 0, -h1,  w1 }, { -w1, -h1, 0 },
        /* handle edges */
        { 0, 0, 0 }, { -w2, -h2,   0 }, {   0, -h2, -w2 },
        { 0, 0, 0 }, {   0, -h2, -w2 }, {  w2, -h2,   0 },
        { 0, 0, 0 }, {  w2, -h2,   0 }, {   0, -h2,  w2 },
        { 0, 0, 0 }, {   0, -h2,  w2 }, { -w2, -h2,   0 },
        /* handle bottom */
        { -w2, -h2, 0 }, { 0, -h2, -w2 }, {  w2, -h2, 0 },
        { -w2, -h2, 0 }, { 0, -h2,  w2 }, { -w2, -h2, 0 },
    };
#if 0
    const osg::Vec3 normalsRaw[VERT_COUNT] = {
        /* bottom */
        {  0,  0, -1 }, {  0,  0, -1 }, {  0,  0, -1 }, {  0,  0, -1 },
        /* top */
        {  0,  0,  1 }, {  0,  0,  1 }, {  0,  0,  1 }, {  0,  0,  1 },
        /* sides */
        { -1,  0,  0 }, { -1,  0,  0 }, { -1,  0,  0 }, { -1,  0,  0 },
        {  0,  1,  0 }, {  0,  1,  0 }, {  0,  1,  0 }, {  0,  1,  0 },
        {  1,  0,  0 }, {  1,  0,  0 }, {  1,  0,  0 }, {  1,  0,  0 },
        {  0, -1,  0 }, {  0, -1,  0 }, {  0, -1,  0 }, {  0, -1,  0 },
    };
#endif
    osg::Vec3Array* vertices = new osg::Vec3Array(VERT_COUNT, verticesRaw);
    //osg::Vec3Array* normals = new osg::Vec3Array(VERT_COUNT, normalsRaw);
    osg::DrawArrays* prim = new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, VERT_COUNT);

    // Create a geometry for the cursor
    osg::Geometry* geom = new osg::Geometry;
    geom->setVertexArray(vertices);
    //geom->setNormalArray(normals, osg::Array::BIND_PER_VERTEX);
    geom->addPrimitiveSet(prim);

    // Create a geode for the cursor
    osg::Geode* geode = new osg::Geode;
    geode->setName("Cursor");
    geode->addDrawable(geom);

    addChild(geode);
    setNodeMask(~simgear::PICK_BIT);
}
