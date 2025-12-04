#include "Models.h"
#include <vector>

// -------------------------
// coord (unit wireframe)
// -------------------------
static std::vector<Vec3> coordVerts = {
  {0,0,0}, {1,0,0}, {0,1,0}, {0,0,1},  // front (z = -1)
};
static std::vector<int> coordFaces = {
  0,1,-1,
  0,2,-1,
  0,3,-1
};

WireframeModel coordModel = {
  coordVerts,
  coordFaces,
  coordVerts.size(),
  coordFaces.size()
};

// -------------------------
// Cube (unit wireframe)
// -------------------------
static std::vector<Vec3> cubeVerts = {
  {-1,-1,-1}, { 1,-1,-1}, { 1, 1,-1}, {-1, 1,-1},  // front (z = -1)
  {-1,-1, 1}, { 1,-1, 1}, { 1, 1, 1}, {-1, 1, 1}   // rear  (z =  1)
};
static std::vector<int> cubeFaces = {
  0,1,2,3,-1, // front
  7,6,5,4,-1, // rear
  4,5,1,0,-1, // bottom
  6,7,3,2,-1, // top
  0,3,7,4,-1, // left
  5,6,2,1,-1  // right
};

WireframeModel cubeModel = {
  cubeVerts,
  cubeFaces,
  cubeVerts.size(),
  cubeFaces.size()
};

// -------------------------
// Ship (closed tapered hull)
// -------------------------
// Layout:
//  - Vertex 0: nose
//  - 1..4: front rectangle (-1..1, -0.5..0.5, z=-1)
//  - 5..8: rear rectangle  (-2..2, -1..1,   z=+1)
// Faces:
//  - 4 triangles forming the nose cap
//  - 5 quads: left, right, top, bottom, rear

static std::vector<Vec3> shipVerts = {
  { 0.0f,  0.0f, -2.0f}, // 0 nose
  {-1.0f, -0.5f, -1.0f}, // 1 front LL
  {-1.0f,  0.5f, -1.0f}, // 2 front UL
  { 1.0f,  0.5f, -1.0f}, // 3 front UR
  { 1.0f, -0.5f, -1.0f}, // 4 front LR
  {-2.0f, -1.0f,  1.0f}, // 5 rear  LL
  {-2.0f,  1.0f,  1.0f}, // 6 rear  UL
  { 2.0f,  1.0f,  1.0f}, // 7 rear  UR
  { 2.0f, -1.0f,  1.0f}  // 8 rear  LR
};
static std::vector<int> shipFaces = {
  // Nose cap (triangles)
  0,2,1,-1, // left
  0,3,2,-1, // top
  0,4,3,-1, // right
  0,1,4,-1, // bottom
  // Hull (quads)
  1,2,6,5,-1, // left side
  3,4,8,7,-1, // right side
  2,3,7,6,-1, // top
  4,1,5,8,-1, // bottom
  5,6,7,8,-1  // rear
};

WireframeModel shipModel = {
  shipVerts,
  shipFaces,
  shipVerts.size(),
  shipFaces.size()
};

