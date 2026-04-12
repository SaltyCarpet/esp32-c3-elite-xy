#include "Models.h"
#include <vector>

// --- Glyph definitions (static, private) ---
static std::vector<Vec3> glyphAVerts = { {0,0,0}, {0,7,0}, {4,7,0}, {4,0,0}, {0,4,0}, {4,4,0} };
static std::vector<std::vector<int>> glyphAFaces = { {0,1}, {1,2}, {2,3}, {3,0}, {4,5} };
static WireframeModel glyphAModel = { glyphAVerts, glyphAFaces };

// Digit 0
static std::vector<Vec3> digit0Verts = {
    {0,0,0},{0,7,0},{4,7,0},{4,0,0}
};
static std::vector<std::vector<int>> digit0Faces = {
    {0,1},{1,2},{2,3},{3,0}
};
static WireframeModel digit0Model = { digit0Verts, digit0Faces };

// Digit 1
static std::vector<Vec3> digit1Verts = {
    {2,0,0},{2,7,0}
};
static std::vector<std::vector<int>> digit1Faces = {
    {0,1}
};
static WireframeModel digit1Model = { digit1Verts, digit1Faces };

// Digit 2
static std::vector<Vec3> digit2Verts = {
    {0,7,0},{4,7,0},{4,4,0},{0,0,0},{4,0,0}
};
static std::vector<std::vector<int>> digit2Faces = {
    {0,1},{1,2},{2,3},{3,4}
};
static WireframeModel digit2Model = { digit2Verts, digit2Faces };

// Digit 3
static std::vector<Vec3> digit3Verts = {
    {0,7,0},{4,7,0},{4,0,0},{0,0,0},{4,4,0}
};
static std::vector<std::vector<int>> digit3Faces = {
    {0,1},{1,2},{2,3},{1,4},{4,2}
};
static WireframeModel digit3Model = { digit3Verts, digit3Faces };

// Digit 4
static std::vector<Vec3> digit4Verts = {
    {0,7,0},{0,4,0},{4,4,0},{4,7,0},{4,0,0}
};
static std::vector<std::vector<int>> digit4Faces = {
    {0,1},{1,2},{2,3},{2,4}
};
static WireframeModel digit4Model = { digit4Verts, digit4Faces };

// Digit 5
static std::vector<Vec3> digit5Verts = {
    {4,7,0},{0,7,0},{0,4,0},{4,4,0},{4,0,0},{0,0,0}
};
static std::vector<std::vector<int>> digit5Faces = {
    {0,1},{1,2},{2,3},{3,4},{4,5}
};
static WireframeModel digit5Model = { digit5Verts, digit5Faces };

// Digit 6
static std::vector<Vec3> digit6Verts = {
    {4,7,0},{0,7,0},{0,0,0},{4,0,0},{4,4,0},{0,4,0}
};
static std::vector<std::vector<int>> digit6Faces = {
    {0,1},{1,2},{2,3},{3,4},{4,5},{5,2}
};
static WireframeModel digit6Model = { digit6Verts, digit6Faces };

// Digit 7
static std::vector<Vec3> digit7Verts = {
    {0,7,0},{4,7,0},{2,0,0}
};
static std::vector<std::vector<int>> digit7Faces = {
    {0,1},{1,2}
};
static WireframeModel digit7Model = { digit7Verts, digit7Faces };

// Digit 8
static std::vector<Vec3> digit8Verts = {
    {0,0,0},{0,7,0},{4,7,0},{4,0,0},{0,4,0},{4,4,0}
};
static std::vector<std::vector<int>> digit8Faces = {
    {0,1},{1,2},{2,3},{3,0},{4,5}
};
static WireframeModel digit8Model = { digit8Verts, digit8Faces };

// Digit 9
static std::vector<Vec3> digit9Verts = {
    {0,0,0},{0,7,0},{4,7,0},{4,0,0},{0,4,0},{4,4,0}
};
static std::vector<std::vector<int>> digit9Faces = {
    {0,1},{1,2},{2,3},{3,4},{4,5}
};
static WireframeModel digit9Model = { digit9Verts, digit9Faces };


// --- Glyph lookup map (static, private) ---
static std::unordered_map<char, WireframeModel*> glyphModels = {
    {'A', &glyphAModel},
    {'0', &digit0Model},
    {'1', &digit1Model},
    {'2', &digit2Model},
    {'3', &digit3Model},
    {'4', &digit4Model},
    {'5', &digit5Model},
    {'6', &digit6Model},
    {'7', &digit7Model},
    {'8', &digit8Model},
    {'9', &digit9Model}
    // add more glyphs...
};

void drawText(const std::string& text, int startX, int startY, int spacing, uint16_t brightness) {
    int x = startX;
    for (char c : text) {
        auto it = glyphModels.find(c);
        if (it == glyphModels.end()) {
            x += spacing; // skip unknown char
            continue;
        }

        ModelBuf buf;
        buf.model = it->second;
        buf.vertbuf.clear();

        // Project glyph verts into screen space
        for (const auto& v : buf.model->verts) {
            VertexBuf vb;
            vb.pos3D = v;
            vb.pos2D.x = x + v.x;
            vb.pos2D.y = startY + v.y;
            buf.vertbuf.push_back(vb);
        }

        wireframeDrawAll(&buf, brightness);
        x += spacing;
    }
}


// -------------------------
// coord (unit wireframe)
// -------------------------
static std::vector<Vec3> coordVerts = {
  {0,0,0}, {1,0,0}, {0,1,0}, {0,0,1},  // front (z = -1)
};
static std::vector<std::vector<int>> coordFaces = {
  {0,1},
  {0,2},
  {0,3}
};

WireframeModel coordModel = {
  coordVerts,
  coordFaces,
};

// -------------------------
// Cube (unit wireframe)
// -------------------------
static std::vector<Vec3> cubeVerts = {
  {-1,-1,-1}, { 1,-1,-1}, { 1, 1,-1}, {-1, 1,-1},  // front (z = -1)
  {-1,-1, 1}, { 1,-1, 1}, { 1, 1, 1}, {-1, 1, 1}   // rear  (z =  1)
};
static std::vector<std::vector<int>> cubeFaces = {
  {0,1,2,3}, // front
  {7,6,5,4}, // rear
  {4,5,1,0}, // bottom
  {6,7,3,2}, // top
  {0,3,7,4}, // left
  {5,6,2,1}  // right
};

WireframeModel cubeModel = {
  cubeVerts,
  cubeFaces,
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
static std::vector<std::vector<int>> shipFaces = {
  // Nose cap (triangles)
  {0,2,1}, // left
  {0,3,2}, // top
  {0,4,3}, // right
  {0,1,4}, // bottom
  // Hull (quads)
  {1,2,6,5}, // left side
  {3,4,8,7}, // right side
  {2,3,7,6}, // top
  {4,1,5,8}, // bottom
  {5,6,7,8}  // rear
};

WireframeModel shipModel = {
  shipVerts,
  shipFaces,
};

