#include <sys/types.h>
#pragma once
#include <Arduino.h>
#include <vector>
#include <string>
#include "Drawing.h"  

struct Quaternion { float w = 1, x = 0, y = 0, z = 0; };
struct Vec3 { float x = 0, y = 0, z = 0; };
struct Vec2 { int x = 0, y = 0; };

// A face is defined by 3 or 4 vertex indices
struct Face {
    int a, b, c, d;   // if d < 0, treat as triangle
};

struct WireframeModel {
    std::vector<Vec3> verts;
    std::vector<int>  faces;
    uint vertCount = 0;
    uint faceIndexCount = 0;
};

struct VertexBuf {
    Vec3 pos3D;  // rotated
    Vec2 pos2D;  // projected
};

struct ModelBuf {
    const WireframeModel* model = nullptr;
    VertexBuf vertbuf[64];
};

struct MoveBuf {
    Quaternion orientation;
    Vec3 pos;    // linear position
    Vec3 angVel; // angular velocity
    Vec3 linVel; // linear velocity
    Vec3 angAcc; // angular acceleration
    Vec3 linAcc; // linear acceleration
};

struct KeyDir {
    Vec3 angle;  //key dir angle
    Vec3 trans;  //key dir trans
};

struct MaxMove {
    Vec3 maxangacc{5,5,5};          //max angular acc
    Vec3 maxtransacc{50,50,50};     //max trans acc
    Vec3 maxangvel{50,50,50};       //max angular vel
    Vec3 maxtransvel{500,500,500};  //max trans vel
    Vec3 maxangstep{10,10,10};      //max angular step
    Vec3 maxtransstep{10,10,10};    //max trans step
    Vec3 maxangdamp{5,5,5};         //max angular damp
    Vec3 maxtransdamp{5,5,5};       //max trans damp
};


// Core API
void wireframeInit(int centerX, int centerY, int scale);
bool loadWRL(const char* path, WireframeModel& model);
void listLittleFS();
void applyMouseInput(MoveBuf& mov, float dx, float dy, float sensitivity);
void applyMouseInputDirect(MoveBuf& mov, float dx, float dy, float sensitivity);
void centerAndScale(WireframeModel& model, float targetSize);
void moveBufUpdater(MoveBuf& mov, const KeyDir& key, const MaxMove& lim, float dt);
void clearMovBuf(MoveBuf& mov);
void transformModel(ModelBuf* buf, const MoveBuf& mov);
void wireframeDrawCulled(const ModelBuf* buf, uint16_t brightness);
void wireframeDrawAll(const ModelBuf* buf, uint16_t b);
