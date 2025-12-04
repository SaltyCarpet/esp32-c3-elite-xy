#pragma once

#include <sys/types.h>
#include <Arduino.h>
#include <vector>
#include "Drawing.h"  
#include "MathHelpers.h"


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
    float radius;
};

struct MoveBuf {
    Quaternion orientation;
    Vec3 pos;    // linear position
    Vec3 angVel; // angular velocity
    Vec3 linVel; // linear velocity
    Vec3 angAcc; // angular acceleration
    Vec3 linAcc; // linear acceleration
    int health = 1;
};

struct KeyDir {
    Vec3 angle;  //key dir angle
    Vec3 trans;  //key dir trans
};

struct MaxMove {
    Vec3 maxangacc = {5,5,5};          //max angular acc
    Vec3 maxtransacc = {50,50,50};     //max trans acc
    Vec3 maxangvel = {5,5,5};       //max angular vel
    Vec3 maxtransvel = {15,15,15};  //max trans vel
    Vec3 maxangstep = {10,10,10};      //max angular step
    Vec3 maxtransstep = {10,10,10};    //max trans step
    Vec3 maxangdamp = {1,1,1};         //max angular damp
    Vec3 maxtransdamp = {5,5,5};       //max trans damp
};


// Core API
bool initModelBuf(const char* path, float targetSize, ModelBuf &buf);
void wireframeInit(int centerX, int centerY, int scale);
bool loadWRL(const char* path, WireframeModel& model);
void listLittleFS();
bool centerAndScale(ModelBuf& buf, WireframeModel& model, float targetSize);
//bool collideSphere(const ModelBuf& a, const ModelBuf& b);
void applyRotInput(MoveBuf& mov, float dax, float day, float daz, float sensitivity);
void applyRotInputAxis(MoveBuf& mov, Vec3 input, const Vec3& worldAxis, const Vec3& shipRot, const float sensitivity);
void applyRotInputAxis2(MoveBuf& mov, Vec3 input, const Vec3& worldAxis, const Vec3& shipRot, const float sensitivity);
void applyRotInputDirect(MoveBuf& mov, Vec3 input, Vec3 screenUp, float sensitivity);
void moveBufUpdater(MoveBuf& mov, const KeyDir& key, const MaxMove& lim, float dt);
void clearMovBufs(MoveBuf& mov, KeyDir& kd);
void transformModel(ModelBuf* buf, const MoveBuf& mov);
void wireframeDrawCulled(const ModelBuf* buf, uint16_t brightness);
void wireframeDrawAll(const ModelBuf* buf, uint16_t b);
