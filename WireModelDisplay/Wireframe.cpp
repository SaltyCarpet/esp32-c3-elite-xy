#include <Arduino.h>
#include <math.h>
#include <vector>
#include "FS.h"
#include "LittleFS.h"
#include "Drawing.h"
#include "Wireframe.h"
#include "MathHelpers.h"


static int g_cX = (1<<11);
static int g_cY = (1<<11);
static int g_scale = (1<<12);

void wireframeInit(int centerX, int centerY, int scale) {
    g_cX = centerX;
    g_cY = centerY;
    g_scale   = scale;
}

void listLittleFS() {
    File root = LittleFS.open("/vrml");
    if (!root) {
        Serial.println("Failed to open root directory");
        return ;
    }
    if (!root.isDirectory()) {
        Serial.println("Root is not a directory");
        return;
    }
    Serial.println("List Files:");
    File file = root.openNextFile();
    while (file) {
        Serial.print("FILE: ");
        Serial.print(file.name());
        Serial.print("  SIZE: ");
        Serial.println(file.size());
        file = root.openNextFile();
    }
}

bool initModelBuf(const char* path, float targetSize, ModelBuf &buf)
{
    WireframeModel* model = new WireframeModel();
    if (!loadWRL(path, *model)) return false;
    return centerAndScale(buf, *model, targetSize);
}

bool loadWRL(const char* path, WireframeModel& model) {
    File file = LittleFS.open(path, "r");
    if (!file) {
        Serial.printf("Failed to open %s\n", path);
        return false;
    }
    model.verts.clear();
    model.faces.clear();

    enum Section { NONE, POINTS, INDICES } section = NONE;
    String line;

    while (file.available()) {
        line = file.readStringUntil('\n');
        //Serial.println(line);
        int comment = line.indexOf('#');
        if (comment >= 0) line = line.substring(0, comment);

        if (line.indexOf("point") >= 0) { section = POINTS; continue; }
        if (line.indexOf("coordIndex") >= 0) { section = INDICES; continue; }
        if (line.indexOf("]") >= 0) { section = NONE; continue; }

        line.replace(",", " "); // commas → spaces
        line.trim();
        if (line.length() == 0) continue;

        if (section == POINTS) {
            float x,y,z;
            const char* cstr = line.c_str();
            while (sscanf(cstr, "%f %f %f", &x, &y, &z) == 3) {
                model.verts.push_back({x,y,z});
                // advance pointer to next numbers
                // find next space after z
                for (int skip=0; skip<3 && *cstr; ) {
                    if (isspace(*cstr)) { ++cstr; continue; }
                    while (*cstr && !isspace(*cstr)) ++cstr;
                    ++skip;
                }
            }
        }
        else if (section == INDICES) {
            int idx;
            const char* cstr = line.c_str();
            while (sscanf(cstr, "%d", &idx) == 1) {
                model.faces.push_back(idx);
                // advance pointer to next number
                while (*cstr && !isspace(*cstr) && *cstr!=',') ++cstr;
                while (*cstr && (isspace(*cstr) || *cstr==',')) ++cstr;
            }
        }

    }
    model.vertCount = model.verts.size();
    model.faceIndexCount = model.faces.size();
    file.close();
    return true;
}

bool centerAndScale(ModelBuf& buf, WireframeModel& model, float targetSize)
{
    if (model.verts.empty()) return false;
    
    // Step 1: bounding box
    float minX = model.verts[0].x, maxX = model.verts[0].x;
    float minY = model.verts[0].y, maxY = model.verts[0].y;
    float minZ = model.verts[0].z, maxZ = model.verts[0].z;

    for (auto& v : model.verts) {
        if (v.x < minX) {minX = v.x;}
        if (v.x > maxX) {maxX = v.x;}
        if (v.y < minY) {minY = v.y;}
        if (v.y > maxY) {maxY = v.y;}
        if (v.z < minZ) {minZ = v.z;}
        if (v.z > maxZ) {maxZ = v.z;}
    }

    // Step 2: center
    Vec3 c = {(minX + maxX) * 0.5f, (minY + maxY) * 0.5f, (minZ + maxZ) * 0.5f};

    // Step 3: subtract center
    for (auto& v : model.verts) {
        v -= c;
    }

    buf.radius = targetSize * 0.5f;
    buf.model = &model;

    // Step 4: largest extent
    float extentX = maxX - minX;
    float extentY = maxY - minY;
    float extentZ = maxZ - minZ;
    float maxExtent = std::max({extentX, extentY, extentZ});

    if (maxExtent < 1e-6f) return false; // avoid div by zero

    // Step 5: scale factor
    float scale = targetSize / maxExtent;

    // Step 6: apply scale
    for (auto& v : model.verts) {
        v *= scale;
    }
    buf.model = &model;
    return true;
}


void moveBufUpdater(MoveBuf& mov, const KeyDir& key, const MaxMove& lim, float dt)
{
    if (dt <= 0) return;

    // --- helper for one axis ---
    auto updateAxis = [&](float& acc, float& vel, float keyDir,
                          float maxAcc, float step, float damp, float maxVel)
    {
        float targetAcc = keyDir * maxAcc;
        float damping = (1.0f - damp * dt * (1.0f - fabs(keyDir)));
        // ramp acceleration toward target
        acc += (targetAcc - acc) * step * dt;
        // integrate velocity
        vel += acc * dt;
        // damping
        vel *= (damping>0)?damping:0.0f;
        // clamp velocity
        if (vel >  maxVel) vel =  maxVel;
        if (vel < -maxVel) vel = -maxVel;
    };

    // --- angular axes ---
    updateAxis(mov.angAcc.x, mov.angVel.x, key.angle.x,
               lim.maxangacc.x, lim.maxangstep.x, lim.maxangdamp.x, lim.maxangvel.x);
    updateAxis(mov.angAcc.y, mov.angVel.y, key.angle.y,
               lim.maxangacc.y, lim.maxangstep.y, lim.maxangdamp.y, lim.maxangvel.y);
    updateAxis(mov.angAcc.z, mov.angVel.z, key.angle.z,
               lim.maxangacc.z, lim.maxangstep.z, lim.maxangdamp.z, lim.maxangvel.z);

    // --- translational axes ---
    updateAxis(mov.linAcc.x, mov.linVel.x, key.trans.x,
               lim.maxtransacc.x, lim.maxtransstep.x, lim.maxtransdamp.x, lim.maxtransvel.x);
    updateAxis(mov.linAcc.y, mov.linVel.y, key.trans.y,
               lim.maxtransacc.y, lim.maxtransstep.y, lim.maxtransdamp.y, lim.maxtransvel.y);
    updateAxis(mov.linAcc.z, mov.linVel.z, key.trans.z,
               lim.maxtransacc.z, lim.maxtransstep.z, lim.maxtransdamp.z, lim.maxtransvel.z);

    // --- integrate orientation from angular velocity ---
    integrateOrientation(mov.orientation, mov.angVel, dt);

    // --- integrate position from linear velocity ---
    // Rotate local velocity into world space using quaternion
    Vec3 worldVel = rotateVector(mov.orientation, mov.linVel);
    mov.pos += (worldVel * dt);
}

void clearMovBufs(MoveBuf& mov, KeyDir& kd)
{
    mov.orientation = {1,-1,0,0};
    mov.pos = {0,0,10};
    mov.angVel = {0,0,0};
    mov.linVel = {0,0,0};
    mov.angAcc = {0,0,0};
    mov.linAcc = {0,0,0};
    kd.angle = {0,0,0};
    kd.trans = {0,0,0};
}
/*
// -------------------- Collision Check --------------------
bool collideSphere(const ModelBuf& a, const ModelBuf& b) {
    float dx = a.pos.x - b.center.x;
    float dy = a.pos3D.y - b.center.y;
    float dz = a.center.z - b.center.z;
    float dist2 = dx*dx + dy*dy + dz*dz;
    float rsum = a.radius + b.radius;
    return dist2 <= rsum*rsum;
}
*/

void applyRotInput(MoveBuf& mov, float dax, float day, float daz, float sensitivity)
{
    // scale deltas to radians
    float yaw   = dax * sensitivity;
    float pitch = day * sensitivity;
    float roll  = daz * sensitivity;

    // local axes in ship space
    Vec3 up      = {0,1,0};  // yaw axis
    Vec3 right   = {1,0,0};  // pitch axis
    Vec3 forward = {0,0,1};  // roll axis

    // build incremental quaternions
    Quaternion qYaw   = quatFromAxisAngle(up.x,      up.y,      up.z,      yaw);
    Quaternion qPitch = quatFromAxisAngle(right.x,   right.y,   right.z,   pitch);
    Quaternion qRoll  = quatFromAxisAngle(forward.x, forward.y, forward.z, roll);

    // apply them to orientation
    mov.orientation = (mov.orientation * qYaw);
    mov.orientation = (mov.orientation * qPitch);
    mov.orientation = (mov.orientation * qRoll);

    // normalize to avoid drift
    mov.orientation = quatNormalize(mov.orientation);
}

// Rotate ship around an arbitrary world axis using joystick input
void applyRotInputAxis(MoveBuf& mov, Vec3 input, const Vec3& worldAxis, const Vec3& shipRot, const float sensitivity)
{
    // Compute desired angle from joystick
    // Example: use atan2 for 2D joystick input
    if (input.x == 0 && input.y == 0) return;
    float angle = atan2f(input.x, input.y); // radians

    // Normalize world axis
    if (worldAxis.x == 0 && worldAxis.y == 0 && worldAxis.z == 0) return;
    Vec3 rotAxis = normalize(worldAxis);

    // Build target quaternion from axis + angle
    Quaternion qTarget = quatFromAxisAngle(rotAxis.x, rotAxis.y, rotAxis.z, angle);
    qTarget = quatNormalize(qTarget);

    // --- Extract ship axes in world space after base rotation ---
    Vec3 chosenRot = rotateVector(qTarget, shipRot);

    // --- Alignment correction: chosenRot axis must align with worldRotAxis ---
    Vec3 corrAxis = cross(chosenRot, rotAxis);
    float corrMag = magnitude(&corrAxis.x);
    if (corrMag > 1e-8f) {
        corrAxis = {corrAxis.x/corrMag, corrAxis.y/corrMag, corrAxis.z/corrMag};
        float dotVal = fmaxf(-1.0f, fminf(1.0f, dot(&chosenRot.x, &rotAxis.x)));
        float corrAngle = acosf(dotVal);
        Quaternion qCorr = quatFromAxisAngle(corrAxis.x, corrAxis.y, corrAxis.z, corrAngle);
        qTarget = (qCorr * qTarget);
    }

    // Interpolate toward target orientation
    float dp = dot(&mov.orientation.w, &qTarget.w, 4);
    if (dp < 0.0f) {
        qTarget *= -1.0f;
    }
    Quaternion result = mov.orientation + (sensitivity*(qTarget - mov.orientation));
    mov.orientation = quatNormalize(result);
    mov.orientation = quatNormalize(mov.orientation);
}

void applyRotInputAxis2(MoveBuf& mov, Vec3 input, const Vec3& worldAxis, const Vec3& shipRot, const float sensitivity)
{
    // Compute desired angle from joystick
    // Example: use atan2 for 2D joystick input
    if (input.x == 0 && input.y == 0) return;
    float angle = atan2f(input.x, input.y); // radians

    // Normalize world axis
    if (worldAxis.x == 0 && worldAxis.y == 0 && worldAxis.z == 0) return;
    Vec3 rotAxis = normalize(worldAxis);

    // Build target quaternion from axis + angle
    Quaternion qTarget = quatFromAxisAngle(rotAxis.x, rotAxis.y, rotAxis.z, angle);
    qTarget = quatNormalize(qTarget);

    // Interpolate toward target orientation
    float dp = dot(&mov.orientation.w, &qTarget.w, 4);
    if (dp < 0.0f) {
        qTarget *= -1.0f;
    }
    Quaternion step = mov.orientation + (sensitivity*(qTarget - mov.orientation));
    step = quatNormalize(step);
    Vec3 currentAxis = rotateVector(step, shipRot);
    currentAxis = normalize(currentAxis);
    Quaternion qAlign = quatAlign(currentAxis, rotAxis);
    qAlign = quatNormalize(qAlign);
    mov.orientation = quatNormalize(qAlign * step);
}

void applyRotInputDirect(MoveBuf& mov, Vec3 input, Vec3 screenUp, float sensitivity)
{
    // Desired forward direction from joystick
    Vec3 forward = normalize(input);
    if (forward.x == 0 && forward.y == 0 && forward.z == 0) return;

    // Build orthonormal basis
    Vec3 right = normalize(cross(screenUp, forward));
    if (right.x == 0 && right.y == 0 && right.z == 0) {
        // Degenerate case: forward parallel to up
        right = {1,0,0};
    }
    Vec3 up = cross(forward, right);

    // Rotation matrix
    float m[3][3] = {
        { right.x,   right.y,   right.z },
        { up.x,      up.y,      up.z    },
        { forward.x, forward.y, forward.z }
    };

    // Convert to quaternion
    Quaternion q = quatFromRotationMatrix(m);
    Quaternion qTarget = quatNormalize(q);
    // Assign orientation incrementally
    float dp = dot(&mov.orientation.w, &qTarget.w, 4);
    if (dp < 0.0f) {
        dp = -dp;
        qTarget *= -1.0f;
    }
    Quaternion result = mov.orientation + (sensitivity*(qTarget - mov.orientation));
    mov.orientation = quatNormalize(result);
}

// -----------------------------
// Transform: rotate in ship space + translate in ship space + project
// -----------------------------
void transformModel(ModelBuf* buf, const MoveBuf& mov)
{
    // Convert orientation quaternion to rotation matrix
    //float R[3][3];
    //quatToMatrix(mov.orientation, R);

    for (int i=0; i<buf->model->vertCount; i++) {
        // Original model-space vertex
        Vec3 v = buf->model->verts[i];

        // Rotate vertex by ship orientation
        Vec3 vr = rotateVector(mov.orientation, v);

        // Translate by ship’s world position
        vr += mov.pos;

        buf->vertbuf[i].pos3D = vr;

        // --- Project to 2D ---
        float denom = vr.z;
        if (denom < 0.1f) {
            buf->vertbuf[i].pos2D.x = (int)g_cX;
            buf->vertbuf[i].pos2D.y = (int)g_cY;
            continue;
        }
        buf->vertbuf[i].pos2D.x = g_cX + (int)lrintf(vr.x * (float)g_scale / denom);
        buf->vertbuf[i].pos2D.y = g_cY + (int)lrintf(vr.y * (float)g_scale / denom);
    }
}

inline bool faceVisible(const VertexBuf* buf,
                        const int* indices, int count)
{
    if (count < 3) return false;
    Vec3 a = buf[indices[0]].pos3D;
    Vec3 b = buf[indices[1]].pos3D;
    Vec3 c = buf[indices[2]].pos3D;
    Vec3 u = b - a;
    Vec3 v = c - a;
    Vec3 n = { u.y*v.z - u.z*v.y,
               u.z*v.x - u.x*v.z,
               u.x*v.y - u.y*v.x };
    return (n.z > 1e-5f); // flip sign if needed
}

void wireframeDrawCulled(const ModelBuf* buf,
                         uint16_t brightness)
{
    int start = 0;
    for (int i=0; i<buf->model->faceIndexCount; i++) {
        if (buf->model->faces[i] == -1) {
            int end = i;
            int count = end - start;
            if ((count >= 3 && faceVisible(buf->vertbuf, &buf->model->faces[start], count))||count == 2) {
                for (int j=0; j<count; j++) {
                    int a = buf->model->faces[start + j];
                    int b = buf->model->faces[start + ((j+1)%count)];
                    drawLineBresenham(buf->vertbuf[a].pos2D.x, buf->vertbuf[a].pos2D.y,
                                      buf->vertbuf[b].pos2D.x, buf->vertbuf[b].pos2D.y,
                                      brightness);
                }
            }
            start = i+1;
        }
    }
}

void wireframeDrawAll(const ModelBuf* buf,
                      uint16_t brightness)
{
    int start = 0;
    for (int i=0; i<buf->model->faceIndexCount; i++) {
        if (buf->model->faces[i] == -1) {
            int end = i;
            int count = end - start;
            if (count >= 2) {
                for (int j=0; j<count; j++) {
                    int a = buf->model->faces[start + j];
                    int b = buf->model->faces[start + ((j+1)%count)];
                    drawLineBresenham(buf->vertbuf[a].pos2D.x, buf->vertbuf[a].pos2D.y,
                                      buf->vertbuf[b].pos2D.x, buf->vertbuf[b].pos2D.y,
                                      brightness);
                }
            }
            start = i+1;
        }
    }
}
