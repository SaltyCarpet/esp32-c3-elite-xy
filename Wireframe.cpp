#include <Arduino.h>
#include <math.h>
#include <vector>
//#include <fstream>
//#include <sstream>
#include <string>
//#include <cctype>
#include "FS.h"
#include "LittleFS.h"
#include "Drawing.h"
#include "Wireframe.h"


static int g_cX = (1<<11);
static int g_cY = (1<<11);
static int g_scale = (1<<12);

void wireframeInit(int centerX, int centerY, int scale) {
    g_cX = centerX;
    g_cY = centerY;
    g_scale   = scale;
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

void listLittleFS() {
    File root = LittleFS.open("/vrml");
    if (!root) {
        Serial.println("Failed to open root directory");
        return;
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

// -----------------------------
// Math helpers
// -----------------------------
static inline Vec3 cross(const Vec3& u, const Vec3& v) {
    return { u.y*v.z - u.z*v.y,
             u.z*v.x - u.x*v.z,
             u.x*v.y - u.y*v.x };
}
static inline float dot(const Vec3& u, const Vec3& v) {
    return u.x*v.x + u.y*v.y + u.z*v.z;
}

inline Quaternion quatIdentity() {
    return {1.0f, 0.0f, 0.0f, 0.0f};
}

inline Quaternion quatNormalize(const Quaternion& q) {
    float mag = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (mag < 1e-8f) return quatIdentity();
    return { q.w/mag, q.x/mag, q.y/mag, q.z/mag };
}

inline Quaternion quatMultiply(const Quaternion& a, const Quaternion& b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

inline Quaternion quatConjugate(const Quaternion& q) {
    return { q.w, -q.x, -q.y, -q.z };
}

inline Quaternion quatFromAxisAngle(float ax, float ay, float az, float angle) {
    float half = 0.5f * angle;
    float s = sinf(half);
    return quatNormalize({ cosf(half), ax*s, ay*s, az*s });
}

inline void quatToMatrix(const Quaternion& q, float M[3][3]) {
    float xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
    float xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
    float wx = q.w*q.x, wy = q.w*q.y, wz = q.w*q.z;

    M[0][0] = 1 - 2*(yy + zz);
    M[0][1] = 2*(xy - wz);
    M[0][2] = 2*(xz + wy);

    M[1][0] = 2*(xy + wz);
    M[1][1] = 1 - 2*(xx + zz);
    M[1][2] = 2*(yz - wx);

    M[2][0] = 2*(xz - wy);
    M[2][1] = 2*(yz + wx);
    M[2][2] = 1 - 2*(xx + yy);
}

inline Vec3 rotateVector(const Quaternion& q, const Vec3& v) {
    // v' = q * (0,v) * q^-1
    Quaternion vq = {0, v.x, v.y, v.z};
    Quaternion qConj = quatConjugate(q);
    Quaternion rq = quatMultiply(quatMultiply(q, vq), qConj);
    return { rq.x, rq.y, rq.z };
}

inline void integrateOrientation(Quaternion& q, const Vec3& angVel, float dt) {
    float mag = sqrtf(angVel.x*angVel.x + angVel.y*angVel.y + angVel.z*angVel.z);
    if (mag > 1e-6f) {
        float half = 0.5f * mag * dt;
        float s = sinf(half) / mag;
        Quaternion dq = { cosf(half), angVel.x*s, angVel.y*s, angVel.z*s };
        q = quatNormalize(quatMultiply(q, dq));
    }
}


void moveBufUpdater(MoveBuf& mov, const KeyDir& key, const MaxMove& lim, float dt)
{
    if (dt <= 0) return;

    // --- helper for one axis ---
    auto updateAxis = [&](float& acc, float& vel, int keyDir,
                          float maxAcc, float step, float damp, float maxVel)
    {
        float targetAcc = keyDir * maxAcc;
        // ramp acceleration toward target
        acc += (targetAcc - acc) * step * dt;
        // integrate velocity
        vel += acc * dt;
        // damping
        vel *= (1.0f - damp * dt);
        // clamp velocity
        if (vel >  maxVel) vel =  maxVel;
        if (vel < -maxVel) vel = -maxVel;
    };

    // --- angular axes ---
    updateAxis(mov.angAcc.x, mov.angVel.x, (int)key.angle.x,
               lim.maxangacc.x, lim.maxangstep.x, lim.maxangdamp.x, lim.maxangvel.x);
    updateAxis(mov.angAcc.y, mov.angVel.y, (int)key.angle.y,
               lim.maxangacc.y, lim.maxangstep.y, lim.maxangdamp.y, lim.maxangvel.y);
    updateAxis(mov.angAcc.z, mov.angVel.z, (int)key.angle.z,
               lim.maxangacc.z, lim.maxangstep.z, lim.maxangdamp.z, lim.maxangvel.z);

    // --- translational axes ---
    updateAxis(mov.linAcc.x, mov.linVel.x, (int)key.trans.x,
               lim.maxtransacc.x, lim.maxtransstep.x, lim.maxtransdamp.x, lim.maxtransvel.x);
    updateAxis(mov.linAcc.y, mov.linVel.y, (int)key.trans.y,
               lim.maxtransacc.y, lim.maxtransstep.y, lim.maxtransdamp.y, lim.maxtransvel.y);
    updateAxis(mov.linAcc.z, mov.linVel.z, (int)key.trans.z,
               lim.maxtransacc.z, lim.maxtransstep.z, lim.maxtransdamp.z, lim.maxtransvel.z);

    // --- integrate orientation from angular velocity ---
    integrateOrientation(mov.orientation, mov.angVel, dt);

    // --- integrate position from linear velocity ---
    // Rotate local velocity into world space using quaternion
    Vec3 worldVel = rotateVector(mov.orientation, mov.linVel);
    mov.pos.x += worldVel.x * dt;
    mov.pos.y += worldVel.y * dt;
    mov.pos.z += worldVel.z * dt;
}


void clearMovBuf(MoveBuf& mov)
{
    mov.orientation = {1,0,0,0};
    mov.pos = {0,0,3};
    mov.angVel = {0,0,0};
    mov.linVel = {0,0,0};
    mov.angAcc = {0,0,0};
    mov.linAcc = {0,0,0};
}

void applyMouseInput(MoveBuf& mov, float dx, float dy, float sensitivity)
{
    // treat mouse deltas as angular velocity impulses in local space
    mov.angVel.y += dx * sensitivity; // yaw
    mov.angVel.x += dy * sensitivity; // pitch
}

void applyMouseInputDirect(MoveBuf& mov, float dx, float dy, float sensitivity)
{
    // scale deltas to radians
    float yaw   = dx * sensitivity;
    float pitch = dy * sensitivity;

    // local axes in ship space
    Vec3 up    = {0,1,0};
    Vec3 right = {1,0,0};

    // build incremental quaternions
    Quaternion qYaw   = quatFromAxisAngle(up.x,    up.y,    up.z,    yaw);
    Quaternion qPitch = quatFromAxisAngle(right.x, right.y, right.z, pitch);

    // apply them to orientation
    mov.orientation = quatMultiply(mov.orientation, qYaw);
    mov.orientation = quatMultiply(mov.orientation, qPitch);
    mov.orientation = quatNormalize(mov.orientation);
}


void centerAndScale(WireframeModel& model, float targetSize)
{
    if (model.verts.empty()) return;

    // Step 1: bounding box
    float minX = model.verts[0].x, maxX = model.verts[0].x;
    float minY = model.verts[0].y, maxY = model.verts[0].y;
    float minZ = model.verts[0].z, maxZ = model.verts[0].z;

    for (auto& v : model.verts) {
        if (v.x < minX) minX = v.x;
        if (v.x > maxX) maxX = v.x;
        if (v.y < minY) minY = v.y;
        if (v.y > maxY) maxY = v.y;
        if (v.z < minZ) minZ = v.z;
        if (v.z > maxZ) maxZ = v.z;
    }

    // Step 2: center
    float cx = (minX + maxX) * 0.5f;
    float cy = (minY + maxY) * 0.5f;
    float cz = (minZ + maxZ) * 0.5f;

    // Step 3: subtract center
    for (auto& v : model.verts) {
        v.x -= cx;
        v.y -= cy;
        v.z -= cz;
    }

    // Step 4: largest extent
    float extentX = maxX - minX;
    float extentY = maxY - minY;
    float extentZ = maxZ - minZ;
    float maxExtent = std::max({extentX, extentY, extentZ});

    if (maxExtent < 1e-6f) return; // avoid div by zero

    // Step 5: scale factor
    float scale = targetSize / maxExtent;

    // Step 6: apply scale
    for (auto& v : model.verts) {
        v.x *= scale;
        v.y *= scale;
        v.z *= scale;
    }
}

// -----------------------------
// Transform: rotate in ship space + translate in ship space + project
// -----------------------------
void transformModel(ModelBuf* buf, const MoveBuf& mov)
{
    // Convert orientation quaternion to rotation matrix
    float R[3][3];
    quatToMatrix(mov.orientation, R);

    for (int i=0; i<buf->model->vertCount; i++) {
        // Original model-space vertex
        Vec3 v = buf->model->verts[i];

        // Rotate vertex by ship orientation
        Vec3 vr = rotateVector(mov.orientation, v);

        // Translate by ship’s world position
        vr.x += mov.pos.x;
        vr.y += mov.pos.y;
        vr.z += mov.pos.z;

        buf->vertbuf[i].pos3D = vr;

        // --- Project to 2D ---
        float denom = vr.z + 4.0f;
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
    Vec3 u = { b.x - a.x, b.y - a.y, b.z - a.z };
    Vec3 v = { c.x - a.x, c.y - a.y, c.z - a.z };
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
