/*3D预览图视角*/
#include <math.h>
#include "camera.h"

#define SENSITIVITY                      0.01f
#define CameraMoveExponential            0.8f

float yaw, pit, dist;
Vector2 mousePosPre;
const char moveControl[4] = { 'W', 'S', 'D', 'A' };
typedef enum {
    MOVE_UP = 0,
    MOVE_DOWN,
    MOVE_RIGHT,
    MOVE_LEFT,
} KeyMoves;

void Init_Camera(Camera *camera)
{
    Vector2 vec;
    camera->position = (Vector3){ 0.0f, -20.0f, 20.0f };
    camera->target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera->up = (Vector3){ 0.0f, 0.0f, 1.0f };
    camera->fovy = 45.0f;
    vec.x = camera->target.x - camera->position.x;
    vec.y = camera->target.y - camera->position.y;
    yaw = atan2f(vec.y, vec.x);
    vec.x = sqrtf(vec.x*vec.x + vec.y*vec.y);
    vec.y = camera->target.z - camera->position.z;
    pit = atan2f(vec.y, vec.x);
    dist = sqrtf(vec.x*vec.x + vec.y*vec.y);
}

void Update_Camera(Camera *camera)
{
    Vector2 mousePosNew, mousePosDelta;
    float mouseWheelMove = GetMouseWheelMove();
    char direction[2] = {
        IsKeyDown(moveControl[MOVE_RIGHT]) - IsKeyDown(moveControl[MOVE_LEFT]),
        IsKeyDown(moveControl[MOVE_UP])    - IsKeyDown(moveControl[MOVE_DOWN]),
    };
    dist *= pow(CameraMoveExponential, mouseWheelMove);
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        mousePosPre = GetMousePosition();
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) return;
    mousePosNew = GetMousePosition();
    mousePosDelta.x = mousePosNew.x - mousePosPre.x;
    mousePosDelta.y = mousePosNew.y - mousePosPre.y;
    mousePosPre = mousePosNew;
    yaw += -SENSITIVITY * mousePosDelta.x;
    pit += -SENSITIVITY * mousePosDelta.y;
    Vector3 vec = (Vector3){cosf(pit)*cosf(yaw), cosf(pit)*sinf(yaw), sinf(pit)};
    vec = Vector3Scale(vec, dist);
    camera->position = Vector3Subtract(camera->target, vec);
}
