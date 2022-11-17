#ifndef UTILS_H
#define UTILS_H
#include "raylib.h"

#if defined(__cplusplus)
extern "C" {
#endif

void Init_Camera(Camera *camera);
void Update_Camera(Camera *camera);

#if defined(__cplusplus)
}
#endif

#endif // UTILS_H
