#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 跨模块共享状态 */
extern volatile int s_mode;
extern volatile int s_desired_mode;
extern volatile int s_angles[6];
extern float s_model[6];
extern int s_mirror;
extern volatile bool s_line_active;
extern volatile uint32_t s_line_gen;
extern volatile int s_line_err;
extern float s_line_from[3];
extern float s_line_to[3];
extern float s_line_pitch;
extern bool s_line_auto;
extern int s_line_ms;
extern float s_cart_xyz[3];
extern bool s_cart_valid;

void apply_mirror(int a[6]);
void apply_mirror_f(float f[6]);
void mirror_save(void);
void mirror_load(void);
void pose_send(const int logic[6], int ms);
void pose_send_f(const float logic[6], int ms);
void cmd_pose(const int a_in[6], int ms);
void line_task(void *arg);
