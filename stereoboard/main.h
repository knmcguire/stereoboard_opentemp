
/**
 * @brief This file contains global includes for all parameters computed in the main loop
 */


#ifndef _MAIN_H
#define _MAIN_H

#include "inttypes.h"

// TODO: move somewhere better
struct vec2_t {
  int32_t x;
  int32_t y;
};

struct vec3_t {
  int32_t x;
  int32_t y;
  int32_t z;
};

struct rot_t {
  int32_t phi;
  int32_t theta;
  int32_t psi;
};

struct cam_state_t {
  float phi;
  float theta;
  float psi;
  float alt;
  int32_t us_timestamp;
};


extern uint32_t frame_dt;     // Time between previous camera frame and current frame in ms
extern uint32_t frame_rate;   // Frequency of main loop time averaged over one second

extern struct image_t disparity_image;
extern struct image_t current_image_pair;

extern struct cam_state_t cam_state;


#endif // _MAIN_H
