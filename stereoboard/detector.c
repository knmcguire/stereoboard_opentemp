/*
 * detector.c
 *
 *  Created on: Oct 29, 2014
 *      Author: Kevin
 */


#include "detector.h"
#include "led.h"
#include "usart.h"
#include "main_parameters.h"
#include "color.h"
#include "stereo_vision.h"
#include <stdio.h>


const uint16_t n_channels = 2;

// Stereo parameters:
const uint32_t disparity_range = 20; // Should be a multiple of 4 for efficiency. At a distance of 1m, disparity is 7-8.
const uint8_t thr1 = 4;
const uint8_t thr2 = 5;

const uint8_t disparity_threshold = 5;
const uint32_t disparities_high = 0;


/* perfect red detector */
// const uint8_t min_U = 100;
// const uint8_t min_V = 141;
// const uint8_t max_U = 150;
// const uint8_t max_V = 170;


/* perfect blue detector */
uint8_t min_U = 130;
uint8_t min_V = 100;
uint8_t max_U = 200;
uint8_t max_V = 140;

uint8_t max_Y = 200;
uint8_t min_Y = 140;


void detect_roof(uint8_t *in, uint8_t *out, uint32_t image_width, uint32_t image_height)
{

  struct img_struct input;
  input.buf = in;
  input.w = image_width;
  input.h = image_height;
  input.seq = 0;

  struct img_struct output;
  output.buf = out;
  output.w = image_width;
  output.h = image_height;
  output.seq = 0;

  uint32_t results[2] = {};

  uint32_t cnt =  colorfilt_uyvy(&input, &output, min_Y, max_Y, min_U, max_U, min_V, max_V, results);


  results[0] = results[0] >> 6;
  results[1] = results[1] >> 6;
  if (results[0] > 63) {
    results[0] = 63;
  }
  if (results[1] > 63) {
    results[1] = 63;
  }

  //uint8_t total = floordisp_cnt + forwarddisp_cnt;




  // cnt= (cnt>>8) + 128;
  // if (cnt > 255) {
  //  cnt = 255;
  // } else if (cnt < 128) {
  //  cnt = 128;
  // }

#if (SEND_COMMANDS==1)

  uint8_t data[8];
  data[0] = 'c';
  data[1] = 'o';
  data[2] = 'l';
  data[3] = results[0];
  data[4] = results[1];
  data[5] = 255;
  data[6] = results[0];
  data[7] = results[1];

  print_string(data, 8);

#endif
#if (SEND_COMMANDS_HUMAN ==1 )
  char str[64];
  int len = sprintf(str, "Left: %u, Right %u, Total %u;\n\r", results[0], results[1], cnt);
  print_string(str, len);
#endif

  // if (cnt < 7) {
  //  led_switch();
  // }
}

void detect_objects(uint8_t *in, uint8_t *out, uint32_t image_width, uint32_t image_height)
{

  stereo_vision(in, out, image_width, image_height, disparity_range, thr1, thr2, 0, 70);

  uint32_t disp_cnt = evaluate_disparities(out, image_width, image_height, disparity_threshold, disparities_high, 30, 70);

  //no objects ~1000-2000, objects ~3000 - 4000
  disp_cnt = disp_cnt >> 6;
  if (disp_cnt > 63) {
    disp_cnt = 63;
  }
  // floordisp_cnt = floordisp_cnt>>6;
  // if (floordisp_cnt >63) {
  //  floordisp_cnt = 63;
  // }
  // floordisp_cnt = floordisp_cnt+64;
  //uint8_t total = floordisp_cnt + forwarddisp_cnt;



#if (SEND_COMMANDS==1)
  uint8_t data[6];
  data[0] = 'd';
  data[1] = 'i';
  data[2] = 's';
  data[3] = disp_cnt;
  data[4] = 255;
  data[5] = disp_cnt;
  print_string(data, 6);
#endif
#if (SEND_COMMANDS_HUMAN ==1 )
  char str[64];
  int len = sprintf(str, "D_forward: %u \n\r", disp_cnt);
  //print_string(str,len);
#endif



}

