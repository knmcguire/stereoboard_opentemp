/*
 * gate_detection.h
 *
 *  Created on: Sep 10, 2016
 *      Author: Guido de Croon
 */

#ifndef STEREOBOARD_GATE_DETECTION_FP_H_
#define STEREOBOARD_GATE_DETECTION_FP_H_

#include "inttypes.h"
#include "data_types.h"

#define GOOD_FIT 0.04
#define BAD_FIT 0.12
#define MAX_POINTS 250  

  void gate_detection_fp(struct image_i* disparity_image, q15_t* x_center, q15_t* y_center, q15_t* radius, q15_t* fitness, int initialize_fit_with_pars, int min_sub_disparity);

	// "private" functions:
	void convert_disparitymap_to_points_fp(struct image_i* disparity_image, int min_sub_disparity);
	void fit_window_to_points_fp(q15_t* x0, q15_t* y0, q15_t* size0, q15_t* fitness);
	q15_t mean_distance_to_circle_fp(q15_t* genome);
  q15_t distance_to_vertical_segment_fp(struct point_i Q1, struct point_i Q2, q15_t x, q15_t y);

  // utility functions: should probably be placed in some other file:
	q15_t get_mutation_fp(void);

  // drawing functions:
  void draw_circle_fp(struct image_i* Im, q15_t x_center, q15_t y_center, q15_t radius, uint8_t* color);
  void draw_stick_fp(struct image_i* Im, q15_t x_center, q15_t y_center, q15_t radius, uint8_t* color);

  // calculating the color fit cannot be done with the current stereo output:
	// float check_color_fit();


#endif /* STEREOBOARD_GATE_DETECTION_FP_H_ */
