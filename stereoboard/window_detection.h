/*
 * window_detection.h
 *
 *  Created on: Sep 9, 2013
 *      Author: mavlab
 */

#ifndef WINDOW_DETECTION_H_
#define WINDOW_DETECTION_H_

#include <arm_math.h>

typedef enum {MODE_DISPARITY, MODE_ILLUMINANCE, MODE_FILTER} MODE;

void window_init(void);
uint16_t detect_window_sizes(uint8_t *in, uint32_t image_width, uint32_t image_height, uint8_t *coordinate,
                             uint8_t *window_size,
                             uint32_t *integral_image, uint8_t MODE, uint8_t disparity_max);
uint16_t detect_window(uint8_t *in, uint32_t image_width, uint32_t image_height, uint8_t *coordinate,
                       uint8_t determine_size, uint16_t *size, uint8_t calculate_integral_image, uint32_t *integral_image, uint8_t MODE,
                       uint8_t disparity_max);
uint16_t detect_escape(uint8_t *in, uint32_t image_width, uint32_t image_height, uint16_t *escape_coordinate,
                       uint32_t *integral_image, uint8_t n_cells);
uint16_t get_window_response(uint16_t x, uint16_t y, uint16_t feature_size, uint16_t border, uint32_t *integral_image,
                             uint16_t image_width, uint16_t image_height, uint16_t px_inner, uint16_t px_border);
uint16_t get_border_response(uint16_t x, uint16_t y, uint16_t feature_size, uint16_t window_size, uint16_t border_size,
                             uint32_t *integral_image, uint16_t image_width, uint16_t image_height, uint16_t px_inner, uint16_t px_outer);
void filter_bad_pixels(uint8_t *in, uint32_t image_width, uint32_t image_height);
void transform_illuminance_image(uint8_t *in, uint8_t *out, uint32_t image_width, uint32_t image_height, uint8_t n_bits,
                                 uint8_t bright_win);

#endif /* WINDOW_DETECTION_H_ */
