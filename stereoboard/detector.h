/*
 * detector.h
 *
 *  Created on: Oct 29, 2014
 *      Author: Kevin
 */

#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <arm_math.h>

void detect_roof(uint8_t *in, uint8_t *out, uint32_t image_width, uint32_t image_height);
void detect_objects(uint8_t *in, uint8_t *out, uint32_t image_width, uint32_t image_height);


#endif