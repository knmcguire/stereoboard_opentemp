/*
 * main_parameters.h
 *
 *  Created on: Sep 18, 2013
 *      Author: mavlab
 */

#ifndef MAIN_PARAMETERS_H_
#define MAIN_PARAMETERS_H_

/*****************
 * MAIN PARAMETERS
 *****************/

// uncomment for communication with the microcrontroller, this is for sending images:


#define SEND_COMMANDS 0     //commands to ppz
#define SEND_COMMANDS_HUMAN 0 //same commands, but readable for humans in a terminal
#define SEND_TIMING 0     //send timing information (in human readable format)
#define SEND_IMAGE_STEREO 1   // to send the raw stereo image
#define SEND_IMAGE_COLOR 0    // to send the raw UYVY color image
#define SEND_FILTER 0     // to send the processed color image result (UYUY filtered color image)
#define SEND_DISPARITY_MAP 0  // to send the processed stereo image result (grayscale disparity map)

#if (SEND_IMAGE_STEREO || SEND_IMAGE_COLOR || SEND_DISPARITY_MAP || SEND_FILTER)
#define USART_3000000   // only works for send image. destroys other types of comm.
//standard uart speed is 9600
#endif


#endif /* MAIN_PARAMETERS_H_ */
