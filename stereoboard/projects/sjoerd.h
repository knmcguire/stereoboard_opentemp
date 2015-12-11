/*
 * project parameters
 *
 *  This file contains the board configuration defaults for this project
 *
 */

#ifndef PROJECT_HEADER_H_
#define PROJECT_HEADER_H_

/*****************
 * Project parameters
 *****************/
#define DEFAULT_BOARD_FUNCTION SEND_IMAGE
#define CAMERA_CPLD_STEREO camera_cpld_stereo_pixmux
#define CAPTURE_MODE_SNAPSHOT 1   // snapshot! Mostly for debugging

//////////////////////////////////////////////////////
// Settings
#define MAX_RATIO 10 // 10
#define STEREO_ALGORITHM 0 // 1 = Dense   0 = Sparse
#define LARGE_IMAGE




#endif /* PROJECT_HEADER_H_ */