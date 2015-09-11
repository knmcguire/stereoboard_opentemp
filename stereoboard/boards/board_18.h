#define BOARD_NUMBER	18

//#define CAMERA_CPLD_STEREO camera_cpld_stereo_left
//#define CAMERA_CPLD_STEREO camera_cpld_stereo_right
#define CAMERA_CPLD_STEREO camera_cpld_stereo_pixmux
//#define CAMERA_CPLD_STEREO camera_cpld_stereo_linemux
#define DISPARITY_OFFSET_LEFT  3
#define DISPARITY_OFFSET_RIGHT  3
#define DISPARITY_BORDER  96		// width threshold for using offset left and right
#define DISPARITY_OFFSET_HORIZONTAL -4
#define RESOLUTION_FACTOR 6
#define CAPTURE_MODE_SNAPSHOT 1   // snapshot! Mostly for debugging

