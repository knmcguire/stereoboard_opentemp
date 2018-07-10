/**
 * Color.c
 * @author: Kirk Scheper
 * @details
 *
 * - Sends color images over serial
 * - Can optionally filter image before sending
 *
 */

#include "edgeFS.h"
#include "edgeflow.h"
#include "main_parameters.h"
#include "main.h"
#include "dcmi.h"
#include "image.h"
#include "raw_digital_video_stream.h"


void init_project(void)
{
	edgeflow_init(IMAGE_WIDTH, IMAGE_HEIGHT, USE_MONOCAM, &cam_state);
}

void run_project(void)
{
    edgeflow_total((uint8_t *)current_image_pair.buf, current_image_pair.pprz_ts);
    send_edgeflow();
}
