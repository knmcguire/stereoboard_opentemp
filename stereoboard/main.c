/**
  ******************************************************************************
  * @file    main.c
  * @author  kevin
  * @version V1.0.0
  * @date    2013
  * @brief   Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "dcmi.h"
#include "cpld.h"
#include "usart.h"
#include "tcm8230.h"
#include "stm32f4xx_conf.h"
#include "jpeg.h"
#include <arm_math.h>

#include "window_detection.h"
#include "filter_color.h"
#include "entropy_patches.h"
#include "usbd_usr.h"

#include "detector.h"

#include "main_parameters.h"

#include "color.h"

#include <stdint.h>



__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

/** @addtogroup StereoCam
  * @{
  */

/* Private function prototypes -----------------------------------------------*/
void Delay(volatile uint32_t nCount);
/* Private functions ---------------------------------------------------------*/

/*
In case of USE_COLOR sends the raw UYVY image over the UART, otherwise the gray scale stereo image
*/
void SendImage(uint8_t *b, uint8_t headerbyte)
{
  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = headerbyte;

  uint16_t width = IMAGE_WIDTH;
  uint16_t height = IMAGE_HEIGHT;

  int j = 0;
  for (j = 0; j < height; j++) {
    code[3] = 0x80;
    while (usart_tx_ringbuffer_push(code, 4) == 0)
      ;
    while (usart_tx_ringbuffer_push(b + width * j * 2, width) == 0)
      ;
    while (usart_tx_ringbuffer_push(b + width * j * 2 + width, width) == 0)
      ;

    code[3] = 0xDA;
    while (usart_tx_ringbuffer_push(code, 4) == 0)
      ;
  }

  code[3] = 0xAB;
  while (usart_tx_ringbuffer_push(code, 4) == 0)
    ;
}

void SendDisparityMap(uint8_t *b)
{
  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = 0x02;

  int j = 0;
  for (j = 0; j < 96; j++) {
    code[3] = 0x80;
    while (usart_tx_ringbuffer_push(code, 4) == 0)
      ;
    while (usart_tx_ringbuffer_push(b + 128 * j, 128) == 0)
      ;

    code[3] = 0xDA;
    while (usart_tx_ringbuffer_push(code, 4) == 0)
      ;
  }

  code[3] = 0xAB;
  while (usart_tx_ringbuffer_push(code, 4) == 0)
    ;
}


void init_timer2()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef TIM_InitStruct;
  TIM_InitStruct.TIM_Prescaler = 42000 - 1;                // This will configure the clock to 2 kHz
  TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;     // Count-up timer mode
  TIM_InitStruct.TIM_Period = 20000 - 1;                    // 10 seconds
  TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;        // Divide clock by 1
  TIM_InitStruct.TIM_RepetitionCounter = 0;                // Set to 0, not used
  TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
  TIM_Cmd(TIM2, ENABLE);
}

/**************
 * MAIN DEFINES
 **************/

uint8_t *filtered_image = ((uint8_t *) 0x10000000); // 0x10000000 - 0x1000 FFFF = CCM data RAM
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*
    At this stage the microcontroller clock setting is already configured,
    this is done through SystemInit() function which is called from startup
    file (startup_stm32f4xx.s) before to branch to application main.
    To reconfigure the default setting of SystemInit() function, refer to
    system_stm32f4xx.c file
  */


  /****************
   * INITIALIZATION
   ****************/

  // Initialize the LED
  led_init();
  led_set();
  // Initialize the serial communication (before the camera so we can print status)
  usart_init();
  // Initialize the CPLD
  camera_cpld_stereo_init();


  //camera_cpld_stereo_left();
  camera_cpld_stereo_pixmux();

  //camera_cpld_stereo_right();
  //camera_cpld_stereo_linemux();
  //camera_cpld_stereo_framemux();


  // Reset the camera's
  camera_reset_init();
  camera_reset();
  // Make a 21MHz clock signal to the camera's
  camera_clock_init();
  // Wait for at least 100 clock cycles
  Delay(0x07FFFF);
  // Stop resetting the camera (pin high)
  camera_unreset();
  // Initialize all camera GPIO and I2C pins
  camera_dcmi_bus_init();
  camera_tcm8230_i2c_init();
  // Start listening to DCMI frames
  camera_dcmi_init();
  camera_dcmi_dma_enable();
  // Wait for at least 2000 clock cycles after reset
  Delay(0x07FFFF);
  // Communicate with camera, setup image type and start streaming
  camera_tcm8230_config();
  // Start DMA image transfer interrupts
  camera_dma_it_init();
  // Print welcome message
  char comm_buff[128] = " --- Stereo Camera --- \n\r";
  usart_tx_ringbuffer_push((uint8_t *)&comm_buff, strlen(comm_buff));

  uint8_t disparity_image[FULL_IMAGE_SIZE / 2] = {};


  uint32_t image_width = IMAGE_WIDTH;
  uint32_t image_height = IMAGE_HEIGHT;

  uint32_t start;
  init_timer2();

  /*******************
   * MINOR PARAMETERS:
   *******************/

  //uint8_t modeswitcher = 0; // local alternater switch between stereo and color mode


  /***********
   * MAIN LOOP
   ***********/
  int frame_processed = 0;
  while (1) {

    /* wait for a new frame */
    while (frame_processed == frame_counter);
    ;

#if SEND_TIMING
    uint32_t currenttime = TIM_GetCounter(TIM2);
    print_number(currenttime - start, 1); //print round trip time, 78 = ~30fps = 33ms, 271 = ~ 9.3fps
    start = currenttime;
#endif
    if (cameraMode == COLOR) {
      cameraMode = STEREO;
      detect_roof(current_image_buffer2, filtered_image, image_width, image_height);
      led_clear();
#if(SEND_FILTER)
      SendImage(filtered_image, cameraMode);
#endif
#if (SEND_IMAGE_COLOR)
      SendImage(current_image_buffer2, cameraMode);
#endif
    } else { // stereo vision
      cameraMode = COLOR;
      detect_objects(current_image_buffer1, disparity_image, image_width, image_height);
      led_set();
#if (SEND_DISPARITY_MAP)
      SendDisparityMap(disparity_image);
#endif
#if (SEND_IMAGE_STEREO)
      SendImage(current_image_buffer1, cameraMode);
#endif
    }

    frame_processed = frame_counter; //frame_counter is updated from the cam interrupt

  } // while(1)
} //main

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(volatile uint32_t nCount)
{
  while (nCount--) {
  }
}

