
#include "dcmi.h"

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#include "stereo_utils.h"
#include "camera_type.h"

void camera_init(void)
{
  // Reset the camera's
  camera_reset_init();
  camera_reset();
  // Make a 21MHz clock signal to the camera's
  camera_clock_init();
  // Stop resetting the camera (pin high)

  camera_unreset();
  // Initialize all camera GPIO and I2C pins
  camera_dcmi_bus_init();
  camera_control_bus_init();
  // Start listening to DCMI frames
  camera_dcmi_init();
  // Start DCMI interrupts (interrupts on frame ready)
  camera_dcmi_it_init();
  camera_dcmi_dma_enable();

  // Start DMA image transfer interrupts (interrupts on buffer full)
  camera_dma_it_init();
  Delay(0x07FFFF);

  camera_unreset();
  // Wait for at least 2000 clock cycles after reset
  Delay(CAMERA_CHIP_UNRESET_TIMING);
  // Communicate with camera, setup image type and start streaming
  camera_chip_config();
}

void camera_reset_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //////////////////////////////////////////
  // Reset Line on PD2
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Configure PD2 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

}

// Set Camera Reset pin LOW = Reset mode
void camera_reset(void)
{
  GPIO_ResetBits(GPIOD, GPIO_Pin_2);
}

// Set Camera Reset pin High
void camera_unreset(void)
{
  // Wait for at least 100 clock cycles
  Delay(0x07FFFF);

  GPIO_SetBits(GPIOD, GPIO_Pin_2);

}

void camera_clock_init(void)
{
  //////////////////////////////////////////
  // Make a clock signal on PA7
  // 21MHz: ABP1 42MHz 50% Duty Cycle counter to 2

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  /* TIM config */
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Set up alternate function on pin A7 to TIM3
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

  /*
   * timer_tick_frequency = Timer_default_frequency / (prescaler_set + 1)
   * frequency = timer_tick_frequency / (TIM_Period + 1)
   *
   * With the settings below, the resultant clock will be 21 MHz
   * for the TCM8230 has a nominal clock frequency of 25.54Mhz
   * Therefore this setting will result in a clock speed of max 25.67Hz
   * 30*(21/25.54) = 25.67
   *
   */

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = 1;
  TIM_TimeBaseStructure.TIM_Prescaler = 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3,  &TIM_TimeBaseStructure);


  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3,  ENABLE);

  // todo: needed?
  TIM_CtrlPWMOutputs(TIM3, ENABLE);
  TIM_Cmd(TIM3,  ENABLE);
}

void camera_dcmi_bus_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable DCMI GPIOs clocks
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

  // Connect DCMI pins to AF
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI); //DCMI_HSYNC
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_PIXCL

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_D5
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI); //DCMI_VSYNC
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_DCMI); //DCMI_D6
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_DCMI); //DCMI_D7

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_D0
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI); //DCMI_D1
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_DCMI); //DCMI_D2
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_DCMI); //DCMI_D3
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_DCMI); //DCMI_D4
#ifdef DCMI_TEN_BITS
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_DCMI); //DCMI_D9
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_DCMI); //DCMI_D8
#endif

  // Setup Electrical Pin characteristics: GPIO configuration
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_11
#ifdef DCMI_TEN_BITS
                                | GPIO_Pin_10 | GPIO_Pin_12
#endif
                                ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

#define DCMI_DR_ADDRESS       0x50050028

uint8_t dcmi_image_buffer_8bit_1[FULL_IMAGE_SIZE];
#ifdef DCMI_DOUBLE_BUFFER
uint8_t dcmi_image_buffer_8bit_2[FULL_IMAGE_SIZE];
#endif


void camera_snapshot(void)
{
  DCMI_CaptureCmd(ENABLE);
}

void camera_dcmi_init(void)
{
  // TODO: implement frame counter
  // reset_frame_counter();

  DCMI_InitTypeDef DCMI_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  // Enable DCMI clock
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);

  camera_crop(0);

  // DCMI configuration
#ifdef CAPTURE_MODE_SNAPSHOT
  DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot;
#else
  DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_Continuous;
#endif
  DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
  DCMI_InitStructure.DCMI_PCKPolarity = DCMI_CLOCK_POLARITY;
  DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_Low;
  DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
  DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
#ifdef DCMI_TEN_BITS
  DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_10b;
#else
  DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
#endif

  // Configures the DMA2 to transfer Data from DCMI
  // Enable DMA2 clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  // DMA2 Stream1 Configuration
  DMA_DeInit(DMA2_Stream1);

  DMA_InitStructure.DMA_Channel = DMA_Channel_1;
  DMA_InitStructure.DMA_PeripheralBaseAddr = DCMI_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) dcmi_image_buffer_8bit_1;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = FULL_IMAGE_SIZE / 4; // buffer size in data unit (word)
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

#ifdef DCMI_DOUBLE_BUFFER
  // 128 x 96, double buffer mode possible: for larger images comment out:
  DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t) dcmi_image_buffer_8bit_2, DMA_Memory_0);
  DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);
#endif

  // DCMI configuration
  DCMI_Init(&DCMI_InitStructure);

  // DMA2 IRQ channel Configuration
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);

}

void camera_crop(uint16_t offset)
{
  DCMI_CROPInitTypeDef DCMI_CROPInitStructure;
  DCMI_CROPInitStructure.DCMI_VerticalLineCount = IMAGE_HEIGHT - 1;
  DCMI_CROPInitStructure.DCMI_HorizontalOffsetCount = 0;
  DCMI_CROPInitStructure.DCMI_VerticalStartLine = offset;
  DCMI_CROPInitStructure.DCMI_CaptureCount = IMAGE_WIDTH * 2 - 1;   // In Pixel-clock (not bytes)
  DCMI_CROPConfig(&DCMI_CROPInitStructure);
  DCMI_CROPCmd(ENABLE);
}

void camera_dcmi_dma_enable(void)
{
  /* Enable DMA2 stream 1 and DCMI interface then start image capture */
  DMA_Cmd(DMA2_Stream1, ENABLE);
  DCMI_Cmd(ENABLE);
  DCMI_CaptureCmd(ENABLE);
  // dma_it_init();
}

void camera_dcmi_dma_disable(void)
{
  /* Disable DMA2 stream 1 and DCMI interface then stop image capture */
  DMA_Cmd(DMA2_Stream1, DISABLE);
  DCMI_Cmd(DISABLE);
  DCMI_CaptureCmd(DISABLE);
}

void camera_dcmi_it_init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the DCMI global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  DCMI_ITConfig(DCMI_IT_VSYNC, ENABLE);
}

volatile int frame_counter = 0;

void dcmi_isr(void)
{
  if (DCMI_GetITStatus(DCMI_IT_VSYNC) != RESET) {
    DCMI_ClearITPendingBit(DCMI_IT_VSYNC);

    //frame_counter++;  /////////////////

    /*if ( frame_counter % 30 == 0 )
      camera_crop( 60);

    if ( frame_counter % 60 == 0 )
      camera_crop( 0);*/

    //camera_crop( 0);
  }

  return;
}

void camera_dma_it_init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the DMA global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //DMA_ITConfig(DMA2_Stream1, DMA_IT_HT, ENABLE); // half transfer interrupt
  DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE); // transfer complete interrupt
}

uint8_t *current_image_buffer = dcmi_image_buffer_8bit_1;

void dma2_stream1_isr(void)
{
  // Transfer Complete
  if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET) {
    DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);

    frame_counter++;
#ifdef NO_WAIT_FOR_FRAME
    camera_snapshot();
#endif
  }

  /* We do not use the half transfer interrupt!!

  // Half Transfer
  if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_HTIF1) != RESET) {
    DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_HTIF1);
  }
  */

  // Get the currently used buffer
#ifdef DCMI_DOUBLE_BUFFER
  // 128 x 96: comment out for larger images:
  if (DMA_GetCurrentMemoryTarget(DMA2_Stream1) == DMA_Memory_0) {
    current_image_buffer = dcmi_image_buffer_8bit_2;
  } else {
    current_image_buffer = dcmi_image_buffer_8bit_1;
  }
#else
  current_image_buffer = dcmi_image_buffer_8bit_1;
#endif

}