

#include "stm32f4xx_conf.h"

#include "led.h"

int led_state = 0;
void led_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOB Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* Configure PB12 in output push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void led_set(void)
{
  led_state = 1;
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

void led_clear(void)
{
  led_state = 0;
  GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

void led_toggle(void)
{
  if (led_state) {
    led_clear();
  } else {
    led_set();
  }
}
