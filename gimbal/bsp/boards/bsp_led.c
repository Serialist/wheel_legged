#include "tim.h"
#include "bsp_led.h"
  /*
  ==============================================================================
文件作用：此文件为LED RGB流光任务的实现文件，主要包含了LED RGB流光的颜色变化和显示函数。
文件说明：2025赛季未使用
  ==============================================================================
  */
extern TIM_HandleTypeDef htim5;
/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
/**
  * @brief          锟斤拷示RGB
  * @param[in]      aRGB:0xaaRRGGBB,'aa' 锟斤拷透锟斤拷锟斤拷,'RR'锟角猴拷色,'GG'锟斤拷锟斤拷色,'BB'锟斤拷锟斤拷色
  * @retval         none
  */
void aRGB_led_show(uint32_t aRGB)
{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}


