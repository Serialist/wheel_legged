/*==============================================================================
文件作用：用单片机的标准电压comparator模块ADC采集电压，并计算电压值。
文件注释：
要采样的电压值两个，1个是电池电压，这个在大疆的例程中作为学习的例子，但工程中没有实际应用价值
另一个电压转换为单片机内部温度数据，2024赛季的IMU温度控制数据是用此处的温度数据，2025赛季用的是BMI088返回的
温度数据，具体这两者的差别我也不记得了

  ==============================================================================*/
#include "bsp_adc.h"
#include "adc.h"
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;


volatile fp32 voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;
/**
  * @brief   获取指定ADC通道的转换值
  * @param   ADCx: ADC外设句柄指针 (例: &hadc1)
  * @param   ch: 目标ADC通道号 (取值范围: ADC_CHANNEL_0~ADC_CHANNEL_15)
  * @retval  ADC转换数值 (12位分辨率，范围: 0x0000~0x0FFF)
  * @说明  
  * 1. 配置单通道单次转换模式：通道号|单次转换|3时钟周期采样时间
  * 2. 启动ADC转换并同步等待结果（超时10ms）
  * 3. 每次调用均会重新初始化通道配置，不适用于多通道轮询场景
  * 4. 快速采样模式：3时钟周期适合<1MHz信号源（根据ADC时钟频率计算）
  * 5. 错误处理：当HAL库返回异常时，触发系统错误处理函数
  */
static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
    static ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;//ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(ADCx);

    HAL_ADC_PollForConversion(ADCx, 10);
    return (uint16_t)HAL_ADC_GetValue(ADCx);

}
/**
  * @brief   初始化内部参考电压校准系数
  * @说明  
  * 1. 功能定位：通过ADC采样内部基准电压（VREFINT），计算电压换算系数
  * 2. 实现原理：
  *    - 对VREFINT通道进行200次采样取均值（消除噪声）
  *    - 已知VREFINT理论值为1.2V，计算实际ADC值->电压的换算比例
  *    - 公式推导：voltage = adc_raw * (200*1.2V)/total_adc
  * 3. 典型应用：
  *    - 用于后续ADC测量时的电压校准（如电池电压检测）
  *    - 需在系统初始化阶段调用，确保校准系数生效
  * 4. 硬件依赖：
  *    - 要求芯片内置VREFINT功能（STM32全系支持）
  *    - ADC1外设已正确初始化
  * 5. 误差因素：
  *    - ADC参考电压波动会影响校准精度
  *    - 采样次数不足可能导致均值偏差
  */
void init_vrefint_reciprocal(void)
{
    uint8_t i = 0;
    uint32_t total_adc = 0;
    for(i = 0; i < 200; i++)
    {
        total_adc += adcx_get_chx_value(&hadc1, ADC_CHANNEL_VREFINT);
    }

    voltage_vrefint_proportion = 200 * 1.2f / total_adc;

}
fp32 temperate;
/**
函数作用: 获取单片机内部温度传感器的温度值
参数：无
返回值：温度值，单位摄氏度
 */
fp32 get_temprate(void)
{
    uint16_t adcx = 0;
    

    adcx = adcx_get_chx_value(&hadc1, ADC_CHANNEL_TEMPSENSOR);
    temperate = (fp32)adcx * voltage_vrefint_proportion;
    temperate = (temperate - 0.76f) * 400.0f + 25.0f;

    return temperate;
}

/**
函数作用：获取电池电压值
参数：无
返回值：电池电压值
 */
fp32 get_battery_voltage(void)
{
    fp32 voltage;
    uint16_t adcx = 0;

    adcx = adcx_get_chx_value(&hadc3, ADC_CHANNEL_8);
    voltage =  (fp32)adcx * voltage_vrefint_proportion * 10.090909090909090909090909090909f;

    return voltage;
}

