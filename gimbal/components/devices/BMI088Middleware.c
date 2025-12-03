/**/
/*==============================================================================
文件作用：bmi088传感器底层文件
文件注释：此文件为bmi088传感器的最底层文件，基本上就是将设置高地电平和数据读取进行再封装
  ==============================================================================*/

#include "BMI088Middleware.h"
#include "spi.h"
extern SPI_HandleTypeDef hspi1;
/*  
函数作用：封装延时函数，读写过程中需延时，具体原因见bmi088数据手册
参数：ms: 延时时间，单位ms
*/
void BMI088_delay_ms(uint16_t ms)
{
    osDelay(ms);
}
/*  
函数作用：封装延时函数，读写过程中需延时，具体原因见bmi088数据手册
参数：us: 延时时间，单位us
*/
void BMI088_delay_us(uint16_t us)
{
    delay_us(us);
}
/*  
函数作用：片选加速度计
*/
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
/*  
函数作用：释放片选加速度计
*/
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}
/*  
函数作用：片选陀螺仪
*/
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
/*  
函数作用：释放片选陀螺仪
*/
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}
/*  
函数作用：spi交换一字节数据，spi传输数据时无论发送接收均以交换数据的方式执行
参数：txdata: 发送数据
返回值：接收数据
*/
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
