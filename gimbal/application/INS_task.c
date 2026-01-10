/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
	
	/*****************************注意！！！*****************************/
	/*2025赛季改用王洪玺开源的扩展卡尔曼滤波
	1.温漂较小
	2.解算的角度更准确，让视觉建立的模型更准确
  3.保留了大疆源码中用中断获取数据
	4.校准为每次上电校准3到4秒，校准时云台需保持静止
  此处的校准用WHX开源的校准，在BMI088driver文件中。
  5.整个获取IMU角速度和加速度的过程比较复杂，涉及SPI_DMA和中断等，建议刚入手的不要深究
  */
/*****************************注意！！！*****************************/
  
//系统层
#include "main.h"
#include "cmsis_os.h"
//外设层
#include "can.h"
#include "iwdg.h"
//应用硬件层
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bsp_adc.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
//数学计算层
#include "pid.h"
#include "ahrs.h"
#include "QuaternionEKF.h"
//任务层
#include "INS_task.h"
#include "gimbal_task.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart6;

//定义三轴角度，加速度和角速度
float IMU_angle_fdb[3] = {0, 0, 0};
float IMU_gyro_fdb[3] = {0, 0, 0};
float IMU_accel_fdb[3] = {0, 0, 0};
uint32_t imu_time = 0;
//定义三轴角度，加速度和角速度

//卡尔曼滤波相关
IMU_Param_t IMU_Param;
//WHX开源的陀螺仪结构体
INS_t INS;
//原始数据和IMU校准相关数据
bmi088_real_data_t bmi088_real_data;

//机体系基向量转换到导航坐标系，本例选取惯性系为导航系
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

//DWT定时器定时时间，获取任务运行时间做积分
uint32_t INS_DWT_Count = 0;
 float dt = 0, t = 0;

//获取任务句柄唤醒任务
static TaskHandle_t INS_task_local_handler;

//新卡尔曼滤波相关
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定



//控制bmi088的温度
static void imu_temp_control(fp32 temp);

//根据imu_update_flag的值开启SPI DMA
static void imu_cmd_spi_dma(void);

//将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);



uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};


//*********中断和SPI_DMA获取数据相关标志位**********//
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;
//*********中断和SPI_DMA获取数据相关标志位**********//

//温度控制pid相关
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;



void INS_Init(void)
{
    IMU_Param.scale[0] = 1;
    IMU_Param.scale[1] = 1;
    IMU_Param.scale[2] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    INS.AccelLPF = 0.0085;

}
//把这个函数放到陀螺仪任务中
void WHX_INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {

        INS.Accel[0] = bmi088_real_data.accel[0];
        INS.Accel[1] = bmi088_real_data.accel[1];
        INS.Accel[2] = bmi088_real_data.accel[2];
        INS.Gyro[0] = bmi088_real_data.gyro[0];
        INS.Gyro[1] = bmi088_real_data.gyro[1];
        INS.Gyro[2] = bmi088_real_data.gyro[2];

        // demo function,用于修正安装误差,可以不管,本demo暂时没用
//        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

//        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
//        INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
//        INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[0], INS.Gyro[1], INS.Gyro[2], INS.Accel[0], INS.Accel[1], INS.Accel[2], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

        // 获取最终数据
        INS.Yaw   =  QEKF_INS.Yaw;
        INS.Pitch =  QEKF_INS.Pitch;
        INS.Roll  =  QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
    }

    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        //IMU_Temperature_Ctrl();
    }

    if ((count % 1000) == 0)
    {
        // 200hz
    }

    count++;
}
//把这个函数放到陀螺仪任务中



/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

void INS_task(void const *pvParameters)
{
    //wait a time
    //-------------------------------------------
    //IMU和SPI以及SPI DMA初始化
    //-------------------------------------------
    osDelay(INS_TASK_INIT_TIME);
	  INS.Calibrate_flag = 0;
    while(BMI088_init())
    {
        osDelay(100);
    }
		Calibrate_MPU_Offset(&bmi088_real_data);
		INS.Calibrate_flag = 1;
//		 MX_CAN1_Init();
//		 MX_CAN2_Init();
		 INS_Init();

    PID_Init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
		
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf,(uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;
    //-------------------------------------------
    //IMU和SPI以及SPI DMA初始化
    //-------------------------------------------

    while (1)
    {
        //wait spi DMA tansmit done
        //等待SPI DMA传输
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
			
        }
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {   
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
					//在此减去校准的零漂
            bmi088_real_data.gyro[0]=bmi088_real_data.gyro[0]-bmi088_real_data.GyroOffset[0];
						bmi088_real_data.gyro[1]=bmi088_real_data.gyro[1]-bmi088_real_data.GyroOffset[1];
						bmi088_real_data.gyro[2]=bmi088_real_data.gyro[2]-bmi088_real_data.GyroOffset[2];
					//在此减去校准的零漂
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {  
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
            
        }
 
        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {   
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }
	//核心解算函数****************************			
		WHX_INS_Task();
  //核心解算函数****************************

				//*********兼容新版旧版角度和角速度赋值**********//
    IMU_gyro_fdb[PITCH] = bmi088_real_data.gyro[1] * 57.29578f;			
		IMU_gyro_fdb[YAW] = bmi088_real_data.gyro[2]* 57.29578f;
		IMU_angle_fdb[YAW]   = INS.Yaw;
		IMU_angle_fdb[PITCH] = INS.Roll;
		IMU_angle_fdb[ROLL] = INS.Pitch;
		//喂狗
		HAL_IWDG_Refresh(&hiwdg);
		//喂狗
    osDelay(1);
		
    } 
    
}


uint32_t GetIMUPitchAngleFdb(float* angle)
{
	*angle = IMU_angle_fdb[PITCH];
	return imu_time;
}
uint32_t GetIMUYawAngleFdb(float* angle)
{
	*angle = IMU_angle_fdb[YAW];
	return imu_time;
}
uint32_t GetIMUPitchGyroFdb(float* gyro)
{
	*gyro = IMU_gyro_fdb[PITCH];
	return imu_time;
}
uint32_t GetIMUYawGyroFdb(float* gyro)
{
	*gyro = IMU_gyro_fdb[YAW];
	return imu_time;
}
uint32_t GetIMUPitchAccelFdb(float* accel)
{
	*accel = IMU_accel_fdb[PITCH];
	return imu_time;
}
uint32_t GetIMUYawAccelFdb(float* accel)
{
	*accel = IMU_accel_fdb[YAW];
	return imu_time;
}
void Get_Calibrate_flag(uint8_t* flag)
{
*flag = INS.Calibrate_flag;
}
/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
	  static uint8_t first_temperate=0;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid,40.f,temp);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp >40.f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        mag_update_flag |= 1 << IMU_DR_SHFITS;
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        //wake up the task
        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }


}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
int spi_tran = 0;
static void imu_cmd_spi_dma(void)
{
		spi_tran++;
        //开启陀螺仪的DMA传输
        if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
            return;
        }
        //开启加速度计的DMA传输
        if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
            return;
        }   
        if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
            return;
        }
}

int dma2 = 0;
void DMA2_Stream2_IRQHandler(void) //定义DMA2 Stream2的中断处理函数
{
    dma2++; //用于统计中断次数

    // 检查DMA接收流是否传输完成
    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        // 清除DMA传输完成标志
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        // 检查陀螺仪数据是否完成读取
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS)) // 如果陀螺仪数据正在通过SPI DMA传输
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS); // 清除正在传输标志
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS); // 设置数据已更新标志

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET); // 关闭陀螺仪的片选信号，结束数据传输
        }

        // 检查加速度计数据是否完成读取
        if(accel_update_flag & (1 << IMU_SPI_SHFITS)) // 如果加速度计数据正在通过SPI DMA传输
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS); // 清除正在传输标志
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS); // 设置数据已更新标志

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET); // 关闭加速度计的片选信号，结束数据传输
        }

        // 检查温度数据是否完成读取
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)) // 如果温度数据正在通过SPI DMA传输
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS); // 清除正在传输标志
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS); // 设置数据已更新标志

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET); // 关闭加速度计的片选信号，结束数据传输
        }

        imu_cmd_spi_dma(); // 根据需要开启新的SPI DMA传输

        // 如果陀螺仪数据已更新，生成软件中断请求
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0); // 生成软件中断请求，以通知其他任务数据已准备好
        }
    }
}
