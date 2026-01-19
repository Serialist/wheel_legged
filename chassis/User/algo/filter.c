/************************
 * @file filter.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief 自定义的一些滤波器方法，可能是练手用的
 * @version 0.1.0
 * @date 2025-11-29
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

#include "filter.h"

/************************
 * @brief 初始化平均滤波器
 *
 * @param filter
 * @param width
 * @return true
 * @return false
 ************************/
bool Filter_Average_Init(struct Filter_Average *filter, uint8_t width)
{
    if (width > FILTER_AVERAGE_FLOAT_WINDOW_MAX_SIZE || width <= 0)
    {
        return false;
    }

    for (uint8_t i = 0; i < width; i++)
    {
        filter->window[i] = 0;
    }
    filter->width = width;
    filter->index = 0;
    filter->average = 0;

    return true;
}

/************************
 * @brief 更新平均滤波器
 *
 * @param filter
 * @param calibrate
 * @return float
 *
 * @date 2025-11-30
 * @bug
 * 浮点数运算精度问题
 * 发现在数值极大极小有可能爆炸
 * 如果发现出现 inf 或 nan 的情况，请检查数据范围
 * **尤其是第一次轮填充的数值！**
 * 建议在初始化后开updata
 * @todo
 * - 解决在 inf 时出现的爆炸问题
 * - 检查数据范围
 * - 检查window有没有被篡改
 * - 在window在填满前不允许输出
 ************************/
float Filter_Average_Update(struct Filter_Average *filter, float calibrate)
{
    static uint32_t reCalc_cnt = 0; // 定期重新计算计数

    /// @brief 一种更好的方法，但是 bug 在上面，而且如果 window 中被篡改，可能会导致数据偏移
    if (reCalc_cnt % 79u) // 随便选了个数
    {
        filter->average = filter->average + (calibrate - filter->window[filter->index]) / (float)filter->width;
    }
    /// @brief 定期重新计算平均值，防止计算时炸掉
    else
    {
        for (uint8_t i = filter->index; i < (filter->index + filter->width) % filter->width; i++)
        {
            filter->average += filter->window[i] / filter->width;
        }
    }

    /// @brief 循环列表更新
    filter->window[filter->index] = calibrate;
    filter->index = (filter->index + 1) % filter->width;

    reCalc_cnt++;

    return filter->average;
}
