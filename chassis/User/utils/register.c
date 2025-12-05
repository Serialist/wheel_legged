/************************
 * @file register.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-11-28
 *
 * @copyright Copyright (c) VGD 2025
 *
 ************************/

#include "register.h"

/************************
 * @brief 初始化注册器
 *
 * @param reg
 * @param Execute
 * @return true
 * @return false
 ************************/
bool Register_Init(struct Reg *reg, bool (*Execute)(struct Reg, uint8_t index))
{
    if (reg == NULL || Execute == NULL)
    {
        return false;
    }

    reg->Execute = Execute;
    reg->length = 0;

    return true;
}

/************************
 * @brief 在注册器中添加值
 *
 * @param reg
 * @param value
 * @return true
 * @return false
 ************************/
bool Register_Add(struct Reg *reg, void *value)
{
    if (reg == NULL || reg->length >= REG_MAX_LENGTH)
    {
        return false;
    }

    reg->value[reg->length] = value;
    reg->length++;

    return true;
}
