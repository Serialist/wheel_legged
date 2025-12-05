/************************
 * @file register.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-11-28
 *
 * @copyright Copyright (c) VGD 2025
 *
 ************************/

#include "user_lib.h"

#define REG_MAX_LENGTH 16

#define Reg_Execute(reg, index) (reg.Execute(reg, index))

struct Reg
{
    void *value[REG_MAX_LENGTH];
    uint8_t length;
    uint8_t index;
    bool (*Execute)(struct Reg, uint8_t index);
};

bool Register_Init(struct Reg *reg, bool (*Execute)(struct Reg, uint8_t index));

bool Register_Add(struct Reg *reg, void *value);
