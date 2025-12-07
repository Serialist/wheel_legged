/************************
 * @file fsm.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-10-07
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

#ifndef __FSM_H
#define __FSM_H

/* ================================================================ import ================================================================*/

#include "user_lib.h"

/* ================================================================ macro ================================================================*/

/* ================================================================ typedef ================================================================*/

struct FSM_State
{
    void (*Procedure)(void);
    struct FSM_State *next;
};

struct FSM_Core
{
    struct FSM_State *root;
    struct FSM_State *now;
};

struct FSM_Transition
{
    void (*Condition)(void);
    struct FSM_State *next;
};

/* ================================================================ variable ================================================================*/

/* ================================================================ prototype ================================================================*/

/* ================================================================ function ================================================================*/

bool FSM_Init(struct FSM_Core *fsm, struct FSM_State *root_fsmState);

bool FSM_Execution(struct FSM_Core *fsm);

bool FSM_State_Init(struct FSM_State *state, void (*Procedure)(void));

#endif
