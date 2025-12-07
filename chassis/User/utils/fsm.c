/************************
 * @file fsm.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-12-05
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

/* ================================================================ import ================================================================*/

#include "fsm.h"

/* ================================================================ macro ================================================================*/

/* ================================================================ typedef ================================================================*/

/* ================================================================ variable ================================================================*/

/* ================================================================ prototype ================================================================*/

/* ================================================================ function ================================================================*/

/************************
 * @brief 状态机初始化
 *
 * @param fsm
 * @param rootState
 * @return true
 * @return false
 ************************/
bool FSM_Init(struct FSM_Core *fsm, struct FSM_State *rootState)
{
    if (fsm == NULL || rootState == NULL)
    {
        return false;
    }

    fsm->root = rootState;
    fsm->now = rootState;
		
		return true;
}

/************************
 * @brief 状态机执行
 *
 * @param fsm
 * @return true
 * @return false
 ************************/
bool FSM_Execution(struct FSM_Core *fsm)
{
    if (fsm->now == NULL || fsm->now->Procedure == NULL)
    {
        fsm->now = fsm->root;
        return false;
    }

    fsm->now = fsm->now->next; // 切换任务
    fsm->now->Procedure();     // 执行任务

    return true;
}

/************************
 * @brief 状态初始化
 *
 * @param state
 * @param Procedure
 * @return true
 * @return false
 ************************/
bool FSM_State_Init(struct FSM_State *state, void (*Procedure)(void))
{
    if (state == NULL || Procedure == NULL)
    {
        return false;
    }

    state->Procedure = Procedure;
    state->next = state;

    return true;
}
