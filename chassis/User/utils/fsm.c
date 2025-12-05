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

/* ================================================================ include ================================================================ */

#include "fsm.h"

/* ================================================================ define ================================================================ */

/* ================================================================ struct ================================================================ */

/* ================================================================ proto ================================================================ */

/* ================================================================ value ================================================================ */

/* ================================================================ function ================================================================ */

/************************************************ fsm ************************************************/

/************************
 * @brief 状态机初始化
 *
 * @param fsm
 * @param root_fsmStatus
 ************************/
void FSM_Init(struct FSM_Core *fsm, struct FSM_Status *root_fsmStatus)
{
    fsm->root = root_fsmStatus;
    fsm->now = root_fsmStatus;
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
    /// @brief 切换任务
    if (fsm->now == NULL)
    {
        fsm->now = fsm->root;
        return false;
    }
    fsm->now = fsm->now->next;

    /// @brief 执行任务
    if (fsm->now->Procedure == NULL)
    {
        fsm->now = fsm->root;
        return false;
    }
    fsm->now->Procedure();

    return true;
}

/************************************************ status ************************************************/

/************************
 * @brief 状态初始化
 *
 * @param status
 * @param Procedure
 * @return true
 * @return false
 ************************/
bool FSM_Status_Init(struct FSM_Status *status, FSM_Status_Procedure_t Procedure)
{
    if (status == NULL || Procedure == NULL)
    {
        return false;
    }

    status->Procedure = Procedure;
    status->next = status;
    return true;
}
