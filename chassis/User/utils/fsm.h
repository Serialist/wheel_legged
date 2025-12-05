/************************************************
 * @file fsm.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-10-07
 *
 * @copyright Copyright (c) VGD Serialist 2025
 *
 ********************************/

/**************************************************************** include ****************************************************************/

#include "user_lib.h"

/**************************************************************** define ****************************************************************/

#define FSM_DELAY_MS(ms) (osDelay(ms))
#define FSM_DELAY_OSTICK(tick) (vsRtosDelay(tick))

#define FSM_NEXT_STATUS_SET(fsm)

/**************************************************************** struct ****************************************************************/

typedef void (*FSM_Status_Procedure_t)(void);

struct FSM_Status
{
    FSM_Status_Procedure_t Procedure;
    struct FSM_Status *next;
};

struct FSM_Core
{
    struct FSM_Status *root;
    struct FSM_Status *now;
};

/**************************************************************** extren ****************************************************************/

/**************************************************************** proto ****************************************************************/

/************************************************ fsm ************************************************/

void FSM_Init(struct FSM_Core *fsm, struct FSM_Status *root_fsmStatus);

bool FSM_Execution(struct FSM_Core *fsm);

/************************************************ status ************************************************/

bool FSM_Status_Init(struct FSM_Status *status, FSM_Status_Procedure_t Procedure);
