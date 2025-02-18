#ifndef DRIVER_STM
#define DRIVER_STM

/***********************************************************************/
/*Include*/ 
/***********************************************************************/
#include "Ifx_Types.h"
#include "IfxStm.h"
#include "IfxCpu_Irq.h"


/***********************************************************************/
/*Typedef*/ 
/***********************************************************************/
typedef struct
{
    uint8 u8nuScheduling1msFlag;
    uint8 u8nuScheduling10msFlag;
    uint8 u8nuScheduling50msFlag;
    uint8 u8nuScheduling100msFlag;
    uint8 u8nuScheduling500msFlag;
    uint8 u8nuScheduling1000msFlag;
    uint8 u8nuScheduling5000msFlag;
}SchedulingFlag;

/***********************************************************************/
/*Define*/ 
/***********************************************************************/

/***********************************************************************/
/*External Variable*/ 
/***********************************************************************/
extern SchedulingFlag stSchedulingInfo;

/***********************************************************************/
/*Global Function Prototype*/ 
/***********************************************************************/
extern void Driver_Stm_Init(void);


#endif /* DRIVER_STM */
