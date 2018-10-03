//——————————————————————————————————————————————————————————————————————————————
// Generic CAN Message
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————

#ifndef ACAN_CALL_BACK_ROUTINE_TYPE_DEFINED
#define ACAN_CALL_BACK_ROUTINE_TYPE_DEFINED

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#include <CANMessage.h>

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

typedef void (*ACANCallBackRoutine) (const CANMessage & inMessage) ;

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#endif
