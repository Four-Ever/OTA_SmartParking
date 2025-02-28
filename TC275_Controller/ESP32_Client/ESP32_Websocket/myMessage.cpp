#include "myMessage.h"

pthread_mutex_t lock[MESSAGE_NUM]; 

Message msg;
Flag flag ={0};
Nano_Message nano_msg;
Nano_Flag nano_flag ={0};