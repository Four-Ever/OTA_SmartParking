#ifndef BSW_IO_TOF_H_
#define BSW_IO_TOF_H_

#define NUM_TOF 2
#define OFFSET -30

typedef enum
{
    TOF0,
    TOF1
}ToFId;



void Init_ToF(void);
void IsrUart1RxHandler_tof(void);
void ToF_get_All_Distance(void);


extern int Distance[NUM_TOF];
#endif /* BSW_IO_TOF_H_ */
