#ifndef _LIN_H_
#define _LIN_H_

#include "main.h"

#define BOARD_180W_ID  0X23

extern int global_target_speed;
extern int global_actual_speed;

void Lin_SendHead(uint8_t id);
void Lin_SendAnswer(uint8_t id ,uint8_t data[]);
void Lin_DataProcess(void);

#endif