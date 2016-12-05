/*
 * can.h
 *
 * Created on: Dec 4, 2016
 *     Author: jamesliu
 *       Note: the HAL CAN driver is a complete friggin hack job. No respect. Pisses me off.
 */

#ifndef CAN_H_
#define CAN_H_

#include "main.h"

#define CAN_BANKS 14
#define CAN_BUFFER_LENGTH 16 //HAL ain't even using fifo hw properly, so neither shall I.

typedef struct
{
  uint32_t id;
  uint8_t isExt;
  uint8_t isRemote;
  uint8_t dlc;
  uint8_t Data[8];
  int filterNum;
}Can_frame_t;

typedef struct
{
  uint32_t id;
  uint32_t mask;
  uint8_t isRemote;
  uint8_t maskRemote;
  uint8_t isExt;
  uint8_t isMasked;
  int filterNum;
}Can_filter_t;

CAN_FilterConfTypeDef Can_filters[CAN_BANKS];
uint8_t Can_filterCapacity[CAN_BANKS];
uint8_t Can_filterUsage[CAN_BANKS]; //one bit for each slot, msb is msb of reg1
Can_frame_t Can_rxBuffer[CAN_BUFFER_LENGTH];

uint8_t Can_head;
uint8_t Can_tail;
uint8_t Can_ovf;

Can_frame_t *Can_headAddr();
Can_frame_t *Can_tailAddr();

void Can_begin();

int Can_addMaskedFilterStd(uint16_t id, uint16_t mask, int isRemote/*-1 = don't care*/);
int Can_addMaskedFilterExt(uint32_t id, uint32_t mask, int isRemote/*-1 = don't care*/);
int Can_addFilterStd(uint16_t id, uint8_t isRemote);
int Can_addFilterExt(uint32_t id, uint8_t isRemote);
int Can_getFilter(int filterNum);
int Can_removeFilter(int filterNum);
int Can_getFilterNum(uint32_t fmi);

int Can_available();
void Can_read(Can_frame_t *target);
void Can_readNum(Can_frame_t *target, int filterNum);

int Can_availableForTx();
void Can_writeStd(uint16_t id, uint8_t isRemote, uint8_t *data, uint8_t dlc);
void Can_writeExt(uint32_t id, uint8_t isRemote, uint8_t *data, uint8_t dlc);
int Can_resend();

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

#endif /* CAN_H_ */
