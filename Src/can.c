/*
 * can.c
 *
 *  Created on: Dec 4, 2016
 *      Author: jamesliu
 */

#include "can.h"

extern CAN_HandleTypeDef hcan1;

/*
 * This is how HAL handles the filter values:
 *
 * 16 bit filters:
 * FR1 = ((0x0000FFFF & FilterMaskIdLow) << 16) | (0x0000FFFF & FilterIdLow);
 * FR2 = ((0x0000FFFF & FilterMaskIdHigh) << 16) | (0x0000FFFF & FilterIdHigh);
 *
 * 32 bit filters:
 * FR1 = ((0x0000FFFF & FilterIdHigh) << 16) | (0x0000FFFF & FilterIdLow);
 * FR2 = ((0x0000FFFF & FilterMaskIdHigh) << 16) | (0x0000FFFF & FilterMaskIdLow);
 *
 * A mask is also more significant than its corresponding ID, apparently.
 */

uint8_t Can_head = 0, Can_tail = 0, Can_ovf = 0;

Can_frame_t *Can_headAddr(){
	return &Can_rxBuffer[Can_head];
}

Can_frame_t *Can_tailAddr(){
	return &Can_rxBuffer[Can_tail];
}

void Can_begin(){
	HAL_CAN_Receive_IT(&hcan1,0);
}

int Can_addMaskedFilterStd(uint16_t id, uint16_t mask, int isRemote){ //2 slots per bank
	uint8_t rtr = (isRemote==0)?0:1;
	uint8_t rtrm = (isRemote<0)?0:1; //0 = don't care, 1 = must match
	for(int i=0; i<CAN_BANKS; i++){ //add to existing available bank
		uint8_t *usage = &Can_filterUsage[i];
		uint8_t *capacity = &Can_filterCapacity[i];
		if(*capacity==2 && *usage>0 && *usage<0x03 &&
				Can_filters[i].FilterScale==0){ //if in use but unfilled
			if((*usage&0x02) == 0){ //if slot 2 unused, so 3 must be in use
				Can_filters[i].FilterIdLow = id<<5 | rtr<<4 | 0<<3; //id|rtr|ide
				Can_filters[i].FilterMaskIdLow = mask<<5 | rtrm << 4 | 1<<3;
				*usage |= 0x02;
				HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
				return 4*i + 2; //"usage" index = xxxx0123
			}else{ //slot 2 in use, so slot 3 must be empty
				Can_filters[i].FilterIdHigh = id<<5 | rtr<<4 | 0<<3; //id|rtr|ide
				Can_filters[i].FilterMaskIdHigh = mask<<5 | rtrm << 4 | 1<<3;
				*usage |= 0x01;
				HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
				return 4*i + 3; //"usage" index = xxxx0123
			}
		}
	}
	for(int i=0; i<CAN_BANKS; i++){ //open up a fresh bank
		if(Can_filterUsage[i]==0){ //find unused bank
			Can_filterUsage[i] |= 0x02; //use highest slot first
			Can_filterCapacity[i] = 2;
			Can_filters[i].FilterIdLow = id<<5 | rtr<<4 | 0<<3; //id|rtr|ide
			Can_filters[i].FilterMaskIdLow = mask<<5 | rtrm << 4 | 1<<3;
			Can_filters[i].FilterIdHigh = 0; //0 should not be a valid addr
			Can_filters[i].FilterMaskIdHigh = ~0;
			Can_filters[i].FilterActivation = ENABLE;
			Can_filters[i].FilterFIFOAssignment = 0;
			Can_filters[i].BankNumber = i+1; //This one isn't even used us HAL...
			Can_filters[i].FilterNumber = i;
			Can_filters[i].FilterMode = 0; //0 for mask, 1 for list
			Can_filters[i].FilterScale = 0; //0 for 16, 1 for 32
			HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
			return 4*i + 2; //"usage" index = xxxx0123
		}
	}
	return -1;
}

int Can_addMaskedFilterExt(uint32_t id, uint32_t mask, int isRemote){ //1 slot per bank
	uint8_t rtr = (isRemote==0)?0:1;
	uint8_t rtrm = (isRemote<0)?0:1; //0 = don't care, 1 = must match
	for(int i=0; i<CAN_BANKS; i++){ //open up a fresh bank
		if(Can_filterUsage[i]==0){ //find unused bank
			Can_filterUsage[i] |= 0x01; //use highest slot first
			Can_filterCapacity[i] = 1;
			Can_filters[i].FilterIdHigh = id>>13; //id[28:13]
			Can_filters[i].FilterIdLow = ((id<<3)&0xffff) | 1<<2 | rtr<<1; //id[12:0]|ide|rtr|0
			Can_filters[i].FilterMaskIdHigh = mask>>13; //mask[28:13]
			Can_filters[i].FilterMaskIdLow = ((mask<<3)&0xffff) | 1<<2 | rtrm<<1; //mask[12:0]|1|rtrm|0
			Can_filters[i].FilterActivation = ENABLE;
			Can_filters[i].FilterFIFOAssignment = 0;
			Can_filters[i].BankNumber = i+1; //This one isn't even used us HAL...
			Can_filters[i].FilterNumber = i;
			Can_filters[i].FilterMode = 0; //0 for mask, 1 for list
			Can_filters[i].FilterScale = 1; //0 for 16, 1 for 32
			HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
			return 4*i + 3; //"usage" index = xxxx0123
		}
	}
	return -1;
}

int Can_addFilterStd(uint16_t id, uint8_t isRemote){ //4 slots per bank
	isRemote = (isRemote==0)?0:1;
	for(int i=0; i<CAN_BANKS; i++){ //add to existing available bank
		uint8_t *usage = &Can_filterUsage[i];
		uint8_t *capacity = &Can_filterCapacity[i];
		if(*capacity==4 && *usage>0 && *usage<0x0f){ //if in use but unfilled
			uint8_t openSlot;
			if((*usage&0x08)==0){
				openSlot = 0;
				Can_filters[i].FilterMaskIdLow = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			}else if((*usage&0x04)==0){
				openSlot = 1;
				Can_filters[i].FilterIdLow = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			}else if((*usage&0x02)==0){
				openSlot = 2;
				Can_filters[i].FilterMaskIdHigh = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			}else{
				openSlot = 3;
				Can_filters[i].FilterIdHigh = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			}
			*usage |= 0x08>>openSlot;
			HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
			return 4*i + openSlot;
		}
	}
	for(int i=0; i<CAN_BANKS; i++){ //open up a fresh bank
		if(Can_filterUsage[i]==0){ //find unused bank
			Can_filterUsage[i] |= 0x08; //use highest slot first
			Can_filterCapacity[i] = 4;
			Can_filters[i].FilterMaskIdLow = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			Can_filters[i].FilterIdLow = 0; //0 should not be a valid addr
			Can_filters[i].FilterMaskIdHigh = 0;
			Can_filters[i].FilterIdHigh = 0;
			Can_filters[i].FilterActivation = ENABLE;
			Can_filters[i].FilterFIFOAssignment = 0;
			Can_filters[i].BankNumber = i+1; //This one isn't even used us HAL...
			Can_filters[i].FilterNumber = i;
			Can_filters[i].FilterMode = 1; //0 for mask, 1 for list
			Can_filters[i].FilterScale = 0; //0 for 16, 1 for 32
			HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
			return 4*i + 0; //"usage" index = xxxx0123
		}
	}
	return -1;
}

int Can_addFilterExt(uint32_t id, uint8_t isRemote){ //2 slots per bank
	isRemote = (isRemote==0)?0:1;
	for(int i=0; i<CAN_BANKS; i++){ //add to existing available bank
		uint8_t *usage = &Can_filterUsage[i];
		uint8_t *capacity = &Can_filterCapacity[i];
		if(*capacity==2 && *usage>0 && *usage<0x03 &&
				Can_filters[i].FilterScale==1){ //if in use but unfilled
			if((*usage&0x02) == 0){ //if slot 2 unused, so 3 must be in use
				Can_filters[i].FilterIdHigh = id>>13; //id[28:13]
				Can_filters[i].FilterIdLow = ((id<<3)&0xffff) | 1<<2 | isRemote << 1; //id[12:0]|ide|rtr|0
				*usage |= 0x02;
				HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
				return 4*i + 2; //"usage" index = xxxx0123
			}else{ //slot 2 in use, so slot 3 must be empty
				Can_filters[i].FilterMaskIdHigh = id>>13; //id[28:13]
				Can_filters[i].FilterMaskIdLow = ((id<<3)&0xffff) | 1<<2 | isRemote << 1; //id[12:0]|ide|rtr|0
				*usage |= 0x01;
				HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
				return 4*i + 3; //"usage" index = xxxx0123
			}
		}
	}
	for(int i=0; i<CAN_BANKS; i++){ //open up a fresh bank
		if(Can_filterUsage[i]==0){ //find unused bank
			Can_filterUsage[i] |= 0x02; //use highest slot first
			Can_filterCapacity[i] = 2;
			Can_filters[i].FilterIdHigh = id>>13; //id[28:13]
			Can_filters[i].FilterIdLow = ((id<<3)&0xffff) | 1<<2 | isRemote << 1; //id[12:0]|ide|rtr|0
			Can_filters[i].FilterMaskIdHigh = 0; //0 should not be a valid addr
			Can_filters[i].FilterMaskIdLow = 0;
			Can_filters[i].FilterActivation = ENABLE;
			Can_filters[i].FilterFIFOAssignment = 0;
			Can_filters[i].BankNumber = i+1; //This one isn't even used us HAL...
			Can_filters[i].FilterNumber = i;
			Can_filters[i].FilterMode = 1; //0 for mask, 1 for list
			Can_filters[i].FilterScale = 1; //0 for 16, 1 for 32
			HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
			return 4*i + 2; //"usage" index = xxxx0123
		}
	}
	return -1;
}

int Can_getFilter(int filterNum);
int Can_removeFilter(int filterNum);

int Can_getFilterNum(uint32_t fmi){
	int result = 0; //fmi is 0 indexed
	for(int i=0; i<CAN_BANKS; i++){
		if(Can_filterCapacity[i] > fmi){ //if target is in ith bank
			result += 4-Can_filterCapacity[i]+fmi;
			return result;
		}
		result += 4;
		fmi -= Can_filterCapacity[i];
	}
	return -1;
}

int Can_available(){
	if(Can_ovf==0){
		return Can_head - Can_tail;
	}else if((Can_ovf==1) && (Can_head <= Can_tail)){
		return CAN_BUFFER_LENGTH - (Can_tail - Can_head);
	}else{
		Can_tail = Can_head;
		Can_ovf = 1;
		return CAN_BUFFER_LENGTH;
	}
}

void Can_read(Can_frame_t *target);
void Can_readNum(Can_frame_t *target, int filterNum);

int Can_availableForTx();
void Can_sendStd(uint16_t id, uint8_t isRemote, uint8_t *data, uint8_t dlc);
void Can_sendExt(uint32_t id, uint8_t isRemote, uint8_t *data, uint8_t dlc);
int Can_resend();

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan);

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	Can_frame_t *newFrame = Can_headAddr();
	newFrame->isExt = hcan->pRxMsg->IDE;
	newFrame->id = (newFrame->isExt) ? hcan->pRxMsg->ExtId : hcan->pRxMsg->StdId;
	newFrame->isRemote = hcan->pRxMsg->RTR;
	newFrame->dlc = hcan->pRxMsg->DLC;
	if(newFrame->isRemote == 0){
		for(int i=0; i<newFrame->dlc; i++){
			newFrame->Data[i] = hcan->pRxMsg->Data[i];
		}
	}
	newFrame->filterNum = Can_getFilterNum(hcan->pRxMsg->FMI);
	Can_head++;
	if(Can_head == CAN_BUFFER_LENGTH){
		Can_head = 0;
		if(Can_ovf < 3) Can_ovf++;
	}
	HAL_CAN_Receive_IT(&hcan1,0);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
