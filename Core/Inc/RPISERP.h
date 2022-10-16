/*
 * RPISERP.h
 *
 *  Created on: 16. 10. 2022
 *      Author: marti
 *      Brief:  Low layer of serial protocol for communication with RaspberryPi
 */

#ifndef INC_RPISERP_H_
#define INC_RPISERP_H_

#include "main.h"


#define RSP_PACKET_SIZE 14
#define RSP_BAUD_RATE 57600

void RSP_Init(UART_HandleTypeDef* Uart, DMA_Channel_TypeDef* TxDma, DMA_Channel_TypeDef* RxDma);

void RSP_Send(uint8_t* data, uint8_t length);


// function to be called periodically from the scheduler (i.e every 5ms)
void RSP_TransmitFromFifo(void);


bool RSP_GetMessage(uint8_t* data, uint8_t* length);

#endif /* INC_RPISERP_H_ */
