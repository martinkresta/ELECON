/*
 * RPISERP.h
 *
 *  Created on: 16. 10. 2022
 *      Author: marti
 *      Brief:  Low layer of serial protocol for communication with RaspberryPi
 *      The single packet is up to 16 bytes long
 *      The first byte is length of the payload, the last byte is the checksum over the length and payload
 *      packet structure  SYNC(2bytes) | LENGTH (1byte) | PAYLOAD (0-12 Bytes)  | CHSUM (1byte) |

 */

#ifndef INC_RPISERP_H_
#define INC_RPISERP_H_

#include "main.h"
#include <stdbool.h>


#define RSP_PACKET_SIZE 14
#define RSP_BAUD_RATE 115200

#define  RSP_MSG_START_B1             0x7F
#define  RSP_MSG_START_B2             0xAA

void RSP_Init(UART_HandleTypeDef* Uart, DMA_Channel_TypeDef* TxDma, DMA_Channel_TypeDef* RxDma);

void RSP_Send(uint8_t* data, uint8_t length);


// function to be called periodically from the scheduler (i.e every 5ms)
void RSP_TransmitFromFifo(void);


bool RSP_GetMessage(uint8_t* data, uint8_t* length);

#endif /* INC_RPISERP_H_ */
