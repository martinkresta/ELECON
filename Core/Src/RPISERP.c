/*
 * RPISERP.c
 *
 *  Created on: 16. 10. 2022
 *      Author: marti
 *      Brief:  Low layer of serial protocol for communication with RaspberryPi
 */

#include "RPISERP.h"
#include "circbuf.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>




CB_handle mReceiveFifo;

// TX FIFO: ring of whole RPISERP frames. Producer = main loop (RSP_Send), consumer =
// main loop (RSP_TransmitFromFifo) and the UART TC interrupt (chains the next frame).
// Every access to the indices is done with interrupts disabled.
static uint8_t mTxFifo[RSP_TX_FIFO_SIZE][RSP_PACKET_SIZE];
static uint8_t mTxHead;
static uint8_t mTxTail;
static uint8_t mTxCount;

volatile uint32_t gRspTxOverflow;   // frames dropped because the TX FIFO was full (debug variable)
volatile uint32_t gRspTxFrames;     // frames handed to the UART (debug variable)


UART_HandleTypeDef* mRspUart;
DMA_Channel_TypeDef* mTxDma;
DMA_Channel_TypeDef* mRxDma;

volatile bool mTxBusy;   // written from main loop and interrupts
bool mRxBusy;

uint8_t mRxData[RSP_PACKET_SIZE];
uint8_t mTxData[RSP_PACKET_SIZE];


void InitUartPeripheral(void);
void StartReciever(void);
void StartTransmiter(uint8_t size);
static void TryStartTx(void);
uint8_t Checksum(uint8_t* data, uint8_t length);


// initialization of UART interface
// It is assumed that the linkage of DMA channel with UART is already done by CubeMX generated code !
void RSP_Init(UART_HandleTypeDef* Uart, DMA_Channel_TypeDef* TxDma, DMA_Channel_TypeDef* RxDma)
{

  mRspUart = Uart;
  mTxDma = TxDma;
  mRxDma = RxDma;


  mTxHead = 0;
  mTxTail = 0;
  mTxCount = 0;
  mReceiveFifo = CB_Create(RSP_PACKET_SIZE, 10);
  if (mReceiveFifo == NULL)
  {
      // TBD error
  }

  InitUartPeripheral();

  mTxBusy = false;
  mRxBusy = false;

  // enable receiver

  //configure RX DMA
  mRxDma->CCR = DMA_CCR_MINC;  // memory auto increment, no rx DMA interupts are enabled
  mRxDma->CPAR = (uint32_t)(&(Uart->Instance->RDR));
  mRxDma->CMAR = (uint32_t)(mRxData);
  StartReciever();

  // configure TX DMA
  mTxDma->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_DIR;
  mTxDma->CMAR = (uint32_t)(mTxData);
  mTxDma->CPAR = (uint32_t)(&(Uart->Instance->TDR));


  mRspUart->Instance->CR1 &= ~USART_CR1_UE;  // disable uart
  mRspUart->Instance->CR2 = 0x00;
  mRspUart->Instance->CR3 = USART_CR3_OVRDIS| USART_CR3_DMAR | USART_CR3_DMAT;  // enable DMAs
  mRspUart->Instance->CR1 = USART_CR1_IDLEIE | USART_CR1_RE | USART_CR1_UE;  // Idle interrupt enabled

}


void RSP_Send(uint8_t* data, uint8_t length)
{
  uint8_t txPacket[RSP_PACKET_SIZE];
  if (length <= RSP_PACKET_SIZE - 4)
  {
    txPacket[0] = RSP_MSG_START_B1;
    txPacket[1] = RSP_MSG_START_B2;
    txPacket[2] = length;
    memcpy(&(txPacket[3]), data, length);
    txPacket[length + 3] = Checksum(txPacket, length +3 );

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (mTxCount < RSP_TX_FIFO_SIZE)
    {
      memcpy(mTxFifo[mTxHead], txPacket, RSP_PACKET_SIZE);
      mTxHead = (mTxHead + 1) % RSP_TX_FIFO_SIZE;
      mTxCount++;
    }
    else
    {
      gRspTxOverflow++;   // FIFO full: the new frame is dropped
    }
    TryStartTx();   // start immediately if the transmitter is idle
    __set_PRIMASK(primask);
  }
  else
  {
    // TBD error
  }
}


// Starts the next frame from the FIFO if the transmitter is idle.
// Must be called with interrupts disabled (or from the UART interrupt).
static void TryStartTx(void)
{
  if (mTxBusy == true || mTxCount == 0)
  {
    return;
  }
  memcpy(mTxData, mTxFifo[mTxTail], RSP_PACKET_SIZE);
  mTxTail = (mTxTail + 1) % RSP_TX_FIFO_SIZE;
  mTxCount--;
  gRspTxFrames++;
  StartTransmiter(mTxData[2] + 4); // size = num of databytes + 2 header bytes + length byte + checksum
}


// Frames are chained on UART TX-complete; this periodic call (scheduler) only kicks an idle transmitter.
void RSP_TransmitFromFifo(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  TryStartTx();
  __set_PRIMASK(primask);
}


uint32_t RSP_GetTxOverflowCount(void)
{
  return gRspTxOverflow;
}


bool RSP_GetMessage(uint8_t* data, uint8_t* length)
{
  uint8_t rxPacket[RSP_PACKET_SIZE];
  if (0 == CB_Get(mReceiveFifo, rxPacket))
  {
    *length = rxPacket[0];
    memcpy(data ,&(rxPacket[1]),rxPacket[0]);
    return true;
  }
  else
  {
    return false;
  }
}



void InitUartPeripheral(void)
{
 // mRspUart->Instance = USART2;
  mRspUart->Init.BaudRate = RSP_BAUD_RATE;
  mRspUart->Init.WordLength = UART_WORDLENGTH_8B;
  mRspUart->Init.StopBits = UART_STOPBITS_1;
  mRspUart->Init.Parity = UART_PARITY_NONE;
  mRspUart->Init.Mode = UART_MODE_TX_RX;
  mRspUart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  mRspUart->Init.OverSampling = UART_OVERSAMPLING_16;
  mRspUart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  mRspUart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(mRspUart) != HAL_OK)
  {
    Error_Handler();
  }
}


void StartReciever(void)
{
  mRxDma->CNDTR = RSP_PACKET_SIZE;
  mRxDma->CCR |= DMA_CCR_EN;  // enable DMA channel
  mRspUart->Instance->CR1 |= USART_CR1_IDLEIE | USART_CR1_RE | USART_CR1_UE;  // Idle interrupt enabled
  mRxBusy = true;
}

void StartTransmiter(uint8_t size)
{
  if (size <= RSP_PACKET_SIZE)
  {
    mTxBusy = true;
    mTxDma->CCR &= ~DMA_CCR_EN;  // disable dma
    mTxDma->CNDTR = size;
    mRspUart->Instance->ICR = USART_ICR_TCCF;
    mTxDma->CCR |= DMA_CCR_EN;  // enable DMA channel
    mRspUart->Instance->CR1 |= USART_CR1_TCIE | USART_CR1_TE | USART_CR1_UE;  // TCinterrupt enabled
  }
}


uint8_t Checksum(uint8_t* data, uint8_t length)
{
  uint8_t i,chsum;
  chsum = 0;
  for(i = 0;i < length; i++)
  {
    chsum += data[i];
  }
  return chsum;
}


/* Hard-coded IRQ handlers of DMA channels and UART*/

void USART1_IRQHandler(void)
{
  if (mRspUart->Instance->ISR & USART_ISR_IDLE)   // Rx Idle interrupt
  {
    uint8_t rxSize = RSP_PACKET_SIZE - mRxDma->CNDTR;
    mRxDma->CCR &= ~DMA_CCR_EN; // disable the DMA channel even when it is not completed transfer
    mRspUart->Instance->ICR = USART_ICR_IDLECF;  // clear the IDLE flag
    if(mRxData[0] == RSP_MSG_START_B1  && mRxData[1] == RSP_MSG_START_B2)
    {
      if (rxSize == mRxData[2] + 4)   // if full packet was received
      {
         //check checksum
        if(mRxData[rxSize - 1] == Checksum(mRxData, rxSize-1))
        {
          CB_Put(mReceiveFifo, &(mRxData[2]));
        }
      }
    }

    // reenable the receiver for next packet reception
    StartReciever();
  }

  if (mRspUart->Instance->ISR & USART_ISR_TC)   // Transfer Complete
  {
    mRspUart->Instance->ICR = USART_ICR_TCCF; // clear the TC flag
    mTxBusy = false;
    TryStartTx();   // chain the next frame without waiting for the scheduler
  }
}

/* TX DMA*/
void DMA1_Channel4_IRQHandler(void)
{
  mTxDma->CCR &= ~DMA_CCR_EN;  // disable the interrupt
  DMA1->IFCR |= DMA_IFCR_CGIF4;  // clear all DMA flags
  // mTxBusy is released by the UART TC interrupt (last byte physically shifted out)
}

/* RX DMA*/
void DMA1_Channel5_IRQHandler(void)
{
  // nothing here this interrupt is not enabled
}
