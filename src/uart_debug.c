/***************************************************************************//**
 * @file
 * @brief UART communication to PC side host.
 * @version 1.0.2
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "em_device.h"
#include "em_cmu.h"
#include "em_leuart.h"
#include "dmadrv.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "em_usart.h"
#include <stdint.h>

/* DEFINES */

#define MAX_FRAME_SIZE    1000
#ifdef EFR32MG12P332F1024GL125
  #define TX_PORT           gpioPortA
  #define RX_PORT           gpioPortA
  #define TX_PIN            0
  #define RX_PIN            1
  #define VCOM_EN_PORT      gpioPortA
  #define VCOM_EN_PIN       5
  #define US0_TX_LOC        USART_ROUTELOC0_TXLOC_LOC0
  #define US0_RX_LOC        USART_ROUTELOC0_RXLOC_LOC0

#else
  #define TX_PORT           gpioPortA
  #define RX_PORT           gpioPortA
  #define TX_PIN            0
  #define RX_PIN            1
  #define VCOM_EN_PORT      gpioPortA
  #define VCOM_EN_PIN       5
  #define US0_TX_LOC        USART_ROUTELOC0_TXLOC_LOC0
  #define US0_RX_LOC        USART_ROUTELOC0_RXLOC_LOC0
#endif

/* GLOBAL VARIABLES */
static volatile bool transferInProcess = false;
static uint8_t TxFrame[MAX_FRAME_SIZE];
static uint8_t buf[MAX_FRAME_SIZE];
static uint8_t rxbuf[20];
static unsigned int           txDmaId;
static unsigned int           rxDmaId;
#define NULL 0
#define DEBUG_UART USART0
/* serial output */
#if 0
#define DMA_UART_SIGNAL dmadrvPeripheralSignal_LEUART0_TXBL
#else
#define DMA_UART_SIGNAL dmadrvPeripheralSignal_USART0_TXBL
#endif

static bool RxDmaCallback(unsigned int channel,
                          unsigned int sequenceNo,
                          void *userParam);

/** Flag used to determine if UART is initialized */
static bool enabled = false;
static uint8_t usb_data_buffer[((520) + 3) & ~3];

#if 0
/* Defining the LEUART0 initialization data */
static LEUART_Init_TypeDef leuart0Init =
{
  .enable   = leuartEnable,       /* Activate data reception on LEUn TX and RX pins. */
  .refFreq  = 0,                  /* Inherit the clock frequenzy from the LEUART clock source */
  .baudrate = 115200,               /* Baudrate = 9600 bps */
  .databits = leuartDatabits8,    /* Each LEUART frame containes 8 databits */
  .parity   = leuartNoParity,     /* No parity bits in use */
  .stopbits = leuartStopbits1,    /* Setting the number of stop bits in a frame to 1 bitperiods */
};
#endif

/***************************************************************************//**
 * @brief Start a UART DMA receive operation.
 ******************************************************************************/
static void StartReceiveDma(void)
{
  int length = 1;
  DMADRV_PeripheralMemory(rxDmaId,
                          dmadrvPeripheralSignal_USART0_RXDATAV,
                          &rxbuf,
                          (void*)&(DEBUG_UART->RXDATA),
                          true,
                          length,
                          dmadrvDataSize1,
                          RxDmaCallback,
                          NULL);
}

static bool RxDmaCallback(unsigned int channel,
                          unsigned int sequenceNo,
                          void *userParam)
{
  (void) sequenceNo;
  (void) userParam;
  int8_t ibuf[20];
  int i;
  if (channel == rxDmaId) {
    for (i = 0; i < 1; i++) {
      ibuf[i] = rxbuf[i];
    }

    /* Start a new DMA receive transfer. */
    StartReceiveDma();
  }
  (void) ibuf;
  return true;
}

/**************************************************************************//**
* @brief  DMA Callback function
*
* When the DMA transfer is completed, disables the DMA wake-up on TX in the
* LEUART to enable the DMA to sleep even when the LEUART buffer is empty.
*
******************************************************************************/
static bool TxDmaCallback(unsigned int channel,
                          unsigned int sequenceNo,
                          void *userParam)
{
  (void)sequenceNo;
  (void)userParam;
  if (channel == txDmaId) {
    transferInProcess = false;
  }

  // return false to stop transfers in ping-pong transfers
  return false;
}

// Statements like:
// #pragma message(Reminder "Fix this problem!")
// Which will cause messages like:
// C:\Source\Project\main.cpp(47): Reminder: Fix this problem!
// to show up during compiles. Note that you can NOT use the
// words "error" or "warning" in your reminders, since it will
// make the IDE think it should abort execution. You can double
// click on these messages and jump to the line in question.

#define Stringize(L)     #L
#define MakeString(M, L) M(L)
#define $Line MakeString(Stringize, __LINE__)
#define Reminder __FILE__ "(" $Line ") : Reminder: "

/**************************************************************************//**
 * @brief BLE_UART_Send
 *****************************************************************************/
bool UART_Debug_IsConnected(void)
{
  return enabled;
}

void UART_Send(uint8_t *pMsg, int MsgLength)
{
  uint16_t  i;
  if (UART_Debug_IsConnected() == false) {
    return;
  }
  while (transferInProcess) {
    EMU_EnterEM1();
  }
  transferInProcess = true;
  /* Copy the message */
  for (i = 0; i < MsgLength; i++) {
    buf[i] = pMsg[i];
  }

  /* Start the DMA transfer. */
  DMADRV_MemoryPeripheral(txDmaId,
                          DMA_UART_SIGNAL,
                          (void*)&(DEBUG_UART->TXDATA),
                          &buf,
                          true,
                          MsgLength,
                          dmadrvDataSize1,
                          TxDmaCallback,
                          NULL);

  while (transferInProcess);                          /* Wait for the DMA transfer to complete. */
  while ((USART0->STATUS & USART_STATUS_TXIDLE));     /* Wait for the USART to begin transfer. */
  while (!(USART0->STATUS & USART_STATUS_TXIDLE));    /* Wait for the USART to finish transfer. */
}

/**************************************************************************//**
 * @brief  Setup Low Energy UART with DMA operation
 *
 * The LEUART/DMA interaction is defined, and the DMA, channel and descriptor
 * is initialized. Then DMA starts to wait for and receive data, and the LEUART0
 * is set up to generate an interrupt when it receives the defined signal frame.
 * The signal frame is set to '\r', which is the "carriage return" symbol.
 *
 *****************************************************************************/
static void DMA_Init()
{
  DMADRV_Init();
  DMADRV_AllocateChannel(&txDmaId, NULL);
  DMADRV_AllocateChannel(&rxDmaId, NULL);
  StartReceiveDma();
}

/**************************************************************************//**
 * @brief byteToHexChar - Converts hex byte to ASCII character
 *****************************************************************************/
static uint8_t byteToHexChar(uint8_t hex)
{
  if ( hex < 0x0A ) {
    return (hex + 0x30);
  } else {
    return (hex + 0x37);
  }
}

/**************************************************************************//**
 * @brief  Initialize LEUART 0
 *
 * Here the LEUART is initialized with the chosen settings.
 *
 *****************************************************************************/
void UART_Debug_Init()
{
/* Enable the required clocks */
  CMU_ClockEnable(cmuClock_USART0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

/* Configure GPIO */
  GPIO_PinModeSet(TX_PORT, TX_PIN, gpioModePushPull, 1);
  GPIO_PinModeSet(RX_PORT, RX_PIN, gpioModeInput, 0);
  GPIO_PinModeSet(VCOM_EN_PORT,                /* GPIO port */
                  VCOM_EN_PIN,                 /* GPIO port number */
                  gpioModePushPull,       /* Pin mode is set to push pull */
                  1);

/* Configure USART peripheral. Default configuration
 * is fine. We only need to set the baud rate.  */
  USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;
  uartInit.baudrate = 115200;
  USART_InitAsync(USART0, &uartInit);

/* Enable RX and TX and set location */
/* Set up RX pin */
  USART0->ROUTELOC0 = (USART0->ROUTELOC0 & (~_USART_ROUTELOC0_RXLOC_MASK))
                      | US0_RX_LOC;
  USART0->ROUTEPEN = USART0->ROUTEPEN | USART_ROUTEPEN_RXPEN;

  /* Set up TX pin */
  USART0->ROUTELOC0 = (USART0->ROUTELOC0 & (~_USART_ROUTELOC0_TXLOC_MASK))
                      | US0_TX_LOC;

  USART0->ROUTEPEN = USART0->ROUTEPEN | USART_ROUTEPEN_TXPEN;
  DMA_Init();
  enabled = true;
  //UART_Debug_IsConnected();
}

/**************************************************************************//**
 * @brief Output data buffer to USB port
 * @param bytes_to_output: # of bytes to read from data buffer. 2*bytes_to_output + 8 MUST BE LESS THAN size of usb_data_buffer
 *****************************************************************************/
void UART_Debug_ProcessOutput(int8_t *data_buffer, uint32_t bytes_to_output, uint16_t timestamp, uint8_t marker)
{
  uint32_t counter;
  //uint8_t *Usb_Buffer[1] = {usb_data_buffer};
  uint8_t *usb_data_ptr;

  usb_data_buffer[0] = 0x10;
  usb_data_buffer[1] = marker;
  usb_data_buffer[2] = byteToHexChar((timestamp >> 12) & 0xF);
  usb_data_buffer[3] = byteToHexChar((timestamp >> 8) & 0xF);
  usb_data_buffer[4] = byteToHexChar((timestamp >> 4) & 0xF);
  usb_data_buffer[5] = byteToHexChar(timestamp & 0xF);

  usb_data_ptr = &usb_data_buffer[6];
  for ( counter = 0; counter < bytes_to_output; counter++ ) {
    usb_data_ptr[counter * 2] = byteToHexChar((uint8_t)data_buffer[counter] >> 4);
    usb_data_ptr[counter * 2 + 1] = byteToHexChar((uint8_t)data_buffer[counter] & 0x0F);
  }
  usb_data_ptr[counter * 2] = 0x10;
  usb_data_ptr[counter * 2 + 1] = 0x0D;
  UART_Send(usb_data_buffer, counter * 2 + 1 + 6 + 1);
}

uint32_t UART_Debug_WriteBinaryBytes(uint8_t *buf, int8_t size)
{
  UART_Send(buf, size);
  return 0;
}

uint32_t UART_Debug_WriteDataBuffer(char*   buf, int size)
{
  char c;
  int i;
  for (i = 0; i < size; i++) {
    c = *buf;
    TxFrame[i * 3] = byteToHexChar(c >> 4);
    TxFrame[i * 3 + 1] = byteToHexChar(c & 0xf);
    TxFrame[i * 3 + 2] = ' ';
    buf++;
  }
  TxFrame[i * 3] = '\n';

  UART_Send(TxFrame, (size * 3) + 1);
  return 0;
}

uint32_t USB_WriteString(char*   str)
{
  char c;
  int size = 0;
  while (*str) {
    c = *str;
    TxFrame[size++] = c;
    str++;
  }
  UART_Send(TxFrame, size);
  return 0;
}
