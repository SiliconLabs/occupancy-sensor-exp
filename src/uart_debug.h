/***************************************************************************//**
 * @file  uart_debug.h
 * @brief UART communication to PC side host header.
 * @version 1.0.0
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

#ifndef UARTDEBUG_H_
#define UARTDEBUG_H_

#ifdef __cplusplus
extern "C" {
#endif

void UART_Debug_Init(  );
void UART_Debug_ProcessOutput(int8_t *data_buffer, uint32_t bytes_to_output, uint16_t timestamp, uint8_t marker);
void UART_Send(uint8_t *pMsg, int MsgLength);

#ifdef __cplusplus
}
#endif

#endif /* UARTDEBUG_H_ */
