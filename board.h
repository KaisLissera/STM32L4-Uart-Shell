/*
 * board.h
 *
 *  Created on: 2023.07.31
 *      Author: Kais Lissera
 */

#ifndef INC_BOARD_H_
#define INC_BOARD_H_

#define USE_FREE_RTOS
#define SYS_CLK 				2000000U // system clock
#define FREE_RTOS_TICK			10000U
#define USE_SYSTICK 			(0) //FreeRTOS using SysTick, disable to avoid conflicts

#define UART_PARAMS 			USART2, GPIOA, 2, GPIOA, 3
#define UART_SPEED 				(115200UL)
#define DMA_UART_TX_CHANNEL 	DMA1_Channel7
#define DMA_UART_RX_CHANNEL 	DMA1_Channel6

#define	LED_GREEN_PARAMS		GPIOA, 5, NoPullUpDown

#define TIM2_CHANNEL_1			GPIOA, 5, AF1

#define BUTTON_1_PARAMS			GPIOC, 13, PullUp
#define BUTON_POLL_DELAY_MS		100

#endif /* INC_BOARD_H_ */
