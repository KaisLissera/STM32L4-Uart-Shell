/*
 * main.cpp
 *
 *  Created on: 2023.07.31
 *      Author: Kais Lissera
 */

#include <stm32l4xx.h>
//
#include <stdint.h>
#include <cstdio>
//
#include <ezhlib.h>
#include <ezhrcc.h>
#include <gpio.h>
#include <ezhuart.h>
#include <ezhtim.h>
#include <board.h>
//
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#define ENABLE_SYSCALLS	0

//UART IRQ uses FreeRTOS API functions,
//priority must be equal or lower then configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
#define UART_IRQ_PRIO			(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
//DMA IRQ does not use any FreeRTOS API
#define DMA_IRQ_PRIO			(0)
#define BLINK_TASK_PRIO			(tskIDLE_PRIORITY + 1)
#define BUTTON_TASK_PRIO		(tskIDLE_PRIORITY + 1)
#define ON_COMMAND_TASK_PRIO	(tskIDLE_PRIORITY + 1)

//Various
void RccInit();
void UartInit();
void LedInit();
void TimInit();
void ButtonsInit();
void ProcessCommand(UartCli_t* Cli);

//Shell commands
/////////////////////////////////////////////////////////////////////

// Commands without arguments
void HelpCallback();
void ResetCallback();
void StandbyCallback();
void TestCallback();

const struct
{
    const char* Command;
    const char* CommandDesc;
    void (*CommandCallback)(void);
} CommandsTable[] = {
		{"help",	"Prints available commands description\r",	HelpCallback},
		{"reset",	"Software reset MCU\r",						ResetCallback},
		{"standby",	"Put MCU in standby mode, MCU exit standby mode on push of button\r", StandbyCallback},
		{"test",	"test\r",									TestCallback},
		{"", "", NULL} // End marker
};

// Commands with arguments
void EchoCallback(uint32_t arg);
void LedOnOffCallback(uint32_t arg);
void LedOverrideCallback(uint32_t arg);

const struct
{
    const char* Command;
    const char* CommandDesc;
    void (*CommandCallback)(uint32_t);
} CommandsWithArgumentTable[] = {
		{"echo",		"0/1 - Disable/enable echo in shell\r",				EchoCallback},
		{"ledoverride",	"0/1 - Disable/enable led blinking\r",				LedOverrideCallback},
		{"led",			"0/1 - Turn off/on led and disable blinking\r",		LedOnOffCallback},
		{"", "", NULL} // End marker
};

//Free RTOS
/////////////////////////////////////////////////////////////////////

TaskHandle_t OnCommandTaskHandle;
void OnCommandTask(void *pvParametrs);

TaskHandle_t BlinkTaskHandle;
void BlinkTask(void *pvParametrs);

TaskHandle_t ButtonTaskHandle;
void ButtonTask(void *pvParametrs);

BaseType_t xHigherPriorityTaskWoken;
SemaphoreHandle_t CommandReady = xSemaphoreCreateBinary();

//Peripheral
/////////////////////////////////////////////////////////////////////

UartDma_t UartCmd(UART_PARAMS, DMA_UART_TX_CHANNEL, DMA_UART_RX_CHANNEL); //UART
UartCli_t Shell(&UartCmd); //UART shell
Timer_t Timer2(TIM2);
Timer_t Timer4(TIM4);
Button_t Button1(BUTTON_1_PARAMS);

//IRQ Handler wrapper for UART
extern "C"
void USART2_IRQHandler() { //Character match detected
	UartCmd.UartIrqHandler();
#if ENABLE_SYSCALLS == 1
	Shell.Printf((char*)"[SYS] Interrupt on command receive\r");
#endif
	xSemaphoreGiveFromISR(CommandReady,&xHigherPriorityTaskWoken);
#if ENABLE_SYSCALLS == 1
	if(xHigherPriorityTaskWoken == pdTRUE)
		Shell.Printf((char*)"[SYS] Semaphore given\r");
	else
		Shell.Printf((char*)"[SYS] Semaphore not given\r");
#endif
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//IRQ Handler wrapper for Dma Tx channel
extern "C"
void DMA1_Channel7_IRQHandler() { //DMA TX
	UartCmd.DmaIrqHandler();
}

//Tasks and main
/////////////////////////////////////////////////////////////////////

void OnCommandTask(void *pvParametrs) {
	while(1) {
#if ENABLE_SYSCALLS == 1
		Shell.Printf("[SYS] On command task\r");
#endif
		xSemaphoreTake(CommandReady, portMAX_DELAY);
#if ENABLE_SYSCALLS == 1
		Shell.Printf("[SYS] On command task take semaphore\r");
#endif
		ProcessCommand(&Shell);
	}//while
	vTaskDelete(NULL);
}

void BlinkTask(void *pvParametrs) {
	while(1) {
#if ENABLE_SYSCALLS == 1
		Shell.Printf("[SYS] Blink task\r");
#endif

//Timer PWM
//		for(uint32_t i = 0; i <= 255; i++) {
//			Timer2.LoadCompareValue(1,i);
//			vTaskDelay(2);
//		}
//		for(uint32_t i = 255; i > 0; i--) {
//			Timer2.LoadCompareValue(1,i);
//			vTaskDelay(2);
//		}

		ezhgpio::TogglePin(GPIOA, 5);
		vTaskDelay(MsToFreeRtosTick(500));
	}//while
	vTaskDelete(NULL);
}

void ButtonTask(void *pvParametrs) {
	while(1) {
//		Shell.Printf("[SYS] Button task\r");
		uint8_t State = Button1.CheckState();
		switch(State) {
//		case(Idle):
//			Shell.Printf("Idle\r");
//			break;
//		case(HoldDown):
//			Shell.Printf("HoldDown\r");
//			break;
		case(Pressed):
			Shell.Printf("Pressed\r");
			vTaskSuspend(BlinkTaskHandle);
			ezhgpio::ActivatePin(GPIOA, 5);
			break;
		case(Released):
			Shell.Printf("Released\r");
			vTaskResume(BlinkTaskHandle);
			ezhgpio::DeactivatePin(GPIOA, 5);
			break;
		}
		vTaskDelay(MsToFreeRtosTick(BUTON_POLL_DELAY_MS));
	}
	vTaskDelete(NULL);
}

int main() {
	RccInit();
	UartInit();
//	LedInit();
	TimInit(); //For timer PWM on LED
	ButtonsInit();
	//
	Shell.Printf("\rBase CLI functions\r"
				"Author: Kais Lissera\r");
	Shell.PrintBusFrequencies();
	//
	NVIC_SetPriorityGrouping(0); //Disable sub-priorities (disabled by default)
	//Tasks create
	xTaskCreate(&OnCommandTask, "Command", configMINIMAL_STACK_SIZE, NULL, BLINK_TASK_PRIO, &OnCommandTaskHandle);
//	xTaskCreate(&BlinkTask, "Blink", configMINIMAL_STACK_SIZE, NULL, ON_COMMAND_TASK_PRIO, &BlinkTaskHandle);
	xTaskCreate(&ButtonTask, "Button", configMINIMAL_STACK_SIZE, NULL, BUTTON_TASK_PRIO, &ButtonTaskHandle);

	vTaskStartScheduler();

	while(1);
}//main

//Peripheral initialization functions
/////////////////////////////////////////////////////////////////////

void UartInit() {
	UartCmd.Init(UART_SPEED);
	UartCmd.EnableCharMatch('\r', UART_IRQ_PRIO);
	UartCmd.InitDmaTx(DMA_IRQ_PRIO);
	UartCmd.InitDmaRx();
	UartCmd.StartDmaRx();
}

void RccInit() {
//	ezhrcc::SetSystemClk80MHz();
	ezhrcc::SetSystemClk2MHz(); // 2 MHz low power run
//	ezhrcc::SetSystemClk200kHz(); // 200 kHz low power run
	// APB1 clock must be 16 times greater then UART bode rate (115200)
	// 200 kHz support 9600 UART bode rate, 2 MHz - 115200
	ezhrcc::SetBusDividers(ahbDiv1, apbDiv1, apbDiv16);
}

void LedInit() {
	ezhgpio::SetupPin(LED_GREEN_PARAMS, GeneralOutput);
}

void TimInit() {
	Timer4.Init(500, 1000, UpCounter);
	Timer4.ConfigureChannel(TIM2_CHANNEL_1, 1, PWM1); // Rewrite ConfigureChannel
	Timer4.LoadCompareValue(1,500);

	TIM4->CR2 &= ~TIM_CR2_MMS_Msk;
	TIM4->CR2 |= 0b100 << TIM_CR2_MMS_Pos; // Channel 1 compare used as output
	//
	Timer2.Init(10000, 1000, UpCounter);
	Timer2.ConfigureChannel(TIM2_CHANNEL_1, 1, PWM1);
	Timer2.LoadCompareValue(1,500);

	TIM2->SMCR &= TIM_SMCR_TS_Msk;
	TIM2->SMCR |= 0b011 << TIM_SMCR_TS_Pos;
	TIM2->SMCR |= 0b0101 << TIM_SMCR_SMS_Pos;
	TIM2->SMCR |= TIM_SMCR_MSM;
	//
	Timer2.StartCount();
	Timer4.StartCount();
}

void ButtonsInit() {
	Button1.Init();
}

//Commands callbacks
/////////////////////////////////////////////////////////////////////

void HelpCallback() {
	Shell.Printf("List of available commands:\r");
	// Print description of commands without arguments
	uint32_t i = 0;
	while(CommandsTable[i].CommandCallback != NULL) {
		Shell.Printf("'%s' - %s",
				CommandsTable[i].Command,
				CommandsTable[i].CommandDesc);
		i++;
	}
	// Print description of commands with uint32_t argument
	i = 0;
	while(CommandsWithArgumentTable[i].CommandCallback != NULL) {
		Shell.Printf("'%s' - %s",
				CommandsWithArgumentTable[i].Command,
				CommandsWithArgumentTable[i].CommandDesc);
		i++;
	}
}

void StandbyCallback() {
	Shell.Printf("Entering standby\r");
	while(UartCmd.CheckDmaStatus(DMA_UART_TX_CHANNEL));
	PWR->PUCRC |= PWR_PUCRC_PC13; // Wake up pin has external pull-up
	PWR->PDCRA |= PWR_PDCRA_PA2; // Apply pull-down to UART
	PWR->CR3 |= PWR_CR3_APC;

	power::EnableWakeup2(DetectionOnLow);
	power::EnterStandby();
}

void TestCallback() { // Stack overflow???
	Shell.Printf("Test Callback\r");
	while(UartCmd.CheckDmaStatus(DMA_UART_TX_CHANNEL));
	UartCmd.WriteToBuffer((uint8_t)'a');
	UartCmd.StartDmaTxIfNotYet();
	Shell.Printf("Current System Clock %d Hz\r", ezhrcc::GetCurrentSystemClock());
//	Shell.Printf("Current System Clock %d Hz\r", ezhrcc::GetCurrentSystemClock());
//	Shell.Printf("Current System Clock %d Hz\r", ezhrcc::GetCurrentSystemClock());
//	Shell.Printf("Current System Clock %d Hz\r", ezhrcc::GetCurrentSystemClock());
//	Shell.Printf("Current System Clock %d Hz\r", ezhrcc::GetCurrentSystemClock());
//	Shell.Printf("Current System Clock %d Hz\r", ezhrcc::GetCurrentSystemClock());
//	Shell.Printf("Current System Clock %d Hz\r", ezhrcc::GetCurrentSystemClock());
//	Shell.Printf("Current System Clock %d Hz\r", ezhrcc::GetCurrentSystemClock());
//	Shell.Printf("Current System Clock %d Hz\r", ezhrcc::GetCurrentSystemClock());
//	Shell.Printf("Current System Clock %d Hz\r", ezhrcc::GetCurrentSystemClock());
}

void ResetCallback() {
	Shell.Printf("Performing software reset\r");
	while(UartCmd.CheckDmaStatus(DMA_UART_TX_CHANNEL));
	NVIC_SystemReset();
}

void EchoCallback(uint32_t arg) {
	if(arg == 0) {
		Shell.Printf("Echo disabled\r");
		Shell.EchoEnabled = 0;
	} else {
		Shell.Printf("Echo enabled\r");
		Shell.EchoEnabled = 1;
	}
}

void LedOnOffCallback(uint32_t arg) {
	vTaskSuspend(BlinkTaskHandle);
	if (arg == 0) {
		ezhgpio::DeactivatePin(GPIOA, 5);
		Shell.Printf("Led off\r");
	}
	else {
		ezhgpio::ActivatePin(GPIOA, 5);
		Shell.Printf("Led on\r");
	}
}

void LedOverrideCallback(uint32_t arg) {
	if (arg == 0) {
		vTaskResume(BlinkTaskHandle);
		Shell.Printf("Led blink override disabled\r");
	} else {
		vTaskSuspend(BlinkTaskHandle);
		Shell.Printf("Led blink override enabled\r");
	}
}

/////////////////////////////////////////////////////////////////////

void ProcessCommand(UartCli_t* Cli) {
	Cli->ReadCommand(); // Read command from DMA buffer
	uint8_t OnComandFoundFl = 0;

	// Checking commands without arguments
	uint32_t i = 0;
	while(CommandsTable[i].CommandCallback != NULL) {
		if(strcmp(CommandsTable[i].Command, Cli->CommandBuffer) == 0) {
			CommandsTable[i].CommandCallback();
			OnComandFoundFl = 1;
			break;
		}
		i++;
	} // while

	// Checking commands with arguments
	i = 0;
	if(OnComandFoundFl == 0) {
		while(CommandsWithArgumentTable[i].CommandCallback != NULL) {
			if(strcmp(CommandsWithArgumentTable[i].Command, Cli->CommandBuffer) == 0) {
				uint32_t Arg = std::atoi(Cli->ArgBuffer); // Returns 0 if conversion can't be performed
				if((Cli->ArgBuffer[0] != '0') && (Arg == 0))
					Shell.Printf("Error '%s' is not a number\r", Cli->ArgBuffer);
				else
					CommandsWithArgumentTable[i].CommandCallback(Arg);
				OnComandFoundFl = 1;
				break;
			}
			i++;
		} // while
	} // If

	// Can't find command in buffer
	if(OnComandFoundFl == 0) {
		Shell.Printf("Error '%s' is not a valid command\r", Cli->CommandBuffer);
	}
}

// FreeRTOS hooks
/////////////////////////////////////////////////////////////////////

void vApplicationIdleHook() {
	Shell.Printf("[SYS] Idle Task\r");
}

void vApplicationMallocFailedHook() {
	Shell.Printf("[SYS] Malloc failed\r");
	while(1) {};
}
