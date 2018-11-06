
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

// Libcanard includes
#include "libcanard_wrapper.h"
#include "canard.h"
#include "canard_stm32.h"

// Message type includes
#include "uavcan/protocol/NodeStatus.h"

#include "spear/arm/JointCommand.h"
#include "spear/arm/JointStatus.h"
#include "spear/drive/DriveCommand.h"
#include "spear/drive/DriveStatus.h"

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;

static uint8_t m_transfer_id;
uint8_t encoded_buf[10] = {[0 ... 9] = 0};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);

static void TIM_Init(void);
static void UART_Init(void);


/** @brief Implementation of a usleep function to avoid errors
 *
 */
void usleep(useconds_t __useconds) {
	// This only has to last in increments of 1 millisecond
	HAL_Delay(__useconds / 1000);
}


bool should_accept(const CanardInstance* ins,
					uint64_t* out_data_type_signature,
					uint16_t data_type_id,
					CanardTransferType transfer_type,
					uint8_t source_node_id) {

	return true;
}


void on_reception(CanardInstance* ins,
					CanardRxTransfer* transfer) {
	uint8_t tx_buf[64];

	int data_type_id = transfer->data_type_id;
	int node_id = transfer->source_node_id;
	int data_len = transfer->payload_len;

	int length = sprintf((char*)tx_buf, "Message received - node ID: %d\n\r", node_id);
	HAL_UART_Transmit(&huart1, tx_buf, length, 100);

	length = sprintf((char*)tx_buf, "Data type ID: %d\n\r", data_type_id);
	HAL_UART_Transmit(&huart1, tx_buf, length, 100);

	length = sprintf((char*)tx_buf, "Data length: %d\n\r", data_len);
	HAL_UART_Transmit(&huart1, tx_buf, length, 100);

	if (data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID) {
		uavcan_protocol_NodeStatus msg;

		uavcan_protocol_NodeStatus_decode(transfer, transfer->payload_len,
				&msg, NULL);

		length = sprintf((char*)tx_buf, "NodeStatus Message ->\n\r");
		HAL_UART_Transmit(&huart1, tx_buf, length, 100);

		length = sprintf((char*)tx_buf, "    Health: %d\n\r", msg.health);
		HAL_UART_Transmit(&huart1, tx_buf, length, 100);

		length = sprintf((char*)tx_buf, "    Mode: %d\n\r", msg.mode);
		HAL_UART_Transmit(&huart1, tx_buf, length, 100);

	} else if (data_type_id == SPEAR_ARM_JOINTCOMMAND_ID) {
		spear_arm_JointCommand msg;

		spear_arm_JointCommand_decode(transfer, transfer->payload_len,
				&msg, NULL);

		length = sprintf((char*)tx_buf, "JointCommand Message ->\n\r");
		HAL_UART_Transmit(&huart1, tx_buf, length, 100);

		length = sprintf((char*)tx_buf, "    Joint: %d\n\r", msg.joint);
		HAL_UART_Transmit(&huart1, tx_buf, length, 100);

		length = sprintf((char*)tx_buf, "    Angle: %d\n\r", msg.angle);
		HAL_UART_Transmit(&huart1, tx_buf, length, 100);
	} else if (data_type_id == SPEAR_DRIVE_DRIVECOMMAND_ID) {
		spear_drive_DriveCommand msg;

		spear_drive_DriveCommand_decode(transfer, transfer->payload_len,
				&msg, NULL);

		length = sprintf((char*)tx_buf, "DriveCommand Message ->\n\r");
		HAL_UART_Transmit(&huart1, tx_buf, length, 100);

		// Need to change DSDL
		length = sprintf((char*)tx_buf, "    Wheel: %d\n\r", msg.wheel);
		HAL_UART_Transmit(&huart1, tx_buf, length, 100);

		length = sprintf((char*)tx_buf, "    Speed: %d\n\r", msg.speed);
		HAL_UART_Transmit(&huart1, tx_buf, length, 100);
	}
}


int16_t setup_hardware_can_filters(void) {
	const CanardSTM32AcceptanceFilterConfiguration conf[1] = {
			{ .id = 0, .mask = 0 }
	};


	return canardSTM32ConfigureAcceptanceFilters((const CanardSTM32AcceptanceFilterConfiguration* const) &conf, 1);
}


/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  UART_Init();
  TIM_Init();


  HAL_UART_Transmit(&huart1, (uint8_t*) "Running...\n\r", 12, 1000);

  libcanard_init( on_reception,
				  should_accept,
				  NULL,
				  8000000,
				  250000);

  setup_hardware_can_filters();


  while (1)
  {

	  tx_once();
	  rx_once();
	   volatile uint8_t thingie = TIM2->CNT;

  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_1TQ;
  hcan.Init.BS2 = CAN_BS2_1TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void TIM_Init(void) {
	__HAL_RCC_TIM2_CLK_ENABLE();

	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);


	TIM_MasterConfigTypeDef master;
	TIM_ClockConfigTypeDef clock;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 8000;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	htim2.Init.Period = 1000;
	HAL_TIM_Base_Init(&htim2);

	clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim2, &clock);

	master.MasterOutputTrigger = TIM_TRGO_RESET;
	master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	HAL_TIMEx_MasterConfigSynchronization(&htim2, &master);

	HAL_TIM_Base_Start_IT(&htim2);

}



void UART_Init(void) {
	// Initialize clocks
	__HAL_RCC_USART1_CLK_ENABLE();

	// Configure pins for UART
	// PA9 -> TX
	// PA10 -> RX

	GPIO_InitTypeDef GPIO_InitStruct;

	// Configure Pin 9
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure Pin 10
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Initialize UART interface
	huart1.Instance = USART1;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.BaudRate = 115200;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	HAL_UART_Init(&huart1);

}

void TIM2_IRQHandler(void) {
	__disable_irq();

	__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_UPDATE);

	static uavcan_protocol_NodeStatus status = {
			.uptime_sec = 0,
			.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK,
			.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL,
			.sub_mode = 0,
			.vendor_specific_status_code = 0
	};

    uint8_t len = uavcan_protocol_NodeStatus_encode(&status, (void*) encoded_buf);

	canardBroadcast(&m_canard_instance,
		  123,
		  UAVCAN_PROTOCOL_NODESTATUS_ID,
		  &m_transfer_id,
		  0,
		  &encoded_buf,
		  len);

	HAL_UART_Transmit(&huart1, "\n\r", 2, 100);

	__enable_irq();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
