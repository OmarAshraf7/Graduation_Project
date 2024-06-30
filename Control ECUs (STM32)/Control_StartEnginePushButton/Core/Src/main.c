/*********************************************************************************************************
 *                      				     Includes                                                    *
 *********************************************************************************************************/

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*********************************************************************************************************
 *                      				  Private Variables                                              *
 *********************************************************************************************************/

CAN_HandleTypeDef hcan;

/*********************************************************************************************************
 *                                   Private Function Prototypes                                         *
 *********************************************************************************************************/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);

/*********************************************************************************************************
 *                      				  Global Variables                                               *
 *********************************************************************************************************/

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

volatile bool txFlag = 0; /* msg sent flag */
volatile bool rxFlag = 0; /* msg recieved flag */

uint8_t TxData[8];
uint8_t RxData[8];

unit_8  inside_size  = 6 ;
unit_8  outside_size = 7 ;
char    inside  [outside_size] = "inside";
char    outside [outside_size] = "outside";
unit_8	inside_flag  = 0;
unit_8	outside_flag = 1;

uint32_t TxMailbox;

/*********************************************************************************************************
 *                      			Global Functions Prototypes                                          *
 *********************************************************************************************************/

uint8_t check_can_message (void)

/*********************************************************************************************************
 *                      			Global Function Definitions                                          *
 *********************************************************************************************************/

uint8_t check_can_message (void)
{
	uint_8 inside_counter  = 0;
	uint_8 outside_counter = 0;
	unit_8 i;
	
	for (i = 0 ; i < inside_size ; i++)
	{
		if(RxData[i] == unlock[i] )
		{
			inside_counter ++ ;
		}
	}
	
	if (inside_counter == inside_size)
	{
		outside_flag = 0;
		inside_flag  = 1;
		return 1;
	}

	for (i = 0 ; i < outside_size ; i++)
	{
		if(RxData[i] == lock[i] )
		{
			outside_counter ++ ;
		}
	}
	
	if (outside_counter == outside_size)
	{
		inside_flag  = 0;
		outside_flag = 1;
		return 1;
	}
	
	return 0;
}

/*********************************************************************************************************
 *                      				 CAN interrupt Callback                                          *
 *********************************************************************************************************/

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	rxFlag = 1;
}

/*********************************************************************************************************
 *                      					Main Function                                                *
 *********************************************************************************************************/

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();

  HAL_CAN_Start(&hcan);

  /* Activate the notification */
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  
  TxHeader.DLC = 8;  /* data length */
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x103;  /* ID */
  
//  TxData[0] = ; 
//  TxData[1] = ;  


  while (1)
  {
	  if(rxFlag == 1) /* rx interrupt has occured */
	  {
		 rxFlag = 0; /* clear rx flag */
		 check_state();
	  }
	  
	  if((push_button == pressed) && inside_flag ==1)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	  }
  }
}


/*********************************************************************************************************
 *                      			Private Function Definitions                                         *
 *********************************************************************************************************/
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;  /* which filter bank to use from the assigned ones */
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;  /* doesn't matter in single can controllers */

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
