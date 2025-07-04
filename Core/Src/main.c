/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "memorymap.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // Necessário para printf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Tempo de pisca-pisca em milissegundos (meio segundo aceso, meio segundo apagado)
#define BLINK_DELAY_MS 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
 uint8_t rx_data;
 // Variável de estado para controlar o modo do LED
 volatile uint8_t led_mode = 0; // 0: off, 1: on, 2: blinking
 uint32_t last_blink_time = 0; // Para controle de tempo do pisca-pisca sem bloquear
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Display_Menu(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Opcional: Acenda ou apague o LED no início para indicar estado inicial
  HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_RESET); // LED começa apagado
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    //Display_Menu(); // Exibe o menu a cada iteração do loop principal

	    // Tenta receber entrada do usuário sem bloquear (timeout 100ms)
	    if (HAL_UART_Receive(&huart3, &rx_data, 1, 100) == HAL_OK)
	    {
	      switch (rx_data)
	      {
	        case '1':
	          printf(">> Opcao 1: LED ACESO!\r\n");
	          HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_SET); // Acende o LED
	          led_mode = 1; // Define o modo LED ON
	          break;

	        case '2':
	          printf(">> Opcao 2: LED APAGADO!\r\n");
	          HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_RESET); // Apaga o LED
	          led_mode = 0; // Define o modo LED OFF
	          break;

	        case '3':
	          printf(">> Opcao 3: LED PISCANDO CONTINUAMENTE! Pressione outra tecla para parar.\r\n");
	          led_mode = 2; // Define o modo LED PISCANDO
	          last_blink_time = HAL_GetTick(); // Reseta o timer do pisca-pisca
	          break;

	        case '4':
	          printf(">> Incremento de 10%% por botao. (Funcionalidade a implementar)\r\n");
	          // Certifica-se de que o LED não esteja piscando ou aceso/apagado por outras opções
	          HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_RESET);
	          led_mode = 0;
	          break;

	        default:
	          printf(">> Opcao invalida. Tente novamente.\r\n");
	          // Se uma opção inválida for digitada, certifique-se de que o LED não fique preso em um estado de piscar
	          if (led_mode == 2) {
	             HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_RESET); // Apaga o LED
	             led_mode = 0; // Sai do modo pisca-pisca
	             printf(">> Modo pisca-pisca interrompido por opcao invalida.\r\n");
	          }
	          break;
	      }
	    }

        // Lógica para piscar o LED se o modo for 2
        if (led_mode == 2) {
            if (HAL_GetTick() - last_blink_time >= BLINK_DELAY_MS) {
                HAL_GPIO_TogglePin(ld1_GPIO_Port, ld1_Pin); // Inverte o estado do LED
                last_blink_time = HAL_GetTick();            // Atualiza o tempo da última piscada
            }
        }
        // O HAL_Delay(500) aqui pode ser removido, pois o HAL_UART_Receive já tem um timeout.
        // Se você mantiver um delay aqui, ele pode afetar a responsividade do sistema.
        // Se desejar um delay geral para exibir o menu com menos frequência, pode usar um timer.
        // HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1; // <<-- Linha corrigida aqui
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief Exibe o menu de opções via UART.
  * @retval None
  */
void Display_Menu(void) {
    printf("\r\n"); // Nova linha para melhor formatação
    printf("===== CONTROLE DE PWM E LED =====\r\n");
    printf("[1] Acender LED Verde (PA5)\r\n");
    printf("[2] Apagar LED Verde (PA5)\r\n");
    printf("[3] Piscar LED Verde Continuamente (PA5)\r\n"); // Texto atualizado
    printf("[4] +10%% por clique no botao (A implementar)\r\n");
    printf("Selecione a opcao: \r\n");
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
