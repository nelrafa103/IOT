/* USER CODE BEGIN Header */
#define SIZE 36

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include "FreeRTOS.h"
// #include <stdio.h>

void multiplicarMatrices(int a[SIZE][SIZE], int b[SIZE][SIZE], int resultado[SIZE][SIZE]) {
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            resultado[i][j] = 0;
            for (int k = 0; k < SIZE; k++) {
                resultado[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}


void imprimirMatriz(int matriz[SIZE][SIZE]) {
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
         //   printf("%d ", matriz[i][j]);
        }
       // printf("\n");
    }
}


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// Definición de los pines de los LEDs y botón
#define LED_PIN1 GPIO_PIN_0  // LED 1 (puedes cambiar según el pin que uses)
#define LED_PIN2 GPIO_PIN_1  // LED 2 (puedes cambiar según el pin que uses)
#define BUTTON_PIN GPIO_PIN_13  // Botón de usuario

// Declaración de las funciones de las tareas
void LED_Task(void *pvParameters);
void Button_Task(void *pvParameters);

// Variable para controlar la pausa de la secuencia de LEDs
volatile uint8_t sequencePaused = 0;
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  // Configuración de los pines para LEDs y botón
   __HAL_RCC_GPIOB_CLK_ENABLE();  // Habilita el reloj para los GPIO
   GPIO_InitTypeDef GPIO_InitStruct = {0};

   // Configura los LEDs como salida
   GPIO_InitStruct.Pin = LED_PIN1 | LED_PIN2;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   // Configura el botón de usuario como entrada
   GPIO_InitStruct.Pin = BUTTON_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);



  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    int matriz_a[SIZE][SIZE], matriz_b[SIZE][SIZE], resultado[SIZE][SIZE];


    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            matriz_a[i][j] = i + j;
            matriz_b[i][j] = i - j;
        }
    }


    multiplicarMatrices(matriz_a, matriz_b, resultado);


//    printf("Resultado de la multiplicación de las matrices:\n");
    imprimirMatriz(resultado);
  xTaskCreate(LED_Task, "LED Task", 128, NULL, 1, NULL);
  xTaskCreate(Button_Task, "Button Task", 128, NULL, 1, NULL);
  vTaskStartScheduler();

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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

void LED_Task(void *pvParameters) {
    while (1) {
        if (!sequencePaused) {
            // Secuencia de encendido y apagado de los LEDs
            HAL_GPIO_WritePin(GPIOB, LED_PIN1, GPIO_PIN_SET);  // Encender LED 1
            vTaskDelay(500 / portTICK_PERIOD_MS);              // Esperar 500 ms
            HAL_GPIO_WritePin(GPIOB, LED_PIN1, GPIO_PIN_RESET);  // Apagar LED 1

            HAL_GPIO_WritePin(GPIOB, LED_PIN2, GPIO_PIN_SET);  // Encender LED 2
            vTaskDelay(500 / portTICK_PERIOD_MS);              // Esperar 500 ms
            HAL_GPIO_WritePin(GPIOB, LED_PIN2, GPIO_PIN_RESET);  // Apagar LED 2
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Pequeño retardo antes de volver a verificar
    }
}

// Tarea encargada de monitorear el botón
void Button_Task(void *pvParameters) {
    uint8_t buttonState = 0;
    uint8_t lastButtonState = 0;

    while (1) {
        buttonState = HAL_GPIO_ReadPin(GPIOC, BUTTON_PIN);

        // Si se detecta un cambio de estado en el botón
        if (buttonState == GPIO_PIN_SET && lastButtonState == GPIO_PIN_RESET) {
            sequencePaused = !sequencePaused;  // Cambia el estado de pausa
        }

        lastButtonState = buttonState;
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Evitar rebotes con un retardo
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
