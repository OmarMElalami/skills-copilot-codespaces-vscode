/* USER CODE BEGIN Header */
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BNO055_STM32.h"
#include "Flash.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t OffsetDatas[22];   /*!BNO055 data structure definition which hold euler, quaternion, linearaccel, gyro etc. parameters*/
BNO055_Sensors_t BNO055;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
bool offsetsSaved = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Sensor_Init(void);
void Display_IMU_Data(const BNO055_Sensors_t *sensorData);
//void Calculate_Tilt_Compensation_Display(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}
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
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  printf("Start des Programms.\n");

  // Schritt 1: Sensor initialisieren
  BNO055_Init((BNO055_Init_t){
      .Unit_Sel = UNIT_TEMP_CELCIUS | UNIT_EUL_DEG | UNIT_GYRO_DPS | UNIT_ACC_MS2,
      .Axis = DEFAULT_AXIS_REMAP,
      .Axis_sign = DEFAULT_AXIS_SIGN,
      .Mode = BNO055_NORMAL_MODE,
      .OP_Modes = NDOF,
      .Clock_Source = CLOCK_EXTERNAL,
      .ACC_Range = Range_16G
  });

  // Schritt 2: Sensor zurücksetzen
  Restart_Sensor();

  // Schritt 3: Versuch, Offset-Daten aus dem Flash zu laden
  uint8_t loadedOffsets[22] = {0};
  printf("Offset-Daten laden...\n");
  if (Load_Offsets(loadedOffsets) == HAL_OK) {
      printf("Offset-Daten erfolgreich geladen.\n");
      // Schritt 4: Offset-Daten im Sensor setzen
      setSensorOffsets(loadedOffsets);
      printf("Offset-Daten im Sensor gesetzt.\n");
      offsetsSaved = true; // Da Offset-Daten geladen und gesetzt wurden
  }
  else {
      printf("Fehler beim Laden der Offset-Daten. Kalibrierung erforderlich.\n");
      // Schritt 5: Kalibrierung durchführen
      if (Calibrate_BNO055()) {
          printf("Kalibrierung erfolgreich.\n");
          // Schritt 6: Offset-Daten nach der Kalibrierung auslesen
          getSensorOffsets(OffsetDatas);
          // Schritt 7: Offset-Daten im Flash speichern
          if (Save_Offsets(OffsetDatas) == HAL_OK) {
              printf("Offset-Daten gespeichert.\n");
              offsetsSaved = true;
          }
          else {
              printf("Fehler beim Speichern.\n");
          }
      }
      else {
          printf("Kalibrierung fehlgeschlagen.\n");
      }
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	    // Optional: Check_Status durchführen
	    BNO_Status_t status;
	    Check_Status(&status);
	    printf("Systemstatus: STresult=%d, SYSStatus=%d, SYSError=%d\n", status.STresult, status.SYSStatus, status.SYSError);

	    // Schritt 8: Sensordaten auslesen (Euler, Accel, Gyro, Magneto, Linear Acceleration, Gravity, Quaternion)
	    ReadData(&BNO055, SENSOR_EULER | SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_LINACC | SENSOR_GRAVITY | SENSOR_QUATERNION);

	    // Schritt 9: Sensordaten anzeigen
	    Display_IMU_Data(&BNO055);

	    // Schritt 10: Neigungskompensation berechnen und anzeigen
	    Calculate_Tilt_Compensation_Display();

	    // Schritt 11: Überprüfen, ob der Sensor vollständig kalibriert ist und Offset-Daten speichern, falls noch nicht geschehen
	    if (isFullyCalibrated() && !offsetsSaved) {
	        printf("Sensor ist kalibriert. Speichere Offset-Daten...\n");
	        getSensorOffsets(OffsetDatas); // Offset-Daten auslesen
	        if (Save_Offsets(OffsetDatas) == HAL_OK) {
	            offsetsSaved = true; // Setze das Flag, um eine weitere Speicherung zu verhindern
	            printf("Offset-Daten gespeichert.\n");
	        }
	        else {
	            printf("Fehler beim Speichern der Offset-Daten.\n");
	        }
	    }

	    HAL_Delay(500); // Verzögerung nach Bedarf
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Restarts the BNO055 sensor by performing a software reset.
  * @retval None
  */
void Restart_Sensor(void){
    printf("Software-Reset...\n");
    ResetBNO055();
    printf("Sensor Restarted.\n");
}

/**
  * @brief  Displays the IMU data on the console.
  * @param  sensorData Pointer to the BNO055_Sensors_t structure containing sensor data.
  * @retval None
  */
void Display_IMU_Data(const BNO055_Sensors_t *sensorData) {
    printf("Euler Angles: Pitch=%.2f°, Roll=%.2f°, Heading=%.2f°\n",
           sensorData->Euler.X, sensorData->Euler.Y, sensorData->Euler.Z);
    printf("Accelerometer: X=%.2f m/s², Y=%.2f m/s², Z=%.2f m/s²\n",
           sensorData->Accel.X, sensorData->Accel.Y, sensorData->Accel.Z);
    printf("Gyroscope: X=%.2f dps, Y=%.2f dps, Z=%.2f dps\n",
           sensorData->Gyro.X, sensorData->Gyro.Y, sensorData->Gyro.Z);
    printf("Magnetometer: X=%.2f uT, Y=%.2f uT, Z=%.2f uT\n",
           sensorData->Magneto.X, sensorData->Magneto.Y, sensorData->Magneto.Z);
    printf("Linear Acceleration: X=%.2f m/s², Y=%.2f m/s², Z=%.2f m/s²\n",
           sensorData->LineerAcc.X, sensorData->LineerAcc.Y, sensorData->LineerAcc.Z);
    printf("Gravity: X=%.2f m/s², Y=%.2f m/s², Z=%.2f m/s²\n",
           sensorData->Gravity.X, sensorData->Gravity.Y, sensorData->Gravity.Z);
    printf("Quaternion: W=%.4f, X=%.4f, Y=%.4f, Z=%.4f\n",
           sensorData->Quaternion.W, sensorData->Quaternion.X, sensorData->Quaternion.Y, sensorData->Quaternion.Z);
    printf("------------------------------------------------------------\n");
}

/**
  * @brief  Berechnet und gibt die Neigungskompensation aus.
  * @retval None
  */
void Calculate_Tilt_Compensation_Display(void) {
    float pitch, roll;
    // Berechnung von Pitch und Roll basierend auf Accelerometer-Daten
    pitch = atan2(BNO055.Accel.Y, sqrt(pow(BNO055.Accel.X, 2) + pow(BNO055.Accel.Z, 2))) * (180.0 / M_PI);
    roll = atan2(BNO055.Accel.X, sqrt(pow(BNO055.Accel.Y, 2) + pow(BNO055.Accel.Z, 2))) * (180.0 / M_PI);

    // Beispiel: Neigungskompensation (Anpassung basierend auf der Länge des Stabes)
    float pole_length = 2.0f; // Länge des Stabes in Metern
    float dx = pole_length * tan(pitch * (M_PI / 180.0f));
    float dy = pole_length * tan(roll * (M_PI / 180.0f));

    printf("Pitch: %.2f°, Roll: %.2f°\n", pitch, roll);
    printf("Neigungskompensation: dx=%.2f m, dy=%.2f m\n", dx, dy);
    printf("------------------------------------------------------------\n");
}
/* USER CODE END 4 */

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
