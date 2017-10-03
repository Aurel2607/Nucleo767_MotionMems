/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f7xx_hal.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "mainMotionSensors.h"
#include <math.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint8_t verbose              = 0;  /* Verbose output to UART terminal ON/OFF. */

#define MAX_BUF_SIZE 256

typedef struct displayFloatToInt_s {
	int8_t sign; /* 0 means positive, 1 means negative*/
	uint32_t  out_int;
	uint32_t  out_dec;
} displayFloatToInt_t;

uint16_t cmpt = 0;

static void *LSM6DSL_X_0_handle = NULL;
static void *LSM6DSL_G_0_handle = NULL;
static void *LSM303AGR_X_0_handle = NULL;
static void *LSM303AGR_M_0_handle = NULL;
static void *HTS221_H_0_handle  = NULL;
static void *HTS221_T_0_handle  = NULL;
static void *LPS22HB_P_0_handle  = NULL;
static void *LPS22HB_T_0_handle  = NULL;

static RTC_HandleTypeDef RtcHandle;

static char dataOut[MAX_BUF_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void Accelero_Sensor_Handler( void *handle );
static void Gyro_Sensor_Handler( void *handle );
static void Magneto_Sensor_Handler( void *handle );
static void Humidity_Sensor_Handler( void *handle );
static void Temperature_Sensor_Handler( void *handle );
static void Pressure_Sensor_Handler( void *handle );

static void RTC_TimeStampConfig( void );
static void initializeAllSensors( void );
static void enableAllSensors( void );
static void disableAllSensors( void );
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);
static void RTC_Handler( void );

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_value the pointer to the output integer structure
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
{
	if(in >= 0.0f)
	{
		out_value->sign = 0;
	}else
	{
		out_value->sign = 1;
		in = -in;
	}

	out_value->out_int = (int32_t)in;
	in = in - (float)(out_value->out_int);
	out_value->out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}


/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_USART3_UART_Init();
	MX_RTC_Init();

	/* USER CODE BEGIN 2 */
	RTC_TimeStampConfig();
	initializeAllSensors();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		enableAllSensors();
		printf("\r\n\r\nMain While %d\r\n", cmpt++);

		Accelero_Sensor_Handler( LSM6DSL_X_0_handle );
		Gyro_Sensor_Handler( LSM6DSL_G_0_handle );
		Accelero_Sensor_Handler( LSM303AGR_X_0_handle );
		Magneto_Sensor_Handler( LSM303AGR_M_0_handle );
		Humidity_Sensor_Handler( HTS221_H_0_handle );
		Temperature_Sensor_Handler( HTS221_T_0_handle );
		Temperature_Sensor_Handler( LPS22HB_T_0_handle );
		Pressure_Sensor_Handler( LPS22HB_P_0_handle );

		HAL_Delay(1000);
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART3
			|RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/* USER CODE BEGIN 4 */

/**
 * @brief  Handles the accelerometer axes data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Accelero_Sensor_Handler( void *handle )
{

	uint8_t who_am_i;
	float odr;
	float fullScale;
	uint8_t id;
	SensorAxes_t acceleration;
	uint8_t status;
	displayFloatToInt_t out_value;

	BSP_ACCELERO_Get_Instance( handle, &id );

	BSP_ACCELERO_IsInitialized( handle, &status );

	if ( status == 1 )
	{
		if ( BSP_ACCELERO_Get_Axes( handle, &acceleration ) == COMPONENT_ERROR )
		{
			acceleration.AXIS_X = 0;
			acceleration.AXIS_Y = 0;
			acceleration.AXIS_Z = 0;
		}

		snprintf( dataOut, MAX_BUF_SIZE, "ACC_X[%d]: %d, ACC_Y[%d]: %d, ACC_Z[%d]: %d\r\n", (int)id, (int)acceleration.AXIS_X, (int)id,
				(int)acceleration.AXIS_Y, (int)id, (int)acceleration.AXIS_Z );

		printf("%s", ( uint8_t * )dataOut);

		if ( verbose == 1 )
		{
			if ( BSP_ACCELERO_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id );
			}
			else
			{
				snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
			}
			printf("%s", ( uint8_t * )dataOut);

			if ( BSP_ACCELERO_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
			}
			else
			{
				floatToInt( odr, &out_value, 3 );
				snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
			}
			printf("%s", ( uint8_t * )dataOut);

			if ( BSP_ACCELERO_Get_FS( handle, &fullScale ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", id );
			}
			else
			{
				floatToInt( fullScale, &out_value, 3 );
				snprintf( dataOut, MAX_BUF_SIZE, "FS[%d]: %d.%03d g\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
			}
			printf("%s", ( uint8_t * )dataOut);
		}
	}
}



/**
 * @brief  Handles the gyroscope axes data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Gyro_Sensor_Handler( void *handle )
{

	uint8_t who_am_i;
	float odr;
	float fullScale;
	uint8_t id;
	SensorAxes_t angular_velocity;
	uint8_t status;
	displayFloatToInt_t out_value;

	BSP_GYRO_Get_Instance( handle, &id );

	BSP_GYRO_IsInitialized( handle, &status );

	if ( status == 1 )
	{
		if ( BSP_GYRO_Get_Axes( handle, &angular_velocity ) == COMPONENT_ERROR )
		{
			angular_velocity.AXIS_X = 0;
			angular_velocity.AXIS_Y = 0;
			angular_velocity.AXIS_Z = 0;
		}

		snprintf( dataOut, MAX_BUF_SIZE, "GYR_X[%d]: %d, GYR_Y[%d]: %d, GYR_Z[%d]: %d\r\n", (int)id, (int)angular_velocity.AXIS_X, (int)id,
				(int)angular_velocity.AXIS_Y, (int)id, (int)angular_velocity.AXIS_Z );
		printf("%s", ( uint8_t * )dataOut);

		if ( verbose == 1 )
		{
			if ( BSP_GYRO_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id );
			}
			else
			{
				snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
			}
			printf("%s", ( uint8_t * )dataOut);

			if ( BSP_GYRO_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
			}
			else
			{
				floatToInt( odr, &out_value, 3 );
				snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
			}
			printf("%s", ( uint8_t * )dataOut);

			if ( BSP_GYRO_Get_FS( handle, &fullScale ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", id );
			}
			else
			{
				floatToInt( fullScale, &out_value, 3 );
				snprintf( dataOut, MAX_BUF_SIZE, "FS[%d]: %d.%03d dps\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
			}
			printf("%s", ( uint8_t * )dataOut);
		}
	}
}



/**
 * @brief  Handles the magneto axes data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Magneto_Sensor_Handler( void *handle )
{

	uint8_t who_am_i;
	float odr;
	float fullScale;
	uint8_t id;
	SensorAxes_t magnetic_field;
	uint8_t status;
	displayFloatToInt_t out_value;

	BSP_MAGNETO_Get_Instance( handle, &id );

	BSP_MAGNETO_IsInitialized( handle, &status );

	if ( status == 1 )
	{
		if ( BSP_MAGNETO_Get_Axes( handle, &magnetic_field ) == COMPONENT_ERROR )
		{
			magnetic_field.AXIS_X = 0;
			magnetic_field.AXIS_Y = 0;
			magnetic_field.AXIS_Z = 0;
		}

		snprintf( dataOut, MAX_BUF_SIZE, "MAG_X[%d]: %d, MAG_Y[%d]: %d, MAG_Z[%d]: %d\r\n", (int)id, (int)magnetic_field.AXIS_X, (int)id,
				(int)magnetic_field.AXIS_Y, (int)id, (int)magnetic_field.AXIS_Z );
		printf("%s", ( uint8_t * )dataOut);

		if ( verbose == 1 )
		{
			if ( BSP_MAGNETO_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id );
			}
			else
			{
				snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
			}
			printf("%s", ( uint8_t * )dataOut);

			if ( BSP_MAGNETO_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
			}
			else
			{
				floatToInt( odr, &out_value, 3 );
				snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
			}
			printf("%s", ( uint8_t * )dataOut);

			if ( BSP_MAGNETO_Get_FS( handle, &fullScale ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", id );
			}
			else
			{
				floatToInt( fullScale, &out_value, 3 );
				snprintf( dataOut, MAX_BUF_SIZE, "FS[%d]: %d.%03d Gauss\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
			}
			printf("%s", ( uint8_t * )dataOut);
		}
	}
}



/**
 * @brief  Handles the humidity data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Humidity_Sensor_Handler( void *handle )
{

	uint8_t who_am_i;
	float odr;
	uint8_t id;
	float humidity;
	uint8_t status;
	displayFloatToInt_t out_value;

	BSP_HUMIDITY_Get_Instance( handle, &id );

	BSP_HUMIDITY_IsInitialized( handle, &status );

	if ( status == 1 )
	{
		if ( BSP_HUMIDITY_Get_Hum( handle, &humidity ) == COMPONENT_ERROR )
		{
			humidity = 0.0f;
		}

		floatToInt( humidity, &out_value, 2 );
		snprintf( dataOut, MAX_BUF_SIZE, "HUM[%d]: %d.%02d\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
		printf("%s", ( uint8_t * )dataOut);

		if ( verbose == 1 )
		{
			if ( BSP_HUMIDITY_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id );
			}
			else
			{
				snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
			}
			printf("%s", ( uint8_t * )dataOut);

			if ( BSP_HUMIDITY_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
			}
			else
			{
				floatToInt( odr, &out_value, 3 );
				snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
			}
			printf("%s", ( uint8_t * )dataOut);
		}
	}
}



/**
 * @brief  Handles the temperature data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Temperature_Sensor_Handler( void *handle )
{

	uint8_t who_am_i;
	float odr;
	uint8_t id;
	float temperature;
	uint8_t status;
	displayFloatToInt_t out_value;

	BSP_TEMPERATURE_Get_Instance( handle, &id );

	BSP_TEMPERATURE_IsInitialized( handle, &status );

	if ( status == 1 )
	{
		if ( BSP_TEMPERATURE_Get_Temp( handle, &temperature ) == COMPONENT_ERROR )
		{
			temperature = 0.0f;
		}

		floatToInt( temperature, &out_value, 2 );
		snprintf( dataOut, MAX_BUF_SIZE, "TEMP[%d]: %c%d.%02d\r\n", (int)id, ((out_value.sign) ? '-' : '+'), (int)out_value.out_int, (int)out_value.out_dec );
		printf("%s", ( uint8_t * )dataOut);

		if ( verbose == 1 )
		{
			if ( BSP_TEMPERATURE_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id );
			}
			else
			{
				snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
			}
			printf("%s", ( uint8_t * )dataOut);

			if ( BSP_TEMPERATURE_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
			}
			else
			{
				floatToInt( odr, &out_value, 3 );
				snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
			}
			printf("%s", ( uint8_t * )dataOut);
		}
	}
}



/**
 * @brief  Handles the pressure sensor data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Pressure_Sensor_Handler( void *handle )
{

	uint8_t who_am_i;
	float odr;
	uint8_t id;
	float pressure;
	uint8_t status;
	displayFloatToInt_t out_value;

	BSP_PRESSURE_Get_Instance( handle, &id );

	BSP_PRESSURE_IsInitialized( handle, &status );

	if( status == 1 )
	{
		if ( BSP_PRESSURE_Get_Press( handle, &pressure ) == COMPONENT_ERROR )
		{
			pressure = 0.0f;
		}

		floatToInt( pressure, &out_value, 2 );
		snprintf(dataOut, MAX_BUF_SIZE, "PRESS[%d]: %d.%02d\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec);
		printf("%s", ( uint8_t * )dataOut);

		if ( verbose == 1 )
		{
			if ( BSP_PRESSURE_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id );
			}
			else
			{
				snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
			}
			printf("%s", ( uint8_t * )dataOut);

			if ( BSP_PRESSURE_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
			{
				snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
			}
			else
			{
				floatToInt( odr, &out_value, 3 );
				snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
			}
			printf("%s", ( uint8_t * )dataOut);
		}
	}
}



/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors( void )
{

	BSP_ACCELERO_Init( LSM6DSL_X_0, &LSM6DSL_X_0_handle );
	BSP_GYRO_Init( LSM6DSL_G_0, &LSM6DSL_G_0_handle );
	BSP_ACCELERO_Init( LSM303AGR_X_0, &LSM303AGR_X_0_handle );
	BSP_MAGNETO_Init( LSM303AGR_M_0, &LSM303AGR_M_0_handle );
	BSP_HUMIDITY_Init( HTS221_H_0, &HTS221_H_0_handle );
	BSP_TEMPERATURE_Init( HTS221_T_0, &HTS221_T_0_handle );
	BSP_TEMPERATURE_Init( LPS22HB_T_0, &LPS22HB_T_0_handle );
	BSP_PRESSURE_Init( LPS22HB_P_0, &LPS22HB_P_0_handle );
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
static void enableAllSensors( void )
{

	BSP_ACCELERO_Sensor_Enable( LSM6DSL_X_0_handle );
	BSP_GYRO_Sensor_Enable( LSM6DSL_G_0_handle );
	BSP_ACCELERO_Sensor_Enable( LSM303AGR_X_0_handle );
	BSP_MAGNETO_Sensor_Enable( LSM303AGR_M_0_handle );
	BSP_HUMIDITY_Sensor_Enable( HTS221_H_0_handle );
	BSP_TEMPERATURE_Sensor_Enable( HTS221_T_0_handle );
	BSP_TEMPERATURE_Sensor_Enable( LPS22HB_T_0_handle );
	BSP_PRESSURE_Sensor_Enable( LPS22HB_P_0_handle );
}



/**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
static void disableAllSensors( void )
{

	BSP_ACCELERO_Sensor_Disable( LSM6DSL_X_0_handle );
	BSP_GYRO_Sensor_Disable( LSM6DSL_G_0_handle );
	BSP_ACCELERO_Sensor_Disable( LSM303AGR_X_0_handle );
	BSP_MAGNETO_Sensor_Disable( LSM303AGR_M_0_handle );
	BSP_HUMIDITY_Sensor_Disable( HTS221_H_0_handle );
	BSP_TEMPERATURE_Sensor_Disable( HTS221_T_0_handle );
	BSP_TEMPERATURE_Sensor_Disable( LPS22HB_T_0_handle );
	BSP_PRESSURE_Sensor_Disable( LPS22HB_P_0_handle );
}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig( void )
{

	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;

	/*##-3- Configure the Date using BCD format ################################*/
	/* Set Date: Monday January 1st 2000 */
	sdatestructure.Year    = 0x00;
	sdatestructure.Month   = RTC_MONTH_JANUARY;
	sdatestructure.Date    = 0x01;
	sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

	if ( HAL_RTC_SetDate( &RtcHandle, &sdatestructure, FORMAT_BCD ) != HAL_OK )
	{

		/* Initialization Error */
		Error_Handler();
	}

	/*##-4- Configure the Time using BCD format#################################*/
	/* Set Time: 00:00:00 */
	stimestructure.Hours          = 0x00;
	stimestructure.Minutes        = 0x00;
	stimestructure.Seconds        = 0x00;
	stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BCD ) != HAL_OK )
	{

		/* Initialization Error */
		Error_Handler();
	}
}



/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
