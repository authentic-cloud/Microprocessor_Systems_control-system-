/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2c.h"
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_DUTY 9900
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t count;
struct lcd_disp disp;

float U_tmp;
float U, Y, E;



char buff[16];
int sampling_freq = 10;
float total_time;


float k_E=20;
float k_I=3;
float k_D = 1000;

int timer_val;
int time_elapsed;

float frequency;
float old_frequency;
float ratio_frequency=3;

float out_frequency; //output

float rot_freq;
float Duty_test, Duty_test_P, Duty_test_I,Duty_test_D;
float prev_E, prev_I;
float I;
float D;

int new_duty;

char msg_[32];
int status_len;

uint32_t encoder_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*****************************************************************************************************
/* @brief configures printf for debugging
/* @author  Wojciech Piersiala
/* @param[in] file
/* @param[in] *ptr
/* @param[in] len
/* @return len
/* @version V1.0
/* @date    17-Feb-2022

/*****************************************************************************************************/
int _write(int file, char *ptr, int len)
{
	int i=0;
	for(i=0 ; i<len ; i++)
		ITM_SendChar((*ptr++));
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
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM2_Init();
	MX_TIM5_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	//usart
	HAL_UART_Receive_IT(&huart3, &buff,3);

	// frequency measurement
	HAL_TIM_Base_Start(&htim2);

	// H bridge
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,50);
	HAL_GPIO_WritePin(DC1_GPIO_Port, DC1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_SET);

	//encoder
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);


	//sampler

	int new_prescaler = htim5.Init.Prescaler/sampling_freq;
	HAL_TIM_Base_Start_IT(&htim5);
	__HAL_TIM_SET_PRESCALER(&htim5,new_prescaler);


	//LCD

	disp.addr = (0x3F << 1);
	disp.bl = true;
	lcd_init(&disp);


	char LCDdisplay1[17];
	char LCDdisplay2[17];


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		sprintf(LCDdisplay1, "U: %d rpm",(int)(U*60));
		sprintf(LCDdisplay2, "Y: %d rpm",(int)(Y*60));

		sprintf((char *)disp.f_line, LCDdisplay1);
		sprintf((char *)disp.s_line, LCDdisplay2);

		lcd_display(&disp);



		HAL_Delay(500);
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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
			|RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/*****************************************************************************************************
/* @brief  GPIO external interupr callback (Implements interaface with IR sensor and polarity change using USER_Btn)
/* @author  Wojciech Piersiala
/* @param[in] GPIO_Pin GPIO Pin handler
/* @return None
/* @version V1.0
/* @date    17-Feb-2022

/*****************************************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == USER_Btn_Pin){
		HAL_GPIO_TogglePin(DC1_GPIO_Port, DC1_Pin);
		HAL_GPIO_TogglePin(DC2_GPIO_Port, DC2_Pin);

	}
	if(GPIO_Pin == Sensor_Pin){
		timer_val=__HAL_TIM_GET_COUNTER(&htim2);

		if(timer_val>15000){
			time_elapsed=timer_val;
		}

		frequency = 1/(float)time_elapsed*1000000;
		__HAL_TIM_SET_COUNTER(&htim2,0);

		if(frequency>2){
			ratio_frequency=(frequency/old_frequency);
			old_frequency = frequency;
		}

		if(ratio_frequency<1.3){
			out_frequency = frequency;
		}
	}
}




/*****************************************************************************************************
 @brief  Timer IC callback(implements interface with rotary encoder)
 @author  Wojciech Piersiala
 @param[in] htim TIM handler
 @return None/* @version V1.0
 @date    17-Feb-2022

 *****************************************************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	encoder_counter=__HAL_TIM_GET_COUNTER(htim);
	count=((int16_t)encoder_counter)/4;


	if (count>30){
		count=30;
		encoder_counter=(uint32_t)(4*count);
		__HAL_TIM_SET_COUNTER(htim,encoder_counter);
	}else if(count<-30){
		count=-30;
		encoder_counter=(0xFFFFFFFF+4*count);
		__HAL_TIM_SET_COUNTER(htim,encoder_counter);

	}


}
/*****************************************************************************************************
 @brief   UART Callback(Serail communication interface to read and write the values)
 @author  Bhargav Malasani,Wojciech Piersiala
 @param[in] huart huart handler
 @return None
 @version V2.0
 @date    20-Feb-2022

/*****************************************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3){
		HAL_UART_Receive_IT(&huart3, buff, 3);
		char inx0 =buff[0];

		switch(inx0){
		case ('S'):{
			int val0, val1, new_duty;
			val0 = (int)(buff[1]-'0');
			val1 = (int)(buff[2]-'0');
			new_duty=(val0*10+val1)*100;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,new_duty);

			break;
		}
		case ('U'):{
			int val0, val1, new_duty;
			val0 = (int)(buff[1]-'0');
			val1 = (int)(buff[2]-'0');
			U_tmp=val0*10+val1;
			count=0;   // count test
			break;
		}
		case ('R'):{
			char USARTdisp3[17];
			int len2;
			if(buff[2]=='Y'){

			len2=sprintf(USARTdisp3, "Y: %d rpm \n\r",(int)(Y*60));}

			if(buff[2]=='U'){

						len2=sprintf(USARTdisp3, "U: %d rpm \n\r",(int)(U*60));}

			if(buff[2]=='E'){

						len2=sprintf(USARTdisp3, "E: %d rpm \n\r",(int)((E)*60));}

			HAL_UART_Transmit(&huart3, &USARTdisp3, len2, 100);
			break;
		}
	}
}
}





/*****************************************************************************************************
 @brief  Time Callback (implements PID controll)
 @author  Wojciech Piersiala
 @param[in] htim tim handler
 @return None
 @version V1.0
 @date    18-Feb-2022

/**************************************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM5){
		rot_freq=out_frequency;
		total_time+=1/((float)sampling_freq);
		Y=rot_freq;
		U=U_tmp+count;
		if(U<0){
			U=0;
			count=0;
		}
		E=U-Y;

		//I
		I = prev_I+ E+prev_E;
		Duty_test_I=I*k_I;
		if(Duty_test_I<1){
			Duty_test_I=1;
		}
		else
			if(Duty_test_I>MAX_DUTY){
				Duty_test_I=MAX_DUTY;
				I=MAX_DUTY/k_I;
			}

		//P
		Duty_test_P=E*k_E;
		if(Duty_test_P<1){
			Duty_test_P=1;
		}
		else
			if(Duty_test_P>MAX_DUTY){
				Duty_test_P=MAX_DUTY;
			}

		//D
		D = (E-prev_E);
		Duty_test_D=k_D*D;
		if(Duty_test_D<-900){
			Duty_test_D=-900;
		}
		else
			if(Duty_test_D>MAX_DUTY){
				Duty_test_D=MAX_DUTY;
			}

		Duty_test=(int)(Duty_test_P+Duty_test_I+Duty_test_D);

		prev_E=E;
		prev_I=I;

		if(Duty_test<1){
			Duty_test=1;
		}
		else
			if(Duty_test>MAX_DUTY){
				Duty_test=MAX_DUTY;
			}

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,Duty_test);
	}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
