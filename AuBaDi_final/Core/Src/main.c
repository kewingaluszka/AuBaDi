/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "audioI2S.h"
#include "MY_CS43L22.h"
#include "wav_player.h"
#include "lcd16x2_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern ApplicationTypeDef Appli_state;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEPS 1970
#define WAV_FILE_menu "audio/wiosna.wav"
#define WAV_FILE_1 "audio/house.wav"
#define WAV_FILE_2 "audio/wiosna.wav"
#define WAV_FILE_3 "audio/house.wav"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */

volatile int duty = 0;
volatile int confirm = 0;
volatile int selection = 0;
volatile int old_selection = -1;
volatile int audio_flag = 0;
volatile int time_pour = 0;
volatile int flag_endofroad = 0;
volatile int flag_busy = 0;
volatile int last_position = 0;
volatile int step = 2;
volatile int step_beg;
volatile int iter = 5;
volatile int recipe = 0;
volatile int in_progress = 0;
volatile int pour_position = 0;
volatile int go_to_pos;
volatile int time_recipe;

char queue[] = { 0, 5, 4, 3, 2, 1 };
char recipe_1[] = { 3000, 2000, 4000, 1000, 2000 };
char recipe_2[] = { 1000, 2000, 3000, 4000, 5000 };
char recipe_3[] = { 5000, 4000, 3000, 2000, 1000 };



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM3_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim10) {
		if (duty < 1000) {
			duty++;
		} else
			duty = 0;
	}
	if (htim == &htim4) {
		HAL_GPIO_TogglePin(step_GPIO_Port, step_Pin);
		//  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		step--;
		step_beg--;
		in_progress = 1;

		if (step == 0) {
			HAL_TIM_Base_Stop_IT(&htim4);
			//HAL_TIM_Base_Start_IT(&htim3);
			flag_busy = 0;
			//          flag_busy = 0;
			//iter--;
			in_progress = 0;
		}
		if (step_beg == 0) {
			HAL_TIM_Base_Stop_IT(&htim4);
			HAL_TIM_Base_Start_IT(&htim3);
			//flag_busy = 0;
			//flag_busy = 0;
			iter--;
			//in_progress = 0;
		}
		if (iter < 0) {
			iter = 5;
			confirm = 0;
			wake(0); // check
			in_progress = 0;
		}
	}

	if (htim == &htim3) {
		switch (pour_position) {

		case 1:
			HAL_GPIO_WritePin(pump_relay1_GPIO_Port, pump_relay1_Pin, 0);
			break;

		case 2:
			HAL_GPIO_WritePin(pump_relay2_GPIO_Port, pump_relay2_Pin, 0);
			break;
		case 3:
			HAL_GPIO_WritePin(pump_relay3_GPIO_Port, pump_relay3_Pin, 0);
			break;
		case 4:
			HAL_GPIO_WritePin(pump_relay4_GPIO_Port, pump_relay4_Pin, 0);
			break;

		case 5:
			HAL_GPIO_WritePin(pump_relay5_GPIO_Port, pump_relay5_Pin, 0);
			break;

		}

		if (time_pour > 0) {
			time_pour--;
		} else {
			time_pour = 0;

			HAL_GPIO_WritePin(pump_relay1_GPIO_Port, pump_relay1_Pin, 1);
			HAL_GPIO_WritePin(pump_relay2_GPIO_Port, pump_relay2_Pin, 1);
			HAL_GPIO_WritePin(pump_relay3_GPIO_Port, pump_relay3_Pin, 1);
			HAL_GPIO_WritePin(pump_relay4_GPIO_Port, pump_relay4_Pin, 1);
			HAL_GPIO_WritePin(pump_relay5_GPIO_Port, pump_relay5_Pin, 1);
			HAL_TIM_Base_Stop_IT(&htim3);
			flag_busy = 0;

		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//  left button
	if (GPIO_Pin == button_left_Pin) {
		//confirm = 0;
		selection--;
	}
	//  right button
	if (GPIO_Pin == button_right_Pin) {
		//confirm = 0;
		selection++;
	}
	//  menu button
	if (GPIO_Pin == button_select_Pin) {
		confirm = 1;
	}
	// endstop
	if (GPIO_Pin == endstop_Pin)
		flag_endofroad = 1;
	else
		flag_endofroad = 0;
}

void wake(int flag) {
	if (flag == 1) {
		HAL_GPIO_WritePin(slp_rst_GPIO_Port, slp_rst_Pin, 1);
	} else {
		HAL_GPIO_WritePin(slp_rst_GPIO_Port, slp_rst_Pin, 0);
	}
}

void move(int direction, int steps) {
	step = steps;
	switch (direction) {
	//move right
	case 0:
		HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, 0);
		wake(1);
		HAL_TIM_Base_Start_IT(&htim4);
		break;
		//move left
	case 1:
		HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, 1);
		wake(1);
		HAL_TIM_Base_Start_IT(&htim4);
		break;
	}

}

void move_begin() {
	wake(1);
	while (flag_endofroad == 0) {
		move(0, 1);
	}
	flag_endofroad = 0;
	wake(0);
}

void setPosition(int position, int time) {
	pour_position = position;
	flag_busy = 1;
	time_pour = time;
	int target;
	flag_endofroad = 0;

	if (position >= 0) {

		if (last_position > position) {
			wake(1);
			target = STEPS / 2 * abs((position - last_position));
			move(0, target + 10);

		}

		else if (last_position < position) {
			wake(1);
			target = STEPS / 2 * abs((last_position - position));
			move(1, target + 10);

		}

		step_beg = target;
	}
	last_position = position;

}

void pumpInit() {
	HAL_GPIO_WritePin(pump_relay1_GPIO_Port, pump_relay1_Pin, 1);
	HAL_GPIO_WritePin(pump_relay2_GPIO_Port, pump_relay2_Pin, 1);
	HAL_GPIO_WritePin(pump_relay3_GPIO_Port, pump_relay3_Pin, 1);
	HAL_GPIO_WritePin(pump_relay4_GPIO_Port, pump_relay4_Pin, 1);
	HAL_GPIO_WritePin(pump_relay5_GPIO_Port, pump_relay5_Pin, 1);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_I2S3_Init();
	MX_USB_HOST_Init();
	MX_FATFS_Init();
	MX_I2C2_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM10_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	/******************************** MENU INIT ************************************/
	pumpInit();

	/****************************** DAC AUDIO INIT ******************************/
	CS43_Init(hi2c1, MODE_I2S);
	CS43_SetVolume(200); //0-255
	CS43_Enable_RightLeft(CS43_RIGHT_LEFT);
	audioI2S_setHandle(&hi2s3);
	bool isSdCardMounted = 0;

	/****************************** LCD INIT ************************************/
	lcd16x2_i2c_init(&hi2c2);
	lcd16x2_i2c_1stLine();
	lcd16x2_i2c_printf("Barman Dionizos");
	lcd16x2_i2c_2ndLine();
	lcd16x2_i2c_printf(" KALIBRACJA ");

	/****************************** PWM LIGHTNING INIT **************************/
	HAL_TIM_Base_Start_IT(&htim10);
	//blue LED
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	//green LED
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	//red LED
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

	/****************************** MOTOR INIT ***********************************/
	move_begin();
	wake(1);

	/******************************** MENU INIT ************************************/
	iter = 5;
	in_progress = 0;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/*********************************** MENU ************************************/

		if (selection != old_selection) {
			if (selection <= 3 && selection >= 0) {
				switch (selection) {

				case 0:
					lcd16x2_i2c_clear();
					lcd16x2_i2c_1stLine();
					lcd16x2_i2c_printf("Barman Dionizos");
					lcd16x2_i2c_2ndLine();
					lcd16x2_i2c_printf("<-WYBOR NAPOJU->");
					audio_flag = 4;
					recipe = 0;
					confirm = 0;
					break;

				case 1:
					lcd16x2_i2c_clear();
					lcd16x2_i2c_1stLine();
					lcd16x2_i2c_printf("Barman Dionizos");
					lcd16x2_i2c_2ndLine();
					lcd16x2_i2c_printf("<-   NAPOJ 1  ->");
					audio_flag = 1;
					recipe = 1;
					break;

				case 2:
					lcd16x2_i2c_clear();
					lcd16x2_i2c_1stLine();
					lcd16x2_i2c_printf("Barman Dionizos");
					lcd16x2_i2c_2ndLine();
					lcd16x2_i2c_printf("<-   NAPOJ 2  ->");
					audio_flag = 2;
					recipe = 2;
					break;

				case 3:
					lcd16x2_i2c_clear();
					lcd16x2_i2c_1stLine();
					lcd16x2_i2c_printf("Barman Dionizos");
					lcd16x2_i2c_2ndLine();
					lcd16x2_i2c_printf("<-   NAPOJ 3  ->");
					audio_flag = 3;
					recipe = 3;
					break;
				}
				old_selection = selection;
			}

			else if (selection < 0) {
				selection = 3;
			} else if (selection > 3) {
				selection = 0;
			}

		}

		/*********************************** LED PWM  ************************************/
		TIM5->CCR2 = duty;
		TIM5->CCR3 = 1000;
		TIM5->CCR4 = (1000 - duty);

		/*********************************** AUDIO ****************************************/

		if (Appli_state == APPLICATION_START) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		}

		else if (Appli_state == APPLICATION_DISCONNECT) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

		}

		if (Appli_state == APPLICATION_READY) {
			if (!isSdCardMounted) {
				f_mount(&USBHFatFS, (const TCHAR*) USBHPath, 0);
				isSdCardMounted = 1;
			}

			if (audio_flag == 1) {
				if (!wavPlayer_isFinished()) {
					wavPlayer_stop();
				} else {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
					HAL_Delay(500);
					wavPlayer_fileSelect(WAV_FILE_1);
					wavPlayer_play();
					audio_flag = 0;
				}
			}

			else if (audio_flag == 2) {
				if (!wavPlayer_isFinished()) {
					wavPlayer_stop();
				} else {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
					HAL_Delay(500);
					wavPlayer_fileSelect(WAV_FILE_2);
					wavPlayer_play();
					audio_flag = 0;
				}
			}

			else if (audio_flag == 3) {
				if (!wavPlayer_isFinished()) {
					wavPlayer_stop();
				} else {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
					HAL_Delay(500);
					wavPlayer_fileSelect(WAV_FILE_3);
					wavPlayer_play();
					audio_flag = 0;
				}
			}

			else if (audio_flag == 4) {
				if (!wavPlayer_isFinished()) {
					wavPlayer_stop();
				} else {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
					HAL_Delay(500);
					wavPlayer_fileSelect(WAV_FILE_menu);
					wavPlayer_play();
					audio_flag = 0;
				}
			}

		}

		wavPlayer_process();

		/*********************************** MOTOR & PUMPS  ************************************/
		if (confirm == 1) {
			switch (selection) {
			case 1:
				if (flag_busy == 0) {
					if (iter >= 0) {
						go_to_pos = queue[iter];
						time_recipe = recipe_1[iter];
					}
				} else {
					go_to_pos = 0;
					time_recipe = 0;
				}

				setPosition(go_to_pos, time_recipe);

				break;

			case 2:
				if (flag_busy == 0) {
					if (iter >= 0) {
						go_to_pos = queue[iter];
						time_recipe = recipe_2[iter];
					}
				} else {
					go_to_pos = 0;
					time_recipe = 0;
				}

				setPosition(go_to_pos, time_recipe);

				break;

			case 3:
				if (flag_busy == 0) {
					if (iter >= 0) {
						go_to_pos = queue[iter];
						time_recipe = recipe_3[iter];
					}
				} else {
					go_to_pos = 0;
					time_recipe = 0;
				}

				setPosition(go_to_pos, time_recipe);

				break;
			}
		}

		/* USER CODE END WHILE */
		MX_USB_HOST_Process();

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 288;
	PeriphClkInitStruct.PLLI2S.PLLI2SM = 8;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void) {

	/* USER CODE BEGIN I2S3_Init 0 */

	/* USER CODE END I2S3_Init 0 */

	/* USER CODE BEGIN I2S3_Init 1 */

	/* USER CODE END I2S3_Init 1 */
	hi2s3.Instance = SPI3;
	hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
	hi2s3.Init.CPOL = I2S_CPOL_LOW;
	hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2S3_Init 2 */

	/* USER CODE END I2S3_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = TIM3_PRESCALER;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = TIM3_PERIOD;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = TIM4_PRESCALER;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = TIM4_PERIOD;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = TIM5_PRESCALER;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = TIM5_PERIOD;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */
	HAL_TIM_MspPostInit(&htim5);

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = TIM10_PRESCALER;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = TIM10_PERIOD;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, USB_power_Pin | dir_Pin | step_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
			pump_relay1_Pin | pump_relay2_Pin | pump_relay3_Pin
					| pump_relay4_Pin | pump_relay5_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | Audio_RST_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(slp_rst_GPIO_Port, slp_rst_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : button_left_Pin button_select_Pin button_right_Pin */
	GPIO_InitStruct.Pin =
	button_left_Pin | button_select_Pin | button_right_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_power_Pin dir_Pin step_Pin */
	GPIO_InitStruct.Pin = USB_power_Pin | dir_Pin | step_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : pump_relay1_Pin pump_relay2_Pin pump_relay3_Pin pump_relay4_Pin
	 pump_relay5_Pin */
	GPIO_InitStruct.Pin = pump_relay1_Pin | pump_relay2_Pin | pump_relay3_Pin
			| pump_relay4_Pin | pump_relay5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : endstop_Pin */
	GPIO_InitStruct.Pin = endstop_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(endstop_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PD12 PD13 PD14 PD15
	 Audio_RST_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15
			| Audio_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : slp_rst_Pin */
	GPIO_InitStruct.Pin = slp_rst_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(slp_rst_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
