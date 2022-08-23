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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "lcd1602.h"
#include "control_hardware.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef char *String;
typedef enum { // enum 으로 0부터 5까지 스텟을 정의한다.
	OVER, SAFE, OFF, ONN, AUTO, ON
} gstat;
typedef uint8_t boolean;// boolean 으로 0이면 false, 1이면 true를 정의한다.
typedef enum {//false, true를 정의한다.
	false, true
} _BOOL;
typedef struct {// 스텟을 정의한다.
	boolean over, safe, off, onn, au, on;
} Statflag;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SW1 HAL_GPIO_ReadPin(SW_ON_GPIO_Port, SW_ON_Pin) // SW1을 읽는다.
#define SW2 HAL_GPIO_ReadPin(SW_AUTO_GPIO_Port, SW_LOCK_Pin) // SW2를 읽는다.
#define SW3 HAL_GPIO_ReadPin(SW_AUTO_GPIO_Port, SW_AUTO_Pin) // SW3을 읽는다.
#define SW4 HAL_GPIO_ReadPin(SW_A_GPIO_Port, SW_A_Pin) // SW4을 읽는다.
#define SW5 HAL_GPIO_ReadPin(SW_B_GPIO_Port, SW_B_Pin) // SW5을 읽는다.
#define BUZZER(X) HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, X) // BUZZER를 제어한다.
#define LED(N, X) HAL_GPIO_WritePin(LED##N##_GPIO_Port, LED##N##_Pin, !X) // LED(N)를 제어한다.
#define NOW(X) (HAL_GetTick() - X) // 현재시간에 X 를 뺸 값 을 정의한다.
#define TEMPUP(X) (X == 1 ? 900 : X == 2 ? 800 : X == 3 ? 700 : X == 4 ? 600 : X == 5 ? 500 : X == 6 ? 400 : X == 7 ? 300 : X == 8 ? 200 : X == 9 ? 100 : 0)
#define TEMPDOWN(X) (X < 10 ? 2900 : X >= 300 ? 100 : X >= 200 ? 200 : X >= 100 ? 400 : X >= 40 ? 700 : X >= 20 ? 1100 : X >= 15 ? 1600 : X >= 10 ? 2200 : 0)
#define LEDCLEAR LED(1, false); LED(2, false); LED(3, false); LED(4, false); LED(5, false) // LED를 모두 끈다.
/* USER CODE END PD */`

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim6;
/* USER CODE BEGIN PV */
int temp = 20, fire = 0, fireset = 0, altemp = 20, autemp = 80;// 온도와 불의 세기, 알람온도, 오토모드일때의 최대 온도 설정치를 정의한다.
boolean ledRingFlag = false;// LEDRing을켜고 끄는 역할을 하는 변수이다.
Statflag stat;// 스텟을 정의한다.
gstat gasstat = OFF;// 가스레인지의 상태를 정의한다.
uint32_t led_ring_data[10][12] = { //LEDRing 의 데이터를 배열안에 저장한다 (0xFF 이런식으로도 가능)
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 13, 0, 0,0, 13, 0, 0, 0, 13, 0, 0, 0 },
		{ 76, 0, 0, 0, 76, 0, 0, 0, 76, 0, 0, 0 },
		{ 255, 0, 0, 0, 255, 0, 0, 0,255, 0, 0, 0 },
		{ 255, 0, 13, 0, 255, 0, 13, 0, 255, 0, 13, 0 },
		{ 255, 0, 76, 0, 255, 0, 76, 0, 255, 0, 76, 0 },
		{ 255, 0, 255, 0, 255,0, 255, 0, 255, 0, 255, 0 },
		{ 255, 13, 255, 13, 255, 13, 255,13, 255, 13, 255, 13 },
		{ 255, 76, 255, 76, 255, 76, 255, 76,255, 76, 255, 76 },
		{ 255, 255, 255, 255, 255, 255, 255, 255,255, 255, 255, 255 }
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM6_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void read_adc(uint16_t *cds, uint16_t *vr) { // ADC를 읽어서 cds와 vr에 저장한다.
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1000);
	*cds = HAL_ADC_GetValue(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1000);
	*vr = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
}

void lcd_print() {// LCD에 온도와 불의 세기를 출력한다.
	String statfont[6] = { "OVER HEAT", "SAFE LOCK", "OFF      ", "ON(NONE) ",
		"AUTO ADJ ", "ON       " };
	String bf = (char *)malloc(sizeof(char) * 16);
	if (stat.over)
		gasstat = OVER;
	else if (stat.safe)
		gasstat = SAFE;
	else if (stat.off)
		gasstat = OFF;
	else if (stat.onn)
		gasstat = ONN;
	else if (stat.au)
		gasstat = AUTO;
	else if (stat.on)
		gasstat = ON;
	sprintf(bf, "TEMP:%03d%cC  %c:%d ", temp, 0xDF, 1, fireset);
	lcd_gotoxy(0, 1);
	lcd_puts(bf);
	sprintf(bf, "[%.9s][%03d]", statfont[gasstat], altemp);
	lcd_gotoxy(0, 0);
	lcd_puts(bf);
	free(bf);
}

void led(uint16_t vr) {// LED를 켜고 끄는 역할을 한다.
	vr = ((uint8_t) ((float) vr / 1023.75) + 1);
	LED(1, (vr == 1));
	LED(2, (vr == 2));
	LED(3, (vr == 3));
	LED(4, (vr == 4));
	LED(5, (vr == 5));
	autemp = (vr == 2 ? 100 : vr == 3 ? 140 : vr == 4 ? 180 : vr == 5 ? 220 : 80);
}

typedef struct _IO{// 함수에 대한 포인터를 구조체에 정의하여 사용한다.
	void (*Lcd_Print)();
	void (*Led)(uint16_t);
	void (*Read_ADC)(uint16_t*, uint16_t*);

}IOcon;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { // TIM6의 이벤트 콜백 함수
	if (htim->Instance == htim6.Instance && ledRingFlag) { // LEDRing 제어를 한다.
		ledRingFlag = false;// LEDRing에대한 플레그를 false 로 설정.
		led_ring_update(led_ring_data[fire]);// LEDRing을 업데이트한다.
	}
}

void setUp(IOcon *io){ // 초기화를 수행한다.
	io->Lcd_Print = lcd_print; // LCD에 출력할 함수를 설정한다.
	io->Led = led;// LED에 출력할 함수를 설정한다.
	io->Read_ADC = read_adc;// ADC의 값을 받아오는 함수를 설정한다.
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	IOcon io; // 함수를 저장하는 구조체를 선언한다.
	boolean swFlag = false, buzflag = false, alflag = false; // 스위치, 부저, 알람 플레그를 선언한다.
	uint16_t cds, vr; // CDS, VR의 값을 저장하는 변수를 선언한다.
	uint32_t last = NOW(0); // 마지막 시간을 저장하는 변수를 선언한다.
	uint32_t flast = NOW(0); // 마지막 시간을 저장하는 변수를 선언한다.
	uint32_t tuplast = NOW(0); // 마지막 시간을 저장하는 변수를 선언한다.
	uint32_t tdownlast = NOW(0); // 마지막 시간을 저장하는 변수를 선언한다.
	uint32_t buzlast = NOW(0); // 마지막 시간을 저장하는 변수를 선언한다.
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
	MX_ADC_Init();
	MX_TIM6_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	setUp(&io);// 초기화를 수행한다.
	/* USER CODE BEGIN 2 */
	LcdInit();
	lcd_cgram(1, 0); // 1이란 아스키 코드에 출력할 문자를 설정한다.
	lcd_puts("\fSmart Gas Range\n             001"); // LCD에 출력한다.
	HAL_Delay(2000); // 2초 대기한다.
	HAL_TIM_Base_Start_IT(&htim6); // TIM6의 이벤트 콜백 함수를 시작한다.
	ledRingFlag = true; // LEDRing에대한 플레그를 true로 설정한다.
	io.Lcd_Print(); // LCD에 출력한다.

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		io.Read_ADC(&cds, &vr); // CDS, VR의 값을 읽어온다.
		if (!SW2) // 스위치2를 누르면
			stat.safe = true; // 안전상태를 true로 설정한다.
		else { // 스위치2를 누르지 않으면
			if (!SW1) { // 스위치1을 누르면
				if (cds > 3000) { // CDS의 값이 3000이상이면
					stat.on = true; // 전원상태를 true로 설정한다.
					stat.onn = false; // 켜졌지만 아무상태도 아닌상태를 false로 설정한다.
				} else { // CDS의 값이 3000미만이면
					stat.onn = true; // 켜졌지만 아무상태도 아닌상태를 true로 설정한다.
					stat.on = false; // 전원 상태를 false로 설정한다.
				}
				stat.off = false; // 꺼진 상태를를 false로 설정한다.
			} else { // 스위치1을 누르지 않으면
				if (temp < 150) { // 온도가 150미만이면
					stat.over = false; // 온도가 150도미만이면 과열상태를 false로 설정한다.
					buzflag = false; // 부저를 끈다.
				}
				stat.safe = false; // 안전상태를 false로 설정한다.
				stat.onn = false; // 켜졌지만 아무상태도 아닌상태를 false로 설정한다.
				stat.on = false; // 전원상태를 false로 설정한다.
				stat.off = true; // 꺼진 상태를 true로 설정한다.
			}
		}
		if (stat.off || stat.over || stat.safe) { // 꺼진 상태, 과열상태, 안전상태이면
			fireset = 0; // 불의 세기 설정을 0으로 설정한다.
			LEDCLEAR; // LED를 모두 끈다.
		} else if (stat.onn) { // 켜졌지만 아무상태도 아닌상태이면
			LEDCLEAR; // LED를 모두 끈다.
			fireset = 1; // 불의 세기 설정을 1으로 설정한다.
		} else if (stat.au && !stat.onn && cds > 3000) { // 켜진 상태이면서 켜졌지만 아무상태도 아닌상태이면서 CDS의 값이 3000이상이면
			io.Led(vr); // VR의 값에 따라 LED를 켜거나 끈다.
			if (1 < autemp - temp) // 온도가 1도 초과이면
				fireset = 9; // 불의 세기 설정을 9으로 설정한다.
			if (autemp - temp < -1) // 온도가 -1도 미만이면
				fireset = 1; // 불의 세기 설정을 1으로 설정한다.
		} else if (stat.on) { // 전원이 켜진 상태이면
			LEDCLEAR; // LED를 모두 끈다.
			fireset = vr / 511.875 + 1; // 불의 세기 설정을 VR의 값에 따라 설정한다.
		}
		if (!SW3 || !SW4 || !SW5) { // 스위치3,4,5를 누르면
			if (!swFlag) { // 스위치를 누르지 않았으면
				if (!SW3) // 스위치3을 누르면
					stat.au = (!stat.au ? true : false); // 안전상태를 true로 설정하거나 false로 설정한다.
				if (!SW4) // 스위치4를 누르면
					altemp -= 20; // 알람 온도를 20도 내링다.
				if (!SW5) // 스위치5를 누르면
					altemp += 20; // 알람 온도를 20도 올린다.
				if (altemp > 280) // 알람 온도가 280도 초과이면
					altemp = 280; // 알람 온도를 280로 설정한다.
				if (altemp < 20) // 알람 온도가 20도 미만이면
					altemp = 20; // 알람 온도를 20로 설정한다.
			}
			swFlag = true; // 스위치를 누른 상태로 설정한다.
		} else { // 스위치를 누르지 않았으면
			swFlag = false; // 스위치를 누른 상태로 설정하지 않는다.
		} 
		if (NOW(last) >= 10) { // 10ms 이상 시간이 지났으면
			if (NOW(flast) >= 100) { // 100ms 이상 시간이 지났으면
				if (fire < fireset) { // 불의 세기가 불의 세기 설정값보다 작으면
					fire++; // 불의 세기를 1씩 증가시킨다.
					io.Lcd_Print(); // LCD를 출력한다.
					ledRingFlag = true; // LEDRING을 업데이트 하는 플레그를 켠다.
				}
				if (fire > fireset) { // 불의 세기가 불의 세기 설정값보다 크면
					fire--; // 불의 세기를 1씩 감소시킨다.
					io.Lcd_Print(); // LCD를 출력한다.
					ledRingFlag = true; // LEDRING을 업데이트 하는 플레그를 켠다.
				}
				flast = NOW(0); // 시간을 현제로 초기화한다.
			}
			last = NOW(0); // 시간을 현제로 초기화한다.
		}
		if (NOW(tuplast)>= TEMPUP(fire) && TEMPUP(fire) != 0) { // 온도업데이트 시간이 지났으면
			temp++; // 온도를 1씩 증가시킨다.
			tuplast = NOW(0); // 시간을 현제로 초기화한다.
			if (temp > 300) // 온도가 300도 초과이면
				stat.over = true; // 과열상태를 true로 설정한다.
			io.Lcd_Print(); // LCD를 출력한다.
		}
		if (NOW(tdownlast) >= TEMPDOWN(temp - 20) && TEMPDOWN(temp - 20) != 0) { // 온도업데이트 시간이 지났으면
			temp--; // 온도를 1씩 감소시킨다.
			if (temp < 20) // 온도가 20도 미만이면
				temp = 20; // 온도를 20로 설정한다.
			tdownlast = NOW(0); // 시간을 현제로 초기화한다.
			io.Lcd_Print(); // LCD를 출력한다.
		}
		if (!buzflag && stat.over) { // 불이 과열상태이고  부저 플레그가 아니면
			buzlast = NOW(0); // 부저 시간을 현제로 초기화한다.
			buzflag = true; // 부저 플레그를 true로 설정한다.
		}
		if (altemp < temp && altemp > 20) { // 알람 온도가 온도보다 작고 20도 미만이면
			if (!alflag) { // 알람 플레그가 아니면
				buzlast = NOW(0); // 부저 시간을 현제로 초기화한다.
				alflag = true; // 알람 플레그를 true로 설정한다.
			}
		} else { // 알람 온도가 온도보다 작거나 20도 미만이면
			BUZZER(false); // 부저를 끈다.
			alflag = false; // 알람 플레그를 false로 설정한다.
		}
		if (buzflag) { // 부저 플레그가 true이면
			if ((NOW(buzlast) >= 100 && NOW(buzlast) <= 200) || (NOW(buzlast) >= 300 && NOW(buzlast) <= 400) || (NOW(buzlast) >= 500 && NOW(buzlast) <= 600)) // 부저 시간이 100~200, 300~400, 500~600이면
				BUZZER(true); // 부저를 켠다.
			else // 부저 시간이 100~200, 300~400, 500~600이 아니면
				BUZZER(false); // 부저를 끈다.
		} else if (alflag) { // 알람 플레그가 true이면
			if ((NOW(buzlast) >= 0 && NOW(buzlast) <= 100) // 부저 시간이 0~100이면
					|| (NOW(buzlast) >= 200 && NOW(buzlast) <= 300)) // 부저 시간이 200~300이면
				BUZZER(true); // 부저를 켠다.
			else // 부저 시간이 0~100, 200~300이 아니면
				BUZZER(false); // 부저를 끈다.
			if (NOW(buzlast) >= 1000) // 부저 시간이 1000이상이면
				buzlast = NOW(0); // 부저 시간을 현제로 초기화한다.
		}
		io.Lcd_Print(); // LCD를 출력한다.
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* TIM6_DAC_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.OversamplingMode = DISABLE;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerFrequencyMode = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 31;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 9999;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED_RING_Pin | BUZZ_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin | LED5_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			LCD_RS_Pin | LCD_RW_Pin | LCD_EN_Pin | LCD_D4_Pin | LCD_D5_Pin
					| LCD_D6_Pin | LCD_D7_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : SW_A_Pin SW_B_Pin */
	GPIO_InitStruct.Pin = SW_A_Pin | SW_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : SW_ON_Pin SW_AUTO_Pin SW_LOCK_Pin */
	GPIO_InitStruct.Pin = SW_ON_Pin | SW_AUTO_Pin | SW_LOCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_RING_Pin */
	GPIO_InitStruct.Pin = LED_RING_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LED_RING_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin
	 LED5_Pin BUZZ_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin | LED5_Pin
			| BUZZ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_RS_Pin LCD_RW_Pin LCD_EN_Pin LCD_D4_Pin
	 LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
	GPIO_InitStruct.Pin = LCD_RS_Pin | LCD_RW_Pin | LCD_EN_Pin | LCD_D4_Pin
			| LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
