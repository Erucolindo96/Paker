/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "Silnik.h"
#include "Robot.h"

#include "Analogowy_Czujnik_Odleglosci.h"
#include "Cyfrowy_Czujnik_Odleglosci.h"
#include "Serwo.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
Silnik silnik_lewy, silnik_prawy;

Robot minisumo;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void uruchomienie_PWM_silnikow(void);
void aktualizacja_silnika_lewego(void);
void aktualizacja_silnika_prawego(void);

void HAL_SYSTICK_Callback(void); //obsluga przerwania timera systick
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); //obsluga przerwania timera htim
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);//obsluga przerwan przychodzacych z zewnatrz do pinow

void pierwsza_faza_pomiaru_Analogowy_Czujnik_Odleglosci(void);
void druga_faza_pomiaru_Analogowy_Czujnik_Odleglosci(void);
void trzecia_faza_pomiaru_Analogowy_Czujnik_Odleglosci(void);
void zapis_wyniku_pomiaru_Analogowy_Czujnik_Odleglosci(void);

void pomiar_odleglosci_cyfrowy(void);

void aktualizacja_polozenia_serwomechanizmow(void);

void inicjalizacja_urzadzen_obslugiwanych(void);
void uruchom_PWM_serwomechanizmow(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//extern typedef enum{nie, tak} Bool;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  inicjalizacja_urzadzen_obslugiwanych();


  //kod testowy
  int wartosc_kata = 0;

  /**
   * inicjalizacja silników
   */
/*  Silnik_init(&silnik_lewy);
  Silnik_init(&silnik_prawy);
  uruchomienie_PWM_silnikow();
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
/*		//do przodu i ca³a moc
	  	Silnik_setKierunek(&silnik_lewy, PRZOD);
	  	Silnik_setKierunek(&silnik_prawy, PRZOD);
	    Silnik_setMoc(&silnik_lewy, MAX_CCR_SILNIK);
		Silnik_setMoc(&silnik_prawy, MAX_CCR_SILNIK);
		HAL_Delay(1000);

		//do przodu mocy
		Silnik_setMoc(&silnik_lewy, MAX_CCR_SILNIK>>1 );
		Silnik_setMoc(&silnik_prawy, MAX_CCR_SILNIK>>1 );
		HAL_Delay(1000);

		//do ty³u i po³owa mocy
	  	Silnik_setKierunek(&silnik_lewy, TYL);
	  	Silnik_setKierunek(&silnik_prawy, TYL);
	  	HAL_Delay(1000);

	  	//do ty³u i ca³a moc
	    Silnik_setMoc(&silnik_lewy, MAX_CCR_SILNIK);
		Silnik_setMoc(&silnik_prawy, MAX_CCR_SILNIK);
		HAL_Delay(1000);

	  Silnik_setKierunek(&silnik_lewy, TYL);
	  Silnik_setKierunek(&silnik_prawy, TYL);

	  int i;
	  for(i=0;i<MAX_CCR_SILNIK; i+=100)
	  {
		  Silnik_setMoc(&silnik_lewy, i);
		  Silnik_setMoc(&silnik_prawy, i);
		  HAL_Delay(50);
	  }
	  Silnik_setKierunek(&silnik_lewy, PRZOD);
	  Silnik_setKierunek(&silnik_prawy, PRZOD);

	  for(i=MAX_CCR_SILNIK; i>=200; i-=100)
	  {
	  		  Silnik_setMoc(&silnik_lewy, i);
	  		  Silnik_setMoc(&silnik_prawy, i);
	  		  HAL_Delay(50);
	  }
	  //kod testowy
	 wartosc_kata = 180;
	 Serwo_ustaw_nowy_kat(&(minisumo.serwo_lewe_), wartosc_kata);
	 Serwo_ustaw_nowy_kat(&(minisumo.serwo_prawe_), wartosc_kata);
	 HAL_Delay(1500);
	 wartosc_kata = 0;
	 Serwo_ustaw_nowy_kat(&(minisumo.serwo_lewe_), wartosc_kata);
	 Serwo_ustaw_nowy_kat(&(minisumo.serwo_prawe_), wartosc_kata);
	 HAL_Delay(1500);

	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  HAL_Delay(5000);
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  HAL_Delay(5000);
*/	  //koniec kodu testowego
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* TIM3 init function */
/*static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 127;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
*/
/*  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
*/

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 127;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 450;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
       Error_Handler();
    }


  HAL_TIM_MspPostInit(&htim3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SILNIK_LEWY_KIER_Pin SILNIK_PRAWY_KIER_Pin */
  GPIO_InitStruct.Pin = SILNIK_LEWY_KIER_Pin|SILNIK_PRAWY_KIER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pin : PRAWY_CZUJNIK_Pin */
  GPIO_InitStruct.Pin = PRAWY_CZUJNIK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PRAWY_CZUJNIK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEWY_CZUJNIK_Pin */
  GPIO_InitStruct.Pin = LEWY_CZUJNIK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LEWY_CZUJNIK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SILNIK_LEWY_KIER_Pin|SILNIK_PRAWY_KIER_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*
void HAL_SYSTICK_Callback(void)
{
	static int iterator_silnika = 0;
	iterator_silnika++;
	if(iterator_silnika >= OKRES_AKTUALIZACJI_SILNIKA)
	{
		iterator_silnika = 0;
		aktualizacja_silnika_lewego();
		aktualizacja_silnika_prawego();
	}

}

void aktualizacja_silnika_lewego(void)
{
	uint16_t moc = Silnik_getMoc(&silnik_lewy);
	htim3.Instance->CCR3 = moc;

	Kierunek kier = Silnik_getKierunek(&silnik_lewy);
	if(kier == PRZOD)
	{
		HAL_GPIO_WritePin(SILNIK_LEWY_KIER_GPIO_Port, SILNIK_LEWY_KIER_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(SILNIK_LEWY_KIER_GPIO_Port, SILNIK_LEWY_KIER_Pin, GPIO_PIN_RESET);
	}
}
void aktualizacja_silnika_prawego(void)
{
	uint16_t moc = Silnik_getMoc(&silnik_prawy);
	htim3.Instance->CCR4 = moc;

	Kierunek kier = Silnik_getKierunek(&silnik_prawy);
	if(kier == PRZOD)
	{
		HAL_GPIO_WritePin(SILNIK_PRAWY_KIER_GPIO_Port, SILNIK_PRAWY_KIER_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(SILNIK_PRAWY_KIER_GPIO_Port, SILNIK_PRAWY_KIER_Pin, GPIO_PIN_RESET);
	}

}
void uruchomienie_PWM_silnikow(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}
*/
//definicje fcji
void HAL_SYSTICK_Callback(void)
{
	/**
	 * Aktualizacja silnika
	 */
	static int iterator_silnika = 0;
	iterator_silnika++;
	if(iterator_silnika >= OKRES_AKTUALIZACJI_SILNIKA)
	{
		iterator_silnika = 0;
		aktualizacja_silnika_lewego();
		aktualizacja_silnika_prawego();
	}
	/**
	 * Aktualizacja poriamu odleg³osci cyfrowego
	 */
	//moze zajsc koniecznosc mierzenia z mniejsza czestoscia, by nie blokowac dzialania STMa
	pomiar_odleglosci_cyfrowy();
	/**
	 * Aktualizacja pomiaru odleg³osci  analogowego
	 */
	static int iterator_pomiaru_odleglosci=0;
	if(iterator_pomiaru_odleglosci == OKRES_POMIARU_ODLEGLOSCI)
	{
		pierwsza_faza_pomiaru_Analogowy_Czujnik_Odleglosci();
		iterator_pomiaru_odleglosci = 0;
	}
	++iterator_pomiaru_odleglosci;

	/**
	 * Aktualizacja serwomechanizmu
	 */
	static int iterator_serwomechanizmu = 0;
	++iterator_serwomechanizmu;
	if(iterator_serwomechanizmu == OKRES_AKTUALIZACJI_SERWOMECHANIZMOW)
	{
		aktualizacja_polozenia_serwomechanizmow();
		iterator_serwomechanizmu = 0;
	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
		druga_faza_pomiaru_Analogowy_Czujnik_Odleglosci();
		return;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == ECHO_Pin && HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
	{
		trzecia_faza_pomiaru_Analogowy_Czujnik_Odleglosci();
	}
	else if(GPIO_Pin == ECHO_Pin && HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET)
	{
		zapis_wyniku_pomiaru_Analogowy_Czujnik_Odleglosci();
	}
}
void aktualizacja_silnika_lewego(void)
{
	uint16_t moc = Silnik_getMoc(&silnik_lewy);
	htim3.Instance->CCR3 = moc;

	Kierunek kier = Silnik_getKierunek(&silnik_lewy);
	if(kier == PRZOD)
	{
		HAL_GPIO_WritePin(SILNIK_LEWY_KIER_GPIO_Port, SILNIK_LEWY_KIER_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(SILNIK_LEWY_KIER_GPIO_Port, SILNIK_LEWY_KIER_Pin, GPIO_PIN_RESET);
	}
}
void aktualizacja_silnika_prawego(void)
{
	uint16_t moc = Silnik_getMoc(&silnik_prawy);
	htim3.Instance->CCR4 = moc;

	Kierunek kier = Silnik_getKierunek(&silnik_prawy);
	if(kier == PRZOD)
	{
		HAL_GPIO_WritePin(SILNIK_PRAWY_KIER_GPIO_Port, SILNIK_PRAWY_KIER_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(SILNIK_PRAWY_KIER_GPIO_Port, SILNIK_PRAWY_KIER_Pin, GPIO_PIN_RESET);
	}

}
void uruchomienie_PWM_silnikow(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}


void pierwsza_faza_pomiaru_Analogowy_Czujnik_Odleglosci(void)
{
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	//TIM_Co->count
	htim4.Init.Period = 639;//wg wyliczen przy takim okresie i preskalerze timer odliczy 10us
	htim4.Init.Prescaler = 0;
	HAL_TIM_Base_Init(&htim4);
	HAL_TIM_Base_Start_IT(&htim4);
	minisumo.czujnik_analogowy_.faza_pomiaru = pierwsza;
}
void druga_faza_pomiaru_Analogowy_Czujnik_Odleglosci(void)
{
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	//tim_co->stop
	HAL_TIM_Base_Stop_IT(&htim4);
	minisumo.czujnik_analogowy_.faza_pomiaru = druga;
}
void trzecia_faza_pomiaru_Analogowy_Czujnik_Odleglosci(void)
{
	//tim_co->zliczanieDlugosciImpulsu
	htim4.Init.Period = 0xFFFF;
	htim4.Init.Prescaler = 50;//bylo 15 - licznik sie zerowal
	HAL_TIM_Base_Init(&htim4);
	HAL_TIM_Base_Start(&htim4);
	minisumo.czujnik_analogowy_.faza_pomiaru = trzecia;
}
void zapis_wyniku_pomiaru_Analogowy_Czujnik_Odleglosci(void)
{
	//faza pomiaru czujnika analogowego odleglosci - brak(zapis wyniku i wylaczenie timera)
	uint16_t pomiar = htim4.Instance->CNT;
	minisumo.czujnik_analogowy_.odleglosc = Analogowy_Czujnik_Odleglosci_oblicz_odleglosc(pomiar, htim4.Init.Prescaler, htim4.Init.Period );//odleglosc w milimetrach
	HAL_TIM_Base_Stop(&htim4);
	minisumo.czujnik_analogowy_.faza_pomiaru = brak;
}

void pomiar_odleglosci_cyfrowy(void)
{
	minisumo.czujnik_cyfrowy_lewy_.czy_jest_cos_widoczne = nie;
	minisumo.czujnik_cyfrowy_prawy_.czy_jest_cos_widoczne = nie;

	if(HAL_GPIO_ReadPin(LEWY_CZUJNIK_GPIO_Port, LEWY_CZUJNIK_Pin)== GPIO_PIN_RESET)//zdaje sie, ze czujniki jesli cos widza to daja stan niski, w przeciwnym razie - wysoki
		minisumo.czujnik_cyfrowy_lewy_.czy_jest_cos_widoczne = tak;
	if(HAL_GPIO_ReadPin(PRAWY_CZUJNIK_GPIO_Port, PRAWY_CZUJNIK_Pin)== GPIO_PIN_RESET)
		minisumo.czujnik_cyfrowy_prawy_.czy_jest_cos_widoczne = tak;
}
void uruchom_PWM_serwomechanizmow(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}


void aktualizacja_polozenia_serwomechanizmow(void)//powinna byc wolana co najmniej co 20 ms(taki jest okres sygnaly sterujacego serwami)
{
	uint8_t kat_lewego = Serwo_wartosc_kata( &(minisumo.serwo_lewe_));
	uint16_t CCR_lewego = Serwo_oblicz_wartosc_CCR(kat_lewego);
	htim3.Instance->CCR1 = CCR_lewego;

	uint8_t kat_prawego = Serwo_wartosc_kata(&(minisumo.serwo_prawe_));
	uint16_t CCR_prawego = Serwo_oblicz_wartosc_CCR(kat_prawego);
	htim3.Instance->CCR2 = CCR_prawego;

}
void inicjalizacja_urzadzen_obslugiwanych(void)
{
	Robot_init(&minisumo);
	Silnik_init(&silnik_lewy);
	Silnik_init(&silnik_prawy);
	uruchom_PWM_serwomechanizmow();
	uruchomienie_PWM_silnikow();

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  HAL_Delay(200);

  }
  /* USER CODE END Error_Handler */ 
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
