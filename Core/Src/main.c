/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

#define FRONT_DOOR 0
#define BACK_DOOR 1

#define KEYPAD_DEF 0
#define KEYPAD_INC 1
#define KEYPAD_COR 2

// #define SENSOR_WIN 0
// #define SENSOR_DOOR 4

#define STATUS_DISARM 0
#define STATUS_ARM 1
#define STATUS_ALARM 2

#define UART_RECV_BUFFER 30

uint8_t g_uartRecvByte = 0;
uint8_t g_uartBuffer[UART_RECV_BUFFER] = {0};
uint8_t g_commandBuffer[UART_RECV_BUFFER+1] = {0};

uint16_t g_uartBufferSize = 0;
uint16_t g_uartCommands = 0;

//Setup
//Mode: 0 -> Setup mode
uint8_t g_bIsRunMode = 0;

uint16_t g_FrontDoorDist = 2;
uint16_t g_BackDoorDist = 2;
int16_t g_AlarmDelay = 2;

//Status
//Armed/Disarmed Items. Treat as BOOLEANS!
uint8_t SystemArmed = 1;

uint8_t SensorArmed[6] = {1,1,1,1,  1,1};
uint8_t DistanceArmed[2] = {1, 1};


//Door/Window sensors and LEDs:
uint8_t SystemTriggered = 0;

int32_t nCurTime2 = 0;
int32_t SAINextTriggerTime = 0;
int32_t AlarmTriggerTime = 0;
int32_t SystemArmTime = 0;
int32_t LCDPrintTime = 0;

uint8_t SensorStatus[6] = { 0, 0, 0, 0, 0, 0 };

uint16_t KeypadCombination[4] = { 1,2,3,4 };
int16_t EnteredCombination[4] = {0};
int16_t EnteredCombinationSize = 0;

//Time stuff
int32_t g_timeHour = 12;
int32_t g_timeMin = 30;
int32_t g_timeSec = 0;
// int32_t g_elapsedTime = 0;

//Lcd buffer
char g_LcdLine1[21] = {0};
char g_LcdLine2[21] = {0};
char g_LcdLine3[21] = {0};
char g_LcdLine4[21] = {0};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// float min (float a, float b) {
// 	return (a<b) ? a : b;
// }
// float max (float a, float b) {
// 	return (a>b) ? a : b;
// }

void ADCSelectChannel(uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}



/* This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
	  HAL_UART_Receive_IT(&huart6, &g_uartRecvByte, 1);
    
    if (g_uartRecvByte >= ' ' && g_uartRecvByte <= 'z') {
      int bEndInstruction = (g_uartRecvByte == ';') ? 1 : 0;
      if (bEndInstruction) {
        g_uartCommands++;
      }
      //Last character of buffer is reserved for end of instruction
      if (g_uartBufferSize < UART_RECV_BUFFER-1 || bEndInstruction) {
        g_uartBuffer[g_uartBufferSize] = g_uartRecvByte;
        g_uartBufferSize++;
      }
    }

  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  if (htim->Instance == TIM10)
	  {
			g_timeSec++;
      if (g_timeSec >= 60) {
        g_timeMin++;
        g_timeSec -= 60;
      }
      if (g_timeMin >= 60) {
        g_timeHour++;
        g_timeMin -= 60;
      }
      if (g_timeHour >= 24) {
        g_timeHour -= 24;
      }
	  }
}

char GetStatusLet(int i) {
  if (SensorStatus[i] == STATUS_ALARM)
    return 'X';
  else if (SensorStatus[i] == STATUS_ARM)
    return 'A';
  return 'D';
}

void UpdateLEDs() {
	//ALARM: Red    01
	//ARMED: Yellow 11
	//Disarmed: Green 10

	//Window LEDs
  uint8_t s = SensorStatus[0];
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, (s==STATUS_ARM || s==STATUS_ALARM)); //RED
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, (s==STATUS_ARM || s==STATUS_DISARM)); //GREEN
  
  s = SensorStatus[1];
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, (s==STATUS_ARM || s==STATUS_ALARM)); //RED
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, (s==STATUS_ARM || s==STATUS_DISARM)); //GREEN

  s = SensorStatus[2];
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (s==STATUS_ARM || s==STATUS_ALARM)); //RED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (s==STATUS_ARM || s==STATUS_DISARM)); //GREEN

  s = SensorStatus[3];
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (s==STATUS_ARM || s==STATUS_ALARM)); //RED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (s==STATUS_ARM || s==STATUS_DISARM)); //GREEN

	//Door LEDs
  s = SensorStatus[4];
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (s==STATUS_ARM || s==STATUS_ALARM)); //RED
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (s==STATUS_ARM || s==STATUS_DISARM)); //GREEN

  s = SensorStatus[5];
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, (s==STATUS_ARM || s==STATUS_ALARM)); //RED
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (s==STATUS_ARM || s==STATUS_DISARM)); //GREEN

}

void ClearScreenLCD() {
		uint8_t LCD_Instcode = 0xFE;
		uint8_t LCD_CLS = 1;
		uint8_t LCD_home = 0;

	//Clear the LCD screen at the beginning
	HAL_UART_Transmit(&huart6,&LCD_Instcode,1,1000);
	HAL_UART_Transmit(&huart6,&LCD_home,1,1000);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart6,&LCD_Instcode,1,1000);
	HAL_UART_Transmit(&huart6,&LCD_CLS,1,1000);
	HAL_Delay(100);
}

void WriteLine(const char* strLine, int nLineNum) {
	uint8_t LCD_Instcode = 0xFE;
	uint8_t LCD_rows[4] = { 128, 192, 148, 212 };

	HAL_UART_Transmit(&huart6, &LCD_Instcode, 1, 1000);
	HAL_UART_Transmit(&huart6, &LCD_rows[nLineNum], 1, 1000);
	// HAL_Delay(100);
	HAL_UART_Transmit(&huart6, (uint8_t*)strLine, strlen((char*)strLine), 1000);
	// HAL_Delay(150);
}

void PadSpaces(char line[21]) {
	int nLen = strlen(line);

	for (int i = nLen; i < 20; i++) {
		line[i] = ' ';
	}
	line[20] = 0;
}
void UpdateLCD() {
	char line1[21];
	char line2[21];
	char line3[21];
	char line4[21];

	if (g_bIsRunMode) {
		sprintf(line1, "M:RUN %02ld:%02ld:%02ld", g_timeHour, g_timeMin, g_timeSec);
		sprintf(line2, "W1:%c W2:%c W3:%c W4:%c", GetStatusLet(0), GetStatusLet(1), GetStatusLet(2), GetStatusLet(3));
    sprintf(line3, "FD:%c BD:%c FL:%c BL:%c", GetStatusLet(4), GetStatusLet(5), (DistanceArmed[0] ? 'A' : 'D'), (DistanceArmed[1] ? 'A' : 'D'));
    
    sprintf(line4, "SysArm:%c  ", (SystemArmed) ? 'A' : 'D');

    // sprintf(line4, "SysArm:%c  %c%c%c%c", 
    //   (SystemArmed) ? 'A' : 'D',
    //   (EnteredCombinationSize > 0) ? EnteredCombination[0] : '_',
    //   (EnteredCombinationSize > 1) ? EnteredCombination[1] : '_',
    //   (EnteredCombinationSize > 2) ? EnteredCombination[2] : '_',
    //   (EnteredCombinationSize > 3) ? EnteredCombination[3] : '_'
    // );
    
    for (int i = 0; i < EnteredCombinationSize; i++) {
      sprintf(line4+strlen(line4), "%d", EnteredCombination[i]);
    }
    for (int i = 0; i < 4-EnteredCombinationSize; i++) {
      sprintf(line4+strlen(line4), "_");
    }

	}
	else {
		sprintf(line1, "M:S %02ld:%02ld:%02ld K:%d%d%d%d", g_timeHour, g_timeMin, g_timeSec, KeypadCombination[0], KeypadCombination[1], KeypadCombination[2], KeypadCombination[3]);
		sprintf(line2, "W1:%c W2:%c W3:%c W4:%c", GetStatusLet(0), GetStatusLet(1), GetStatusLet(2), GetStatusLet(3));
		sprintf(line3, "FD:%c BD:%c FL:%c BL:%c", GetStatusLet(4), GetStatusLet(5), (DistanceArmed[0] ? 'A' : 'D'), (DistanceArmed[1] ? 'A' : 'D'));
		sprintf(line4, "FDD:%d BDD:%d AD:%d", g_FrontDoorDist, g_BackDoorDist, g_AlarmDelay);
	}

	PadSpaces(line1);
	PadSpaces(line2);
	PadSpaces(line3);
	PadSpaces(line4);

	if (strcmp(g_LcdLine1, line1)) {
    if ( nCurTime2 >= LCDPrintTime) {

      LCDPrintTime = nCurTime2 + 100;
      WriteLine(line1, 0);
		  strcpy(g_LcdLine1, line1);
    }
		
	}
	if (strcmp(g_LcdLine2, line2)) {
    if ( nCurTime2 >= LCDPrintTime) {
      LCDPrintTime = nCurTime2 + 100;
		  WriteLine(line2, 1);
		  strcpy(g_LcdLine2, line2);
    }
	}
	if (strcmp(g_LcdLine3, line3)) {
    if ( nCurTime2 >= LCDPrintTime) {
      LCDPrintTime = nCurTime2 + 100;
		  WriteLine(line3, 2);
		  strcpy(g_LcdLine3, line3);
    }
	}
	if (strcmp(g_LcdLine4, line4)) {
     if ( nCurTime2 >= LCDPrintTime) {
      LCDPrintTime = nCurTime2 + 100;
		  WriteLine(line4, 3);
		  strcpy(g_LcdLine4, line4);
     }
	}
}


uint16_t keypadColState = 0;
uint16_t keypadLastState = 0;

GPIO_TypeDef* const mapKeypadInput[4] = { GPIOB, GPIOB, GPIOB, GPIOB };
uint16_t const mapKeypadInputNum[4] = { GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13, GPIO_PIN_12 };
GPIO_TypeDef* const mapKeypadOutput[3] = { GPIOA, GPIOC, GPIOA };
uint16_t const mapKeypadOutputNum[3] = { GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_15 };

uint16_t const keymap [12]= { 1,2,3,4,5,6,7,8,9,11,0,12 }; //11=*, 12 = #
uint16_t combinationIndex = 0; //What digit in the KeypadCombination is being tested for.
uint16_t combinationCorrect = 0; //How many digits in the current KeypadCombination have been correct

int CheckKeypad() {
  int nRet = KEYPAD_DEF;
  for (int i = 0; i < 4; i++) { //For all rows
    short nBitPos = keypadColState + 3*i;
    if (HAL_GPIO_ReadPin(mapKeypadInput[i], mapKeypadInputNum[i])) //Button i is down
    {
      if(!((keypadLastState >> nBitPos) & 1)){//button at i was just pressed.
        keypadLastState |= (1 << nBitPos);

        if (keymap[nBitPos] >= 10) {
          continue;
        }
        //Basic code to figure out whether the KeypadCombination has been typed in. Invulnerable to De Brujin Sequence.
        if(keymap[nBitPos] == KeypadCombination[combinationIndex]){
          combinationCorrect++;
        }
        else{
          combinationCorrect = 0;
        }

        EnteredCombination[EnteredCombinationSize++] = keymap[nBitPos];
        //Debug code
        // char buf[21];
        // sprintf(buf, "Value: %d", keymap[nBitPos]);
        // WriteLine(buf, 0);

        combinationIndex++;
        if (combinationIndex == 4){
          nRet = (combinationCorrect == 4) ? KEYPAD_COR : KEYPAD_INC;
          combinationIndex = 0;
          combinationCorrect = 0;

          EnteredCombinationSize = 0;
        }
      }
    }
    else {
      keypadLastState &= ~(1 << nBitPos);
    }
  }

  keypadColState = (keypadColState+1) % 3; //Move to next column
  for (int i = 0; i < 3; i++) {
    short nValue = (keypadColState == i);
    HAL_GPIO_WritePin(mapKeypadOutput[i], mapKeypadOutputNum[i], nValue);
  }

  return nRet;
}

void UpdateIndividualState(int16_t sensorNum, GPIO_TypeDef* PinLetter, uint16_t PinNum) {
  // uint8_t* armed = (sensorNum < 4) ? WinArmed : DoorArmed;
  // if (sensorNum >= 4)
  //   sensorNum -= 4;
  
  if (SensorStatus[sensorNum] == STATUS_ALARM) {
    return; //If it is alarm then just stay at alarm
  }
  if (SensorArmed[sensorNum]) {
    SensorStatus[sensorNum] = (SystemArmed && !HAL_GPIO_ReadPin(PinLetter, PinNum)) ? STATUS_ALARM : STATUS_ARM;
  }
  else {
    SensorStatus[sensorNum] = STATUS_DISARM;
  }
}

void UpdateSensorState() {
  // SensorStatus
  // UpdateIndividualState

  UpdateIndividualState(0, GPIOC, GPIO_PIN_0);
  UpdateIndividualState(1, GPIOC, GPIO_PIN_1);
  UpdateIndividualState(2, GPIOC, GPIO_PIN_2);
  UpdateIndividualState(3, GPIOC, GPIO_PIN_3);
  
  //Door
  UpdateIndividualState(4, GPIOB, GPIO_PIN_9);
  UpdateIndividualState(5, GPIOB, GPIO_PIN_8);
  
}

int ProcessCommand(const char* strCommand) {
  //run; -> Switch to run mode
  //set; -> Switch to setup mode

  if (strCommand == 0 || *strCommand == 0)
    return 0;
  
  // char buf[21];
  char letter;
  int a,b,c,d;

  // WriteLine(strCommand, 1);

  //During run mode, only process the setup command
  if (g_bIsRunMode) {
    if (strcmp(strCommand, "setup") == 0) {
      // if (!SystemTriggered) 
      {
        SystemTriggered = 0;
        g_bIsRunMode = 0;
        SystemArmed = 1;
  
        for (int i = 0; i < 6; i++) {
          SensorStatus[i] = (SensorArmed[i] ? STATUS_ARM : STATUS_DISARM);
        }
        
        //Turn off the flashing system alarm indicaor if it is on
        SAINextTriggerTime = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0);

        //Turn off the alarm relay if it is on
        AlarmTriggerTime = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);

        //Turn off lighting relay if it is on 
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);

        //Reset LCD screen
        g_LcdLine1[0] = 0;
        g_LcdLine2[0] = 0;
        g_LcdLine3[0] = 0;
        g_LcdLine4[0] = 0;

      }
    }

    return 0;
  }

  //Start Run mode
  if (strcmp(strCommand, "run") == 0) {
    // sprintf(buf, "Run Mode");

    g_bIsRunMode = 1;
    SystemTriggered = 0;
    SystemArmed = 1;


    //Reset LCD screen
    g_LcdLine1[0] = 0;
    g_LcdLine2[0] = 0;
    g_LcdLine3[0] = 0;
    g_LcdLine4[0] = 0;
    
  }
  //Start setup mode
  // else if (strcmp(strCommand, "setup") == 0) {
  //   // sprintf(buf, "Setup Mode");
  //   if (!SystemTriggered) {
  //     g_bIsRunMode = 0;
  //     SystemArmed = 1;
  //   }
  // }
  //Window
  else if (sscanf(strCommand, "w%d %c", &a, &letter) == 2) {
    // sprintf(buf, "Window: %d %c", a, letter);
    if (a >= 1 && a <= 4) {
      if (letter=='d')
        SensorArmed[a-1] = 0;
      else if (letter=='a')
        SensorArmed[a-1] = 1;

      // SensorArmed[a-1] = (letter=='d') ? 0 : 1;
    }
  }
  //Front Door Distance
  else if (sscanf(strCommand, "fdd %d", &a) == 1) {
    // sprintf(buf, "FDD: %d", a);
    if (a >= 1 && a <= 4)
      g_FrontDoorDist = a;

  }
  //Back Door Distance
  else if (sscanf(strCommand, "bdd %d", &a) == 1) {
    // sprintf(buf, "BDD: %d", a);
    if (a >= 1 && a <= 4)
      g_BackDoorDist = a;
  }
  //Front Door
  else if (sscanf(strCommand, "fd %c", &letter) == 1) {
    // sprintf(buf, "FD: %c", letter);
    if (letter=='d')
      SensorArmed[4] = 0;
    else if (letter=='a')
      SensorArmed[4] = 1;

    // SensorArmed[4] = (letter=='d') ? 0 : 1;
  }
  //Back Door
  else if (sscanf(strCommand, "bd %c", &letter) == 1) {
    // sprintf(buf, "BD: %c", letter);
    if (letter=='d')
      SensorArmed[5] = 0;
    else if (letter=='a')
      SensorArmed[5] = 1;

    // SensorArmed[5] = (letter=='d') ? 0 : 1;
  }
  else if (sscanf(strCommand, "fl %c", &letter) == 1) {
    // sprintf(buf, "BD: %c", letter);
    if (letter=='d')
      DistanceArmed[0] = 0;
    else if (letter=='a')
      DistanceArmed[0] = 1;
  }
  else if (sscanf(strCommand, "bl %c", &letter) == 1) {
    // sprintf(buf, "BD: %c", letter);
    if (letter=='d')
      DistanceArmed[1] = 0;
    else if (letter=='a')
      DistanceArmed[1] = 1;
  }
  //Alarm Delay
  else if (sscanf(strCommand, "ad %d", &a) == 1) {
    if (a >= 0) {
      g_AlarmDelay = a;
    }

    // sprintf(buf, "delay: %d", a);
    // PadSpaces(buf);
    // WriteLine(buf, 0);
  }
  //Time
  else if (sscanf(strCommand, "time %d:%d:%d", &a, &b, &c) == 3) {
    // sprintf(buf, "Time: %d %d %d", a, b, c);
    g_timeHour = a;
    g_timeMin = b;
    g_timeSec = c;
  }
  //Keypad password
  else if (sscanf(strCommand, "key %1d%1d%1d%1d", &a, &b, &c, &d) == 4) {
    // sprintf(buf, "Keypad: %d %d %d %d", a, b, c, d);
    KeypadCombination[0] = a;
    KeypadCombination[1] = b;
    KeypadCombination[2] = c;
    KeypadCombination[3] = d;
  }
  else {
    // sprintf(buf, ": %s", strCommand);
  }

  // PadSpaces(buf);
  // WriteLine(buf, 0);

  return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//Keypad Variables
	int32_t keypadLastTimerCount = 0;



//	uint16_t LCDLastTimerCount = 0;
	int32_t LEDLastTimerCount = 0;
	int32_t ADCLastTimerCount = 0;
	uint8_t ADCChannelNum = 0;	//This chooses which ADC to sample from

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
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart6, &g_uartRecvByte, 1);
  HAL_TIM_Base_Start_IT(&htim10);// start the TIM10 timer and enable the Timer10 interrupt

  //Initialize timer
  uint16_t nLastTime = 0;
  g_uartBufferSize = 0;

  HAL_TIM_Base_Start(&htim2);


  ClearScreenLCD();
  nCurTime2 = __HAL_TIM_GET_COUNTER(&htim2);
  keypadLastTimerCount = nCurTime2;
  LEDLastTimerCount = keypadLastTimerCount + 10;
  ADCLastTimerCount = keypadLastTimerCount + 20;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint16_t time = __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t deltaTime;
    if (time > nLastTime) {
      deltaTime = (time - nLastTime);
      nCurTime2 += deltaTime;
    }
    else {
      //Overflow occured
      deltaTime = (0xffff - (nLastTime - time) + 1); 
      nCurTime2 += deltaTime;
    }
    nLastTime = time;

    // g_elapsedTime += deltaTime; 

    /////////////////////////////////
    ///////////         Update Time
    //Doesnt work for some reason
    // for (; g_elapsedTime >= 1000; g_elapsedTime -= 1000) {
    //   //Increment a second
    //   char buff1[21];
    //   char buff2[21];

    //   sprintf (buff1, "%ld sec: %ld", g_elapsedTime, (g_timeSec/60));
    //   sprintf (buff2, "min: %ld hour: %ld", (g_timeMin/60), g_timeMin);
    //   PadSpaces(buff1);
    //   PadSpaces(buff2);

    //   WriteLine(buff1, 0);
    //   WriteLine(buff2, 1);


    //   g_timeSec++;
    //   g_timeMin += (int)(g_timeSec/60);
    //   g_timeHour += (int)(g_timeMin/60);

    //   g_timeSec %= 60;
    //   g_timeMin %= 60;
    //   g_timeHour %= 24;
    // }

    // char buf1[21];
    // sprintf(buf1, "elap: %ld  %ld:%ld:%ld", g_elapsedTime, g_timeHour, g_timeMin, g_timeSec);
    // PadSpaces(buf1);
    // WriteLine(buf1, 0);


    // for (; g_elapsedTime >= 1000; g_elapsedTime -= 1000) {
    //   //Increment a second
    //   g_timeSec++;
    //   if (g_timeSec >= 60) {
    //     g_timeMin++;
    //     g_timeSec -= 60;
    //   }
    //   if (g_timeMin >= 60) {
    //     g_timeHour++;
    //     g_timeMin -= 60;
    //   }
    //   if (g_timeHour >= 24) {
    //     g_timeHour -= 24;
    //   }

    //   // g_timeSec %= 60;
    //   // g_timeMin %= 60;
    //   // g_timeHour %= 24;
    // }


    ////////////////////////////////////////////////////////////////////////////////
    ///////////////              Run Mode
    ////////////////////////////////////////////////////////////////////////////////

    if (g_bIsRunMode) {

      //KEYPAD LOGIC
      if (nCurTime2 >= keypadLastTimerCount) //Only scan keypad every 10ms
      {
        keypadLastTimerCount = nCurTime2 + 5;
        int nRes = CheckKeypad();
        if (nRes == KEYPAD_COR) {
          
          if (SystemTriggered) {
            SystemTriggered = 0;
            SystemArmed = 0;
            
            //Turn off the flashing system alarm indicaor if it is on
            SAINextTriggerTime = 0;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0);

            //Turn off the alarm
            AlarmTriggerTime = 0;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
            
            for (int i = 0; i < 6; i++) {
              SensorStatus[i] = (SensorArmed[i] ? STATUS_ARM : STATUS_DISARM);
            }

            // char buf[21];
            // PadSpaces(buf);
            // sprintf(buf, "Disarmed system");
            // WriteLine(buf, 1);

          }
          else {
            if (SystemArmed) {
              SystemArmed = 0;
              SystemArmTime = 0;
            }
            else {
              //Set it to arm mode AFTER a delay
              SystemArmTime = nCurTime2 + (int32_t)(g_AlarmDelay * 1000); //in ms

            }
            
          }

        }
      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////			Push Button Input

      //Button Logic
      //GREEN on Disarmed and Armed (!Alarm)
      //Red on Armed and ALARM (!Disarmed)
      //Currently, everything is armed all the time, and there is no way to disarm stuff.

      //Update at xHz to avoid lag in Proteus.
      if(nCurTime2 >= LEDLastTimerCount){
        LEDLastTimerCount = nCurTime2 + 50;

        //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);

        //Window/Door is_open
        // WinTriggered[0]  = ( (!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))) && WinArmed[0] && SystemArmed );
        // WinTriggered[1]  = ( (!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))) && WinArmed[1] && SystemArmed);
        // WinTriggered[2]  = ( (!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2))) && WinArmed[2] && SystemArmed);
        // WinTriggered[3]  = ( (!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))) && WinArmed[3] && SystemArmed);
        // DoorTriggered[0] = ( (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))) && DoorArmed[0] && SystemArmed);
        // DoorTriggered[1] = ( (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))) && DoorArmed[1] && SystemArmed);
        // int8_t bOldAlarm = 0;
        // for (int i = 0; i < 6; i++) {
        //   if (SensorStatus[i] == STATUS_ALARM)
        //     bOldAlarm = 1;
        // }
        UpdateSensorState();
        UpdateLEDs();

        uint8_t bNewAlarm = 0;
        for (int i = 0; i < 6; i++) {
          if (SensorStatus[i] == STATUS_ALARM)
            bNewAlarm = 1;
        }

        if (!SystemTriggered && bNewAlarm) {
          SystemTriggered = 1;
          SAINextTriggerTime = 0;

          AlarmTriggerTime = nCurTime2 + (int32_t)(g_AlarmDelay * 1000); //in ms

          // char buf[21];
          // sprintf(buf, "tt: %d, dt: %d", AlarmTriggerTime, deltaTime);
          // PadSpaces(buf);
          // WriteLine(buf, 2);
          
        }
      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	    //////////////////////////////			ADC
      if (nCurTime2 >= ADCLastTimerCount) {
        ADCLastTimerCount = nCurTime2 + 100;
        if (DistanceArmed[ADCChannelNum]) {
          uint32_t nChannel = (ADCChannelNum == FRONT_DOOR) ? ADC_CHANNEL_6 : ADC_CHANNEL_7;

          ADCSelectChannel (nChannel);
          HAL_ADC_Start(&hadc1); //Start conversion
          HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); //Poll for completion
          uint32_t sampledValue = HAL_ADC_GetValue(&hadc1); //Get sampled value
          HAL_ADC_Stop(&hadc1); //Done conversion

          float fVoltage = (float)sampledValue/255.0 * 3.3; //Max value is 255 which represents 3.3V

          float mapDistToVoltage[6] = { 2.769412, 1.436471, 0.983529, 0.750588 };
          //Print voltage (Debug stuff)
          // char buffer[21];
          // sprintf(buffer, "Count: %ld", sampledValue);
          // char buffer2[21];
          // sprintf(buffer2, "Val: %f", fVoltage);
          // PadSpaces(buffer);
          // PadSpaces(buffer2);
          // if (ADCChannelNum == FRONT_DOOR) {
          //   WriteLine(buffer, 0);
          //   WriteLine(buffer2, 1);
          // }
          // else {
          //   WriteLine(buffer, 2);
          //   WriteLine(buffer2, 3);
          // }

          GPIO_TypeDef* relayLetter = GPIOB;
          uint16_t relayPinNum = (ADCChannelNum == FRONT_DOOR) ? GPIO_PIN_5 : GPIO_PIN_4;
          uint16_t nTargetDist = (ADCChannelNum == FRONT_DOOR) ? g_FrontDoorDist : g_BackDoorDist;
          float fTarget = mapDistToVoltage[nTargetDist-1];
          
          //Print voltage (Debug stuff)
          // char buffer[21];
          // sprintf(buffer, "Target: %f", fTarget);
          // char buffer2[21];
          // sprintf(buffer2, "Val: %f", fVoltage);
          // PadSpaces(buffer);
          // PadSpaces(buffer2);
          // if (ADCChannelNum == FRONT_DOOR) {
          //   WriteLine(buffer, 0);
          //   WriteLine(buffer2, 1);
          // }
          // else {
          //   WriteLine(buffer, 2);
          //   WriteLine(buffer2, 3);
          // }
          
          HAL_GPIO_WritePin(relayLetter, relayPinNum, (fVoltage >= fTarget) ? 1 : 0);
          
          // if (fVoltage > 1) {
          //   HAL_GPIO_WritePin(relayLetter, relayPinNum, 1);
          // }
          // else {
          //   HAL_GPIO_WritePin(relayLetter, relayPinNum, 0);
          // }
        }

        //Switch ADC Channel
        ADCChannelNum = (ADCChannelNum+1) % 2;
      }

      if (SystemTriggered) {
        //Set rate of system alarm indicator
        if (nCurTime2 >= SAINextTriggerTime) {
          SAINextTriggerTime = nCurTime2 + 100;
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
        }

        if (nCurTime2 >= AlarmTriggerTime && AlarmTriggerTime > 0) {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
          AlarmTriggerTime = 0;
        }
      }

      if (SystemArmTime > 0 && nCurTime2 >= SystemArmTime) {
        SystemArmed = 1;
        SystemArmTime = 0;
      }

    }
    ////////////////////////////////////////////////////////////////////////////////
    ///////////////              Setup Mode
    ////////////////////////////////////////////////////////////////////////////////
    else {
      UpdateSensorState();
      UpdateLEDs();
    
      //Turn off the flashing system alarm indicaor if it is on
      // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0);

      //Turn off the alarm relay if it is on
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);

      //Turn off lighting relay if it is on 
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);

    }


    ////////////////////////////////////////////////////////////////////////////////
    ///////////////              Process commands from keyboard
    ////////////////////////////////////////////////////////////////////////////////
    if (g_uartCommands > 0) {
      // g_uartBuffer[g_uartBufferSize] = 0;
      // PadSpaces(g_uartBuffer);
      // WriteLine(g_uartBuffer, 2);
      // g_uartBufferSize = 0;
      // g_uartCommands = 0;
      
      int nTotalBufferSize = g_uartBufferSize;
      g_uartCommands--;

      int i = 0;

      for (; i < nTotalBufferSize; i++) {
        char c = g_uartBuffer[i];
        if (c == ';') {
          g_commandBuffer[i] = 0;
          ProcessCommand((char*)(g_commandBuffer));
          break;
        }
        g_commandBuffer[i] = c;
      }

      i++;  //i points to the ';'.. Go to next character
      int j = 0;
      for (; i < g_uartBufferSize; i++) {
        g_uartBuffer[j++] = g_uartBuffer[i];
      }
      g_uartBufferSize = j;
    }
    

	  
    //////////////////////////////
	  ////		LCD display
	  //This function will only update the LCD if the output has changed. So its okay to call it without a timer variable
	  UpdateLCD();



	  HAL_Delay(5);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 16000-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Keypad_Input___Column_1_Pin|Door_2_Status_LED_Pin|Door_2_Status_LEDA10_Pin
                          |Door_1_Status_LED_Pin|Door_1_Status_LEDA12_Pin|Keypad_Input___Column_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Window_2_Status_LED_Pin|Window_2_Status_LEDC5_Pin|Keypad_Input___Column_2_Pin|Window_1_Status_LED_Pin
                          |Window_1_Status_LEDC11_Pin|SystemAlarm_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Window_3_Status_LED_Pin|Window_3_Status_LEDB1_Pin|Window_4_Status_LED_Pin|Window_4_Status_LEDB10_Pin
                          |Lighting_Relay_2_Pin|Lighting_Relay_1_Pin|Alarm_Relay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Window_Switch_1_Pin Window_Switch_2_Pin Window_Switch_3_Pin Window_Switch_4_Pin */
  GPIO_InitStruct.Pin = Window_Switch_1_Pin|Window_Switch_2_Pin|Window_Switch_3_Pin|Window_Switch_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Keypad_Input___Column_1_Pin Door_2_Status_LED_Pin Door_2_Status_LEDA10_Pin
                           Door_1_Status_LED_Pin Door_1_Status_LEDA12_Pin Keypad_Input___Column_3_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Keypad_Input___Column_1_Pin|Door_2_Status_LED_Pin|Door_2_Status_LEDA10_Pin
                          |Door_1_Status_LED_Pin|Door_1_Status_LEDA12_Pin|Keypad_Input___Column_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Window_2_Status_LED_Pin Window_2_Status_LEDC5_Pin Keypad_Input___Column_2_Pin Window_1_Status_LED_Pin
                           Window_1_Status_LEDC11_Pin SystemAlarm_Pin */
  GPIO_InitStruct.Pin = Window_2_Status_LED_Pin|Window_2_Status_LEDC5_Pin|Keypad_Input___Column_2_Pin|Window_1_Status_LED_Pin
                          |Window_1_Status_LEDC11_Pin|SystemAlarm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Window_3_Status_LED_Pin Window_3_Status_LEDB1_Pin Window_4_Status_LED_Pin Window_4_Status_LEDB10_Pin
                           Lighting_Relay_2_Pin Lighting_Relay_1_Pin Alarm_Relay_Pin */
  GPIO_InitStruct.Pin = Window_3_Status_LED_Pin|Window_3_Status_LEDB1_Pin|Window_4_Status_LED_Pin|Window_4_Status_LEDB10_Pin
                          |Lighting_Relay_2_Pin|Lighting_Relay_1_Pin|Alarm_Relay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Keypad_Output___Row_D_Pin Keypad_Output___Row_C_Pin Keypad_Output___Row_B_Pin Keypad_Output___Row_A_Pin
                           Door_Switch_2_Pin Door_Switch_1_Pin */
  GPIO_InitStruct.Pin = Keypad_Output___Row_D_Pin|Keypad_Output___Row_C_Pin|Keypad_Output___Row_B_Pin|Keypad_Output___Row_A_Pin
                          |Door_Switch_2_Pin|Door_Switch_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



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
