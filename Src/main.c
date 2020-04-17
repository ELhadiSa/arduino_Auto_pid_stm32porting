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
 #include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum { false, true } boolean;
#define bool boolean

#define byte int8_t

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
extern uint32_t milisecond;
boolean useSimulation = true;
double theta[50];
double outputStart;
unsigned long  modelTime;



//setmode


//AUTOPID
bool isMax, isMin;
double input, output;
double setpoint;
double noiseBand;
int controlType;
bool running;
unsigned long peak1, peak2, lastTime;
int sampleTime;
int nLookBack;
int peakType;
double lastInputs[101];
double peaks[10];
int peakCount;
bool justchanged;
bool justevaled;
double absMax, absMin;
double oStep;
double outputStart;
double Ku, Pu;



//PID
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1


double dispKp;				// * we'll hold on to the tuning parameters in user-entered
double dispKi;				//   format for display purposes
double dispKd;				//

double kp;                  // * (P)roportional Tuning Parameter
double ki;                  // * (I)ntegral Tuning Parameter
double kd;                  // * (D)erivative Tuning Parameter

int controllerDirection;
int pOn;

double input;       //modification      *input;   // * Pointers to the Input, Output, and Setpoint variables
double output;      //modification      *output;   //   This creates a hard link between the variables and the
double setpoint;     //modification     *setpoint;   //   PID, freeing the user from having to constantly tell us
                              //   what these values are.  with pointers we'll just know.



unsigned long lastTime;
double outputSum, lastInput;

unsigned long SampleTime;
double outMin, outMax;
bool inAuto, pOnE;


//setmode

//changeAutotune
boolean tuning=1;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;



//autotune helper
byte ATuneModeRemember;



//domodel

double kpmodel, taup;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void setup();
void SetMode(int Mode);
void Initialize();
void changeAutoTune();
void SetNoiseBand(double Band);
void SetOutputStep(double Step);
void SetLookbackSec(int value);
void AutoTuneHelper(boolean start);
int GetMode();
void Cancel();
void loop();
unsigned long millis();
double  GetKp();
double  GetKi();
double  GetKd();
void SetTunings(double Kp, double Ki, double Kd);
void SetTunings2(double Kp, double Ki, double Kd, int POn);
bool Compute();
void DoModel();
void FinishUp();
void SetControllerDirection(int Direction);
void SetOutputLimits(double Min, double Max);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Cancel()
{
	running = false;
}
int GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = GetMode();
  else
    SetMode(ATuneModeRemember);
}
void SetLookbackSec(int value)
{
    if (value<1) value = 1;

	if(value<25)
	{
		nLookBack = value * 4;
		sampleTime = 250;
	}
	else
	{
		nLookBack = 100;
		sampleTime = value*10;
	}
}
void SetOutputStep(double Step)
{
	oStep = Step;
}
void SetNoiseBand(double Band)
{
	noiseBand = Band;
}
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

void Initialize()
{
   outputSum = output; //modification *output
   lastInput = input; //modification *input
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}


void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    SetNoiseBand(aTuneNoise);
    SetOutputStep(aTuneStep);
    SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}



void setup()
{
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid
  SetMode(AUTOMATIC);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
//
//  serialTime = 0;
//  Serial.begin(9600);

}
unsigned long millis(){

	return milisecond;
}
void FinishUp()
{
	  output = outputStart; //*output modification
      //we can generate tuning parameters!
      Ku = 4*(2*oStep)/((absMax-absMin)*3.14159);
      Pu = (double)(peak1-peak2) / 1000;
}

int Runtime()
{
	justevaled=false;
	if(peakCount>9 && running)
	{
		running = false;
		FinishUp();
		return 1;
	}
	unsigned long now = millis();

	if((now-lastTime)<sampleTime) return false;
	lastTime = now;
	double refVal = input; //*input modification
	justevaled=true;
	if(!running)
	{ //initialize working variables the first time around
		peakType = 0;
		peakCount=0;
		justchanged=false;
		absMax=refVal;
		absMin=refVal;
		setpoint = refVal;
		running = true;
		outputStart = output; //*output modification
		output = outputStart+oStep;//*output modification
	}
	else
	{
		if(refVal>absMax)absMax=refVal;
		if(refVal<absMin)absMin=refVal;
	}

	//oscillate the output base on the input's relation to the setpoint

	if(refVal>setpoint+noiseBand) output = outputStart-oStep; //*output modification
	else if (refVal<setpoint-noiseBand) output = outputStart+oStep; //*output modification


  //bool isMax=true, isMin=true;
  isMax=true;isMin=true;
  //id peaks
  for(int i=nLookBack-1;i>=0;i--)
  {
    double val = lastInputs[i];
    if(isMax) isMax = refVal>val;
    if(isMin) isMin = refVal<val;
    lastInputs[i+1] = lastInputs[i];
  }
  lastInputs[0] = refVal;
  if(nLookBack<9)
  {  //we don't want to trust the maxes or mins until the inputs array has been filled
	return 0;
	}

  if(isMax)
  {
    if(peakType==0)peakType=1;
    if(peakType==-1)
    {
      peakType = 1;
      justchanged=true;
      peak2 = peak1;
    }
    peak1 = now;
    peaks[peakCount] = refVal;

  }
  else if(isMin)
  {
    if(peakType==0)peakType=-1;
    if(peakType==1)
    {
      peakType=-1;
      peakCount++;
      justchanged=true;
    }

    if(peakCount<10)peaks[peakCount] = refVal;
  }

  if(justchanged && peakCount>2)
  { //we've transitioned.  check if we can autotune based on the last peaks
    double avgSeparation = (abs(peaks[peakCount-1]-peaks[peakCount-2])+abs(peaks[peakCount-2]-peaks[peakCount-3]))/2;
    if( avgSeparation < 0.05*(absMax-absMin))
    {
		FinishUp();
      running = false;
	  return 1;

    }
  }
   justchanged=false;
	return 0;
}

double  GetKp()
{
	return controlType==1 ? 0.6 * Ku : 0.4 * Ku;
}

double  GetKi()
{
	return controlType==1? 1.2*Ku / Pu : 0.48 * Ku / Pu;  // Ki = Kc/Ti
}

double  GetKd()
{
	return controlType==1? 0.075 * Ku * Pu : 0;  //Kd = Kc * Td
}



void SetTunings(double Kp, double Ki, double Kd){
    SetTunings2(Kp, Ki, Kd, pOn);
}

void SetTunings2(double Kp, double Ki, double Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

bool Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
 //     double input = input; //modification  *input
      double error = setpoint - input;//modification *setpoint
      double dInput = (input - lastInput);
      outputSum+= (ki * error);

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!pOnE) outputSum-= kp * dInput;

      if(outputSum > outMax) outputSum= outMax;
      else if(outputSum < outMin) outputSum= outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
	   //double output;
      if(pOnE) output = kp * error;
      else output = 0;

      /*Compute Rest of PID Output*/
      output += outputSum - kd * dInput;

	    if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	    output = output; //modification *output

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
	    return true;
   }
   else return false;
}


void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)(rand() % 20)-10)/100;

}
void loop()
{

  unsigned long now = millis();

  if(!useSimulation)
  { //pull the input in from the real world
    //input = analogRead(0);
  }

  if(tuning)
  {
    byte val = (Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = GetKp();
      ki = GetKi();
      kd = GetKd();
      SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else Compute();

  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)
    {
      modelTime +=100;
      DoModel();
    }
  }
  else
  {
//     analogWrite(0,output);
  }
//
//  //send-receive with processing if it's time
//  if(millis()>serialTime)
//  {
//    SerialReceive();
//    SerialSend();
//    serialTime+=500;
//  }
}


void SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(output > outMax) output = outMax; //modification my output
	   else if(output < outMin) output = outMin;  //modification my output

	   if(outputSum > outMax) outputSum= outMax;
	   else if(outputSum < outMin) outputSum= outMin;
   }
}

void SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
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
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  ATuneModeRemember=2;
 	 input=80;
 	 output=50;
 	 setpoint=180;
 	 kp=2;
 	 ki=0.5;
 	 kd=2;

 	 kpmodel=1.5;
 	 taup=100;

 	 outputStart=5;
 	 aTuneStep=50;
 	 aTuneNoise=1;
 	 aTuneStartValue=100;
 	 aTuneLookBack=20;

 	 tuning = false;

 	  output = output;
 	  input = input;

  	 //creation PID

 	   output = output;
 	    input = input;
 	    setpoint = setpoint;
 	    inAuto = false;

 	    SetOutputLimits(0, 255);				//default output limit corresponds to
 													//the arduino pwm limits

 	    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

 	    SetControllerDirection(controllerDirection);
 	    SetTunings2(kp, ki, kd, pOn);

 	    lastTime = millis()-SampleTime;


 	    //creation auttune


 	   input = input;
 	   	output = output;
 	   	controlType =0 ; //default to PI
 	   	noiseBand = 0.5;
 	   	running = false;
 	   	oStep = 30;
 	   	SetLookbackSec(10);
 	   	lastTime = millis();
  /* USER CODE END 2 */
HAL_TIM_Base_Start_IT(&htim7);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
setup();

  while (1)
  {
	  loop();

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 799;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
