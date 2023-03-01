#include "stm32f10x.h"                  // Device header
#include "RTE_Components.h"             // Component selection
#include "DMA_STM32F10x.h"              // Keil::Device:DMA
#include "GPIO_STM32F10x.h"             // Keil::Device:GPIO
#include "RTE_Device.h"                 // Keil::Device:Startup
#include "misc.h"                       // Keil::Device:StdPeriph Drivers:Framework
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#include "math.h"
#include "ssd1306.h"
#include "flash.h"

#define CntStepFval 3    //кол-во значений для усреднения считывания регулятора оборотов. Чем меньше - тем резче регулируется
#define MinStartVoltage 0 //0 = контроль не испольузется. Минимальное рабочее напряжение (310В пост = 220 переменки! Указывается постоянка!, или переменка * 1.4)
int MotorAmperage = 10;  //0 = контроль не испольщуется. Максимальный рабочий ток двигателя 

//Тип управления запуска частотника:
#define ControlType ControlTwoButtons // ControlSwitch = работает пока нажат пуск; ControlTwoButtons = управление кнопками пуск + стоп

//Скорости разгона-торможения
int StartDelim  = 15; //Чем меньше число, тем быстрее разгон Мин 1 Макс 1000
int StopDelim  = 15;  //Чем меньше число, тем быстрее торможение Мин 1 Макс 1000 0 = торможение на выбеге. Если тормозить слишком быстро... это можеть быть опасно =)

//РАСПИНОВКА!
#define piStart GPIOB, GPIO_Pin_6 //Старт
#define piSetts GPIOB, GPIO_Pin_8 //Настройки
#define piRev GPIOB, GPIO_Pin_9 //Реверс
#define piPlus GPIOB, GPIO_Pin_5 //+
#define piMinus GPIOB, GPIO_Pin_7 //-
#define piError GPIOB, GPIO_Pin_1 //Сигнал ошибки

#define poFanOn GPIOA, GPIO_Pin_12 //Выход на активацию выхода драйвера | Индикация пуска
#define poSoftStart GPIOC, GPIO_Pin_13 // реле мягкого старта
#define poError GPIOC, GPIO_Pin_15 //лампа ошибки
#define poEnable GPIOC, GPIO_Pin_14 //Включение вентилятора
#define poBrake GPIOA, GPIO_Pin_3 // Включение тормозного резистора 

//Служебное
#define PI 				 3.14159 //В военное время может достигать 4
#define TIM_PERIOD 57 //57     
#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

#define ModuleError false //!GPIO_ReadInputDataBit(piError)



#define StartPressed !GPIO_ReadInputDataBit(piStart)
//#define StartReleased GPIO_ReadInputDataBit(piStart)
//#define StopPressed !IsRun //!GPIO_ReadInputDataBit(piStop)
//#define StopReleased GPIO_ReadInputDataBit(piStop)

#define ControlTwoButtons 1 //кнопка старт + стоп
#define ControlSwitch 2 //работает пока нажата старт

int mTIM_Prescaler =  800;  //200
#define TIM_STEPS	84			//120	
bool ErrorState = false;

int FSetVals[CntStepFval];
int nSetVal = 0;

#define cntVoltVals 100
int VoltVals[cntVoltVals];
int nSetVolt = 0;

#define cntAmpVals 80
int AmpVals[cntAmpVals];
int nSetAmp = 0;
int CurrAmp = 0; // значение x10!
int ampOffset = 0;
 
bool IsRev = false; //Фаг обратного хода
float SetHz = 50;	

bool AutoStartMode = false;
#define mMain 0
#define mStart 1
#define mStop 2

int menuMode = 0;
bool HasChangesMenu = false;

//Инициализация АЦП регулятора оборотов на B1
void adc_init() 
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	// настройки ADC
	ADC_InitTypeDef ADC_InitStructure;
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // режим работы - одиночный, независимый
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; // не сканировать каналы, просто измерить один канал
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // однократное измерение
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // без внешнего триггера
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //выравнивание битов результат - прижать вправо
	ADC_InitStructure.ADC_NbrOfChannel = 1; //количество каналов - одна штука
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Cmd(ADC2, ENABLE);

	// калибровка АЦП
	ADC_ResetCalibration(ADC2);
	while (ADC_GetResetCalibrationStatus(ADC2));
	ADC_StartCalibration(ADC2);
	while (ADC_GetCalibrationStatus(ADC2));
	
}

int analogRead(uint8_t Chan)
{
	ADC_RegularChannelConfig(ADC2, Chan, 1, ADC_SampleTime_28Cycles5); 
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);
	while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC2);
}
//Инициализация АЦП контроля силового напряжения
void adcVoltage_init() //Напряжение на А6
{
	if (MinStartVoltage == 0) return; //Значит контроль напряжения не используется.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
		//NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    //ADC
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init (ADC1, &ADC_InitStructure);    
 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5);
 
	/* Ограничители низкого и высокого уровня напряжений*/
  	ADC_AnalogWatchdogThresholdsConfig(ADC1, 330 * 8.2733, MinStartVoltage * 8.2733); //Безопасный диапазон в В * коэф АЦП = 8.2733
    ADC_AnalogWatchdogSingleChannelConfig(ADC1, ADC_Channel_6);
    ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
 
    /* Enable AWD interrupt */
    ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);
 
    // Enable ADC
    ADC_Cmd (ADC1, ENABLE); //enable ADC1
 
    //  ADC calibration (optional, but recommended at power on)
    ADC_ResetCalibration(ADC1); // Reset previous calibration
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1); // Start new calibration (ADC must be off at that time)
    while(ADC_GetCalibrationStatus(ADC1));
 
    // start conversion
    ADC_Cmd (ADC1,ENABLE);  //enable ADC1
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); // start conversion (will be endless as we are in continuous mode)
}

//Иноциализация пинов генерации ШИМа
void initIO(void)
{
	 //------Назначение GPIO------------- 
	 //-----------Ввод структур--------------// 
	 GPIO_InitTypeDef gpio;
	 GPIO_InitTypeDef gpio_n;

 //------------------------------------
	GPIO_StructInit(&gpio); // GPIO A - Tim1 Channel 1P, 2P, 3P, Enable
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	gpio.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &gpio);
 //------------------------------------ 
	GPIO_StructInit(&gpio_n); /* GPIOB Configuration: Channel 1N, 2N , 3N as alternate function push-pull */ 
	gpio_n.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
	gpio_n.GPIO_Mode = GPIO_Mode_AF_PP; 
	gpio_n.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_Init(GPIOB, &gpio_n);

}

//Инициализация основных пинов управления. Ручное управление портами
void InitIO2Manual()
{

	GPIO_InitTypeDef gpioP;
	GPIO_InitTypeDef gpioN;
	
	GPIO_InitTypeDef gpiom;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE); //----------Включение тактирования-------// 

	
	//Общие входы-выходы
	GPIO_StructInit(&gpiom); // GPIO A12 - Enable, A0 - номинальная частота, тормозной резистор
	gpiom.GPIO_Mode = GPIO_Mode_Out_PP;
	gpiom.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_0 | GPIO_Pin_3;
	gpiom.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpiom);
	
	// GPIO A - Fault, 
	gpiom.GPIO_Mode = GPIO_Mode_IPU;
	gpiom.GPIO_Pin = GPIO_Pin_11;
	gpiom.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpiom);
	
	// GPIO B - Start, Stop (кнопки), Error инвертированный!, Реверс
	gpiom.GPIO_Mode = GPIO_Mode_IPU;
	gpiom.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8 | GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_1 | GPIO_Pin_7;
	gpiom.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &gpiom);
	
	// GPIO С - Реле мягкого старта, лампа ошибки, пусковое реле
	gpiom.GPIO_Mode = GPIO_Mode_Out_PP;
	gpiom.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	gpiom.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &gpiom);
	
	
	//Выходы для прямого управления модулем (отключение)
	GPIO_StructInit(&gpioP); // GPIO A - Tim1 Channel 1P, 2P, 3P
	gpioP.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioP.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	gpioP.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &gpioP);
	
	GPIO_StructInit(&gpioN); // GPIO B - Tim1 Channel 1N, 2N, 3N
	gpioN.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioN.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	gpioN.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioN);	
}

void timers(void) { 
 //-----------Ввод структур--------------// 
 TIM_TimeBaseInitTypeDef TIM_Base_1;
 TIM_OCInitTypeDef TIM_PWM;

 //-----Назначение TIM1---------------- 
 TIM_TimeBaseStructInit(&TIM_Base_1);
 TIM_Base_1.TIM_Prescaler = mTIM_Prescaler; //18
 TIM_Base_1.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_Base_1.TIM_Period = TIM_PERIOD;
 TIM_Base_1.TIM_ClockDivision = TIM_CKD_DIV1;
 TIM_TimeBaseInit(TIM1,&TIM_Base_1);
	
 //-------Назначение ШИМ------------------- TIM_OCStructInit(&TIM_PWM);
 TIM_PWM.TIM_OCMode = TIM_OCMode_PWM1;
	
 TIM_PWM.TIM_OutputState = TIM_OutputState_Enable;
 TIM_PWM.TIM_OutputNState = TIM_OutputNState_Enable;
	
 TIM_PWM.TIM_OCNPolarity =  TIM_OCNPolarity_Low;
 TIM_PWM.TIM_OCPolarity =  TIM_OCPolarity_Low;
	
 TIM_OC1Init(TIM1, &TIM_PWM);
 TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
 TIM_OC2Init(TIM1, &TIM_PWM);
 TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
 TIM_OC3Init(TIM1, &TIM_PWM);
 TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
 TIM_Cmd(TIM1, ENABLE);
 TIM_CtrlPWMOutputs(TIM1, ENABLE);
 //разнесение инверсионных выходов
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime = 128; //Много - не мало. Но мало - дорого =)
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
 } 
/*********************************************************************/ 
 //---Фаза А ---// 
 uint16_t sinA[TIM_STEPS] = {0}; //{56,84,113,141,169,197,225,253,281,309,337,365,392, 420,447,474,502,529,556,583,609,636,662,688,714,740,766,791,817,842,867, 891,916,940,964,988,1011,1035,1058,1080,1103,1125,1147,1169,1190,1211, 1232,1252,1272,1292,1312,1331,1350,1368,1386,1404,1422,1439,1456,1472, 1488,1504,1519,1534,1549,1563,1577,1590,1603,1616,1628,1640,1651,1662, 1673,1683,1693,1702,1711,1720,1728,1736,1743,1750,1756,1762,1768,1773, 1777,1782,1785,1789,1792,1794,1796,1798,1799,1799,1800,1799,1799,1798, 1796,1794,1792,1789,1785,1782,1777,1773,1768,1762,1756,1750,1743,1736, 1728,1720,1711,1702,1693,1683,1673,1662,1651,1640,1628,1616,1603,1590, 1577,1563,1549,1534,1519,1504,1488,1472,1456,1439,1422,1404,1386,1368, 1350,1331,1312,1292,1272,1252,1232,1211,1190,1169,1147,1125,1103,1080, 1058,1035,1011,988,964,940,916,891,867,842,817,791,766,740,714,688,662, 636,609,583,556,529,502,474,447,420,392,365,337,309,281,253,225,56};
 //---Фаза B ---// 
 uint16_t sinB[TIM_STEPS] = {0}; //{1529,1514,1499,1483,1467,1450,1433,1416,1398,1380, 1362,1343,1324,1305,1286,1266,1245,1225,1204,1183,1161,1140,1118,1095, 1073,1050,1027,1003,980,956,932,908,883,858,833,808,783,757,732,706,680, 653,627,600,574,547,520,493,465,438,411,383,355,328,300,272,244,216,188, 160,131,103,75,47,18,9,37,65,94,122,150,178,206,234,262,290,318,346,374, 401,429,456,484,511,538,565,591,618,645,671,697,723,749,774,800,825,850, 875,900,924,948,972,996,1019,1042,1065,1088,1110,1132,1154,1176,1197, 1218,1239,1259,1279,1299,1318,1337,1356,1374,1392,1410,1428,1445,1461, 1478,1494,1509,1524,1539,1568,1581,1595,1608,1620,1632,1644,1655,1666, 1677,1687,1696,1705,1714,1723,1731,1745,1752,1758,1764,1769,1774,1779, 1783,1786,1790,1792,1795,1797,1798,1799,1799,1799,1799,1798,1797,1795, 1793,1791,1788,1784,1780,1776,1771,1766,1760,1748,1741,1733,1725,1717, 1708,1699,1690,1680,1670,1659,1529};
 //---Фаза C ---// 
 uint16_t sinC[TIM_STEPS] = {0};//{1599,1612,1624,1636,1648,1659,1670,1680,1690,1699, 1708,1717,1725,1733,1741,1748,1754,1760,1766,1771,1776,1780,1784,1788, 1791,1793,1795,1797,1798,1799,1799,1799,1799,1798,1797,1795,1792,1790, 1786,1783,1779,1774,1769,1764,1758,1752,1745,1738,1731,1723,1714,1705, 1696,1687,1677,1666,1655,1644,1632,1620,1608,1595,1581,1568,1554,1539, 1524,1509,1494,1478,1461,1445,1428,1410,1392,1374,1356,1337,1318,1299, 1279,1259,1239,1218,1197,1176,1154,1132,1110,1088,1065,1042,1019,996, 972,948,924,900,875,850,825,800,774,749,723,697,671,645,618,591,565,538, 511,484,456,429,401,374,346,318,290,262,234,206,178,150,122,94,65,37,9, 18,47,75,103,131,160,188,216,244,272,300,328,355,383,411,438,465,493, 520,547,574,600,627,653,680,706,732,757,783,808,833,858,883,908,932,956, 980,1003,1027,1050,1073,1095,1118,1140,1161,1183,1204,1225,1245,1266, 1286,1305,1324,1343,1362,1380,1398,1416,1433,1586,1586};
 /**********************************************************************/ 

////Расчёт таблиц синуса
static void sin_table_update (float_t koeff)
{
//	int Phase;
//	
//	if (IsRev) Phase = 240; else Phase = 0;
//    for(uint16_t i = 0; i < TIM_STEPS; ++i)
//    {
//    	sinA[i] = (uint16_t)(TIM_PERIOD * koeff * (1 + sin(PI * (2.0f * i / TIM_STEPS + Phase / 180.0f))));
//    }
//		
//	Phase	= 120;
//		for(uint16_t i = 0; i < TIM_STEPS; ++i)
//    {
//    	sinB[i] = (uint16_t)(TIM_PERIOD * koeff * (1 + sin(PI * (2.0f * i / TIM_STEPS + Phase / 180.0f))));
//    }
//		
//	if (IsRev) Phase = 0; else Phase = 240;
//		for(uint16_t i = 0; i < TIM_STEPS; ++i)
//    {
//    	sinC[i] = (uint16_t)(TIM_PERIOD * koeff * (1 + sin(PI * (2.0f * i / TIM_STEPS + Phase / 180.0f))));
//    }
		
		//////////////////
		int A = 0; float B; float PhA; float PhB; float PhC;
		
		if (IsRev) PhA = 0 / 180.0f; else PhA = 240 / 180.0f;
		PhB = 120 / 180.0f;
		if (IsRev) PhC = 240 / 180.0f; else PhC = 0 / 180.0f;
		A = TIM_PERIOD * koeff;
		
		for(uint16_t i = 0; i < TIM_STEPS; ++i)
    {	
			B = 2.0f * i / TIM_STEPS;
			sinA[i] = (uint16_t)( A * (1 + sin(PI * (B + PhA))));
    	sinB[i] = (uint16_t)( A * (1 + sin(PI * (B + PhB))));
			sinC[i] = (uint16_t)( A * (1 + sin(PI * (B + PhC))));
    }
}

//Инициализация каналов DMA для каждой фазы
void sinDMA_PhaseA(void) { 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitTypeDef DMA_struct;
	DMA_StructInit(&DMA_struct);
	DMA_struct.DMA_PeripheralBaseAddr =(uint32_t)&TIM1->CCR1;
	DMA_struct.DMA_MemoryBaseAddr = (uint32_t)&sinA[0];
	DMA_struct.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_struct.DMA_BufferSize=TIM_STEPS;
	DMA_struct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_struct.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_struct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_struct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_struct.DMA_Mode=DMA_Mode_Circular;
	DMA_struct.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel2,&DMA_struct);
	DMA_Cmd(DMA1_Channel2, ENABLE);
	TIM_DMACmd(TIM1,TIM_DMA_CC1, ENABLE);
 } 
void sinDMA_PhaseB(void) { 
	DMA_InitTypeDef DMA_struct1;
	DMA_StructInit(&DMA_struct1);
	DMA_struct1.DMA_PeripheralBaseAddr =(uint32_t)&TIM1->CCR2;
	DMA_struct1.DMA_MemoryBaseAddr = (uint32_t)&sinB[0];
	DMA_struct1.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_struct1.DMA_BufferSize=TIM_STEPS;
	DMA_struct1.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_struct1.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_struct1.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_struct1.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_struct1.DMA_Mode=DMA_Mode_Circular;
	DMA_struct1.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel3,&DMA_struct1);
	DMA_Cmd(DMA1_Channel3, ENABLE);
	TIM_DMACmd(TIM1,TIM_DMA_CC2, ENABLE);
 } 
void sinDMA_PhaseC(void) { 
	DMA_InitTypeDef DMA_struct2;
	DMA_StructInit(&DMA_struct2);
	DMA_struct2.DMA_PeripheralBaseAddr =(uint32_t)&TIM1->CCR3;
	DMA_struct2.DMA_MemoryBaseAddr = (uint32_t)&sinC[0];
	DMA_struct2.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_struct2.DMA_BufferSize=TIM_STEPS;
	DMA_struct2.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_struct2.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_struct2.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_struct2.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_struct2.DMA_Mode=DMA_Mode_Circular;
	DMA_struct2.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel6,&DMA_struct2);
	DMA_Cmd(DMA1_Channel6, ENABLE);
	TIM_DMACmd(TIM1,TIM_DMA_CC3, ENABLE);
 }

 ////////----------------------------------- Логика
 int Abs(int i1)
 {
	 if (i1 < 0)
		 return i1 * -1;
	 else 
		 return i1;
 }
 
 //Функция задержки
 void delay_ms(uint32_t ms)
{
	int32_t ms_count_tick =  ms * (SystemCoreClock/1000);
	//разрешаем использовать счётчик
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
         //обнуляем значение счётного регистра
	DWT_CYCCNT  = 0;
        //запускаем счётчик
	DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; 
	while(DWT_CYCCNT < ms_count_tick);
        //останавливаем счётчик
	DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
}

//контроль тока. 
bool AmpControlOK()
{

	double ret;
	ret = analogRead(ADC_Channel_7);
	if (ret < 0) ret = 0;
	int Curr = MotorAmperage * 41; //620 = 15А = 0.5В на шунте 
	
	//Усреднитель значений
	AmpVals[nSetAmp] = ret;
	nSetAmp++;
	if (nSetAmp == cntAmpVals)
		nSetAmp = 0;
	
	
	
	
	ret = 0;
	for(int i = 0; i < cntAmpVals; i++)
		ret += AmpVals[i];

	ret = ret / cntAmpVals;
	
	if(!GPIO_ReadInputDataBit(poEnable))
	{
		ampOffset = ret;
		return true;
	}
	else
		ret = ret - ampOffset;
	//
	CurrAmp = ret / 11.0f;
	if (MotorAmperage == 0 || !GPIO_ReadInputDataBit(poEnable)) return true;
	return (ret < Curr); //типа ток не превышен
}

//замер значения регулятора частоты и приведение его к значениям для делителя таймера
int GetValFriq()
{
	double ret;
	//ret = analogRead(ADC_Channel_9); //Крутилка на B1
	
	 ret = (float)SystemCoreClock/SetHz/(float)(TIM_PERIOD * TIM_STEPS);
	
	if (!AmpControlOK())
	{
		ret = 500;
		delay_ms(10);
	}
	
	//Усреднитель значений
	FSetVals[nSetVal] = ret;
	nSetVal++;
	if (nSetVal == CntStepFval)
		nSetVal = 0;
	
	ret = 0;
	for(int i = 0; i < CntStepFval; i++)
		ret += FSetVals[i];

	ret = ret / CntStepFval;
	//
	
	return ret;
}

//Изменение делителя частоты (увеличивает частоту срабатывания таймера = увеличивает частоту синуса)
void SetPrescaler(int val)
{
	
	DMA_Cmd(DMA1_Channel6, DISABLE);
	DMA_Cmd(DMA1_Channel3, DISABLE);
	DMA_Cmd(DMA1_Channel2, DISABLE);
	TIM_Cmd(TIM1, DISABLE);

	mTIM_Prescaler = val;
	TIM1->PSC = val;

	TIM_Cmd(TIM1, ENABLE);
	DMA_Cmd(DMA1_Channel6, ENABLE);
	DMA_Cmd(DMA1_Channel3, ENABLE);
	DMA_Cmd(DMA1_Channel2, ENABLE);
}

//Метод остановки частотника. параметр - используется ли плавная остановка (true) или выбег мотора (stop)
void Stop(bool PowerStop) 
{
	if (PowerStop && StopDelim > 0)
	{
		//Торможение мотора
		int ErrCycle = 0;
		int i;
		for (i = mTIM_Prescaler; i < 1500; i++) 
		{
			UpdateLCD(mStop);
			if (ModuleError)
			{
				ErrCycle ++;
				GPIO_SetBits(poError);
				if (ErrCycle > 50)
					break;
			}
			else
			{
				GPIO_ResetBits(poError);
				if (i < 1400)
				{			
					float_t VoltKoeff = fminf(0.5,(((((float)1) / (((float)i / 1500))*0.088)+0.016)*0.93));
					sin_table_update(VoltKoeff);
				}
				else
				{
					sin_table_update(0.05);
				}
				
				i += i / StopDelim; //чем меньше делитель - тем больше шаг торможения
			}
			
			SetPrescaler(i);	
		}
		GPIO_ResetBits(poBrake);
	}
	CurrAmp = 0;
	delay_ms(10);
	DMA_Cmd(DMA1_Channel6, DISABLE);
	DMA_Cmd(DMA1_Channel3, DISABLE);
	DMA_Cmd(DMA1_Channel2, DISABLE);
	TIM_Cmd(TIM1, DISABLE);

	InitIO2Manual();
//	
	GPIO_ResetBits(poEnable);

	GPIO_SetBits(GPIOB,GPIO_Pin_13); //p
	GPIO_SetBits(GPIOB,GPIO_Pin_14); //p
	GPIO_SetBits(GPIOB,GPIO_Pin_15); //p
	delay_ms(1);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);    //n
	GPIO_ResetBits(GPIOA,GPIO_Pin_9);    //n
	GPIO_ResetBits(GPIOA,GPIO_Pin_10);   //n
}

//Переход в состояние ошибки
void DoErrorState()
{
		GPIO_SetBits(poError);
		ErrorState = true;
		Stop(false);
}

//прерывание по значению питания. Значит, что-то идёт не так.
void ADC1_2_IRQHandler(void) 
{
   if(ADC_GetITStatus(ADC1, ADC_IT_AWD) == SET)
   {
		 int val = ADC_GetConversionValue(ADC1);
		 
		 	//Усреднитель значений
			VoltVals[nSetVolt] = val;
			nSetVolt++;
			if (nSetVolt == cntVoltVals)
				nSetVolt = 0;
			
			val = 0;
			for(int i = 0; i < cntVoltVals; i++)
				val += VoltVals[i];

			val = val / cntVoltVals;
			//
	
		 if (val > 3150) //380+ вольт. Тормозной резистор не справился. Тупо вырубаем всё и нехай на выбеге останавливается.
		 {
			 DoErrorState();
		 }
		 else if (val > 2900) // 350+ вольт  - включаем тормозной резистор и вырубаем реле запуска
			 //вырубаем реле, потому что компаратор может упустить момент, когда напряжение резко упадёт ниже его порога. 
		   //И если реле останется включено, то мы тупо получим печку. А так, из-за пускового сопротивления, напряжение быстро упадёт ниже стартового
		   //И начнётя новый цикл заряда. При этом, конечно, тормозной резистор будет отключен.
		 {
				GPIO_SetBits(poBrake);
				//GPIO_ResetBits(poSoftStart);
		 }
		 else if (val > (MinStartVoltage * 8.2733) * 0.85) //Конденсаторы заряжены до 85% мин напряжения старта. Замыкаем реле старта
		 {
			 GPIO_SetBits(poSoftStart);
		 }
		 else if (val > 100 && val < (MinStartVoltage * 8.2733) * 0.7) //Заряжаем конденсаторы мягким стартом
		 {
				GPIO_ResetBits(poSoftStart);
		 }
		 else if (val > 100 && val < 2813) //340- вольт - гасим тормозной резистор
		 {
			 GPIO_ResetBits(poBrake);
			 GPIO_SetBits(poSoftStart);
		 }

		 ADC_ClearITPendingBit(ADC1,ADC_IT_AWD);
   }  
}

//Проверка на старт
bool ToStart()
{
		if (StartPressed && !GPIO_ReadInputDataBit(poEnable) && !ErrorState)
		{
			delay_ms(20);
			return true; //проверка - может проскочила помеха? Против спонтанного запуска
		}
		return false;
}
//Проверка на стоп

bool ToStop()
{
	//свои основные циклы в зависимости от типа запуска
		if (StartPressed && (GPIO_ReadInputDataBit(poEnable) || ErrorState))
		{
			delay_ms(20);
			return true; //проверка - может проскочила помеха? Против спонтанной остановки
		}
		return false;

}
//Метод запуска частотника
void Start()
{
	int i, val;
	
	
////////// инициализация 
	GPIO_SetBits(poSoftStart);
	delay_ms(100);
	
	bool b;
	for(i = 0; i < cntAmpVals; i++)
		b = AmpControlOK();
	
	for(i = 0; i < CntStepFval; i++) //фильтр на CntStepFval значений. Надобно заполнить
		val = GetValFriq();
	
	//0.5 - 100% напряжения на выходе. 0.25 - 50%
	sin_table_update(0.1);
	
	timers();
	initIO();
	
	sinDMA_PhaseA();
	sinDMA_PhaseB();
	sinDMA_PhaseC();
//////////

	GPIO_SetBits(poEnable);

	//Выход на рабочие
	int ErrCycle = 0;
	for (i = 3000; i > val; i--) 
	{
		UpdateLCD(mStart);
		
		if (ModuleError)
		{
			ErrCycle ++;
			GPIO_SetBits(poError);
			if (ErrCycle > 30)
			{
				DoErrorState();
				return;
			}
		}
		else
		{
			GPIO_ResetBits(poError);
			if (i < 2800)
			{			
				float VoltKoeff = fminf(0.5,(((((float)1) / (((float)i / 1500))*0.088)+0.016)*0.93));
				sin_table_update(VoltKoeff);
			}
			
			if (AmpControlOK())
				i -= i / StartDelim; //чем меньше делитель - тем больше шаг разгона
			else
				delay_ms(200);
		}
		
		SetPrescaler(i);
		//Проверяемся на стоп
		if (ToStop())
		{
			Stop(false);
			delay_ms(300);
			return;
		}
	}
}

//Метод измерения частоты
void SetFrequency()
{
	int nVal = GetValFriq();
	
	if (Abs(nVal - mTIM_Prescaler) > 2) //минимальные изменнеия игнорируем. Возможно, тупо дребезг
	{
			if (nVal < 1500)
			{
				float VoltKoeff = fminf(0.5,(((((float)1) / (((float)nVal / 1500))*0.088)+0.016)*0.93));
				sin_table_update(VoltKoeff);
			}
			else
			{
				sin_table_update(0.1);
			}
				
		SetPrescaler(nVal);
		delay_ms(40);
	}
}

//запуск частотника - зарядка конденсаторов при подаче питания
void PowerInit()
{
	GPIO_SetBits(poError);
	
	int i = 0;
	while(i < 10 && (MinStartVoltage == 0 || analogRead(ADC_Channel_6) < MinStartVoltage * 8.2733)) //Либо по датчику напряжения, ну либо по таймеру
	{
		if (i%2==0)
			GPIO_SetBits(poError);
		else
			GPIO_ResetBits(poError);
		delay_ms(300);
		i++;
	}
	GPIO_ResetBits(poError);
	ErrorState = false;
	GPIO_SetBits(poSoftStart);
}

void UpdateLCD(int wType)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	char buf[8];
	
	switch (menuMode)
	{
		case 0:
			SSD1306_GotoXY(3, 0); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
		
			sprintf(buf, "%.1fA", (float)CurrAmp/10.0f);
			SSD1306_Puts(&buf, &Font_11x18, SSD1306_COLOR_WHITE); 
			SSD1306_GotoXY(63, 0); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
				sprintf(buf, "%.0fW", CurrAmp * 22.0f);
			SSD1306_Puts(&buf, &Font_11x18, SSD1306_COLOR_WHITE); 
			SSD1306_GotoXY(10, 21); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
		
			
			float Hz = 0;
			if (GPIO_ReadInputDataBit(poEnable) && wType != mMain)
				Hz = (float)SystemCoreClock / (float)mTIM_Prescaler / (float)(TIM_PERIOD * TIM_STEPS);
			else
				Hz = SetHz;
			
			sprintf(buf, "%.1fHz", Hz);
			SSD1306_Puts(&buf, &Font_7x10, SSD1306_COLOR_WHITE);
		
			SSD1306_GotoXY(60, 21); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			if (GPIO_ReadInputDataBit(poEnable))
			{
				if (!IsRev)
					if (wType == mStart)
						SSD1306_Puts(">> Start", &Font_7x10, SSD1306_COLOR_WHITE);
					else if (wType == mStop)
						SSD1306_Puts(">> Stop", &Font_7x10, SSD1306_COLOR_WHITE);
					else
						SSD1306_Puts(" Forward", &Font_7x10, SSD1306_COLOR_WHITE);
				else
					if (wType == mStart)
						SSD1306_Puts("<< Start", &Font_7x10, SSD1306_COLOR_WHITE);
					else if (wType == mStop)
						SSD1306_Puts("<< Stop", &Font_7x10, SSD1306_COLOR_WHITE);
					else
						SSD1306_Puts(" Reverse", &Font_7x10, SSD1306_COLOR_WHITE);
			}
			else
			{
				if (IsRev)
					SSD1306_Puts("<<<<<<<<", &Font_7x10, SSD1306_COLOR_WHITE);
				else
					SSD1306_Puts(">>>>>>>>", &Font_7x10, SSD1306_COLOR_WHITE);
			}
		break;
		case 1:
			SSD1306_GotoXY(10, 0); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
		SSD1306_Puts("Start Hz:", &Font_7x10, SSD1306_COLOR_WHITE); 
			SSD1306_GotoXY(10, 13); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			sprintf(buf, "%.1fHz", SetHz);
			SSD1306_Puts(&buf, &Font_11x18, SSD1306_COLOR_WHITE);
		break;
		case 2:
			SSD1306_GotoXY(10, 0); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			SSD1306_Puts("Start spin:", &Font_7x10, SSD1306_COLOR_WHITE); 
			SSD1306_GotoXY(10, 13); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			if (IsRev)
				SSD1306_Puts("Reverce", &Font_11x18, SSD1306_COLOR_WHITE);
			else
				SSD1306_Puts("Forward", &Font_11x18, SSD1306_COLOR_WHITE);
		break;
		case 3:
			SSD1306_GotoXY(10, 0); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			SSD1306_Puts("Autostart:", &Font_7x10, SSD1306_COLOR_WHITE); 
			SSD1306_GotoXY(10, 13); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			if (AutoStartMode)
				SSD1306_Puts("Yes", &Font_11x18, SSD1306_COLOR_WHITE);
			else
				SSD1306_Puts("No", &Font_11x18, SSD1306_COLOR_WHITE);
		break;
		case 4:
			SSD1306_GotoXY(10, 0); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			SSD1306_Puts("Speed start:", &Font_7x10, SSD1306_COLOR_WHITE); 
			SSD1306_GotoXY(10, 13); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			sprintf(buf, "%d%%", 101 - StartDelim);
			SSD1306_Puts(&buf, &Font_11x18, SSD1306_COLOR_WHITE);
		break;
		case 5:
			SSD1306_GotoXY(10, 0); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			SSD1306_Puts("Speed stop:", &Font_7x10, SSD1306_COLOR_WHITE); 
			SSD1306_GotoXY(10, 13); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			sprintf(buf, "%d%%", 101 - StopDelim);
			SSD1306_Puts(&buf, &Font_11x18, SSD1306_COLOR_WHITE);
		break;
		case 6:
			SSD1306_GotoXY(10, 0); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			SSD1306_Puts("Motor amperage:", &Font_7x10, SSD1306_COLOR_WHITE); 
			SSD1306_GotoXY(10, 13); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			sprintf(buf, "%dA", MotorAmperage);
			if (MotorAmperage)
				SSD1306_Puts(&buf, &Font_11x18, SSD1306_COLOR_WHITE);
			else
				SSD1306_Puts("Not use", &Font_11x18, SSD1306_COLOR_WHITE);
		break;
		
	}	

		SSD1306_UpdateScreen();	
}

void TempControl()
{
if (analogRead(ADC_Channel_6) < 3720)
		GPIO_SetBits(poFanOn);
	else
		GPIO_ResetBits(poFanOn);
}

void SaveSettings()
{
	if (!HasChangesMenu) return;
	FLASH_Init();

	tpSettings settings;
	
	settings.Hz = (uint16_t)(SetHz*10.0f);
	settings.StartDelim = StartDelim;
	settings.StopDelim = StopDelim;
	if (AutoStartMode) settings.StartMode = 1; else settings.StartMode = 0;
	if (IsRev) settings.SpinMode = 1; else settings.SpinMode = 0;
	settings.MotorAmperage = MotorAmperage;
	
	FLASH_WriteSettings(&settings);
	HasChangesMenu = false;
}

void LoadSettings()
{
	tpSettings settings;
	FLASH_Init();
	
	FLASH_ReadSettings(&settings);

	if (settings.Hz > 65000)
	{
		SetHz = 50;
		StartDelim = 21;
		StopDelim = 21;
		AutoStartMode = false;
		IsRev = false;
		MotorAmperage = 0;
	}
	else
	{
		SetHz = (float)settings.Hz / 10.0f;
		StartDelim = settings.StartDelim;
		StopDelim = settings.StopDelim;
		if (settings.StartMode) AutoStartMode = true; else AutoStartMode = false;
		if (settings.SpinMode) IsRev = true; else IsRev = false;
		MotorAmperage = settings.MotorAmperage;
	}
}

////////////////////////////////////////////
//Основной цикл

int main(void)
{
		InitIO2Manual();
//	
	GPIO_ResetBits(poEnable);

	GPIO_SetBits(GPIOB,GPIO_Pin_13); //p
	GPIO_SetBits(GPIOB,GPIO_Pin_14); //p
	GPIO_SetBits(GPIOB,GPIO_Pin_15); //p
	delay_ms(1);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);    //n
	GPIO_ResetBits(GPIOA,GPIO_Pin_9);    //n
	GPIO_ResetBits(GPIOA,GPIO_Pin_10);   //n
	
		SSD1306_Init();
		delay_ms(100);
		SSD1306_GotoXY(17, 5); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
		SSD1306_Puts("Hello!", &Font_16x26, SSD1306_COLOR_WHITE); 
		SSD1306_UpdateScreen();
	
		LoadSettings();
		adc_init();
		adcVoltage_init();
		Stop(false);	
		PowerInit();
	
	if (AutoStartMode)
		Start();
	
	while(1)
	{
		TempControl();
		
		//Нажали старт?
		if (ToStart())
		{		
			delay_ms(200);
			Start();
		}
		
		//Нажали стоп?
		if (ToStop())
		{
			if (ErrorState)  //Нажатие стоп после ошибки - гасит её
			{
				ErrorState = false;
				GPIO_ResetBits(poError);
				delay_ms(1500);
			}
			else
			{
				delay_ms(150);
				Stop(true);
			}
		}
		
		//меню
		if (!GPIO_ReadInputDataBit(piSetts))
		{
				delay_ms(10);
				if (!GPIO_ReadInputDataBit(piSetts)) //Вдруг помеха? убеждаемся что через 10 мс "никто не передумал" =)
				{
					if (menuMode < 6)
						menuMode ++;
					else
					{
						SaveSettings();
						menuMode = 0;
					}
					
						delay_ms(200);
				}
		}
		
		// +
		if (!GPIO_ReadInputDataBit(piPlus))
		{
				delay_ms(10);
				if (!GPIO_ReadInputDataBit(piPlus)) //Вдруг помеха? убеждаемся что через 10 мс "никто не передумал" =)
				{
					if (menuMode > 0) HasChangesMenu = true;
					switch(menuMode)
					{
						case 0: case 1:	if (SetHz < 85)	SetHz += 0.5; break;
						case 2:	IsRev = !IsRev; break;
						case 3:	AutoStartMode = !AutoStartMode; break;
						case 4:	if (StartDelim > 1)	StartDelim --; break;
						case 5:	if (StopDelim > 1)	StopDelim --; break;
						case 6:	if (MotorAmperage < 15)	MotorAmperage ++; break;
					}
					delay_ms(150);
				}
		}
		
		// -
		if (!GPIO_ReadInputDataBit(piMinus))
		{
				delay_ms(10);
				if (!GPIO_ReadInputDataBit(piMinus)) //Вдруг помеха? убеждаемся что через 10 мс "никто не передумал" =)
				{
					if (menuMode > 0) HasChangesMenu = true;
					switch(menuMode)
					{
						case 0: case 1:	if (SetHz > 25)	SetHz -= 0.5; break;
						case 2:	IsRev = !IsRev; break;
						case 3:	AutoStartMode = !AutoStartMode; break;
						case 4:	if (StartDelim < 100)	StartDelim ++; break;
						case 5:	if (StopDelim < 100)	StopDelim ++; break;
						case 6:	if (MotorAmperage > 0)	MotorAmperage --; break;
					}
					delay_ms(150);
				}
		}
		
		//Переключился на реверс?
		if (!GPIO_ReadInputDataBit(piRev))
		{
				delay_ms(10);
				if (!GPIO_ReadInputDataBit(piRev)) //Вдруг помеха? убеждаемся что через 10 мс "никто не передумал" =)
				{
					if (GPIO_ReadInputDataBit(poEnable)) //Если на ходу
					{
							Stop(true);
							IsRev = !IsRev;
							Start();
					}
					else
					{
						IsRev = !IsRev;
						delay_ms(200);
					}
				}
		}
		
		//Проверка регулятора частоты
		if (GPIO_ReadInputDataBit(poEnable))
			SetFrequency();
		
		
		//Контроль ошибки
		if (ModuleError)
		{
				int ErrCycle = 0;
				for (int i = 0; i < 300; i ++) 
				{
					if (ModuleError)
					{
						ErrCycle ++;
						GPIO_SetBits(poError);
						if (ErrCycle > 30)
						{
							DoErrorState();
							break;
						}
					}
					//Не забываем проверять кнопку стоп!
					if (ToStop())
					{
						Stop(false);
						delay_ms(300);
						break;
					}
					
					delay_ms(10);
				}
				if (!ErrorState)
					GPIO_ResetBits(poError);
		}
			
		UpdateLCD(mMain);
	}
}


