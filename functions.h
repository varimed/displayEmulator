/*
 * functions.h
 *
 *  Created on: Oct 14, 2019
 *      Author: grzegorz
 */

//#include "onewire_hw.h"
//#include "main.h"
//#include "testerStates.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

typedef enum {
	ecThresholdExceeded = 101,
	ecPumpingTooLong,
	ecPumpingTooShort,
} testerErrorCodeTypeDef;

typedef enum {
	jmTest = 1,
	jmMonitor,
} JobModeTypeDef;

typedef enum {
	jrNone = 1,
	jrGood,
	jrBad,
	jrMonitor,
	jrCanceled,
	jrError,
	jrNotReliable
} JobResultTypeDef;

typedef enum {
	bcNone = 	0b00000000,
	bcRed = 	0b00000001,
	bcGreen = 	0b00000010,
	bcYellow =	0b00000011,
	bcBlue = 	0b00000100,
	bcMagenta = 0b00000101,
	bcCyan = 	0b00000110,
	bcWhite =	0b00000111,
} BacklightColor_t;



#define CYCLE_N	4 //15
#define CYCLE_DP			2000 //1000//2000
#define CYCLE_CREEP_TIME	10000
#define CYCLE_TOTAL_TIME 	40000
#define CYCLE_MEASURE_TIME	(CYCLE_TOTAL_TIME-CYCLE_CREEP_TIME)

//#define MEASUREMENTS_N 16
typedef enum {
	trNone = 0,
	trGood,
	trBad,
	trNotReady,
} TestResult_t;

typedef struct Measurement_t{
	uint16_t duration;
	int32_t rough_start;
	int16_t fine_stop;
}Measurement_t;

typedef struct TestLog_t{
	int16_t pump_duration;
	Measurement_t measurements[CYCLE_N];
	int16_t test_duration;
	TestResult_t result;
	uint8_t cycle_iter;
}TestLog_t;

typedef struct {
	float a;
	float b;
	float r2;
	float sd;
	float avg;
}TestStats_t;

#define __BUZZER_ON() 		(HAL_GPIO_WritePin(GPIO_Buzzer_GPIO_Port, GPIO_Buzzer_Pin, GPIO_PIN_SET))
#define __BUZZER_OFF() 		(HAL_GPIO_WritePin(GPIO_Buzzer_GPIO_Port, GPIO_Buzzer_Pin, GPIO_PIN_RESET))
#define __BACKLIGHT_R_ON() 	(HAL_GPIO_WritePin(GPIO_Backlight_R_GPIO_Port, GPIO_Backlight_R_Pin, GPIO_PIN_RESET))
#define __BACKLIGHT_R_OFF() (HAL_GPIO_WritePin(GPIO_Backlight_R_GPIO_Port, GPIO_Backlight_R_Pin, GPIO_PIN_SET))
#define __BACKLIGHT_G_ON() 	(HAL_GPIO_WritePin(GPIO_Backlight_G_GPIO_Port, GPIO_Backlight_G_Pin, GPIO_PIN_RESET))
#define __BACKLIGHT_G_OFF() (HAL_GPIO_WritePin(GPIO_Backlight_G_GPIO_Port, GPIO_Backlight_G_Pin, GPIO_PIN_SET))
#define __BACKLIGHT_B_ON() 	(HAL_GPIO_WritePin(GPIO_Backlight_B_GPIO_Port, GPIO_Backlight_B_Pin, GPIO_PIN_RESET))
#define __BACKLIGHT_B_OFF() (HAL_GPIO_WritePin(GPIO_Backlight_B_GPIO_Port, GPIO_Backlight_B_Pin, GPIO_PIN_SET))
#define __GREEN_LED_ON()	(HAL_GPIO_WritePin(GPIO_GreenLED_GPIO_Port, GPIO_GreenLED_Pin, GPIO_PIN_RESET))
#define __GREEN_LED_OFF()	(HAL_GPIO_WritePin(GPIO_GreenLED_GPIO_Port, GPIO_GreenLED_Pin, GPIO_PIN_SET))
#define __GREEN_LED_TOG()	(HAL_GPIO_TogglePin(GPIO_GreenLED_GPIO_Port, GPIO_GreenLED_Pin))
#define __RED_LED_ON()		(HAL_GPIO_WritePin(GPIO_RedLED_GPIO_Port, GPIO_RedLED_Pin, GPIO_PIN_RESET))
#define __RED_LED_OFF()		(HAL_GPIO_WritePin(GPIO_RedLED_GPIO_Port, GPIO_RedLED_Pin, GPIO_PIN_SET))
#define __RED_LED_TOG()		(HAL_GPIO_TogglePin(GPIO_RedLED_GPIO_Port, GPIO_RedLED_Pin))
#define __POWER_LED_ON()	__GREEN_LED_ON()
#define __POWER_LED_OFF()	__GREEN_LED_OFF()
#define __RFID_LED_ON()		__RED_LED_ON()
#define __RFID_LED_OFF()	__RED_LED_OFF()
#define __BTN1_PRESSED()	(HAL_GPIO_ReadPin(GPIO_BTN1_GPIO_Port, GPIO_BTN1_Pin) == GPIO_PIN_RESET)
#define __BTN1_RELEASED()	(HAL_GPIO_ReadPin(GPIO_BTN1_GPIO_Port, GPIO_BTN1_Pin) == GPIO_PIN_SET)
#define __BTN2_PRESSED()	(HAL_GPIO_ReadPin(GPIO_BTN2_GPIO_Port, GPIO_BTN2_Pin) == GPIO_PIN_RESET)
#define __BTN2_RELEASED()	(HAL_GPIO_ReadPin(GPIO_BTN2_GPIO_Port, GPIO_BTN2_Pin) == GPIO_PIN_SET)
#define __BTN3_PRESSED()	(HAL_GPIO_ReadPin(GPIO_BTN3_GPIO_Port, GPIO_BTN3_Pin) == GPIO_PIN_RESET)
#define __BTN3_RELEASED()	(HAL_GPIO_ReadPin(GPIO_BTN3_GPIO_Port, GPIO_BTN3_Pin) == GPIO_PIN_SET)
#define __BTN_F1_PRESSED()	__BTN1_PRESSED()
#define __BTN_F1_RELEASED()	__BTN1_RELEASED()
#define __BTN_F2_PRESSED()	__BTN2_PRESSED()
#define __BTN_F2_RELEASED()	__BTN2_RELEASED()
#define __BTN_OK_PRESSED()	__BTN3_PRESSED()
#define __BTN_OK_RELEASED()	__BTN3_RELEASED()
#define __VALVE1_CLOSE()	(HAL_GPIO_WritePin(GPIO_Valve1_GPIO_Port, GPIO_Valve1_Pin, GPIO_PIN_RESET))
#define __VALVE1_OPEN()		(HAL_GPIO_WritePin(GPIO_Valve1_GPIO_Port, GPIO_Valve1_Pin, GPIO_PIN_SET))
#define __VALVE2_CLOSE()	(HAL_GPIO_WritePin(GPIO_Valve1_GPIO_Port, GPIO_Valve2_Pin, GPIO_PIN_RESET))
#define __VALVE2_OPEN()		(HAL_GPIO_WritePin(GPIO_Valve1_GPIO_Port, GPIO_Valve2_Pin, GPIO_PIN_SET))
#define __DUMP_VALVE_CLOSE()	__VALVE1_CLOSE()
#define __DUMP_VALVE_OPEN()		__VALVE1_OPEN()
#define __DIFF_VALVE_CLOSE()	__VALVE2_CLOSE()
#define __DIFF_VALVE_OPEN()		__VALVE2_OPEN()

#define ABSOLUTE_PRESSURE_TARGET_PA		19000//19000 	// Pa
#define DIFF_CRITICAL_PA				600 	// Pa
#define POSITIVE_RESULT_DROP_PA			150		// Pa
#define PUMPING_SLOW_DOWN_DELTA_PA		2000	// Pa
#define PUMPING_SLOW_DOWN_PA			(ABSOLUTE_PRESSURE_TARGET_PA-PUMPING_SLOW_DOWN_DELTA_PA)
#define PRESSURE_DROP_DELTA_PA			500
#define PRESSURE_DROP_THRESHOLD_PA		(ABSOLUTE_PRESSURE_TARGET_PA-PRESSURE_DROP_DELTA_PA)
#define MEASUREMENT_DURATION			120UL // seconds UPDATE: this is NO LONGER fuck high to emulate infinity XD
#define DUMP_DURATION					10UL // seconds
#define P_ROUGH_CRITICAL				35000 	//Pa
#define PUMPING_DURATION_MIN			800UL // milliseconds
#define PUMPING_DURATION_MAX			60000UL // milliseconds
#define CREEP_DURATION					60000UL // milliseconds
#define DUMP_TIME						5000UL // milliseconds


//#define

#define INIT_WAIT_TIME					1500 // ms
#define INIT_EXIT_PRESSURE				70 // Pa*10

#define IDLE_P_FINE_MAX					2000 // Pa*10
#define IDLE_P_ROUGH_MAX				2000 // Pa

#define PUMP_SPEED_FAST 45
#define PUMP_SPEED_SLOW 30
#define PUMP_SPEED_ZERO 0


#define LANG_PL

#ifdef LANG_PL
#define MSG_CREEP		"Czekam..."
#define MSG_PUMPING		"Pompowanie"
#define MSG_MEASURE		"Pomiar"
#define MSG_TIME_TO_END "DO KONCA"
#define MSG_P_ROUGH 	"SR"
#define MSG_P_FINE 		"SF"
#define MSG_PRESS_KEY1 	"Btn1 - Exit"
#define MSG_PRESS_KEY2 	"Btn2 - Monitor"
#define MSG_P_START 	"Start"
#define MSG_P_HALF 		"Polowa"
#define MSG_P_END 		"Stop"
#define MSG_F1START		"F1 - test"
#define MSG_F1MONITOR	"F2 - kontrola"
#define MSG_DUMP		"Czekam..."
#define MSG_READY		"Gotowy"
#define MSG_RFID_SCAN	"Skanuj"
#define MSG_DEVICE		"urzadzenie"
#define MSG_USER		"uzytkownik"
#define MSG_CANCEL		"F1 - przerwij"
#define MSG_SKIP		"F2 - pomin"
#define MSG_MONITOR  	"Kontrola"
#define MSG_RESULT_BAD  	"Nieszczelny!"
#define MSG_RESULT_GOOD 	"Szczelny :-)"
#define MSG_RESULT_NOTRELIABLE 	"Niepewny"
#define MSG_RESULT_CANCELED "Przerwany"
#define MSG_RESULT_MONITOR  "Kontrola"
#define MSG_RESULT_ERROR  	"B\214\211d"
#define MSG_OKFINISH	"OK - koniec"
#define MSG_ERROR_SERVICE	"Konieczny serwis!"
#define MSG_ERROR_CODE	"Kod:"
#define MSG_DATE_TIME	"Data i godzina"
#endif


// Error codes
#define EC_OK					0x0000

#define EC_POST_FP_SENSOR		0xA010
#define EC_POST_RP_SENSOR		0xA011
#define EC_POST_BATTERY			0xA012
#define EC_POST_SDCARD			0xA013
#define EC_POST_PUMP_TEST		0xA014
#define EC_POST_DIFF_VALVE_TEST	0xA015
#define EC_POST_FP_SEN_CONN		0xA016

void userSystemInit(void);
void changeState(testerStateStructTypeDef *newState);

void pumpOn(void);
void pumpOff(void);

//uint8_t readRFID(OW_Handle_t *ow, uint8_t *tag);
uint8_t readRFIDNB(uint8_t *tag);


///////////////////
// STATE INDEP. //
///////////////////

void device_Init(void);
uint16_t device_POST(void);
void calculatePressure(void);
void goToDump(Buttons_EventTypeDef);
void goToResult(Buttons_EventTypeDef);
void clearDisplay(void); // czysci bufor ekranowy i rysuje np. ramke, do uzycia przy init=1
uint8_t tagsTheSame(uint8_t *tag1, uint8_t *tag2);
void cancelToHold(Buttons_EventTypeDef);
void resetRFIDTags(void);
void sendPressureToRS485(void);
void clear_testlog(void);
uint8_t mv2batlvl(uint16_t);
uint8_t wait4dump(void);
TestResult_t analyze_data(void);
void set_backlight(BacklightColor_t color);
uint16_t log2sd(char *text);

TestResult_t simple_analysis(TestStats_t *stats);
uint32_t read_rtc(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef write_rtc(RTC_HandleTypeDef *hrtc, uint32_t TimeCounter);
HAL_StatusTypeDef rtc_enterinit(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef rtc_exitinit(RTC_HandleTypeDef *hrtc);
///////////////////
// SELF TEST     //
///////////////////
void stSelfTestRefreshDisplay(void);
void stSelfTestBtn1(Buttons_EventTypeDef _event);
void stSelfTestBtn2(Buttons_EventTypeDef _event);
void stSelfTestBtn3(Buttons_EventTypeDef _event);
void stSelfTestHandleHardware(void);

///////////////////
// CALIB BATTERY //
///////////////////
void stCalibBattRefreshDisplay(void);
void stCalibBattBtn1(Buttons_EventTypeDef _event);
void stCalibBattBtn2(Buttons_EventTypeDef _event);
void stCalibBattBtn3(Buttons_EventTypeDef _event);
void stCalibBattHandleHardware(void);

///////////////////
// INIT //
///////////////////
void stInitRefreshDisplay(void);
void stInitHandleHardware(void);

///////////////////
// RFID scan     //
///////////////////

void stRFIDScanRefreshDisplay(void);
void stRFIDScanToStReady(Buttons_EventTypeDef _event);
void stRFIDScanToStPumping(Buttons_EventTypeDef _event);
void stRFIDScanHandleHardware(void);
void stRFIDScanCalculate(void);

///////////////////
// READY //
///////////////////
void stReadyRefreshDisplay(void);
void stReadyToTest(Buttons_EventTypeDef);
void stReadyToMonitor(Buttons_EventTypeDef);
void stReadyToSetTime(Buttons_EventTypeDef);
void stReadyHandleHardware(void);



///////////////////
// PUMPING //
///////////////////
void stPumpingRefreshDisplay(void); // "trwa pomowanie", "pressure: XX kPa"
void stPumpingHandleHardware(void);



///////////////////
// CREEP //
///////////////////
void stCreepRefreshDisplay(void); // "starting in...", "pressure: XX kPa"
void stCreepHandleHardware(void);  // wait 60s



///////////////////
// MEASURE //
///////////////////
void stMeasureRefreshDisplay(void); // "time to end....", pressure F,R
void stMeasureHandleHardware(void); // czas pomairu 20s, P dla T_0, T_05, T_END, zapis do tablicy[3]


///////////////////
// RESULT //
///////////////////
void stResultRefreshDisplay(void); // wyswietl T_0, T_05, T_END, "wcisnij dowolny btn"
void stResultToStDump(Buttons_EventTypeDef);
void stResultToStMonitor(Buttons_EventTypeDef);
void stResultHandleHardware(void);


///////////////////
// DUMP //
///////////////////

void stDumpRefreshDisplay(void); // "spuszczanie powietrza"
void stDumpHandleHardware(void); // czekaj 10s

///////////////////
// MONITOR //
///////////////////
void stMonitorRefreshDisplay(void);
void stMonitorPumpOnOff(Buttons_EventTypeDef);
void stMonitorValveOnOff(Buttons_EventTypeDef);
void stMonitorToStDump(Buttons_EventTypeDef);
void stMonitorToStHold(Buttons_EventTypeDef);
void stMonitorHandleHardware(void);
void calculatePressure(void);


///////////////////
// HOLD          //
///////////////////

void stHoldRefreshDisplay(void);
void stHoldHandleHardware(void);
void stHoldToStDump(Buttons_EventTypeDef);


///////////////////
// ERROR         //
///////////////////

void stErrorRefreshDisplay(void);
void stErrorHandleHardware(void);


///////////////////
// SET TIME      //
///////////////////
void stSetTimeRefreshDisplay(void);
void stSetTimeUp(Buttons_EventTypeDef);
void stSetTimeDown(Buttons_EventTypeDef);
void stSetTimeOK(Buttons_EventTypeDef);
void stSetTimeHandleHardware(void);

#endif /* FUNCTIONS_H_ */
