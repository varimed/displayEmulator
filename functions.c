/*
 * functions.c
 *
 *  Created on: Oct 14, 2019
 *      Author: Artur Pozniak, Grzegorz Palaszynski
 */

#include "functions.h"
#include "buttons.h"
#include "honeywell_hsc.h"
#include "st7565.h"
#include "dispbuf.h"
#include "font_mono5x8.h"
#include "symbols8pt.h"
#include "font_sansserif11pt.h"
#include "bitmaps.h"
#include "main.h"
#include "ow.h"
#include "onewire.h"
#include <time.h>
#include "eeprom.h"
#include "statlib.h"
#include "fatfs.h"

#define HSC_ROUGH_MIN -40000 // Pa
#define HSC_ROUGH_MAX  40000 // Pa
#define HSC_FINE_MIN -6000   // Pa *10
#define HSC_FINE_MAX  6000   // Pa *10
#define HSC_CNT_MIN (1638)
#define HSC_CNT_MAX (14746)

#define VLINE_X 90
#define HLINE_Y 46

#define BTN_COUNT 3

#define SCAN_MODE_USER		0
#define SCAN_MODE_DEVICE	1

// panel buttons
BUTTONS_ItemTypeDef btnF1, btnF2, btnOK;
BUTTONS_ItemTypeDef *buttons[BTN_COUNT] = {
		&btnF1, &btnF2, &btnOK };
BUTTONS_GroupTypeDef btn_group = {
		.buttons = (BUTTONS_ItemTypeDef*) buttons,
		.btn_count = BTN_COUNT };

// LCD display
TDisplayBuffer display_buffer;
uint8_t st7565_buffer[1024];

// RFID
//OW_Handle_t rfid;
uint8_t user_tag[8];
uint8_t device_tag[8];
uint8_t scan_mode;

// virtual EEPROM
uint16_t VirtAddVarTab[NB_OF_VAR];
int16_t VarDataTab[NB_OF_VAR];


JobModeTypeDef job_mode;
JobResultTypeDef job_result;

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
volatile extern uint16_t lion_adc;

int32_t pressureRough = 0, pressureFine = 0;

testerErrorCodeTypeDef errorCode;

TestLog_t test_log;

#define SAMPLES_N   10
int32_t samples_r[SAMPLES_N];
uint8_t samples_pos_r = 0;
int32_t samples_f[SAMPLES_N];
uint8_t samples_pos_f = 0;
void average_Init(int32_t *samples);
int32_t average_Compute(int32_t *samples, uint8_t *position, int32_t value);

void sendCmdEB(void *payload, uint16_t payloadLen);

typedef enum {
	TINY = 0, SMALL, MEDIUM, LARGE, HUGE
} aparatusTypeT;

typedef struct testReportEB {
	uint8_t cmd;
	uint8_t res;
	uint8_t tag1[8];
	uint8_t tag2[8];
	uint16_t initPress;
	uint16_t finalPress;

} testReportEB;

aparatusTypeT aparatusType;



testerStateStructTypeDef stSelfTest = {
		.refreshDisplay = stSelfTestRefreshDisplay,
		.btnF1Event = stSelfTestBtn1,
		.btnF2Event = stSelfTestBtn2,
		.btnOKEvent = stSelfTestBtn3,
		.handleHardware = stSelfTestHandleHardware,
		.calculate = calculatePressure };

testerStateStructTypeDef stCalibBatt = {
		.refreshDisplay = stCalibBattRefreshDisplay,
		.btnF1Event = stCalibBattBtn1,
		.btnF2Event = stCalibBattBtn2,
		.btnOKEvent = stCalibBattBtn3,
		.handleHardware = stCalibBattHandleHardware,
		.calculate = NULL };

testerStateStructTypeDef stRFIDScan = {
		.refreshDisplay = stRFIDScanRefreshDisplay,
		.btnF1Event = stRFIDScanToStReady,
		.btnF2Event = stRFIDScanToStPumping,
		.btnOKEvent = NULL,
		.handleHardware = stRFIDScanHandleHardware,
		.calculate = calculatePressure };

testerStateStructTypeDef stInit = {
		.refreshDisplay = stInitRefreshDisplay,
		.btnF1Event = NULL,
		.btnF2Event = NULL,
		.btnOKEvent = NULL,
		.handleHardware = stInitHandleHardware,
		.calculate = calculatePressure };

testerStateStructTypeDef stReady = {
		.refreshDisplay = stReadyRefreshDisplay,
		.btnF1Event = stReadyToTest,
		.btnF2Event = stReadyToMonitor,
		.btnOKEvent = stReadyToSetTime,
		.handleHardware = stReadyHandleHardware,
		.calculate = calculatePressure };

testerStateStructTypeDef stPumping = {
		.refreshDisplay = stPumpingRefreshDisplay,
		.btnF1Event = cancelToHold,
		.btnF2Event = NULL,
		.btnOKEvent = NULL,
		.handleHardware = stPumpingHandleHardware,
		.calculate = calculatePressure };

testerStateStructTypeDef stCreep = {
		.refreshDisplay = stCreepRefreshDisplay,
		.btnF1Event = cancelToHold,
		.btnF2Event = NULL,
		.btnOKEvent = NULL,
		.handleHardware = stCreepHandleHardware,
		.calculate = calculatePressure };

testerStateStructTypeDef stMeasure = {
		.refreshDisplay = stMeasureRefreshDisplay,
		.btnF1Event = cancelToHold,
		.btnF2Event = NULL,
		.btnOKEvent = NULL,
		.handleHardware = stMeasureHandleHardware,
		.calculate = calculatePressure };

testerStateStructTypeDef stDump = {
		.refreshDisplay = stDumpRefreshDisplay,
		.btnF1Event = NULL,
		.btnF2Event = NULL,
		.btnOKEvent = NULL,
		.handleHardware = stDumpHandleHardware,
		.calculate = calculatePressure };

testerStateStructTypeDef stMonitor = {
		.refreshDisplay = stMonitorRefreshDisplay,
		.btnF1Event = stMonitorToStHold,
		.btnF2Event = NULL,
		.btnOKEvent = NULL,
		.handleHardware = stMonitorHandleHardware,
		.calculate = calculatePressure };

testerStateStructTypeDef stHold = {
		.refreshDisplay = stHoldRefreshDisplay,
		.btnF1Event = NULL,
		.btnF2Event = NULL,
		.btnOKEvent = goToDump,
		.handleHardware = stHoldHandleHardware,
		.calculate = calculatePressure };

testerStateStructTypeDef stError = {
		.refreshDisplay = stErrorRefreshDisplay,
		.btnF1Event = NULL,
		.btnF2Event = NULL,
		.btnOKEvent = NULL,
		.handleHardware = stErrorHandleHardware,
		.calculate = NULL
};

testerStateStructTypeDef stSetTime = {
		.refreshDisplay = stSetTimeRefreshDisplay,
		.btnF1Event = stSetTimeUp,
		.btnF2Event = stSetTimeDown,
		.btnOKEvent = stSetTimeOK,
		.handleHardware = stSetTimeHandleHardware,
		.calculate = NULL
};

uint32_t timer = 0;

testerStateStructTypeDef *curState;

HoneywellHSC_Sensor Sf = {
		.raw_min = HSC_CNT_MIN,
		.raw_max = HSC_CNT_MAX,
		.calc_min = HSC_FINE_MIN,
		.calc_max = HSC_FINE_MAX,
		.readSensor_f = readHoneywellSPI };

HoneywellHSC_Sensor Sr = {
		.raw_min = HSC_CNT_MIN,
		.raw_max = HSC_CNT_MAX,
		.calc_min = HSC_ROUGH_MIN,
		.calc_max = HSC_ROUGH_MAX,
		.readSensor_f = readHoneywellI2C };

uint8_t uart_tx_buf[100];

int16_t batt_divisor;
uint16_t batt_calib_coef;

uint16_t last_error_code;

void device_Init(void) {
	// one beep on startup
	__BUZZER_ON();
	HAL_Delay(25);
	__BUZZER_OFF();

	// enable LCD backlight - white color
	__BACKLIGHT_R_ON();
	__BACKLIGHT_G_ON();
	__BACKLIGHT_B_ON();

	// set panel LEDs state
	__POWER_LED_ON();
	__RFID_LED_OFF();

	// initialize LCD, display splash screenzq4433
	st7565_Init();
	st7565_WriteBuffer(varimed_logo);

	// initialize graphic display buffer
	display_buffer.width = 128;
	display_buffer.height = 64;
	display_buffer.buffer = st7565_buffer;
	dbuf_Init(&display_buffer);
	dbuf_Fill(&display_buffer, 0);

	// initialize front panel buttons
	btnF1.button_port = GPIO_BTN1_GPIO_Port;
	btnF1.button_pin = GPIO_BTN1_Pin;
	btnF1.repeat_delay = 500;
	btnF1.repeat_period = 200;
	btnF1.onEvent = curState->btnF1Event;
	btnF1.active_high = 0;

	btnF2.button_port = GPIO_BTN2_GPIO_Port;
	btnF2.button_pin = GPIO_BTN2_Pin;
	btnF2.repeat_delay = 500;
	btnF2.repeat_period = 200;
	btnF2.onEvent = curState->btnF2Event;
	btnF2.active_high = 0;

	btnOK.button_port = GPIO_BTN3_GPIO_Port;
	btnOK.button_pin = GPIO_BTN3_Pin;
	btnOK.repeat_delay = 500;
	btnOK.repeat_period = 200;
	btnOK.onEvent = curState->btnOKEvent;
	btnOK.active_high = 0;

	buttons_Init(&btn_group);

	//rfid.huart = &huart3;

	// initialize emulated EEPROM
	for (uint8_t VarIndex = 1; VarIndex <= NB_OF_VAR; VarIndex++) {
		VirtAddVarTab[VarIndex - 1] = VarIndex;
	}
	HAL_FLASH_Unlock();
	if (EE_Init() != HAL_OK) {
		Error_Handler();
	}
	EE_ReadVariable(VirtAddVarTab[0], (uint16_t*)&(VarDataTab[0]));

	// apply battery calibration data
	batt_calib_coef = VarDataTab[0];
	batt_divisor = (uint16_t) (4096 + batt_calib_coef);

	//write_rtc(&hrtc, 1618917042U);
	//write_rtc(&hrtc, 1U);

	if (__BTN_F1_PRESSED()) {
		changeState(&stSelfTest);
	} else if (__BTN_F2_PRESSED()) {
		changeState(&stCalibBatt);
	} else {
		changeState(&stInit);
	}
}

uint16_t device_POST(void) {
	uint8_t data[2];
	int32_t p;
	HoneywellHSC_StatusTypeDef result;

	// test fine pressure sensor, SPI
	if (Sf.readSensor_f(data) != EC_OK) return EC_POST_FP_SENSOR;
	// test rough pressure sensor, i2c
	if (Sr.readSensor_f(data) != EC_OK) return EC_POST_RP_SENSOR;

	// diff valve test
	__DIFF_VALVE_OPEN();
	HAL_Delay(1000);
	result = HoneywellHSC_ReadCalcValue(&Sf, &p);
	if (result != HONEYWELL_HSC_STATUS_OK) return EC_POST_FP_SENSOR;
	if (p < -1000 || p > 1000) return EC_POST_DIFF_VALVE_TEST;

	// pump test
	__DIFF_VALVE_CLOSE();
	setPumpPWM(PUMP_SPEED_SLOW);
	HAL_Delay(100);
	setPumpPWM(PUMP_SPEED_ZERO);
	result = HoneywellHSC_ReadCalcValue(&Sf, &p);
	if (result != HONEYWELL_HSC_STATUS_OK) return EC_POST_FP_SENSOR;
	if (p > 2000) {
		return EC_POST_FP_SEN_CONN;
	} else if (p > -5000){
		return EC_POST_PUMP_TEST;
	}

	// diff valve test 2
	__DIFF_VALVE_OPEN();
	HAL_Delay(500);
	result = HoneywellHSC_ReadCalcValue(&Sf, &p);
	if (result != HONEYWELL_HSC_STATUS_OK) return EC_POST_FP_SENSOR;
	if (p < -1000 || p > 1000) return EC_POST_DIFF_VALVE_TEST;

	return EC_OK;
}

void changeState(testerStateStructTypeDef *newState) {
	curState = newState;
	newState->init = siInit;
	btnF1.onEvent = curState->btnF1Event;
	btnF2.onEvent = curState->btnF2Event;
	btnOK.onEvent = curState->btnOKEvent;
}

///////////////////
// STATE INDEP. //
///////////////////

void calculatePressure(void) {
	int32_t p;

	HoneywellHSC_StatusTypeDef result;

	result = HoneywellHSC_ReadCalcValue(&Sr, &p);
	if (result == HONEYWELL_HSC_STATUS_OK) {
		pressureRough = average_Compute(samples_r, &samples_pos_r, p);
	}

	result = HoneywellHSC_ReadCalcValue(&Sf, &p);
	if (result == HONEYWELL_HSC_STATUS_OK) {
		pressureFine = average_Compute(samples_f, &samples_pos_f, p);
	}
}

void goToDump(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		changeState(&stDump);
		break;
	case BUTTON_RELEASED:
		break;
	}
}

void cancelToHold(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		job_result = jrCanceled;
		changeState(&stHold);
		break;
	case BUTTON_RELEASED:
		break;
	}
}


void clearDisplay(void) {
	char _text[20];
	uint16_t mV;
	time_t now;
	struct tm  *ts;

	dbuf_Fill(&display_buffer, 0);
	dbuf_DrawLine(&display_buffer, VLINE_X, HLINE_Y, VLINE_X, 63, 1);
	dbuf_DrawLine(&display_buffer, 0, HLINE_Y, 127, HLINE_Y, 1);

	now = read_rtc(&hrtc);
	ts = gmtime(&now);
	//strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", ts);
	strftime(_text, sizeof(_text), "%H:%M", ts);
	dbuf_PutString(&display_buffer, 0, HLINE_Y + 2, (uint8_t*)_text, &mono5x8, 1);
	strftime(_text, sizeof(_text), "%d/%m/%Y", ts);
	dbuf_PutString(&display_buffer, 0, HLINE_Y + 10, (uint8_t*)_text, &mono5x8, 1);


	//dbuf_PutString(&display_buffer, 0, HLINE_Y + 2, (uint8_t*)buf, &mono5x8, 1);


	//mV = (lion_adc * 33297) / 4095);
	mV = (lion_adc * 33297) / batt_divisor;
	sprintf((char*) _text, "%c %c", mv2batlvl(mV) + 128, 132);
	dbuf_PutString(&display_buffer, VLINE_X - 23, HLINE_Y + 10, (uint8_t*)_text, &symbols8pt, 1);
}
//
//#define BATT_LEVELS_N 10
//#define BATT_MV_WINDOW 25
//uint16_t batt_levels[BATT_LEVELS_N] = { 6900, 7200, 7350, 7400, 7450, 7500,
//		7600, 7800, 7900, 8100 };

#define BATT_LEVELS_N 3
#define BATT_MV_WINDOW 25
uint16_t batt_levels[BATT_LEVELS_N] = { 6900, 7400, 7600};

uint8_t mv2batlvl(uint16_t mv) {
	static uint16_t last_mv = 7500;
	if (mv > last_mv + BATT_MV_WINDOW) {
		last_mv = mv - BATT_MV_WINDOW;
	} else if (mv < last_mv - BATT_MV_WINDOW) {
		last_mv = mv + BATT_MV_WINDOW;
	}
	for (uint8_t i = 0; i < BATT_LEVELS_N; i++) {
		if (last_mv < batt_levels[i]) {
			return i;
		}
	}
	return BATT_LEVELS_N;
}

///////////////////
// INIT //
///////////////////
void stInitRefreshDisplay(void) {
	st7565_WriteBuffer(varimed_logo);
}
void stInitHandleHardware(void) {
	if (curState->init == siInit) {
		curState->init = siNone;
		timer = HAL_GetTick();
		last_error_code = device_POST();
	}
	if (HAL_GetTick() - timer > INIT_WAIT_TIME) {
		if (last_error_code == EC_OK) {
			changeState(&stReady);
		} else {
			changeState(&stError);
		}
	}
	// no need to open valves, stReady handles that
	/*
	if (HAL_GetTick() - timer > INIT_WAIT_TIME) {
		// close if pressure OK
		if (pressureRough < INIT_EXIT_PRESSURE * 10) {
			__DUMP_VALVE_CLOSE();
			__DIFF_VALVE_CLOSE();

			uint8_t _text[25];
			HAL_GPIO_WritePin(GPIO_RS485_OE_GPIO_Port, GPIO_RS485_OE_Pin,
					GPIO_PIN_SET);
			uint8_t _len = sprintf((char*) _text, "#Czesc to ja, RS485\n");
			HAL_UART_Transmit(&huart2, _text, _len, 1000);
			HAL_GPIO_WritePin(GPIO_RS485_OE_GPIO_Port, GPIO_RS485_OE_Pin,
					GPIO_PIN_RESET);




			changeState(&stReady);
		}
	} else {
		__DIFF_VALVE_OPEN();
		__DUMP_VALVE_OPEN();
	}
	*/
}

///////////////////
// READY //
///////////////////
void stReadyRefreshDisplay(void) {
	clearDisplay();
	dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_READY, &sansserif_11pt, 1);
	dbuf_PutString(&display_buffer, 0, 18, (uint8_t*)MSG_F1START, &mono5x8, 1);
	dbuf_PutString(&display_buffer, 0, 27, (uint8_t*)MSG_F1MONITOR, &mono5x8, 1);
	st7565_WriteBuffer(display_buffer.buffer);
}

void stReadyToTest(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		job_mode = jmTest;
		changeState(&stRFIDScan);
		break;
	case BUTTON_RELEASED:
		break;
	}
}

void stReadyToSetTime(Buttons_EventTypeDef _event){
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		changeState(&stSetTime);
		break;
	case BUTTON_RELEASED:
		break;
	}
}

void stReadyToMonitor(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		job_mode = jmMonitor;
		changeState(&stMonitor);
		break;
	case BUTTON_RELEASED:
		break;
	}
}

void stReadyHandleHardware(void) {
	static uint32_t valve_tick;

	if (curState->init == siInit) {
		set_backlight(bcWhite);
		curState->init = siNone;
		//__DUMP_VALVE_CLOSE();
		//__DIFF_VALVE_CLOSE();
		setPumpPWM(PUMP_SPEED_ZERO);
		//HAL_GPIO_WritePin(GPIO_RedLED_GPIO_Port, GPIO_RedLED_Pin, GPIO_PIN_SET);
		average_Init(samples_r);
		average_Init(samples_f);
		resetRFIDTags();
		valve_tick = HAL_GetTick();
		clear_testlog();
	}
	if (abs(pressureFine) > IDLE_P_FINE_MAX) {
		__DIFF_VALVE_OPEN();
		valve_tick = HAL_GetTick();
	} else if (HAL_GetTick() - valve_tick > 1000) {
		__DIFF_VALVE_CLOSE();
	}
	if (abs(pressureRough) > IDLE_P_ROUGH_MAX) {
		valve_tick = HAL_GetTick();
		__DUMP_VALVE_OPEN();
	} else if (HAL_GetTick() - valve_tick > 1000) {
		__DUMP_VALVE_CLOSE();
	}
}

///////////////////
// PUMPING //
///////////////////
void stPumpingRefreshDisplay(void) {
	uint8_t _text[20] = { 0 };
	uint16_t _elapsed_time;
	int32_t _pressure;

	clearDisplay();
	dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_PUMPING, &sansserif_11pt, 1);
	// display iteration number
	sprintf((char*) _text, "%d/%d", test_log.cycle_iter + 1, CYCLE_N);
	dbuf_PutString(&display_buffer, 95, 0, _text, &mono5x8, 1);
	// display elapsed time
	_elapsed_time = (HAL_GetTick() - timer) / 1000;
	sprintf((char*) _text, "%ds", _elapsed_time);
	dbuf_PutString(&display_buffer, 0, HLINE_Y - 9, _text, &mono5x8, 1);
	// display CANCEL info
	dbuf_PutString(&display_buffer, 0, 18, (uint8_t*)MSG_CANCEL, &mono5x8, 1);
	// display rough pressure
	_pressure = pressureRough / 10;
	if (_pressure < 50)
		_pressure = 0;
	sprintf((char*) _text, "%d.%02d", (int16_t) (_pressure / 100),
			(int16_t) (_pressure % 100));
	dbuf_PutString(&display_buffer, VLINE_X + 2, HLINE_Y + 2, _text,
			&sansserif_11pt, 1);
	st7565_WriteBuffer(display_buffer.buffer);
}
void stPumpingHandleHardware(void) {
	uint32_t pumping_duration;

	if (abs(pressureRough) > P_ROUGH_CRITICAL) {
		errorCode = ecThresholdExceeded;
		job_result = jrError;
		changeState(&stHold);
		return;
	}

	if (curState->init == siInit) {
		curState->init = siNone;
		set_backlight(bcWhite);
		timer = HAL_GetTick();
		__DUMP_VALVE_CLOSE(); // close exhaust valve
		__DIFF_VALVE_OPEN(); // open diff valve
		setPumpPWM(PUMP_SPEED_SLOW);
		log2sd("Pumping started");
	}

	if (pressureRough >= PUMPING_SLOW_DOWN_PA) {
		setPumpPWM(PUMP_SPEED_SLOW);
	} else if (HAL_GetTick() - timer > 2000) {
		setPumpPWM(PUMP_SPEED_FAST);
	}
	if (pressureRough
			>= ABSOLUTE_PRESSURE_TARGET_PA + test_log.cycle_iter * CYCLE_DP) {
		setPumpPWM(PUMP_SPEED_ZERO);
		if (test_log.cycle_iter == 0) {
			pumping_duration = HAL_GetTick() - timer;
			if (pumping_duration < PUMPING_DURATION_MIN){
				errorCode = ecPumpingTooShort;
				job_result = jrError;
				changeState(&stHold);
				return;
			}
			test_log.pump_duration = pumping_duration / 1000; //store first pumping duration in milliseconds
		}
		changeState(&stCreep);
		return;
	}

	if (HAL_GetTick() - timer > PUMPING_DURATION_MAX) {
		// cos poszlo nie tak, nadal po timeoucie nie ma cisnienia ABSOLUTE_PRESSURE_LIMIT_KPA
		// wywolaj Emergency
		errorCode = ecPumpingTooLong;
		job_result = jrError;
		changeState(&stHold);
	}
}

///////////////////
// CREEP //
///////////////////

void stCreepRefreshDisplay(void) {
	int32_t _pressure;
	uint8_t _text[20];
	uint16_t _remaining_time;

	if (curState->init == siInit) {
		return;
	}
	clearDisplay();
	dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_CREEP, &sansserif_11pt, 1);
	// display iteration number
	sprintf((char*) _text, "%d/%d", test_log.cycle_iter + 1, CYCLE_N);
	dbuf_PutString(&display_buffer, 95, 0, _text, &mono5x8, 1);
	// display remaining time
	if (test_log.cycle_iter == 0) {
		_remaining_time = (CREEP_DURATION + timer - HAL_GetTick())
				/ 1000;
	} else {
		_remaining_time = (CYCLE_CREEP_TIME + timer - HAL_GetTick()) / 1000;
	}
	sprintf((char*) _text, "%ds", _remaining_time);
	dbuf_PutString(&display_buffer, 0, HLINE_Y - 9, _text, &mono5x8, 1);
	// display CANCEL info
	dbuf_PutString(&display_buffer, 0, 18, (uint8_t*)MSG_CANCEL, &mono5x8, 1);
	// display rough pressure
	_pressure = pressureRough / 10;
	if (_pressure < 50)
		_pressure = 0;
	sprintf((char*) _text, "%d.%02d", (int16_t) (_pressure / 100),
			(int16_t) (_pressure % 100));
	dbuf_PutString(&display_buffer, VLINE_X + 2, HLINE_Y + 2, _text,
			&sansserif_11pt, 1);
	st7565_WriteBuffer(display_buffer.buffer);
}

void stCreepHandleHardware(void) {
	static uint8_t repumping;

	if (curState->init == siInit) {
		timer = HAL_GetTick();
		curState->init = siNone;
		set_backlight(bcWhite);
		repumping = 0;
	}
	if (test_log.cycle_iter == 0) {
		if (HAL_GetTick() - timer > CREEP_DURATION) {
			changeState(&stMeasure);
		}
	} else {
		if (HAL_GetTick() - timer > CYCLE_CREEP_TIME) {
			changeState(&stMeasure);
		}
	}
	///////////////////
	// RE-PUMP!!!
	///////////////////
	if (repumping) {
		if (pressureRough
				>= ABSOLUTE_PRESSURE_TARGET_PA
						+ test_log.cycle_iter * CYCLE_DP) {
			setPumpPWM(PUMP_SPEED_ZERO);
			repumping = 0;
		}
	} else {
		if (pressureRough
				< PRESSURE_DROP_THRESHOLD_PA + test_log.cycle_iter * CYCLE_DP) {
			setPumpPWM(PUMP_SPEED_SLOW);
			repumping = 1;
		}
	}
}

///////////////////
// MEASURE //
///////////////////

void stMeasureRefreshDisplay(void) {
	uint16_t _remaining_time;
	uint8_t _text[20] = { 0 };
	int32_t _pressure;

	if (curState->init == siNone) {
		clearDisplay();
		dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_MEASURE, &sansserif_11pt, 1);
		// display iteration number
		sprintf((char*) _text, "%d/%d", test_log.cycle_iter + 1, CYCLE_N);
		dbuf_PutString(&display_buffer, 95, 0, _text, &mono5x8, 1);
		// display CANCEL info
		dbuf_PutString(&display_buffer, 0, 18, (uint8_t*)MSG_CANCEL, &mono5x8, 1);
		// display remaining time
		_remaining_time = (CYCLE_MEASURE_TIME + timer - HAL_GetTick()) / 1000;
		sprintf((char*) _text, "%3ds", _remaining_time);
		dbuf_PutString(&display_buffer, 0, HLINE_Y - 9, _text, &mono5x8, 1);
		// dispaly fine pressure
		sprintf((char*) _text, "Fine: %d", (int16_t) (pressureFine / 10));
		dbuf_PutString(&display_buffer, 0, HLINE_Y - 18, _text, &mono5x8, 1);
		// display rought pressure
		_pressure = pressureRough / 10;
		if (_pressure < 50)
			_pressure = 0;
		sprintf((char*) _text, "%d.%02d", (int16_t) (_pressure / 100),
				(int16_t) (_pressure % 100));
		dbuf_PutString(&display_buffer, VLINE_X + 2, HLINE_Y + 2, _text,
				&sansserif_11pt, 1);

		st7565_WriteBuffer(display_buffer.buffer);
	}
}

void clear_testlog(void) {
	uint8_t i;

	test_log.pump_duration = 0;
	test_log.result = trNone;
	test_log.test_duration = 0;
	test_log.cycle_iter = 0;
	for (i = 0; i < CYCLE_N; i++) {
		test_log.measurements[i].duration = 0;
		test_log.measurements[i].fine_stop = 0;
		test_log.measurements[i].rough_start = 0;
	}
}

void stMeasureHandleHardware(void) {
	if (curState->init == siInit) {
		curState->init = siNone;
		set_backlight(bcWhite);
		timer = HAL_GetTick();
		setPumpPWM(PUMP_SPEED_ZERO);
		__DIFF_VALVE_CLOSE();
		test_log.measurements[test_log.cycle_iter].rough_start = pressureRough;
	}

	if (HAL_GetTick() - timer >= CYCLE_MEASURE_TIME) {
		test_log.measurements[test_log.cycle_iter].duration = (HAL_GetTick()
				- timer) / 1000;
		test_log.measurements[test_log.cycle_iter].fine_stop = pressureFine;
		test_log.cycle_iter++;
		if (test_log.cycle_iter == CYCLE_N) {
			switch(analyze_data()){
			case trGood:
				job_result = jrGood;
				break;
			case trBad:
				job_result = jrBad;
				break;
			case trNotReady:
				job_result = jrNotReliable;
				break;
			}
			changeState(&stHold);
			return;
		} else {
			changeState(&stPumping);
			return;
		}
	}

	if (abs(pressureFine) >= DIFF_CRITICAL_PA * 10) {
		test_log.measurements[test_log.cycle_iter].duration = (HAL_GetTick()
				- timer) / 1000;
		test_log.measurements[test_log.cycle_iter].fine_stop = pressureFine;
		test_log.cycle_iter++;
		if (test_log.cycle_iter == CYCLE_N) {
			switch(analyze_data()){
			case trGood:
				job_result = jrGood;
				break;
			case trBad:
				job_result = jrBad;
				break;
			case trNotReady:
				job_result = jrNotReliable;
				break;
			}
			changeState(&stHold);
			return;
		} else {
			changeState(&stPumping);
			return;
		}
	}

}

///////////////////
// DUMP //
///////////////////

void stDumpRefreshDisplay(void) {
	uint8_t _text[20];
	int32_t _pressure;

	if (curState->init == siNone) {
		clearDisplay();
		dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_DUMP, &sansserif_11pt, 1);
		// wyswietl aktualna wartosc zgrubna
		_pressure = pressureRough / 10;
		if (_pressure < 50)
			_pressure = 0;
		sprintf((char*) _text, "%d.%02d", (int16_t) (_pressure / 100),
				(int16_t) (_pressure % 100));
		dbuf_PutString(&display_buffer, VLINE_X + 2, HLINE_Y + 2, _text,
				&sansserif_11pt, 1);
	}
	st7565_WriteBuffer(display_buffer.buffer);
}

void stDumpHandleHardware(void) {
	if (curState->init == siInit) {
		curState->init = siNone;
		__DUMP_VALVE_OPEN();
		__DIFF_VALVE_OPEN();
	}
	if (pressureRough > 100)
		timer = HAL_GetTick();
	if (pressureRough < 100) {
		if (HAL_GetTick() - timer > DUMP_TIME)
			changeState(&stReady);
	}
}

///////////////////
// MONITOR //
///////////////////
void stMonitorRefreshDisplay(void) {
	int32_t _pressure;
	uint16_t _elapTime, m, s;
	uint8_t _text[20];

	if (curState->init == siInit) {
		return;
	}
	clearDisplay();
	_elapTime = (uint16_t) ((HAL_GetTick() - timer) / 1000);
	m = _elapTime / 60;
	s = _elapTime % 60;
	dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_MONITOR, &sansserif_11pt, 1);
	dbuf_PutString(&display_buffer, 0, 18, (uint8_t*)MSG_CANCEL, &mono5x8, 1);
	sprintf((char*) _text, "%d:%02d", m, s);
	dbuf_PutString(&display_buffer, 0, HLINE_Y - 9, _text, &mono5x8, 1);

	// wyswietl aktualna wartosc zgrubna
	_pressure = pressureRough / 10;
	if (_pressure < 50)
		_pressure = 0;
	sprintf((char*) _text, "%d.%02d", (int16_t) (_pressure / 100),
			(int16_t) (_pressure % 100));
	dbuf_PutString(&display_buffer, VLINE_X + 2, HLINE_Y + 2, _text,
			&sansserif_11pt, 1);

	st7565_WriteBuffer(display_buffer.buffer);
}

void stMonitorToStHold(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		//setPumpPWM(PUMP_SPEED_ZERO);
		job_result = jrMonitor;
		changeState(&stHold);
		break;
	case BUTTON_RELEASED:
		break;
	}
}

void stMonitorHandleHardware(void) {
	static uint32_t valve_tick;

	if (curState->init == siInit) {
		curState->init = siNone;
		timer = HAL_GetTick();
		valve_tick = HAL_GetTick();
		log2sd("Monitoring started");
		__DUMP_VALVE_CLOSE();
	}
	///////////////////
	// RE-PUMP!!!
	///////////////////
	if (pressureRough <= PRESSURE_DROP_THRESHOLD_PA + CYCLE_N * CYCLE_DP) {
		setPumpPWM(PUMP_SPEED_FAST);
		__DIFF_VALVE_OPEN();
	}
	if (pressureRough >= ABSOLUTE_PRESSURE_TARGET_PA + CYCLE_N * CYCLE_DP) {
		setPumpPWM(PUMP_SPEED_ZERO);
	} else if (pressureRough >= PUMPING_SLOW_DOWN_PA + CYCLE_N * CYCLE_DP) {
		setPumpPWM(PUMP_SPEED_SLOW); // slow down as closing to the limit
		__DIFF_VALVE_OPEN();
	}

	if (abs(pressureFine) > IDLE_P_FINE_MAX) {
		__DIFF_VALVE_OPEN();
		valve_tick = HAL_GetTick();
	} else if (HAL_GetTick() - valve_tick > 1000) {
		__DIFF_VALVE_CLOSE();
	}
}

///////////////////
// HOLD          //
///////////////////

void stHoldHandleHardware(void) {
	static uint32_t valve_tick;
	// unused__ uint8_t _len;
	// unused__ uint8_t _text[100];

	if (curState->init == siInit) {
		curState->init = siNone;
		valve_tick = HAL_GetTick();
	}
	setPumpPWM(PUMP_SPEED_ZERO);
	__DUMP_VALVE_CLOSE();
	if (abs(pressureFine) > IDLE_P_FINE_MAX) {
		__DIFF_VALVE_OPEN();
		valve_tick = HAL_GetTick();
	} else if (HAL_GetTick() - valve_tick > 1000) {
		__DIFF_VALVE_CLOSE();
	}
}

void stHoldRefreshDisplay(void) {
	uint8_t _text[30] = { 0 };
	int32_t _pressure;

	clearDisplay();

	// wyswietl aktualna wartosc zgrubna
	_pressure = pressureRough / 10;
	if (_pressure < 50)
		_pressure = 0;
	sprintf((char*) _text, "%d.%02d", (int16_t) (_pressure / 100),
			(int16_t) (_pressure % 100));
	dbuf_PutString(&display_buffer, VLINE_X + 2, HLINE_Y + 2, _text,
			&sansserif_11pt, 1);

	dbuf_PutString(&display_buffer, 0, HLINE_Y - 8, (uint8_t*)MSG_OKFINISH, &mono5x8, 1);

	switch (job_result) {
	case jrBad:
		set_backlight(bcRed);
		dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_RESULT_BAD, &sansserif_11pt,
				1);
		dbuf_PutString(&display_buffer, 0, HLINE_Y - 8, (uint8_t*)MSG_OKFINISH, &mono5x8,
				1);
		break;
	case jrCanceled:
		dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_RESULT_CANCELED,
				&sansserif_11pt, 1);
		break;
	case jrError:
		dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_RESULT_ERROR, &sansserif_11pt,
				1);
		switch (errorCode) {
		case ecPumpingTooLong:
			dbuf_PutString(&display_buffer, 0, HLINE_Y - 27,
					(uint8_t*) "Pompowanie nieudane", &mono5x8, 1);
			break;
		case ecPumpingTooShort:
			dbuf_PutString(&display_buffer, 0, HLINE_Y - 27,
					(uint8_t*) "Pompowanie za krotkie", &mono5x8, 1);
			break;
		default:
			dbuf_PutString(&display_buffer, 0, HLINE_Y - 27,
					(uint8_t*) "Nieznany blad", &mono5x8, 1);
		}
		break;
	case jrGood:
		set_backlight(bcGreen);
		dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_RESULT_GOOD, &sansserif_11pt,
				1);
		dbuf_PutString(&display_buffer, 0, HLINE_Y - 8, (uint8_t*)MSG_OKFINISH, &mono5x8,
				1);
		break;
	case jrMonitor:
		dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_RESULT_MONITOR,
				&sansserif_11pt, 1);
		break;
	case jrNone:
		break;
	case jrNotReliable:
		set_backlight(bcYellow);
		dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_RESULT_NOTRELIABLE, &sansserif_11pt,
				1);
		dbuf_PutString(&display_buffer, 0, HLINE_Y - 8, (uint8_t*)MSG_OKFINISH, &mono5x8,
				1);
		break;
	}

	st7565_WriteBuffer(display_buffer.buffer);
}

void stHoldToStDump(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		changeState(&stDump);
		break;
	case BUTTON_RELEASED:
		break;
	}
}

///////////////////
// SELF TEST     //
///////////////////

void stSelfTestRefreshDisplay(void) {
	uint8_t _text[20] = { 0 };
	int32_t _pressure;

	clearDisplay();
	dbuf_Fill(&display_buffer, 0);

	sprintf((char*) _text, "Tick:%lu", timer / 1000);
	dbuf_PutString(&display_buffer, 0, 0, _text, &mono5x8, 1);

	_pressure = pressureRough;
	sprintf((char*) _text, "SR:%2d.%03dkPa", (int16_t) (_pressure / 1000),
			(int16_t) abs(_pressure % 1000));
	dbuf_PutString(&display_buffer, 0, 9, _text, &mono5x8, 1);

	_pressure = pressureFine;
	sprintf((char*) _text, "SF:%4d.%dPa", (int16_t) (_pressure / 10),
			(int16_t) abs(_pressure % 10));
	dbuf_PutString(&display_buffer, 0, 18, _text, &mono5x8, 1);

	sprintf((char*) _text, "Batt: %u mV", (lion_adc * 33297) / 4095);
	dbuf_PutString(&display_buffer, 0, 27, _text, &mono5x8, 1);

	uint32_t czas;
	czas = read_rtc(&hrtc);
	sprintf((char*) _text, "RTC: %lu, %d", czas>>16U, (uint16_t)(czas & 0xFFFF) );
	dbuf_PutString(&display_buffer, 0, 36, _text, &mono5x8, 1);


	//HAL_RTC_SetTime(hrtc, sTime, Format)

	st7565_WriteBuffer(display_buffer.buffer);
}

void stSelfTestBtn1(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		__DUMP_VALVE_OPEN();
		break;
	case BUTTON_RELEASED:
		__DUMP_VALVE_CLOSE();
		break;
	}
}

void stSelfTestBtn2(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		__DIFF_VALVE_OPEN();
		break;
	case BUTTON_RELEASED:
		__DIFF_VALVE_CLOSE();
		break;
	}
}

void stSelfTestBtn3(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		setPumpPWM(PUMP_SPEED_SLOW);
		__BUZZER_ON();
		break;
	case BUTTON_RELEASED:
		setPumpPWM(PUMP_SPEED_ZERO);
		__BUZZER_OFF();
		break;
	}
}

void stSelfTestHandleHardware(void) {
	// unused__ static uint8_t blight = 0;
	if (curState->init == siInit) {
		curState->init = siNone;
		timer = HAL_GetTick();
		__GREEN_LED_ON();
		__RED_LED_ON();
		set_backlight(bcWhite);
	}
}

///////////////////
// CALIB BATTERY //
///////////////////

void stCalibBattRefreshDisplay(void) {
	uint8_t _text[20] = { 0 };
	// unused__ int32_t _pressure;

	clearDisplay();
	dbuf_Fill(&display_buffer, 0);

	sprintf((char*) _text, "Calib val:%d", VarDataTab[0]);
	dbuf_PutString(&display_buffer, 0, 0, _text, &mono5x8, 1);

	sprintf((char*) _text, "Batt: %u mV", (lion_adc * 33297) / batt_divisor);
	dbuf_PutString(&display_buffer, 0, 27, _text, &mono5x8, 1);

	st7565_WriteBuffer(display_buffer.buffer);
}

void stCalibBattBtn1(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		VarDataTab[0]--;
		batt_calib_coef = VarDataTab[0];
		batt_divisor = 4095 + batt_calib_coef;
		break;
	case BUTTON_RELEASED:
		break;
	}
}

void stCalibBattBtn2(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		VarDataTab[0]++;
		batt_calib_coef = VarDataTab[0];
		batt_divisor = 4095 + batt_calib_coef;
		break;
	case BUTTON_RELEASED:
		break;
	}
}

void stCalibBattBtn3(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		HAL_GPIO_WritePin(GPIO_Buzzer_GPIO_Port, GPIO_Buzzer_Pin, GPIO_PIN_SET);
		break;
	case BUTTON_RELEASED:
		EE_WriteVariable(VirtAddVarTab[0], VarDataTab[0]);
		HAL_GPIO_WritePin(GPIO_Buzzer_GPIO_Port, GPIO_Buzzer_Pin,
				GPIO_PIN_RESET);
		break;
	}
}

void stCalibBattHandleHardware(void) {
	// unused__ static uint8_t blight = 0;
	if (curState->init == siInit) {
		curState->init = siNone;

		for (uint8_t VarIndex = 1; VarIndex <= NB_OF_VAR; VarIndex++) {
			VirtAddVarTab[VarIndex - 1] = VarIndex;
		}
		//HAL_FLASH_Unlock();
		//if( EE_Init() != HAL_OK)
		//{
		//  Error_Handler();
		//}
		while (HAL_GPIO_ReadPin(GPIO_BTN1_GPIO_Port, GPIO_BTN2_Pin)
				== GPIO_PIN_RESET)
			;
		EE_ReadVariable(VirtAddVarTab[0], (uint16_t*)&(VarDataTab[0]));

		timer = HAL_GetTick();
		__GREEN_LED_ON();
		__RED_LED_ON();
		__BACKLIGHT_B_OFF();
		__BACKLIGHT_G_OFF();
		__BACKLIGHT_R_ON();
	}
}

///////////////////
// RFID scan     //
///////////////////

void stRFIDScanRefreshDisplay(void) {
	clearDisplay();
	dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_RFID_SCAN, &sansserif_11pt, 1);
	switch (scan_mode) {
	case SCAN_MODE_USER:
		dbuf_PutString(&display_buffer, 0, 17, (uint8_t*)MSG_USER, &mono5x8, 1);
		break;
	case SCAN_MODE_DEVICE:
		dbuf_PutString(&display_buffer, 0, 17, (uint8_t*)MSG_DEVICE, &mono5x8, 1);
		break;
	}
	dbuf_PutString(&display_buffer, 0, 29, (uint8_t*)MSG_CANCEL, &mono5x8, 1);
	dbuf_PutString(&display_buffer, 0, 38, (uint8_t*)MSG_SKIP, &mono5x8, 1);

	st7565_WriteBuffer(display_buffer.buffer);
}

void stRFIDScanToStReady(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		changeState(&stReady);
		break;
	case BUTTON_RELEASED:
		break;
	}
}

void stRFIDScanToStPumping(Buttons_EventTypeDef _event) {
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		resetRFIDTags();
		changeState(&stPumping);
		break;
	case BUTTON_RELEASED:
		break;
	}
}

void stRFIDScanHandleHardware(void) {
	uint8_t card_detected = 0;

	if (curState->init == siInit) {
		curState->init = siNone;
		scan_mode = SCAN_MODE_USER;
		timer = HAL_GetTick();
	}
	if (HAL_GetTick() - timer > 500) {
		switch (scan_mode) {
		case SCAN_MODE_USER:
			card_detected = readRFIDNB(user_tag);
			break;
		case SCAN_MODE_DEVICE:
			card_detected = readRFIDNB(device_tag);
			break;
		}
		HAL_GPIO_TogglePin(GPIO_RedLED_GPIO_Port, GPIO_RedLED_Pin);
		HAL_GPIO_WritePin(GPIO_Buzzer_GPIO_Port, GPIO_Buzzer_Pin, GPIO_PIN_SET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(GPIO_Buzzer_GPIO_Port, GPIO_Buzzer_Pin,
				GPIO_PIN_RESET);
		if (card_detected) {
			if (!tagsTheSame(user_tag, device_tag)) {
				HAL_GPIO_WritePin(GPIO_Buzzer_GPIO_Port, GPIO_Buzzer_Pin,
						GPIO_PIN_SET);
				HAL_Delay(500);
				HAL_GPIO_WritePin(GPIO_Buzzer_GPIO_Port, GPIO_Buzzer_Pin,
						GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIO_RedLED_GPIO_Port, GPIO_RedLED_Pin,
						GPIO_PIN_SET);
				switch (scan_mode) {
				case SCAN_MODE_USER:
					scan_mode = SCAN_MODE_DEVICE;
					break;
				case SCAN_MODE_DEVICE:
					changeState(&stPumping);
					break;
				}
			}
		}
		timer += 500;
	}
}

uint8_t readRFIDNB(uint8_t *tag) {
	ow_readRFID(tag);
	//owstart_wr_rd(wr_ptr, 2, tag, 8);
	while (ow_busy) {
	}; // wait for the answer

	if (ow_state == OWS_NOTPRESENT) {
		return 0;
	} else if (ow_state == OWS_IDLE) {
		return 1;
	}

	return 0;

}

//uint8_t readRFID(OW_Handle_t *ow, uint8_t *tag){
//	uint8_t no_card;
//
//	no_card = OW_Reset(ow);
//	if(no_card == 0){
//		//OW_WriteByte(&ow, 0xCC);
//		OW_WriteByte(ow, 0x33);
//		tag[0] = OW_ReadByte(ow);
//		tag[1] = OW_ReadByte(ow);
//		tag[2] = OW_ReadByte(ow);
//		tag[3] = OW_ReadByte(ow);
//		tag[4] = OW_ReadByte(ow);
//		tag[5] = OW_ReadByte(ow);
//		tag[6] = OW_ReadByte(ow);
//		tag[7] = OW_ReadByte(ow);
//	}else{
//		memset(tag,0,8);
//	}
//	return !(no_card);
//}

uint8_t tagsTheSame(uint8_t *tag1, uint8_t *tag2) {
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if (tag1[i] != tag2[i])
			return 0;
	}
	return 1;
}

void average_Init(int32_t *samples) {
	uint8_t i;

	for (i = 0; i < SAMPLES_N; i++) {
		samples[i] = 0;
	};
}

int32_t average_Compute(int32_t *samples, uint8_t *position, int32_t value) {
	int32_t sum = 0;
	uint8_t i;

	samples[*position] = value;
	(*position)++;
	if ((*position) >= SAMPLES_N)
		(*position) = 0;
	for (i = 0; i < SAMPLES_N; i++) {
		sum += samples[i];
	}
	return (sum / SAMPLES_N);
}

void resetRFIDTags(void) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		user_tag[i] = 0;
		device_tag[i] = 0;
	}
}

void sendCmdEB(void *payload, uint16_t payloadLen) {
	uint8_t frame[100] = { 0 };
	frame[0] = 0xCC; // start
	frame[1] = 0x69; // start

	frame[2] = payloadLen >> 8;
	frame[3] = payloadLen & 0x00FF;

	memcpy(&(frame[4]), (uint8_t*) payload, payloadLen);

	//memcpy(&(frame[payloadLen+4]), 0x12, 1);
	//memcpy(&(frame[payloadLen+4]), 0xcc, 1);

	frame[payloadLen + 4] = 0x96;
	frame[payloadLen + 5] = 0x99;

	HAL_GPIO_WritePin(GPIO_RS485_OE_GPIO_Port, GPIO_RS485_OE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2, frame, payloadLen + 6, 100);
	HAL_GPIO_WritePin(GPIO_RS485_OE_GPIO_Port, GPIO_RS485_OE_Pin,
			GPIO_PIN_RESET);
}

void sendPressureToRS485(void) {
	HAL_GPIO_WritePin(GPIO_RS485_OE_GPIO_Port, GPIO_RS485_OE_Pin, GPIO_PIN_SET);
	sprintf((char*) uart_tx_buf, "%ld\t%ld\n", pressureRough, pressureFine);
	HAL_UART_Transmit(&huart2, uart_tx_buf, strlen((char*) uart_tx_buf), 20);
	HAL_GPIO_WritePin(GPIO_RS485_OE_GPIO_Port, GPIO_RS485_OE_Pin,
			GPIO_PIN_RESET);
}

uint8_t wait4dump(void) {
	int32_t p;

	HoneywellHSC_StatusTypeDef result;

	result = HoneywellHSC_ReadCalcValue(&Sr, &p);
	if (result == HONEYWELL_HSC_STATUS_OK) {
		pressureRough = average_Compute(samples_r, &samples_pos_r, p);
	}

	result = HoneywellHSC_ReadCalcValue(&Sf, &p);
	if (result == HONEYWELL_HSC_STATUS_OK) {
		pressureFine = average_Compute(samples_f, &samples_pos_f, p);
	}
	if (abs(pressureRough) < IDLE_P_ROUGH_MAX/2 && abs(pressureFine) < IDLE_P_FINE_MAX/2) {
		return 0;
	} else {
		__DUMP_VALVE_OPEN();
		__DIFF_VALVE_OPEN();
		return 1;
	}
}

TestResult_t analyze_data(void) {
	float xs[CYCLE_N];
	float ys[CYCLE_N];
	float a = .0, b = .0, r2 = .0, avg = .0, sd = .0;
	int32_t tmp;
	uint8_t i;
	uint8_t _len;
	uint8_t _text[100];

	// calculate as per minute value!
	for (i = 0; i < CYCLE_N; i++) {
		xs[i] = (float) (test_log.measurements[i].rough_start) / 1000.0;
		ys[i] = (float) (test_log.measurements[i].fine_stop) * 60.0
				/ (float) test_log.measurements[i].duration / 10.0;
	}
	a = calcualte_a(xs, ys);
	b = calcualte_b(xs, ys);
	r2 = calculate_r2(xs, ys, a, b);
	avg = mean(ys);
	sd = std_dev_pop(ys);
	//printf("przeciecie: %2.2f, kierunkowy: %2.6f, R2: %2.6f, mean: %2.2f, std_dev: %2.2f\n", a, b, r2, avg, sd);

	// send to RS485
	HAL_GPIO_WritePin(GPIO_RS485_OE_GPIO_Port, GPIO_RS485_OE_Pin, GPIO_PIN_SET);
	_len = sprintf((char*) _text, "#Pump duration: %d\n",
			test_log.pump_duration);
	HAL_UART_Transmit(&huart2, _text, _len, 1000);
	_len = sprintf((char*) _text, "#Test duration: %d\n",
			test_log.test_duration);
	HAL_UART_Transmit(&huart2, _text, _len, 1000);

	tmp = (int32_t) (a * 100.0);
	_len = sprintf((char*) _text, "#A=%ld.%02d\n", tmp / 100, abs(tmp % 100));
	HAL_UART_Transmit(&huart2, _text, _len, 1000);
	tmp = (int32_t) (b * 100.0);
	_len = sprintf((char*) _text, "#B=%ld.%02d\n", tmp / 100, abs(tmp % 100));
	HAL_UART_Transmit(&huart2, _text, _len, 1000);
	tmp = (int32_t) (r2 * 100.0);
	_len = sprintf((char*) _text, "#R2=%ld.%02d\n", tmp / 100, abs(tmp % 100));
	HAL_UART_Transmit(&huart2, _text, _len, 1000);
	tmp = (int32_t) (avg * 100.0);
	_len = sprintf((char*) _text, "#AVG=%ld.%02d\n", tmp / 100,
			abs(tmp % 100));
	HAL_UART_Transmit(&huart2, _text, _len, 1000);
	tmp = (int32_t) (sd * 100.0);
	_len = sprintf((char*) _text, "#SD=%ld.%02d\n", tmp / 100, abs(tmp % 100));
	HAL_UART_Transmit(&huart2, _text, _len, 1000);

	_len = sprintf((char*) _text, "# \tPa/min\tkPa\n");
	HAL_UART_Transmit(&huart2, _text, _len, 1000);
	for (uint8_t i = 0; i < CYCLE_N; i++) {
		/*_len = sprintf((char*)_text, "%d\t%5ld\t%3d\t%6d\n",
		 i+1,
		 test_log.measurements[i].rough_start,
		 test_log.measurements[i].fine_stop/10,
		 test_log.measurements[i].duration);*/
		_len = sprintf((char*) _text, "%d\t%4ld.%02d\t%2ld.%02d\n", i + 1,
				((int32_t) (ys[i] * 100)) / 100,
				abs((int32_t) (ys[i] * 100)) % 100,
				((int32_t) (xs[i] * 100)) / 100,
				abs((int32_t) (xs[i] * 100)) % 100);
		HAL_UART_Transmit(&huart2, _text, _len, 1000);
	}
	HAL_GPIO_WritePin(GPIO_RS485_OE_GPIO_Port, GPIO_RS485_OE_Pin,
			GPIO_PIN_RESET);

	TestStats_t s = { .a = a, .b = b, .r2 = r2, .sd = sd, .avg = avg };
	return simple_analysis(&s);
}

TestResult_t simple_analysis(TestStats_t *stats) {
	// avg between -50 and 50 -- OK
	if (abs(stats->avg) < 50.0 && stats->sd < 30.0) {
		//job_result = jrGood;
		return trGood;
	}

	// compulsive change based on raw observation
	if (stats->avg > -75.) {
		//job_result = jrGood;
		return trGood;
	}

	// large
	if (stats->sd > 200.0) {
		//job_result = jrError;
		return trNotReady;
	}

	// constant leak -- sd means no doubt
	if (stats->avg < -100.0 && stats->sd < 150.0) {
		//job_result = jrBad;
		return trBad;
	}

	if (stats->b < -5 && stats->r2 > 0.7) {
		//job_result = jrBad;
		return trBad;
	}

	// if we got here -- something is obviously wrong!
	//job_result = jrError;
	return trNotReady;

}

void set_backlight(BacklightColor_t color) {
	color = ~color;
	HAL_GPIO_WritePin(GPIO_Backlight_R_GPIO_Port, GPIO_Backlight_R_Pin,
			color & 0x01);
	HAL_GPIO_WritePin(GPIO_Backlight_G_GPIO_Port, GPIO_Backlight_G_Pin,
			color & 0x02);
	HAL_GPIO_WritePin(GPIO_Backlight_B_GPIO_Port, GPIO_Backlight_B_Pin,
			color & 0x04);
}


uint32_t read_rtc(RTC_HandleTypeDef *hrtc)
{
  uint16_t high1 = 0U, high2 = 0U, low = 0U;
  uint32_t timecounter = 0U;

  high1 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);
  low   = READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT);
  high2 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);

  if (high1 != high2)
  {
    /* In this case the counter roll over during reading of CNTL and CNTH registers,
       read again CNTL register then return the counter value */
    timecounter = (((uint32_t) high2 << 16U) | READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT));
  }
  else
  {
    /* No counter roll over during reading of CNTL and CNTH registers, counter
       value is equal to first value of CNTL and CNTH */
    timecounter = (((uint32_t) high1 << 16U) | low);
  }

  return timecounter;
}

HAL_StatusTypeDef write_rtc(RTC_HandleTypeDef *hrtc, uint32_t TimeCounter)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Set Initialization mode */
  if (rtc_enterinit(hrtc) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  else
  {
    /* Set RTC COUNTER MSB word */
    WRITE_REG(hrtc->Instance->CNTH, (TimeCounter >> 16U));
    /* Set RTC COUNTER LSB word */
    WRITE_REG(hrtc->Instance->CNTL, (TimeCounter & RTC_CNTL_RTC_CNT));

    /* Wait for synchro */
    if (rtc_exitinit(hrtc) != HAL_OK)
    {
      status = HAL_ERROR;
    }
  }

  return status;
}

HAL_StatusTypeDef rtc_enterinit(RTC_HandleTypeDef *hrtc)
{
  uint32_t tickstart = 0U;

  tickstart = HAL_GetTick();
  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while ((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if ((HAL_GetTick() - tickstart) >  RTC_TIMEOUT_VALUE)
    {
      return HAL_TIMEOUT;
    }
  }

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);


  return HAL_OK;
}

HAL_StatusTypeDef rtc_exitinit(RTC_HandleTypeDef *hrtc)
{
  uint32_t tickstart = 0U;

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  tickstart = HAL_GetTick();
  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while ((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if ((HAL_GetTick() - tickstart) >  RTC_TIMEOUT_VALUE)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}


///////////////////
// ERROR         //
///////////////////

void stErrorRefreshDisplay(void) {
	uint8_t _text[20] = { 0 };
	// unused__ int32_t _pressure;

	clearDisplay();
	dbuf_Fill(&display_buffer, 0);

	sprintf((char*) _text, MSG_ERROR_SERVICE);
	dbuf_PutString(&display_buffer, 0, 0, _text, &mono5x8, 1);

	sprintf((char*) _text, MSG_ERROR_CODE);
	dbuf_PutString(&display_buffer, 0, 18, _text, &mono5x8, 1);

	sprintf((char*) _text, " %04X", last_error_code);
	dbuf_PutString(&display_buffer, 0, 27, _text, &mono5x8, 1);
	st7565_WriteBuffer(display_buffer.buffer);
}

void stErrorHandleHardware(void) {
	if (curState->init == siInit) {
		curState->init = siNone;

		timer = HAL_GetTick();
		__GREEN_LED_ON();
		__RED_LED_ON();
	}
	if (HAL_GetTick() - timer > 500) {
		timer += 500;
		__RED_LED_TOG();
		__GREEN_LED_TOG();
	}
}


///////////////////
// SET TIME      //
///////////////////
uint8_t settime_position;
struct tm  datetime_tmp;
void stSetTimeRefreshDisplay(void){
	uint8_t _text[20] = { 0 };

	clearDisplay();

	sprintf((char*) _text, MSG_ERROR_SERVICE);
	dbuf_PutString(&display_buffer, 0, 0, (uint8_t*)MSG_DATE_TIME, &sansserif_11pt, 1);

	dbuf_PutString(&display_buffer, 0, 18, (uint8_t*)"  /  /    ", &mono5x8, 1);
	dbuf_PutString(&display_buffer, 0, 27, (uint8_t*)"  :  ", &mono5x8, 1);

	switch(settime_position){
	case 0: // day
		dbuf_DrawFillRect(&display_buffer, 0, 17, 12, 9, 1);
		break;
	case 1: //month
		dbuf_DrawFillRect(&display_buffer, 17, 17, 13, 9, 1);
		break;
	case 2: //year
		dbuf_DrawFillRect(&display_buffer, 35, 17, 25, 9, 1);
		break;
	case 3: //hour
		dbuf_DrawFillRect(&display_buffer, 0, 26, 12, 9, 1);
		break;
	case 4: //minute
		dbuf_DrawFillRect(&display_buffer, 17, 26, 12, 9, 1);
		break;
	default:
		break;
	}

	sprintf((char*) _text, "%2d", datetime_tmp.tm_mday);
	dbuf_PutString(&display_buffer, 0, 18, _text, &mono5x8, settime_position == 0 ? 0 : 1);

	sprintf((char*) _text, "%02d", datetime_tmp.tm_mon+1);
	dbuf_PutString(&display_buffer, 18, 18, _text, &mono5x8, settime_position == 1 ? 0 : 1);

	sprintf((char*) _text, "%04d", datetime_tmp.tm_year+1900);
	dbuf_PutString(&display_buffer, 36, 18, _text, &mono5x8, settime_position == 2 ? 0 : 1);

	sprintf((char*) _text, "%2d", datetime_tmp.tm_hour);
	dbuf_PutString(&display_buffer, 0, 27, _text, &mono5x8, settime_position == 3 ? 0 : 1);

	sprintf((char*) _text, "%02d", datetime_tmp.tm_min);
	dbuf_PutString(&display_buffer, 18, 27, _text, &mono5x8, settime_position == 4 ? 0 : 1);

	st7565_WriteBuffer(display_buffer.buffer);
}

void stSetTimeUp(Buttons_EventTypeDef _event){
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		switch(settime_position){
		case 0: // day
			datetime_tmp.tm_mday++;
			if (datetime_tmp.tm_mday > 31) datetime_tmp.tm_mday = 1;
			break;
		case 1: //month
			datetime_tmp.tm_mon++;
			if (datetime_tmp.tm_mon > 11) datetime_tmp.tm_mon = 0;
			break;
		case 2: //year
			datetime_tmp.tm_year++;
			if (datetime_tmp.tm_year > 2037-1900) datetime_tmp.tm_year = 2020-1900;
			break;
		case 3: //hour
			datetime_tmp.tm_hour++;
			if (datetime_tmp.tm_hour > 23) datetime_tmp.tm_hour = 0;
			break;
		case 4: //minute
			datetime_tmp.tm_min++;
			if (datetime_tmp.tm_min > 59) datetime_tmp.tm_min = 0;
			break;
		default:
			break;
		}
		break;
	case BUTTON_RELEASED:
		break;
	}
}

void stSetTimeDown(Buttons_EventTypeDef _event){
	switch (_event) {
	case BUTTON_PRESSED:
	case BUTTON_REPEATED:
		switch(settime_position){
		case 0: // day
			datetime_tmp.tm_mday--;
			if (datetime_tmp.tm_mday < 1) datetime_tmp.tm_mday = 31;
			break;
		case 1: //month
			datetime_tmp.tm_mon--;
			if (datetime_tmp.tm_mon < 0) datetime_tmp.tm_mon = 11;
			break;
		case 2: //year
			datetime_tmp.tm_year--;
			if (datetime_tmp.tm_year < 2010-1900) datetime_tmp.tm_year = 2037;
			break;
		case 3: //hour
			datetime_tmp.tm_hour--;
			if (datetime_tmp.tm_hour < 0) datetime_tmp.tm_hour = 23;
			break;
		case 4: //minute
			datetime_tmp.tm_min--;
			if (datetime_tmp.tm_min < 0) datetime_tmp.tm_min = 59;
			break;
		default:
			break;
		}
		break;
	case BUTTON_RELEASED:
		break;
	}
}

void stSetTimeOK(Buttons_EventTypeDef _event){
	uint32_t epoch;

	switch (_event) {
	case BUTTON_PRESSED:
		settime_position++;
		if(settime_position > 4){
			epoch = mktime(&datetime_tmp);
			write_rtc(&hrtc, epoch);
			changeState(&stReady);
		}
		break;
	case BUTTON_REPEATED:
		break;
	case BUTTON_RELEASED:
		break;
	}
}

void stSetTimeHandleHardware(void){
	time_t now;

	if (curState->init == siInit) {
		curState->init = siNone;

		now = read_rtc(&hrtc);
		datetime_tmp = *(gmtime(&now));
		settime_position = 0;
	}
}


uint16_t log2sd(char *text){
	time_t epoch;
	struct tm datetime;
	FATFS sdFatFs; /* File system object for User logical drive */
	FIL logFile;   /* File object */
	uint32_t wbytes; /* File write counts */
	char filename[20];
	char logdatetime[40];

	epoch = read_rtc(&hrtc);
	datetime = *(gmtime(&epoch));

	if(f_mount(&sdFatFs, (TCHAR const*)SDPath, 0) == FR_OK)
	{
		sprintf(filename, "%04d%02d%02d.txt", datetime.tm_year+1900, datetime.tm_mon+1, datetime.tm_mday);
		if(f_open(&logFile, filename, FA_OPEN_ALWAYS | FA_WRITE) == FR_OK)
		{
			f_lseek(&logFile, logFile.fsize);
			sprintf(logdatetime, "\r\n%04d/%02d/%02d %2d:%02d:%02d|",
					datetime.tm_year+1900, datetime.tm_mon+1, datetime.tm_mday,
					datetime.tm_hour, datetime.tm_min, datetime.tm_sec);
			if(f_write(&logFile, logdatetime, strlen(logdatetime), (void *)&wbytes) == FR_OK)
			{

			}else{
				f_close(&logFile);
				return 1;
			}
			if(f_write(&logFile, text, strlen(text), (void *)&wbytes) == FR_OK)
			{
				f_close(&logFile);
				return 0;
			}
		}
	}
	return 1;
}
