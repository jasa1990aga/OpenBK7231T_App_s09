#include "../obk_config.h"

#if ENABLE_DRIVER_IRREMOTEESP
// drv_ir_new.cpp - IRremoteESP8266

extern "C" {
	// these cause error: conflicting declaration of 'int bk_wlan_mcu_suppress_and_sleep(unsigned int)' with 'C' linkage
#include "../new_common.h"

#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
#include "../obk_config.h"
#include "../cmnds/cmd_public.h"
#include "../hal/hal_pins.h"
#include "../hal/hal_generic.h"
#include "../hal/hal_hwtimer.h"

#if PLATFORM_BEKEN
#include "include.h"
#include "arm_arch.h"
#elif PLATFORM_LN882H || PLATFORM_LN8825
#define delay_ms OS_MsDelay
#elif PLATFORM_RTL8710B
	int __wrap_atoi(const char* str);
	char* _strncpy(char* dest, const char* src, size_t count);
	int _sscanf_patch(const char* buf, const char* fmt, ...);
//#undef sscanf
#endif

// why can;t I call this?
#include "../mqtt/new_mqtt.h"

	unsigned long ir_counter = 0;
	uint8_t gEnableIRSendWhilstReceive = 0;
	uint32_t gIRProtocolEnable = 0xFFFFFFFF;
	// 0 == active low.  1 = active hi
	uint8_t gIRPinPolarity = 0;

	extern int my_strnicmp(const char* a, const char* b, int len);
	extern unsigned int g_timeMs;
}


#include "drv_ir.h"

//#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE 1
#undef read
#undef write
#undef send
//#define PROGMEM


//#define NO_LED_FEEDBACK_CODE 1

//typedef unsigned char uint_fast8_t;
typedef unsigned short uint16_t;

#define __FlashStringHelper char

// dummy functions
#if PLATFORM_BEKEN
void noInterrupts() { }
void interrupts() { }
void delay(int n) { }
void delayMicroseconds(int n) { }
unsigned long millis()
{
	return 0;
}
unsigned long micros()
{
	return 0;
}
#else
void noInterrupts() { taskENTER_CRITICAL(); }
void interrupts() { taskEXIT_CRITICAL(); }
void delay(int n) { delay_ms(n); }
void delayMicroseconds(int n) { HAL_Delay_us(n); }
unsigned long millis()
{
	return g_timeMs;
}
unsigned long micros()
{
	return g_timeMs * 1000;
}
#endif


class Print {
public:
	void println(const char *p) {
		return;
	}
	void print(...) {
		return;
	}
};

Print Serial;




#define EXTERNAL_IR_TIMER_ISR

//////////////////////////////////////////
// our external timer interrupt stuff
// this will have already been done
#define TIMER_RESET_INTR_PENDING


// #  if defined(ISR)
// #undef ISR
// #  endif
// #define ISR void IR_ISR

// THIS function is defined in src/libraries/IRremoteESP8266/src/IRrecv.cpp
extern "C" void DRV_IR_ISR(void* arg);
extern void IR_ISR(float period_us);

static int8_t ir_chan = -1;
static float ir_periodus = 50;

void timerConfigForReceive() {
	// nothing here`
}

void _timerConfigForReceive() {
	ir_counter = 0;

	ir_chan = HAL_RequestHWTimer(ir_periodus, &ir_periodus, DRV_IR_ISR, NULL);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer %u, %.2f us period", ir_chan, ir_periodus);
}

static void timer_enable() {
}
static void timer_disable() {
}
static void _timer_enable() {
	HAL_HWTimerStart(ir_chan);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer enabled %u", ir_chan);
}
static void _timer_disable() {
	HAL_HWTimerStop(ir_chan);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer disabled %u", ir_chan);
}

#define TIMER_ENABLE_RECEIVE_INTR timer_enable();
#define TIMER_DISABLE_RECEIVE_INTR timer_disable();

//////////////////////////////////////////

class SpoofIrReceiver {
public:
	static void restartAfterSend() {

	}
};

SpoofIrReceiver IrReceiver;

#include "../libraries/IRremoteESP8266/src/IRremoteESP8266.h"
#include "../libraries/IRremoteESP8266/src/IRsend.h"
#include "../libraries/IRremoteESP8266/src/IRrecv.h"
#include "../libraries/IRremoteESP8266/src/IRutils.h"
#ifdef ENABLE_IRAC
#include "../libraries/IRremoteESP8266/src/IRac.h"
#endif
#include "../libraries/IRremoteESP8266/src/IRproto.h"
#include "../libraries/IRremoteESP8266/src/digitalWriteFast.h"

// override aspects of sending for our own interrupt driven sends
// basically, IRsend calls mark(us) and space(us) to send.
// we simply note the numbers into a rolling buffer, assume the first is a mark()
// and then every 50us service the rolling buffer, changing the PWM from 0 duty to 50% duty
// appropriately.
#define SEND_MAXBITS 128

class myIRsend : public IRsend {
public:
	myIRsend(uint_fast8_t aSendPin) :IRsend(aSendPin) {
		sendPin = aSendPin;
		our_us = 0;
		our_ms = 0;
		currentFreq = 0;
		resetsendqueue();
	}
	~myIRsend() { }


	uint32_t millis() {
		return our_ms;
	}

	void delay(long int ms) {
		// add a pure delay to our queue
		space(ms * 1000);
	}

	uint16_t mark(uint16_t aMarkMicros) {
		// sends a high for aMarkMicros
		uint32_t newtimein = (timein + 1) % (SEND_MAXBITS * 2);
		if (newtimein != timeout) {
			// store mark bits in highest +ve bit of count
			times[timein] = aMarkMicros | 0x10000000;
			timein = newtimein;
			timecount++;
			timecounttotal++;
		}
		else {
			overflows++;
		}
		return 1;
	}

	void space(uint32_t aMarkMicros) {
		// sends a low for aMarkMicros
		uint32_t newtimein = (timein + 1) % (SEND_MAXBITS * 2);
		if (newtimein != timeout) {
			times[timein] = aMarkMicros;
			timein = newtimein;
			timecount++;
			timecounttotal++;
		}
		else {
			overflows++;
		}
	}

	void enableIROut(uint32_t freq, uint8_t duty=50) {
		//uint_fast8_t aFrequencyKHz
		if (freq < 1000)  // Were we given kHz? Supports the old call usage.
			freq *= 1000;
		if(duty<1)
			duty=1;
		pwmduty = duty;

		if (currentFreq != freq) {
			ADDLOG_INFO(LOG_FEATURE_IR, (char *)"enableIROut %d freq %d duty",(int)freq, (int)duty);
			currentFreq = freq;
			HAL_PIN_PWM_Start(sendPin, freq);
		}
	}

	void resetsendqueue() {
		// sends a low for aMarkMicros
		timein = timeout = 0;
		timecount = 0;
		overflows = 0;
		currentsendtime = 0;
		currentbitval = 0;
		timecounttotal = 0;
	}
	int32_t times[SEND_MAXBITS * 2]; // enough for 128 bits
	volatile unsigned short timein;
	volatile unsigned short timeout;
	volatile unsigned short timecount;
	volatile unsigned short overflows;
	uint32_t timecounttotal;

	int32_t getsendqueue() {
		int32_t val = 0;
		if (timein != timeout) {
			val = times[timeout];
			timeout = (timeout + 1) % (SEND_MAXBITS * 2);
			timecount--;
		}
		return val;
	}
	int currentsendtime;
	int currentbitval;

	uint8_t sendPin;
	uint32_t pwmduty;
	uint32_t currentFreq;

	uint32_t our_ms;
	float our_us;
};


// our send/receive instances
myIRsend *pIRsend = NULL;
IRrecv *ourReceiver = NULL;

// this is our ISR.
// it is called every 50us, so we need to work on making it as efficient as possible.
extern "C" void DRV_IR_ISR(void* arg)
{
	int sending = 0;
	if (pIRsend) {
		pIRsend->our_us += ir_periodus;
		if (pIRsend->our_us > 1000) {
			pIRsend->our_ms++;
			pIRsend->our_us -= 1000;
		}

		int pinval = 0;
		if (pIRsend->currentsendtime) {
			sending = 1;
			pIRsend->currentsendtime -= ir_periodus;
			if (pIRsend->currentsendtime <= 0) {
				int32_t remains = pIRsend->currentsendtime;
				int32_t newtime = pIRsend->getsendqueue();
				if (0 == newtime) {
					// if it was the last one
					pIRsend->currentsendtime = 0;
					pIRsend->currentbitval = 0;
				}
				else {
					// we got a new time
					// store mark bits in highest +ve bit of count
					pIRsend->currentbitval = (newtime & 0x10000000) ? 1 : 0;
					pIRsend->currentsendtime = (newtime & 0xfffffff);
					// adjust the us value to keep the running accuracy
					// and avoid a running error?
					// note remains is -ve
					pIRsend->currentsendtime += remains;
				}
			}
		}
		else {
			int32_t newtime = pIRsend->getsendqueue();
			if (!newtime) {
				pIRsend->currentsendtime = 0;
				pIRsend->currentbitval = 0;
			}
			else {
				sending = 1;
				pIRsend->currentsendtime = (newtime & 0xfffffff);
				pIRsend->currentbitval = (newtime & 0x10000000) ? 1 : 0;
			}
		}
		pinval = pIRsend->currentbitval;

		uint32_t duty = pIRsend->pwmduty;
		if (!pinval) {
			if (gIRPinPolarity) {
				duty = 50;
			}
			else {
				duty = 0;
			}
		}
		HAL_PIN_PWM_Update(pIRsend->sendPin, duty);
	}

	// is someone really wants rx and TX at the same time, then allow it.
	if (gEnableIRSendWhilstReceive) {
		sending = 0;
	}

	// don't receive if we are currently sending
	if (ourReceiver && !sending){
		IR_ISR(ir_periodus);
	}
	ir_counter++;
}


extern "C" commandResult_t IR_Send_Cmd(const void *context, const char *cmd, const char *args_in, int cmdFlags) {
	if (!args_in) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	char args[128];
	strncpy(args, args_in, sizeof(args) - 1);
	args[sizeof(args) - 1] = 0;

	// split arg at hyphen;
	char *p = args;
	while (*p && (*p != '-') && (*p != ' ')) {
		p++;
	}

	if ((*p != '-') && (*p != ' ')) {
		// try to decode "new" format, separated by comma
		// the format is bits,0xDATA[,repeat]
		char *p = args;
		while (*p && (*p != ',')) {
			p++;
		}
		if(*p==',')
		{
			*p='
