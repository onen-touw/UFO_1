#include <Arduino.h>
#include "UFO_HX711.h"

// TEENSYDUINO has a port of Dean Camera's ATOMIC_BLOCK macros for AVR to ARM Cortex M3.
#define HAS_ATOMIC_BLOCK (defined(ARDUINO_ARCH_AVR) || defined(TEENSYDUINO))

// Whether we are running on either the ESP8266 or the ESP32.
#define ARCH_ESPRESSIF (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))

// Whether we are actually running on FreeRTOS.
#define IS_FREE_RTOS defined(ARDUINO_ARCH_ESP32)

// Define macro designating whether we're running on a reasonable
// fast CPU and so should slow down sampling from GPIO.
#define FAST_CPU \
    ( \
    ARCH_ESPRESSIF || \
    defined(ARDUINO_ARCH_SAM)     || defined(ARDUINO_ARCH_SAMD) || \
    defined(ARDUINO_ARCH_STM32)   || defined(TEENSYDUINO) \
    )


#if FAST_CPU
// Make shiftIn() be aware of clockspeed for
// faster CPUs like ESP32, Teensy 3.x and friends.
// See also:
// - https://github.com/bogde/HX711/issues/75
// - https://github.com/arduino/Arduino/issues/6561
// - https://community.hiveeyes.org/t/using-bogdans-canonical-hx711-library-on-the-esp32/539
uint8_t shiftInSlow(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
    uint8_t value = 0;
    uint8_t i;

    for(i = 0; i < 8; ++i) {
        digitalWrite(clockPin, HIGH);
        delayMicroseconds(1);
        if(bitOrder == LSBFIRST)
            value |= digitalRead(dataPin) << i;
        else
            value |= digitalRead(dataPin) << (7 - i);
        digitalWrite(clockPin, LOW);
        delayMicroseconds(1);
    }
    return value;
}
#endif

#if ARCH_ESPRESSIF
// ESP8266 doesn't read values between 0x20000 and 0x30000 when DOUT is pulled up.
#define DOUT_MODE INPUT
#endif


UFO_HX711::UFO_HX711() {
}

UFO_HX711::~UFO_HX711() {
}

void UFO_HX711::begin(byte dout, byte pd_sck, byte gain) {
	_sckPin = pd_sck;
	_doutPin = dout;

	pinMode(_sckPin, OUTPUT);
	pinMode(_doutPin, INPUT|OUTPUT);

	set_gain(gain);
}

bool UFO_HX711::is_ready() {
	return digitalRead(_doutPin) == LOW; 
}

void UFO_HX711::set_gain(byte gain) {
	switch (gain) {
		case 128:		// channel A, gain factor 128
			GAIN = 1;
			break;
		case 64:		// channel A, gain factor 64
			GAIN = 3;
			break;
		case 32:		// channel B, gain factor 32
			GAIN = 2;
			break;
	}

}

long UFO_HX711::read() {

	// Wait for the chip to become ready.
	wait_ready(10);

	// Define structures for reading data into.
	unsigned long value = 0;
	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;

	
	// Begin of critical section.
	// Critical sections are used as a valid protection method
	// against simultaneous access in vanilla FreeRTOS.
	// Disable the scheduler and call portDISABLE_INTERRUPTS. This prevents
	// context switches and servicing of ISRs during a critical section.
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL(&mux);


	// Pulse the clock pin 24 times to read the data.
	data[2] = shiftInSlow(_doutPin, _sckPin, MSBFIRST);
	data[1] = shiftInSlow(_doutPin, _sckPin, MSBFIRST);
	data[0] = shiftInSlow(_doutPin, _sckPin, MSBFIRST);

	// Set the channel and the gain factor for the next reading using the clock pin.
	for (unsigned int i = 0; i < GAIN; i++) {
		digitalWrite(_sckPin, HIGH);
		delayMicroseconds(1);
		digitalWrite(_doutPin, LOW);
		delayMicroseconds(1);
	}

	
	// End of critical section.
	portEXIT_CRITICAL(&mux);
	

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if (data[2] & 0x80) {
		filler = 0xFF;
	} else {
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ( static_cast<unsigned long>(filler) << 24
			| static_cast<unsigned long>(data[2]) << 16
			| static_cast<unsigned long>(data[1]) << 8
			| static_cast<unsigned long>(data[0]) );

	return static_cast<long>(value);
}

void UFO_HX711::wait_ready(unsigned long delay_ms) {
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the sketch until a load cell is connected.
	while (!is_ready()) {
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		delay(delay_ms);
	}
}

bool UFO_HX711::wait_ready_retry(int retries, unsigned long delay_ms) {
	// Wait for the chip to become ready by
	// retrying for a specified amount of attempts.
	// https://github.com/bogde/HX711/issues/76
	int count = 0;
	while (count < retries) {
		if (is_ready()) {
			return true;
		}
		delay(delay_ms);
		count++;
	}
	return false;
}

bool UFO_HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
	// Wait for the chip to become ready until timeout.
	// https://github.com/bogde/HX711/pull/96
	unsigned long millisStarted = millis();
	while (millis() - millisStarted < timeout) {
		if (is_ready()) {
			return true;
		}
		delay(delay_ms);
	}
	return false;
}

long UFO_HX711::read_average(byte times) {
	long sum = 0;
	for (byte i = 0; i < times; i++) {
		sum += read();
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		delay(0);
	}
	return sum / times;
}

double UFO_HX711::get_value(byte times) {
	return read_average(times) - OFFSET;
}

float UFO_HX711::get_units(byte times) {
	return get_value(times) / SCALE;
}

void UFO_HX711::tare(byte times) {
	double sum = read_average(times);
	set_offset(sum);
}

void UFO_HX711::set_scale(float scale) {
	SCALE = scale;
}

float UFO_HX711::get_scale() {
	return SCALE;
}

void UFO_HX711::set_offset(long offset) {
	OFFSET = offset;
}

long UFO_HX711::get_offset() {
	return OFFSET;
}

void UFO_HX711::power_down() {
	digitalWrite(_sckPin, LOW);
	digitalWrite(_sckPin, HIGH);
}

void UFO_HX711::power_up() {
	digitalWrite(_sckPin, LOW);
}