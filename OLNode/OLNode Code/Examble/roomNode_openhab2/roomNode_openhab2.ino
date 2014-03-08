// New version of the Room Node, derived from rooms.pde
// 2010-10-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// see http://jeelabs.org/2010/10/20/new-roomnode-code/
// and http://jeelabs.org/2010/10/21/reporting-motion/

// The complexity in the code below comes from the fact that newly detected PIR
// motion needs to be reported as soon as possible, but only once, while all the
// other sensor values are being collected and averaged in a more regular cycle.

#include <JeeLib.h>
#include <PortsSHT11.h>
#include <avr/sleep.h>
#include <util/atomic.h>

#define SERIAL  1   // set to 1 to also report readings on the serial port
#define DEBUG   0   // set to 1 to display each loop() run and PIR trigger

#define SHT11_PORT  4   // defined if SHT11 is connected to a port
#define LDR_PORT    1   // defined if LDR is connected to a port's AIO pin
#define PIR_PORT    1   // defined if PIR is connected to a port's DIO pin

#define MEASURE_PERIOD  100 // how often to measure, in tenths of seconds
#define RETRY_PERIOD    10  // how soon to retry if ACK didn't come in
#define RETRY_LIMIT     5   // maximum number of times to retry
#define ACK_TIME        10  // number of milliseconds to wait for an ack
#define REPORT_EVERY    2   // report every N measurement cycles
#define SMOOTH          3   // smoothing factor used for running averages

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 2

// The scheduler makes it easy to perform various tasks at various times:

enum { MEASURE, REPORT, TASK_END };

static word schedbuf[TASK_END];
Scheduler scheduler (schedbuf, TASK_END);

// Other variables used in various places in the code:

static byte reportCount;    // count up until next report, i.e. packet send
static byte myNodeID;       // node ID used for this unit

// This defines the structure of the packets which get sent out by wireless:

struct {
    byte light_type;        //light type L = 30  
    byte light_data;        // light sensor: 0..255
    byte moved_type;        //motion type B = 40 
    byte moved_data;    // motion detector: 0..1 :1
    byte humi_type;        //humi type F = 50 
    byte humi_data;    // humidity: 0..100   :7
    byte temp_type;        //temp type T = 20 
    int temp_data;   // temperature: -500..+500 (tenths)   :10
    byte lobat_type;        //low Batterie type V = 10 
    byte lobat_data;    // supply voltage dropped under 3.1V: 0..1 :1
    
} payload;

// Conditional code, depending on which sensors are connected and how:

#if SHT11_PORT
    SHT11 sht11 (SHT11_PORT);
#endif

#if LDR_PORT
    Port ldr (LDR_PORT);
#endif

#if PIR_PORT
    #define PIR_HOLD_TIME   30  // hold PIR value this many seconds after change
    #define PIR_PULLUP      1   // set to one to pull-up the PIR input pin
    #define PIR_FLIP        0   // 0 or 1, to match PIR reporting high or low
    
    class PIR : public Port {
        volatile byte value, changed;
        volatile uint32_t lastOn;
    public:
        PIR (byte portnum)
            : Port (portnum), value (0), changed (0), lastOn (0) {}

        // this code is called from the pin-change interrupt handler
        void poll() {
            // see http://talk.jeelabs.net/topic/811#post-4734 for PIR_FLIP
            byte pin = digiRead() ^ PIR_FLIP;
            // if the pin just went on, then set the changed flag to report it
            if (pin) {
                if (!state())
                    changed = 1;
                lastOn = millis();
            }
            value = pin;
        }

        // state is true if curr value is still on or if it was on recently
        byte state() const {
            byte f = value;
            if (lastOn > 0)
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                    if (millis() - lastOn < 1000 * PIR_HOLD_TIME)
                        f = 1 ^ PIR_FLIP;
                }
            return f;
        }

        // return true if there is new motion to report
        byte triggered() {
            byte f = changed;
            changed = 0;
            return f;
        }
    };

    PIR pir (PIR_PORT);

    // the PIR signal comes in via a pin-change interrupt
    ISR(PCINT2_vect) { pir.poll(); }
#endif

// has to be defined because we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

// utility code to perform simple smoothing as a running average
static int smoothedAverage(int prev, int next, byte firstTime =0) {
    if (firstTime)
        return next;
    return ((SMOOTH - 1) * prev + next + SMOOTH / 2) / SMOOTH;
}

// spend a little time in power down mode while the SHT11 does a measurement
static void shtDelay () {
    Sleepy::loseSomeTime(32); // must wait at least 20 ms
}

// wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                // see http://talk.jeelabs.net/topic/811#post-4712
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
            return 1;
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }
    return 0;
}

// readout all the sensors and other values
static void doMeasure() {
    byte firstTime = payload.humi_data == 0; // special case to init running avg
    
    payload.lobat_type = 10;
    payload.lobat_data = rf12_lowbat();

    #if SHT11_PORT
#ifndef __AVR_ATtiny84__
        sht11.measure(SHT11::HUMI, shtDelay);        
        sht11.measure(SHT11::TEMP, shtDelay);
        float h, t;
        sht11.calculate(h, t);
        int humi = h + 0.5, temp = 10 * t + 0.5;
#else
        //XXX TINY!
        int humi = 50, temp = 25;
#endif
        payload.humi_type = 50;
        payload.humi_data = smoothedAverage(payload.humi_data, humi, firstTime);
        payload.temp_type = 20;
        payload.temp_data = smoothedAverage(payload.temp_data, temp, firstTime);
    #endif
    #if LDR_PORT
        ldr.digiWrite2(1);  // enable AIO pull-up
        byte light = ~ ldr.anaRead() >> 2;
        ldr.digiWrite2(0);  // disable pull-up to reduce current draw
        payload.light_type = 30;
        payload.light_data = smoothedAverage(payload.light_data, light, firstTime);
    #endif
    #if PIR_PORT
        payload.moved_type = 40;
        payload.moved_data = pir.state();
    #endif
}

static void serialFlush () {
    #if ARDUINO >= 100
        Serial.flush();
    #endif  
    delay(2); // make sure tx buf is empty before going back to sleep
}

// periodic report, i.e. send out a packet and optionally report on serial port
static void doReport() {
    rf12_sleep(RF12_WAKEUP);
    while (!rf12_canSend())
        rf12_recvDone();
    rf12_sendStart(0, &payload, sizeof payload, RADIO_SYNC_MODE);
    rf12_sleep(RF12_SLEEP);

    #if SERIAL
        Serial.print("ROOM ");
        Serial.print((int) payload.light_type);
        Serial.print((int) payload.light_data);
        Serial.print(' ');
        Serial.print((int) payload.moved_type);
        Serial.print((int) payload.moved_data);
        Serial.print(' ');
        Serial.print((int) payload.humi_type);
        Serial.print((int) payload.humi_data);
        Serial.print(' ');
        Serial.print((int) payload.temp_type);
        Serial.print((int) payload.temp_data);
        Serial.print(' ');
        Serial.print((int) payload.lobat_type);
        Serial.print((int) payload.lobat_data);
        Serial.println();
        Serial.print(sizeof payload);
        //Serial.print(&payload);
        //for (byte i = 0; i < n; ++i) {   //Test Darstellung von Data
        //    Serial.print((int) rf12_data[i]);
        //    Serial.print(",");
        //    }
        serialFlush();
    #endif
}

// send packet and wait for ack when there is a motion trigger
static void doTrigger() {
    #if DEBUG
        Serial.print("PIR ");
        Serial.print((int) payload.moved_data);
        serialFlush();
    #endif

    for (byte i = 0; i < RETRY_LIMIT; ++i) {
        rf12_sleep(RF12_WAKEUP);
        while (!rf12_canSend())
            rf12_recvDone();
        rf12_sendStart(RF12_HDR_ACK, &payload, sizeof payload, RADIO_SYNC_MODE);
        byte acked = waitForAck();
        rf12_sleep(RF12_SLEEP);

        if (acked) {
            #if DEBUG
                Serial.print(" ack ");
                Serial.println((int) i);
                serialFlush();
            #endif
            // reset scheduling to start a fresh measurement cycle
            scheduler.timer(MEASURE, MEASURE_PERIOD);
            return;
        }
        
        delay(RETRY_PERIOD * 100);
    }
    scheduler.timer(MEASURE, MEASURE_PERIOD);
    #if DEBUG
        Serial.println(" no ack!");
        serialFlush();
    #endif
}

void blink (byte pin) {
    for (byte i = 0; i < 6; ++i) {
        delay(100);
        digitalWrite(pin, !digitalRead(pin));
    }
}

void setup () {
    #if SERIAL || DEBUG
        Serial.begin(57600);
        Serial.print("\n[roomNode.3]");
        myNodeID = rf12_config();
        serialFlush();
    #else
        myNodeID = rf12_config(0); // don't report info on the serial port
    #endif
    
    rf12_sleep(RF12_SLEEP); // power down
    
    #if PIR_PORT
        pir.digiWrite(PIR_PULLUP);
#ifdef PCMSK2
        bitSet(PCMSK2, PIR_PORT + 3);
        bitSet(PCICR, PCIE2);
#else
        //XXX TINY!
#endif
    #endif

    reportCount = REPORT_EVERY;     // report right away for easy debugging
    scheduler.timer(MEASURE, 0);    // start the measurement loop going
    
    rf12_encrypt(RF12_EEPROM_EKEY);
}

void loop () {
    #if DEBUG
        Serial.print('.');
        serialFlush();
    #endif

    #if PIR_PORT
        if (pir.triggered()) {
            payload.moved_data = pir.state();
            doTrigger();
        }
    #endif
    
    switch (scheduler.pollWaiting()) {

        case MEASURE:
            // reschedule these measurements periodically
            scheduler.timer(MEASURE, MEASURE_PERIOD);
    
            doMeasure();

            // every so often, a report needs to be sent out
            if (++reportCount >= REPORT_EVERY) {
                reportCount = 0;
                scheduler.timer(REPORT, 0);
            }
            break;
            
        case REPORT:
            doReport();
            break;
    }
}
