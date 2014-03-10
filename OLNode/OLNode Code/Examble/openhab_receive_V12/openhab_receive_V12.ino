// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// this version adds flash memory support, 2009-11-19

#include <JeeLib.h>
#include <util/crc16.h>
//#include <util/parity.h>
//#include <avr/eeprom.h>
//#include <avr/pgmspace.h>


int led = 5;           // the pin that the LED is attached to

static byte myNodeID = 5;   // node ID used for this unit
static byte myNetGroup = 212; // netGroup used for this unit

static byte quiet;

#define SERIAL_BAUD 57600

#define DATAFLASH   1   // check for presence of DataFlash memory on JeeLink
#define FLASH_MBIT  16  // support for various dataflash sizes: 4/8/16 Mbit

#define LED_PIN     9   // activity LED, comment out to disable


#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

static unsigned long now () {
    // FIXME 49-day overflow
    return millis() / 1000;
}

static void activityLed (byte on) {
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !on);
#endif
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.print("\n[RF12_Receiver.8]");

    rf12_initialize(myNodeID, RF12_868MHZ, myNetGroup);

    rf12_encrypt(RF12_EEPROM_EKEY);
    
    pinMode(led, OUTPUT);
    digitalWrite(led, LOW);
    
}

void loop() {
    //if (Serial.available())
    //    handleInput(Serial.read());

    if (rf12_recvDone() && rf12_crc == 0) {
        byte n = rf12_len;
        if (rf12_crc == 0) {
            Serial.print("OK");
        } else {
            if (quiet)
                return;
            Serial.print(" ?");
            if (n > 20) // print at most 20 bytes if crc is wrong
                n = 20;
        }
        if (myNetGroup == 0) {
            Serial.print("G ");
            Serial.print((int) rf12_grp);
        }
        Serial.print(' ');
        Serial.print((int) rf12_hdr);
        for (byte i = 0; i < n; ++i) {
            Serial.print(' ');
            Serial.print((int) rf12_data[i]);
        }
        Serial.println();
        
         if (rf12_data[0] == 1) {
            digitalWrite(led, HIGH);
           }
           if (rf12_data[0] == 2) {
            digitalWrite(led, LOW);
           }
        if (RF12_WANTS_ACK && (myNodeID & COLLECT) == 0) {
                
                rf12_sendStart(RF12_ACK_REPLY, 0, 0);
            }
    }
}
