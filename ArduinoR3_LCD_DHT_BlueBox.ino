/*
  LiquidCrystal 16x2 LCD display

  The circuit:
 * 5V to Breadboard
 * Ground to Breadboard

 * LCD VSS pin to ground
 * LCD VDD pin to 5V
 * LCD V0 pin to potentiometer (center pin)
    - potentiometer (pin 1 = 5V, pin 3 = ground)

 * LCD RS pin to digital pin 7
 * LCD RW pin to ground
 * LCD E nable pin to digital pin 8

 * LCD D4 pin to digital pin 9
 * LCD D5 pin to digital pin 10
 * LCD D6 pin to digital pin 11
 * LCD D7 pin to digital pin 12

 * LCD A pin to 220ohm resistor to 5V
 * LCD K pin to ground

 * DHT Sensor (S on front left)
 * DHT Pin 1 to digital pin 2
 * DHT Pin 2 to 5V
 * DHT Pin 3 to gorund

 */

// include the libraries
#include <LiquidCrystal.h>
#include <PulseSensorPlayground.h>
#include "DHT.h"

// GPS
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int RXPin = 3, TXPin = 4;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// end GPS

// Define the DHT Pin
#define DHTPIN 2

// Type of DHT Sensor
#define DHTTYPE DHT11

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated

// constants won't change HH UL * MM UL * SS UL * MILS UL
const unsigned long interval = 15UL * 1000UL;  // Output every 15 seconds

int humidLast = 0;
int tempLast = 0;

// const int OUTPUT_TYPE = SERIAL_PLOTTER;
// const int PULSE_INPUT = A2;
// const int PULSE_BLINK = LED_BUILTIN;
// const int PULSE_FADE = 5;
// const int THRESHOLD = 530;

// /*
//    All the PulseSensor Playground functions.
// */
// PulseSensorPlayground pulseSensor;

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// Calculations for Temp Sensor - BD1020
// Math:  ADC_Voltage = sensorValue * (5V/1024)
//        Tempurature = - (1000 * ADC_Voltage - 1546) / 8.2

// float BD1020_getTemp(unsigned short data) {
//   float tmp, temp;

//   tmp = (float)data * (5 / 1024);
//   temp = -(1000 * tmp - 1546) / 8.2;

//   return (temp);
// }


static const int MAX_SATELLITES = 40;

TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1);  // $GPGSV sentence, first element
TinyGPSCustom messageNumber(gps, "GPGSV", 2);       // $GPGSV sentence, second element
TinyGPSCustom satsInView(gps, "GPGSV", 3);          // $GPGSV sentence, third element
TinyGPSCustom satNumber[4];                         // to be initialized later
TinyGPSCustom elevation[4];
TinyGPSCustom azimuth[4];
TinyGPSCustom snr[4];

struct {
  bool active;
  int elevation;
  int azimuth;
  int snr;
} sats[MAX_SATELLITES];


void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);

  Serial.println(F("SatelliteTracker.ino"));
  Serial.println(F("Monitoring satellite location and signal strength using TinyGPSCustom"));
  Serial.print(F("Testing TinyGPSPlus library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();

  // Initialize all the uninitialized TinyGPSCustom objects
  for (int i = 0; i < 4; ++i) {
    satNumber[i].begin(gps, "GPGSV", 4 + 4 * i);  // offsets 4, 8, 12, 16
    elevation[i].begin(gps, "GPGSV", 5 + 4 * i);  // offsets 5, 9, 13, 17
    azimuth[i].begin(gps, "GPGSV", 6 + 4 * i);    // offsets 6, 10, 14, 18
    snr[i].begin(gps, "GPGSV", 7 + 4 * i);        // offsets 7, 11, 15, 19
  }

  // Setup LCD
  lcd.begin(16, 2);

  // Start DHT Sensor
  dht.begin();

  // pulseSensor.analogInput(PULSE_INPUT);
  // pulseSensor.blinkOnPulse(PULSE_BLINK);
  // pulseSensor.fadeOnPulse(PULSE_FADE);

  // pulseSensor.setSerial(Serial);
  // pulseSensor.setOutputType(OUTPUT_TYPE);
  // pulseSensor.setThreshold(THRESHOLD);

  // // Now that everything is ready, start reading the PulseSensor signal.
  // if (!pulseSensor.begin()) {
  //   /*
  //      PulseSensor initialization failed,
  //      likely because our particular Arduino platform interrupts
  //      aren't supported yet.

  //      If your Sketch hangs here, try PulseSensor_BPM_Alternative.ino,
  //      which doesn't use interrupts.
  //   */
  //   for (;;) {
  //     // Flash the led to show things didn't work.
  //     digitalWrite(PULSE_BLINK, LOW);
  //     delay(50);
  //     Serial.println('!');
  //     digitalWrite(PULSE_BLINK, HIGH);
  //     delay(50);
  //   }
  // }

}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    float humid = dht.readHumidity();
    int humidNow = truncf(humid * 10) / 10;

    if (humidNow != humidLast) {
      lcd.setCursor(0, 0);
      lcd.print("Humidity:       ");
      lcd.setCursor(10, 0);
      lcd.print(humidNow);
      lcd.print(" %");
      humidLast = humidNow;
    }

    float temp = dht.readTemperature();
    int tempNow = (temp * 1.8) + 32;

    if (tempNow != tempLast) {
      lcd.setCursor(0, 1);
      lcd.print("Temp    :       ");
      lcd.setCursor(10, 1);
      lcd.print(tempNow);
      lcd.print(" F");
      tempLast = tempNow;
    }

    if (ss.available() > 0) {
      Serial.print("LAT=");
      Serial.print(gps.location.lat(), 6);
      Serial.print(", ");
      Serial.print("LONG=");
      Serial.print(gps.location.lng(), 6);
      Serial.print(", ");
      Serial.print("ALT=");
      Serial.print(gps.altitude.meters());
      Serial.println();

      previousMillis += interval;
    }
  }

  if (ss.available() > 0) {
    gps.encode(ss.read());
    if (totalGPGSVMessages.isUpdated()) {
      for (int i = 0; i < 4; ++i) {
        int no = atoi(satNumber[i].value());
        // Serial.print(F("SatNumber is ")); Serial.println(no);
        if (no >= 1 && no <= MAX_SATELLITES) {
          sats[no - 1].elevation = atoi(elevation[i].value());
          sats[no - 1].azimuth = atoi(azimuth[i].value());
          sats[no - 1].snr = atoi(snr[i].value());
          sats[no - 1].active = true;
        }
      }

      int totalMessages = atoi(totalGPGSVMessages.value());
      int currentMessage = atoi(messageNumber.value());
      if (totalMessages == currentMessage) {
        Serial.print(F("Sats="));
        Serial.print(gps.satellites.value());
        Serial.print(F(" Nums="));
        for (int i = 0; i < MAX_SATELLITES; ++i) {
          if (sats[i].active) {
            Serial.print(i + 1);
            Serial.print(F(" "));
          }
        }
        Serial.print(F(" Elevation="));
        for (int i = 0; i < MAX_SATELLITES; ++i) {
          if (sats[i].active) {
            Serial.print(sats[i].elevation);
            Serial.print(F(" "));
          }
        }
        Serial.print(F(" Azimuth="));
        for (int i = 0; i < MAX_SATELLITES; ++i) {
          if (sats[i].active) {
            Serial.print(sats[i].azimuth);
            Serial.print(F(" "));
          }
        }
        Serial.print(F(" SNR="));
        for (int i = 0; i < MAX_SATELLITES; ++i) {
          if (sats[i].active) {
            Serial.print(sats[i].snr);
            Serial.print(F(" "));
          }
        }
        Serial.println();

        for (int i = 0; i < MAX_SATELLITES; ++i) {
          sats[i].active = false;
        }
      }
    }
  }
}


// int analogVal = analogRead(A0);
// float BDTemp = BD1020_getTemp(analogVal);

// Serial.print("BD1020HFV = ");
// Serial.println(BDTemp);


//   if (pulseSensor.UsingHardwareTimer) {
//     /*
//      Wait a bit.
//      We don't output every sample, because our baud rate
//      won't support that much I/O.
//   */
//     delay(20);
//     // write the latest sample to Serial.
//     pulseSensor.outputSample();
//   } else {
//     /*
//     When using a software timer, we have to check to see if it is time
//     to acquire another sample. A call to sawNewSample will do that.
// */
//     if (pulseSensor.sawNewSample()) {
//       /*
//         Every so often, send the latest Sample.
//         We don't print every sample, because our baud rate
//         won't support that much I/O.
//     */
//       if (--pulseSensor.samplesUntilReport == (byte)0) {
//         pulseSensor.samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;
//         pulseSensor.outputSample();
//       }
//     }
//   }

//   /*
//      If a beat has happened since we last checked,
//      write the per-beat information to Serial.
//    */
//   if (pulseSensor.sawStartOfBeat()) {
//     pulseSensor.outputBeat();
//   }