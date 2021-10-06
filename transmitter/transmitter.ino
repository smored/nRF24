/* TRANSMITTER CODE
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */

/* Modified transmitter code to send data from a remote
 * to an RC car on the 2.4GHz bandwidth
 * Modified by Kurt Querengesser
 */ 

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define STATLED 9 // LED pin #
#define CE_PIN 6 // Chip Enable pin
#define CSN_PIN 5 // Chip Select Not pin
#define POTLPIN 21 // Left potentiometer analog pin
#define POTRPIN 20 // Right potentiometer analog pin
#define TRANSMITTER true // The role of this device 
#define DEADZONE 25 // How big the potentiometer deadzone should be in one direction (2x for both)
#define CENTER 512 // Center of the potentiometer

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

// Let these addresses be used for the pair
uint8_t address[][6] = {"1Node", "2Node"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 0; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = TRANSMITTER;  // true = TX role, false = RX role

// Payload to be sent that contains all of the pot values
struct potData {
  uint16_t potR;
  uint16_t potL;
};

// Initializing payload
potData payload = {CENTER, CENTER};

// Setting payload size
int plSize = sizeof(payload);


void setup() {

  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} // hold in infinite loop
  }

  // To set the radioNumber via the Serial monitor on startup
  Serial.print(F("radioNumber = "));
  Serial.println((int)radioNumber);

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // Sets payload size
  radio.setPayloadSize(plSize);

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);

   // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

  // additional setup specific to the node's role
  if (role) {
    radio.stopListening();  // put radio in TX mode
  } else {
    radio.startListening(); // put radio in RX mode
  }

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data



  // Setting pinModes
  pinMode(POTLPIN, INPUT);
  pinMode(POTRPIN, INPUT);


} // setup

void loop() {
  
  static uint16_t potL;
  static uint16_t potR;
  potL = analogRead(POTLPIN);
  potR = analogRead(POTRPIN);

  if ((potR > (CENTER + DEADZONE))
  ||  (potR < (CENTER - DEADZONE))) {
    payload.potR = potR;
    sendData();
  } else {
    payload.potR = CENTER;
  }

  if ((potL > (CENTER + DEADZONE))
  ||  (potL < (CENTER - DEADZONE))) {
    payload.potL = potL;
    sendData();
  } else {
    payload.potL = CENTER;
  }

} // loop

bool sendData() {
    if (role) {
    // This device is a TX node

    bool report = radio.write(&payload, plSize);      // transmit & save the report

    if (report) {
      digitalWrite(STATLED, LOW);
      Serial.print(F("Transmission successful! "));          // payload was delivered
      // Serial.print(F(" us. Sent: "));                      // DEBUG ONLY
      // Serial.println(payload);                               // print payload sent 
      return(true);
    } else {
      digitalWrite(STATLED, HIGH);
      Serial.println(F("Transmission failed or timed out")); // payload was not delivered
      return(false);
    }
  }
}
