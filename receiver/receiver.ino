/* RECEIVER CODE
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */

/* Modified receiver code to receive data from a remote
 * on the 2.4GHz bandwidth
 * Modified by Kurt Querengesser
 */ 

#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <Servo.h>

#define STATLED 9 // LED pin #
#define CE_PIN 6 // Chip Enable pin
#define CSN_PIN 5 // Chip Select Not pin
#define RECEIVER false // The role of this device
#define CENTER 512 // Center of the potentiometer
#define SERVOPIN 8 // Pin to attach servo to

// Instantiate Servo object
Servo myservo;

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

// Let these addresses be used for the pair
uint8_t address[][6] = {"1Node", "2Node"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = RECEIVER;  // true = TX role, false = RX role

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

  // print example's introductory prompt
  Serial.println(F("RF24/examples/GettingStarted"));

  // // To set the radioNumber via the Serial monitor on startup
  // Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
  // while (!Serial.available()) {
  //   // wait for user input
  // }
  //char input = Serial.parseInt();
  //radioNumber = input == 1;
  Serial.print(F("radioNumber = "));
  Serial.println((int)radioNumber);

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  radio.setPayloadSize(plSize);

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0

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

  // Initialize IO
  pinMode(STATLED, OUTPUT);
  myservo.attach(SERVOPIN);

} // setup

void loop() {
  receiveData();

} // loop

bool receiveData() {
  if (!role) {
    // This device is a RX node

    uint8_t pipe;
    if (radio.available(&pipe)) {             // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
      radio.read(&payload, bytes);            // fetch payload from FIFO
      Serial.print(F("Received "));
      // Serial.print(bytes);                    // print the size of the payload
      // Serial.print(F(" bytes on pipe "));
      // Serial.print(pipe);                     // print the pipe number // DEBUG ONLY
      digitalWrite(STATLED, LOW); // Turn off the Status LED to indicate transmission was sucessful
    } else {
      digitalWrite(STATLED, HIGH); // Turn on the Status LED to indicate standby mode
      // Also put a timer to automatically shut down if no signal is received for a period of time
    }
  } // role
}

// Returns vehicle to default steering position and zero throttle
void defaultPos() {
  throttle(CENTER);
  steer(CENTER);
}

// Maps ADC value to throttle value
void throttle(int val) {
  val = map(val, 0, 1023, -100, 100); // map(input, adcLow, adcHigh, -100%throttle, +100%throttle)
  // run motor based on val, negative being reverse
}

void steer(int val) {
  val = map(val, 0, 1023, 0, 180); // map(input, adcLow, adcHigh, 0 degrees, 180 degrees) NOTE: will need tweaking for precise values
  myservo.write(val);
}