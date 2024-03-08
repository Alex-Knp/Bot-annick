/*
 * This example code shows how to use the can bus on the teensy
 */

// Can library
#include <FlexCAN_T4.h>
// Can object (can2 as we use pin 0 and 1)
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

// Led
const int led_pin = 21;
int value;

// Time variable
int time_offset;
int time_current;

void setup(void) {
  // Enable serial
  Serial.begin(115200);

  // Can configuration
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  // Function to call when there is a message in the mailbox
  Can0.onReceive(canSniff);
  Can0.mailboxStatus();

  // Pin configuration
  pinMode(led_pin,OUTPUT);

  // Default value
  time_offset = millis();
  value = 1;
}

void canSniff(const CAN_message_t &msg) {
  // Print info about the received message
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

void loop() {
  // Triggers action for the can
  Can0.events();

  time_current = millis();
  if (time_current - time_offset > 2000 ) {
    // Create a message with a random ID
    CAN_message_t msg;
    msg.id = random(0x1,0x7FE);
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = i + 1;
    // Send it
    Can0.write(msg);

    // update time variable
    time_offset = time_current;

    // Flip led
    value = 1 - value;
    digitalWrite(led_pin,value);
  }

}
