#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

#define FREQUENCY     439987500 // 439.9875 MHz
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

#define SERIAL_BAUD      38400

#define RFM69_CS      8
#define RFM69_IRQ     3
#define RFM69_IRQN    0  // Pin 2 is IRQ 0!
#define RFM69_RST     4

#define PTT_LED       12
#define STATUS_LED    13

#define MAX_LEN                512

#define FLASH_DELAY   32000U
RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

String message = "";
boolean tx_mode = false;
char cmessage[MAX_LEN];

bool led = false;
uint32_t count = 0U;

void setup() {
  while (!Serial); // wait until serial console is open, remove if not tethered to computer
  Serial.begin(SERIAL_BAUD);

  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // Initialize radio
  radio.initialize(RF69_433MHZ,0,0);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)

  // Set to ham POCSAQ frequency
  radio.setFrequency(FREQUENCY);

  // Disable encryption
  radio.encrypt(0);

  // Initialize LED pins
  pinMode(PTT_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
}

void loop() {
  count++;
  if (count > FLASH_DELAY) {
    digitalWrite(STATUS_LED, led ? LOW : HIGH);
    led = !led;
    count = 0U;
  }
}

void serialEventRun(void) {
  if (Serial.available()) serialEvent();
}

void serialEvent() {
  while (Serial.available()) {
    if (tx_mode == false) {
      tx_mode = true;
      radio.set_tx_mode(true);
      digitalWrite(STATUS_LED, HIGH);
    }
    char inChar = (char)Serial.read();
    if (inChar != 0x17) { // end of transmission
      message = (inChar ^= 0xFF);
      message.toCharArray(cmessage,message.length() + 1);
      radio.send_pocsag(cmessage, strlen(cmessage));
    }
    else {
      tx_mode = false;
      radio.set_tx_mode(false);
      digitalWrite(STATUS_LED, LOW);
      Serial.write(0x17);
    }
  }
}
