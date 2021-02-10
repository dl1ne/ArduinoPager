#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

#define FREQUENCY     439987500 // 439.9875 MHz
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module
#define POWER_OUT     15 // power output ranges from 0 (5dBm) to 31 (20dBm)

#define SERIAL_BAUD      38400

#define RFM69_CS      8
#define RFM69_IRQ     3
#define RFM69_IRQN    0  // Pin 2 is IRQ 0!
#define RFM69_RST     4

#define PTT_LED       12
#define STATUS_LED    13

#define MAX_LEN       512

#define WORD_EOT      0x17
#define WORD_CFG      0x18

#define FLASH_DELAY   32000U
RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

String message = "";
boolean tx_mode = false;
boolean setup_mode = false;
int setup_step = 0;

uint32_t freq = FREQUENCY;
int power = POWER_OUT;

char cmessage[MAX_LEN];

bool led = false;
uint32_t count = 0U;

void setup() {
  while (!Serial); // wait until serial console is open, remove if not tethered to computer
  Serial.begin(SERIAL_BAUD);
  pinMode(RFM69_RST, OUTPUT);
  pinMode(PTT_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  setup_radio();
}

void setup_radio() {
  // Hard Reset the RFM module
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // Initialize radio
  radio.initialize(RF69_433MHZ,0,0);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(power); // power output ranges from 0 (5dBm) to 31 (20dBm)

  // Set to ham POCSAQ frequency
  radio.setFrequency(freq);

  // Disable encryption
  radio.encrypt(0);
  setup_mode = false;
  tx_mode = false;
  setup_step = 0;
  message = "";
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
    char inChar = (char)Serial.read();

    // normal packets
    if (inChar != WORD_EOT && inChar != WORD_CFG && setup_mode == false) {
      if (tx_mode == false) {
        tx_mode = true;
        radio.set_tx_mode(tx_mode);
        digitalWrite(STATUS_LED, HIGH);
      }
      message = (inChar ^= 0xFF);
      message.toCharArray(cmessage,message.length() + 1);
      radio.send_pocsag(cmessage, strlen(cmessage));
    }

    // end of packet 
    if (inChar == WORD_EOT && tx_mode == true) {
      tx_mode = false;
      radio.set_tx_mode(tx_mode);
      digitalWrite(STATUS_LED, LOW);
      Serial.write(WORD_EOT);
    }

    if (inChar != WORD_CFG && inChar != WORD_EOT && setup_mode == true) {
      message = message + inChar;
    }

    // setup step 1, end, set freq
    if (inChar == WORD_CFG && setup_mode == true && setup_step == 1) {
      freq = (uint32_t)message[0] << 24 |
             (uint32_t)message[1] << 16 |
             (uint32_t)message[2] << 8  |
             (uint32_t)message[3];
      message = "";
      setup_step++;
    }

    // setup step 2, end, set power
    if (inChar == WORD_EOT && setup_mode == true && setup_step == 2) {
      power = message.toInt();
      message = "";
      digitalWrite(STATUS_LED, LOW);
      setup_radio();
    }

    // enable setup mode
    if (inChar == WORD_CFG && setup_mode == false && tx_mode == false) {
      setup_mode = true;
      setup_step++;
      message = "";
      digitalWrite(STATUS_LED, HIGH);
    }
    
  }
}
