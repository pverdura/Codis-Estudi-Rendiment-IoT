#include <SPI.h>
#include <Wire.h>
#include <SSD1306.h>
#include <LoRa.h>
#include "images.h"

//#define LORA_BAND    433
#define LORA_BAND    868
//#define LORA_BAND    915

#define OLED_SDA    21
#define OLED_SCL    22
//#define OLED_RST    16

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     23  // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

#define LED     25

SSD1306 display(0x3c, OLED_SDA, OLED_SCL);

// Forward declarations
void displayLoraData(int packetSize, String packet, String rssi);
void showLogo();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println("LoRa Receiver");

  // Configure the LED an an output
  pinMode(LED, OUTPUT);

  // Configure OLED by setting the OLED Reset HIGH, LOW, and then back HIGH
  /*pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, HIGH);
  delay(100);
  digitalWrite(OLED_RST, LOW);
  delay(100);
  digitalWrite(OLED_RST, HIGH);*/

  display.init();
  display.flipScreenVertically();

  showLogo();
  delay(2000);

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(display.getWidth() / 2, display.getHeight() / 2, "LoRa Receiver");
  display.display();
  delay(2000);

  // Configure the LoRA radio
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(LORA_BAND * 1E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("init ok");

  // Set the radio into receive mode
  LoRa.receive();
  delay(1500);
}

void loop() {
  static long SignalBW[] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, 500E3};
  static int sf = 7;        // Spread Factor
  static int bw = 7;        // Bandwidth
  static int cr = 4;        // Coding Rate
  static bool received = true;

  if (received) {
    if (bw == 9 and cr == 8) {  // We used all combinations of bandwidth and coding rate with this sp
        sf = (sf-5)%7 + 6; // Change the spread spectrum
    }

    if (cr == 8) {  // We used all types of coding rates with this bw
      bw = (bw+1)%10; // Change the bandwidth
    }

    cr = (cr-4)%4 + 5;
  
    LoRa.setSpreadingFactor(sf);
    LoRa.setCodingRate4(cr);
    LoRa.setSignalBandwidth(SignalBW[bw]);
    received = false;
  }

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    received = true;

    String packet = "";
    for (int i = 0; i < packetSize; i++) {
      packet += (char)LoRa.read();
    }
    String rssi = String(LoRa.packetRssi(), DEC);
    String snr = String(LoRa.packetSnr(), DEC);
    printLoraData(packetSize, packet, rssi, snr, sf, SignalBW[bw], cr);

    displayLoraData(packetSize, packet, rssi, cr);
    // toggle the led to give a visual indication the packet was sent
    digitalWrite(LED, !digitalRead(LED));
  }
  delay(10);
}

void displayLoraData(int packetSize, String packet, String rssi, int cr) {
  String packSize = String(packetSize, DEC);

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0 , 15 , "Received " + packSize + " bytes");
  display.drawStringMaxWidth(0 , 26 , 128, packet);
  display.drawString(30, 0, rssi);
  display.drawString(0, 0, "RSSI:");
  display.drawString(80, 0, "CR:");
  display.drawString(100, 0, String(cr, DEC));
  display.display();
}

void printLoraData(int packetSize, String packet, String rssi, String snr, int sf, long bw, int cr) {
  Serial.println("Packet size:      " + String(packetSize, DEC));
  Serial.println("Packet:           " + packet);
  Serial.println("RSSI:             " + rssi);
  Serial.println("SNR:              " + snr);
  Serial.println("Spreading factor: " + String(sf, DEC));
  Serial.println("Bandwidth:        " + String(bw, DEC));
  Serial.println("Coding rate:      " + String(cr, DEC));
  Serial.println("------------------------------------");
}

void showLogo() {
  uint8_t x_off = (display.getWidth() - logo_width) / 2;
  uint8_t y_off = (display.getHeight() - logo_height) / 2;

  display.clear();
  display.drawXbm(x_off, y_off, logo_width, logo_height, logo_bits);
  display.display();
}
