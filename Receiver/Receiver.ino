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
#define RST     23   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

#define LED     25

#define TIME_P  2000
#define TIME_T  10000 

#define MAX(A,B) (A > B ? A : B)

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
  ///////////////////////
  /* INITIALIZE PARAMS */
  ///////////////////////
  static long SignalBW[] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, 500E3};
  
  static int sf = 7;        // Spread Factor
  static int bw = 0;        // Bandwidth
  static int cr = 5;        // Coding Rate

  //////////////////
  /* CHECK PARAMS */
  //////////////////
  double packet_time = getTimePayload(sf, cr, SignalBW[bw]);
  if (packet_time <= TIME_P) {
    LoRa.setSpreadingFactor(sf);
    LoRa.setCodingRate4(cr);
    LoRa.setSignalBandwidth(SignalBW[bw]);

    //////////////////
    /* UPDATE TIMER */
    //////////////////
    long waitTime = millis();

    //////////////////
    /* WAIT FOR MSG */
    //////////////////
    while ((millis() - waitTime) <= TIME_T) {
      // READ PORTS
      int packetSize = LoRa.parsePacket();
      
      // CHECK IF WE RECEIVED THE MSG
      if (packetSize) {
        String packet = "";
        for (int i = 0; i < packetSize; i++) {
          packet += (char)LoRa.read();
        }

        // PRINT THE MSG
        String rssi = String(LoRa.packetRssi(), DEC);
        String snr = String(LoRa.packetSnr(), DEC);
        printLoraData(packetSize, packet, rssi, snr, sf, SignalBW[bw], cr);

        displayLoraData(packetSize, packet, rssi, cr);
        
        // Toggle the led to give a visual indication the packet was sent
        digitalWrite(LED, !digitalRead(LED));
      }
      delay(10);
    }
  }
  
  ///////////////////////
  /* MODIFY PARAMETERS */
  ///////////////////////
  if (bw == 9 and cr == 8) {  // We used all combinations of bandwidth and coding rate with this sp
      sf = (sf-5)%7 + 6; // Change the spread spectrum
  }

  if (cr == 8) {  // We used all types of coding rates with this bw
    bw = (bw+1)%10; // Change the bandwidth
  }

  cr = (cr-4)%4 + 5; // Change the coding rate
}

// Returns the smallest integer bigger than or equal to n
int roundUp(double n) {
  int m = int(n);
  return (double(m) < n ? m+1 : m);
}

// Returns the symbol time in miliseconds
double getSymbolTime(int sf, long bw) {
  return (2 << (sf-1))*1000/bw;
}

// Returns the number of symbols of the payload
int getNumSymbols(int sf, int cr, int pl, int crc, int ih, int de) {
  double num = 8*pl - 4*sf + 28 - 16*crc - 20*ih;
  double denom = 4*(sf-(2*de));
  int n = roundUp(num/denom);
  return 8 + MAX(n*cr,0);
}

double getTimePayload(int sf, int cr, long bw) {
  double ts = getSymbolTime(sf, bw);
  int ih = (sf == 6); // With a SF = 6 we must activate the implicit header
  int de = (ts > 16); // Inrease robustness if the symbol is more than 16ms
  int np = getNumSymbols(sf, cr, 64, 0, ih, de);
  return ts*np;
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
