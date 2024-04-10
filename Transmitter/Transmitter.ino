#include <SPI.h>
#include <Wire.h>
#include <SSD1306.h>
#include <LoRa.h>
#include "images.h"
#include <DS18B20.h>

DS18B20 ds(2);

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

#define MAX_TIME 10000
#define EXTRA_TIME 3500

#define MAX(A,B) (A > B ? A : B)

SSD1306 display(0x3c, OLED_SDA, OLED_SCL);

// Forward declarations
void displayLoraData(String countStr);
void showLogo();

int temp;
float bat;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println("LoRa Transmitter");

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
  display.drawString(display.getWidth() / 2, display.getHeight() / 2, "LoRa Transmitter");
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
}

void loop() {
  ///////////////////////
  /* INITIALIZE PARAMS */
  ///////////////////////
  static long SignalBW[] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, 500E3};

  static int counter = 0;   // To keep track of the number of packages sent
  static int sf = 7;        // Spread Factor
  static int bw = 0;        // Bandwidth
  static int cr = 5;        // Coding Rate
  static int pow = 0;       // Transmission power

  //////////////////
  /* CHECK PARAMS */
  //////////////////
  double packet_time = getTimePayload(sf, cr, SignalBW[bw]);
  if (packet_time <= MAX_TIME) {
    LoRa.setSpreadingFactor(sf);
    LoRa.setCodingRate4(cr);
    LoRa.setSignalBandwidth(SignalBW[bw]);
    LoRa.setTxPower(pow);

    //////////////
    /* SEND MSG */
    //////////////
    Canonicalize the msg, this way it will contain 64 bytes
    String countStr = toStringSize(String(counter, DEC),4);
    String powStr = toStringSize(String(pow, DEC), 3);
    String sfStr = toStringSize(String(sf, DEC), 2);

    bat = analogRead(A7);
    bat = bat * 0.00168;
    String batStr = toStringSize(String(bat, DEC), 7);
    
    // Send packet
    LoRa.beginPacket(sf == 6);
    LoRa.print("counter = ");
    LoRa.print(countStr);
    LoRa.print(", TP = ");
    LoRa.print(powStr);
    LoRa.print(", bat = ");
    LoRa.print(batStr);
    LoRa.print(", sf = ");
    LoRa.print(sfStr);
    LoRa.print(", cr = ");
    LoRa.print(cr);
    LoRa.print(", bw = "); 
    LoRa.print(bw);
    LoRa.endPacket();

    // Display msg
    displayLoraData(countStr, sf, SignalBW[bw], cr, pow);
    
    Serial.print("======= POW: ");
    Serial.print(pow);
    Serial.print(", BW: ");
    Serial.print(bw);
    Serial.print(", SF: ");
    Serial.print(sf);
    Serial.print(", CR: ");
    Serial.print(cr);
    Serial.print(", TIME: ");
    Serial.print(millis());
    Serial.println(" =======");
    
    // Toggle the led to give a visual indication the packet was sent
    digitalWrite(LED, HIGH);  
    delay(250);
    digitalWrite(LED, LOW);
    delay(250);
    
    //////////////////////
    /* WAIT PACKET TIME */
    //////////////////////
    delay(packet_time+EXTRA_TIME);
    counter++;
  }

  ///////////////////////
  /* MODIFY PARAMETERS */
  ///////////////////////
  if (sf == 12 and bw == 9 and cr == 8) { // We used all cambinations with this transmission power
    pow = (pow+1)%21;
  }
  
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

String toStringSize(String num, int size) {
  int n = num.length();

  for (int i = n; i < size; ++i) {
    num = "0" + num;
  }
  return num;
}

void displayLoraData(String countStr, int sf, long bw, int cr, int pow) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  
  display.drawString(0, 0, "Sending packet: ");
  display.drawString(90, 0, countStr);
  display.drawString(0, 10, "Power: ");
  display.drawString(90, 10, String(pow, DEC));
  display.drawString(0, 20, "Bat Volt: ");
  display.drawString(90, 20, String(bat, DEC));
  display.drawString(0, 30, "Spreading factor: ");
  display.drawString(90, 30, String(sf, DEC));
  display.drawString(0, 40, "Bandwidth: ");
  display.drawString(90, 40, String(bw, DEC));
  display.drawString(0, 50, "Coding rate: ");
  display.drawString(90, 50, "4 /");
  display.drawString(104, 50, String(cr, DEC));

  display.display();
}

void showLogo() {
  uint8_t x_off = (display.getWidth() - logo_width) / 2;
  uint8_t y_off = (display.getHeight() - logo_height) / 2;

  display.clear();
  display.drawXbm(x_off, y_off, logo_width, logo_height, logo_bits);
  display.display();
}
