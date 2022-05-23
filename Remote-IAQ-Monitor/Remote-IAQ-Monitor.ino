/* 
  This is an example for testing the Remote Indoor Air Quality Monitor
  Arduino Nano 33 IoT
  Modify Arduino_BHY2Host to support Arduino Nano 33 IoT
  defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_SAMD_MKRWIFI1010)
  to
  defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT)
  Arduino_BHY2Host.h 
  Author: Enrique Albertos
  Date: 2022-05-21
*/

#define DEBUG true

#include "Arduino_BHY2Host.h"
#include <SPI.h>
#include <Wire.h>

// Display includes
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include "TFTSevenSegmentDecimalDisplay.h"
#include "icons.h"
#include "fonts/FreeMonoBoldOblique18pt7b.h"
#include "fonts/FreeMonoBold9pt7b.h"

// NTP Time includes
#include <RTCZero.h>
#define NTPUPDATE true
// Wifi Includes
#ifdef NTPUPDATE
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include "arduino_secrets.h"

// Define NTP Client to get time
WiFiUDP udpSocket;
const long utcOffsetWinter = 3600; // Offset from UTC in seconds (3600 seconds = 1h) -- UTC+1 (Central European Winter Time)
const long utcOffsetSummer = 7200; // Offset from UTC in seconds (7200 seconds = 2h) -- UTC+2 (Central European Summer Time)
unsigned long lastupdate = 0UL;
NTPClient ntpClient(udpSocket, "pool.ntp.org", utcOffsetWinter);
#endif NTPUPDATE

// Assign human-readable names to some common 16-bit color values:
#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GAUGE_GREEN 0x0320
#define GAUGE_YELLOW 0xFFE0
#define GAUGE_ORANGE 0xFC60
#define GAUGE_RED 0xF800
#define ST77XX_GRAY_C8 0xCE59
#define ST77XX_GRAY_FA 0xFFDF


SensorBSEC co2Sensor(SENSOR_ID_BSEC);

float niclaTemperature;
int iaq;
int accuracy;
float niclaHumidity;
int niclaSeconds;
int printTime = 0;
/* Create an rtc object */
RTCZero rtc;

unsigned long lastEnvironmentUpdate;
unsigned long lastTimeUpdate;
unsigned long lastTimeDisplayUpdate;


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 160 // OLED display height, in pixels

// Hardware, PIN assignements
#define TFT_CS        10
#define TFT_RST        -1 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         8
#define LEFT_BUTTON_PIN A1
#define RIGHT_BUTTON_PIN A7
#define ENTER_BUTTON_PIN 9
#define LCD_LED_PWM 3
#define DEBOUNCE_DELAY_MS 200


Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

uint16_t backgroundColor = tft.color565(20, 20, 20);
uint16_t foregroundColor = WHITE;
// virtual 7 segment displays
TFTSevenSegmentDecimalDisplay co2display(&tft, 5,50, 15, 32, foregroundColor, backgroundColor, 4);
TFTSevenSegmentDecimalDisplay tempdisplay(&tft, 0,SCREEN_HEIGHT -30,6, 10, foregroundColor, backgroundColor, 1);
TFTSevenSegmentDecimalDisplay humiditydisplay(&tft, SCREEN_WIDTH - 60,SCREEN_HEIGHT -30, 6, 10, foregroundColor, backgroundColor, 1);

#define DEG2RAD 0.0174532925

// icons
const tImage termo3 = {termo6x16, 6, 16, 8};
const tImage humidity = {humidity6x16, 6, 16, 8};

bool envEnabled = false;
bool niclaEnabled = true;
// defaul TFT PWM LED level, 0 to 255
int tftLedLevel = 128;
///////////////////////////////////////////////////////////////////////////////////////////////
void initTFT();

// states
typedef enum  {IDLE=0, LEFT, RIGHT, ENTER} Actions;

Actions action = IDLE;

void leftButtonISR() {
  unsigned long msNow = millis();
  static unsigned long last;
  if (msNow-last >DEBOUNCE_DELAY_MS) {
    action = LEFT;
    last= msNow;
  }  
}

void rightButtonISR() {
  unsigned long msNow = millis();
   static unsigned long last;
  if (msNow-last >DEBOUNCE_DELAY_MS) {
    action = RIGHT;
    last= msNow;
  }
}

void enterButtonISR() {
  unsigned long msNow = millis();
   static unsigned long last;
  if (msNow-last >DEBOUNCE_DELAY_MS) {
    action = ENTER;
    last= msNow;
  }
}

void setupNavigationButtons() {
  pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ENTER_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_BUTTON_PIN), leftButtonISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_BUTTON_PIN), rightButtonISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENTER_BUTTON_PIN), enterButtonISR, RISING);
}

void setupTftPWMLedControl() {
  pinMode(LCD_LED_PWM, OUTPUT);
  analogWrite(LCD_LED_PWM, tftLedLevel % 256);
}

void setupNiclaBHYHost() {
#if DEBUG
  BHY2Host.debug(Serial);
#endif  
  Serial.println("Configuring Nicla...");
  // Update function should be continuously polled if PASSTHORUGH is ENABLED
  // NiclaWiring NICLA_VIA_BLE
  while (!BHY2Host.begin(false, NICLA_VIA_BLE)) {}
  Serial.println("NICLA device found!");
  co2Sensor.begin();
}

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
  delay(1500);
#endif

  setupNavigationButtons();
  setupTftPWMLedControl();
  setupTime();
  setupNiclaBHYHost();
  printTime = millis();

  delay(2000);        
  initTFT();
  initDisplay();
}

void loop()
{

    unsigned long msNow = millis();
    if(action != IDLE) {
      tft.setCursor(0,0);
    
      switch(action) {
        case LEFT: Serial.println("LEFT!");tftLedLevel-=10; break;
        case RIGHT:Serial.println("RIGHT!");tftLedLevel+=10;break;
        case ENTER:Serial.println("ENTER!");tftLedLevel=128; break;
        default: break;
      }
      tftLedLevel %=256;
       analogWrite(LCD_LED_PWM, tftLedLevel % 256);
      action= IDLE;
    }
  if (envEnabled && msNow - lastTimeDisplayUpdate >= 1000L )
  {
      displayDateTime();
      if( msNow - lastTimeUpdate >= 300000L) { // refresh time each 5 minutes
        //updateTime();
        lastTimeUpdate = msNow;
      }
      lastTimeDisplayUpdate = msNow;
      
  }

  if (niclaEnabled) {
      BHY2Host.update(100);
      
      if (millis() - printTime > 1000) {
        printTime = millis();
        niclaSeconds = millis()/1000;
#ifdef DEBUG
        Serial.println(co2Sensor.toString());
#endif          
        niclaTemperature = co2Sensor.comp_t();
        iaq = co2Sensor.iaq();
        accuracy =  co2Sensor.accuracy();
        niclaHumidity = co2Sensor.comp_h();


        displayLevel((int)iaq);
        displayTemperature((int) niclaTemperature);
        displayHumidity((int) niclaHumidity);
        displayAccuracy((int) accuracy);
      }

      
    }

}

void setupTime() {
  rtc.begin();
  #ifdef NTPUPDATE 
  WiFi.begin(SECRET_SSID, SECRET_PASS);
 
  Serial.print("Connecting to ");
  Serial.print(SECRET_SSID);
 
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(250);
  }
 
  Serial.println(" Done!");
   
  ntpClient.begin();
  envEnabled = true;
  ntpClient.update();
  rtc.setHours(ntpClient.getHours());
  rtc.setMinutes(ntpClient.getMinutes());
  rtc.setSeconds(ntpClient.getSeconds());
  ntpClient.end();
  WiFi.end();
  #endif
}


// Display functions  -----------------------------------------------------------------
void initTFT() {
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);   
}

void initDisplay(void) {

  // Use this initializer if using a 1.8" TFT screen:

  Serial.println(F("Initialized"));
  tft.fillScreen(ST77XX_BLACK);    
  tft.drawRGBBitmap(3,135, (const uint16_t *)termo3.data, termo3.width, termo3.height); // Copy to screen
  tft.drawRGBBitmap( tft.width()/2+15,135, (const uint16_t *)humidity.data, humidity.width, humidity.height); // Copy to screen
}

void displayTemperature(const int temperature){
  tft.setTextSize(0);
  tft.setCursor(6,156);
  tft.setTextColor(ST77XX_GRAY_FA);
  tft.setFont(&FreeMonoBold9pt7b);
  //drawUpdatedValue2d(temperature, 18 ,156);  
  tempdisplay.display(temperature);
  tft.setFont(NULL);
  tft.setCursor(14,146);
  tft.print("C");
}

void displayHumidity(const int humidity) {
  tft.setTextSize(0);
  tft.setTextColor(ST77XX_GRAY_FA);
  tft.setFont(&FreeMonoBold9pt7b);
  //drawUpdatedValue2d(humidity, tft.width()/2 + 28,156);
  humiditydisplay.display(humidity);
  tft.setFont(NULL);
  tft.setCursor(tft.width() -40,146);
  tft.print("%");
}

void displayLevel(const int co2level){
    tft.setFont(NULL);
    //tft.setCursor(55,30);
    //tft.setTextColor(ST77XX_GRAY_FA);
    //tft.print("IAQ");
    tft.setCursor(18,82);
    tft.setTextColor(ST77XX_GRAY_FA);
    tft.setTextSize(0);
    tft.setFont(&FreeMonoBoldOblique18pt7b);
    //drawUpdatedValue4d(co2level,20,88);
    tft.setFont(NULL);
    tft.setCursor(88,90);
    tft.setTextColor(ST77XX_GRAY_FA);
    tft.print("IAQ");
    drawGauge(co2level, 10, 600);
    tft.setFont(NULL);
    co2display.display(co2level);
}

void drawUpdatedValue4d(const int number, int x, int y)
{
    int16_t x1, y1;
    uint16_t w, h;
    char buffer[5]="6666";
    snprintf(buffer,sizeof(buffer), "%4d", number);
    tft.getTextBounds(buffer, x, y, &x1, &y1, &w, &h); //calc width of new string
    tft.fillRect(x,y-h,w+4, h+2,  ST77XX_BLACK);
    tft.setCursor(x,y);
    tft.print(buffer);
}

void drawUpdatedValue2d(const int number, int x, int y)
{
    int16_t x1, y1;
    uint16_t w, h;
    char buffer[3]="66";
    snprintf(buffer,sizeof(buffer), "%2d", number);
    tft.getTextBounds(buffer, x, y, &x1, &y1, &w, &h); //calc width of new string
    tft.fillRect(x,y-h,w+4, h+2,  ST77XX_BLACK);
    tft.setCursor(x,y);
    tft.print(buffer);
}

// format and print time
void displayDateTime()
{ 
    //ntpClient.update();
    char buf[8];
    sprintf(buf, "%.2d:%.2d", (rtc.getHours() + 1) % 24, rtc.getMinutes());
    tft.fillRect(tft.width() - 33, 3,33,12,ST77XX_BLACK);
    tft.setFont(NULL);
    tft.setTextSize(0);
    tft.setCursor(tft.width() - 33, 3);
    tft.setTextColor(ST77XX_GRAY_FA);
    tft.print(buf);
}

void displayAccuracy(const int accuracy)
{ 
    int width = 4;
    for (int i=0; i<4; i++) {
      if (i<=accuracy) {
        tft.fillRect(i*width+2*i, 2,width,6,ST77XX_WHITE);
      } else {
        tft.fillRect(i*width+2*i+1, 2+1,width-2,6-2,ST77XX_BLACK);
        tft.drawRect(i*width+2*i, 2,width,6,ST77XX_WHITE);
      }
    }
}


void drawGauge (const int level, const int min, const int max) {
    static int lastValue;
    int degrees = (log(level)  / log(2) - log(min)  / log(2)) / (log(max)/log(2) - log(min)/log(2)) * 240;
    fillArc2(tft.width()/2, tft.height()/2-8, -120, 20, tft.width()/2-6, tft.width()/2-6, 6, GAUGE_GREEN);
    fillArc2(tft.width()/2, tft.height()/2-8,  -60, 20, tft.width()/2-6, tft.width()/2-6, 6, GAUGE_YELLOW);
    fillArc2(tft.width()/2, tft.height()/2-8,    0, 20, tft.width()/2-6, tft.width()/2-6, 6, GAUGE_ORANGE);
    fillArc2(tft.width()/2, tft.height()/2-8,   60, 20, tft.width()/2-6, tft.width()/2-6, 6, GAUGE_RED);
    fillArc2(tft.width()/2, tft.height()/2-8, lastValue-120, 1, tft.width()/2-16, tft.width()/2-16, 6, ST7735_BLACK );
    fillArc2(tft.width()/2, tft.height()/2-8, degrees-120,   1, tft.width()/2-16, tft.width()/2-16, 6, ST7735_WHITE );
    lastValue = degrees;
}

// #########################################################################
// Draw a circular or elliptical arc with a defined thickness
// #########################################################################

// x,y == coords of centre of arc
// start_angle = 0 - 359
// seg_count = number of 3 degree segments to draw (120 => 360 degree arc)
// rx = x axis radius
// yx = y axis radius
// w  = width (thickness) of arc in pixels
// colour = 16 bit colour value
// Note if rx and ry are the same then an arc of a circle is drawn

void fillArc2(int x, int y, int start_angle, int seg_count, int rx, int ry, int w, unsigned int colour)
{

  byte seg = 3; // Segments are 3 degrees wide = 120 segments for 360 degrees
  byte inc = 3; // Draw segments every 3 degrees, increase to 6 for segmented ring

    // Calculate first pair of coordinates for segment start
    float sx = cos((start_angle - 90) * DEG2RAD);
    float sy = sin((start_angle - 90) * DEG2RAD);
    uint16_t x0 = sx * (rx - w) + x;
    uint16_t y0 = sy * (ry - w) + y;
    uint16_t x1 = sx * rx + x;
    uint16_t y1 = sy * ry + y;

  // Draw colour blocks every inc degrees
  for (int i = start_angle; i < start_angle + seg * seg_count; i += inc) {

    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * DEG2RAD);
    float sy2 = sin((i + seg - 90) * DEG2RAD);
    int x2 = sx2 * (rx - w) + x;
    int y2 = sy2 * (ry - w) + y;
    int x3 = sx2 * rx + x;
    int y3 = sy2 * ry + y;

    tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
    tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);

    // Copy segment end to sgement start for next segment
    x0 = x2;
    y0 = y2;
    x1 = x3;
    y1 = y3;
  }
}
