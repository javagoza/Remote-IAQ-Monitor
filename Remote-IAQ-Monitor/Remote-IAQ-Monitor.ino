/* 
  This is an example for testing the Remote Indoor Air Quality Monitor
  Arduino Nano 33 IoT
  Modify Arduino_BHY2Host to support Arduino Nano 33 IoT
  defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_SAMD_MKRWIFI1010)
  to
  defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT)
  Arduino_BHY2Host.h 


  Hardware
    TFT_CS           10
    TFT_RST       RESET
    TFT_DC            8
    LEFT_BUTTON_PIN  A1  INPUT_PULLUP
    RIGHT_BUTTON_PIN A7  INPUT_PULLUP
    ENTER_BUTTON_PIN  9  INPUT_PULLUP
    SD_CS            A2
    LCD_LED_PWM       3
    TFT_CLOCK      SCK D13
    SD_CLOCK       SCK D13
    TFT_MOSI       MOSI D11
    SD_MOSI        MOSI D11
    SD_MISO        MISO D12
  Author: Enrique Albertos
  Date: 2022-05-21
*/

#define DEBUG true

#include "Arduino_BHY2Host.h"
#include <SPI.h>
#include <Wire.h>

// SD card
#include <SD.h>

// Display includes
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include "TFTSevenSegmentDecimalDisplay.h"
#include "icons.h"
#include "fonts/FreeMonoBoldOblique18pt7b.h"
#include "fonts/FreeMonoBold9pt7b.h"

// NTP Time includes
#include <RTCZero.h>

#include <time.h>
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
NTPClient ntpClient(udpSocket, "pool.ntp.org", utcOffsetSummer);
#endif // NTPUPDATE

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
int logTime = 0;
/* Create an rtc object */
RTCZero rtc;
File dataLogggerFile;


unsigned long lastEnvironmentUpdate;
unsigned long lastTimeUpdate;
unsigned long lastTimeDisplayUpdate;


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 160 // OLED display height, in pixels

// Hardware, PIN assignements
#define TFT_CS           10
#define TFT_RST          -1 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC            8
#define LEFT_BUTTON_PIN  A1
#define RIGHT_BUTTON_PIN A7
#define ENTER_BUTTON_PIN  9
#define SD_CS            A2
#define LCD_LED_PWM       3

#define DEBOUNCE_DELAY_MS 200
#define DATA_LOG_INTERVAL 300000L // 5 minutes


Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

uint16_t backgroundColor = tft.color565(20, 20, 20);
uint16_t foregroundColor = WHITE;
// virtual 7 segment displays
TFTSevenSegmentDecimalDisplay iaqdisplay(&tft, 5,50, 15, 32, foregroundColor, backgroundColor, 4);
TFTSevenSegmentDecimalDisplay tempdisplay(&tft, 0,SCREEN_HEIGHT -30,6, 10, foregroundColor, backgroundColor, 1);
TFTSevenSegmentDecimalDisplay humiditydisplay(&tft, SCREEN_WIDTH - 60,SCREEN_HEIGHT -30, 6, 10, foregroundColor, backgroundColor, 1);

TFTSevenSegmentDecimalDisplay iaqdisplayValues(&tft, 5,SCREEN_HEIGHT/3-32, 15, 32, foregroundColor, backgroundColor, 4);
TFTSevenSegmentDecimalDisplay tempdisplayValues(&tft, 5,SCREEN_HEIGHT/3*2-32, 15, 32, foregroundColor, backgroundColor, 4);
TFTSevenSegmentDecimalDisplay humiditydisplayValues(&tft, 5,SCREEN_HEIGHT-48, 15, 32, foregroundColor, backgroundColor, 4);

#define DEG2RAD 0.0174532925

// icons
const tImage termo3 = {termo6x16, 6, 16, 8};
const tImage humidity = {humidity6x16, 6, 16, 8};

bool envEnabled = false;
bool niclaEnabled = true;
bool isSDReady = false;
// defaul TFT PWM LED level, 0 to 255
int tftLedLevel = 128;
///////////////////////////////////////////////////////////////////////////////////////////////
void initTFT();

// states
typedef enum  {IDLE=0, LEFT, RIGHT, ENTER} Actions;
// displays
typedef enum {IAQDISPLAY =0, VALUESDISPLAY, GRAPHDISPLAY } Layouts;

Actions action = IDLE;
Layouts layout = IAQDISPLAY;

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
  attachInterrupt(digitalPinToInterrupt(LEFT_BUTTON_PIN), leftButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_BUTTON_PIN), rightButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENTER_BUTTON_PIN), enterButtonISR, FALLING);
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

void setupSDCardReader(){
  pinMode(SD_CS, OUTPUT);
    // wait for SD module to start
  if (!SD.begin(SD_CS)) {
    Serial.println("No SD Module Detected");
    isSDReady = false;
  } else {
    Serial.println("SD Module Detected");
    isSDReady = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  setupNavigationButtons();
  setupTftPWMLedControl();
  setupSDCardReader();
  setupTime();
  setupNiclaBHYHost();
  printTime = millis();
  initTFT();
  initDisplay(layout);
}

void sdLogData() {

  char fileName[10];
  char logTime[10];


  sprintf(fileName, "%02d%02d%02d.txt",  rtc.getYear(), rtc.getMonth(),rtc.getDay()); 
  sprintf(logTime, "%02d:%02d:%02d", rtc.getHours() , rtc.getMinutes(), rtc.getSeconds()); 
  Serial.print("fileName :");
  Serial.println(fileName);

    Serial.print("logTime :");
  Serial.println(logTime);
  bool isNewFile = !SD.exists(fileName);

  dataLogggerFile = SD.open(fileName, FILE_WRITE);

   // if the file opened okay, write to it:
  if (dataLogggerFile) {
    if(isNewFile) {
    // write header
      dataLogggerFile.println( F("'time','IAQ','ACCURACY','TEMP','HUMIDITY','CO2EQ','VOCEQ','IAQS'"));
    }

     dataLogggerFile.print( logTime);
     dataLogggerFile.print(",");
     dataLogggerFile.print( co2Sensor.iaq());
     dataLogggerFile.print(",");
     dataLogggerFile.print( co2Sensor.accuracy());
     dataLogggerFile.print(",");
     dataLogggerFile.print( co2Sensor.comp_t());
     dataLogggerFile.print(",");
     dataLogggerFile.print( co2Sensor.comp_h());
     dataLogggerFile.print(",");
     dataLogggerFile.print( co2Sensor.co2_eq());
     dataLogggerFile.print(",");
     dataLogggerFile.print( co2Sensor.b_voc_eq());
     dataLogggerFile.print(",");
     dataLogggerFile.print( co2Sensor.iaq_s());
     dataLogggerFile.println("");           
     // close the file:
     dataLogggerFile.close();
  } else {
      // if the file didn't open, print an error:
      Serial.print("error openning ");
      Serial.println(fileName);
      Serial.print("Log Time ");
      Serial.println(logTime);
  }
}

void loop()
{

    unsigned long msNow = millis();
    if(action != IDLE) {
      tft.setCursor(0,0);
    
      switch(action) {
        case LEFT: tftLedLevel-=10; break;
        case RIGHT: tftLedLevel+=10;break;
        case ENTER: showLogs();nextLayout(); break;
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
    switch (layout) {
      case IAQDISPLAY : updateIAQDisplay(); break;
      case VALUESDISPLAY : updateValuesDisplay(); break;
      case GRAPHDISPLAY : updateGraphDisplay(); break;
      default: updateIAQDisplay();
    } 
    
    if(isSDReady &&  ( millis() - logTime) > DATA_LOG_INTERVAL) {
      logTime = millis();
      sdLogData();
    }
  }
}

void nextLayout() {
    switch (layout) {
        case IAQDISPLAY : layout = VALUESDISPLAY; break;
        case VALUESDISPLAY : layout = GRAPHDISPLAY; break;
        case GRAPHDISPLAY :  layout =IAQDISPLAY; break;
        default: layout = IAQDISPLAY;
    } 
    initDisplay(layout);
}



void initValuesDisplay() {
    tft.setTextSize(0);
  tft.setTextColor(ST77XX_GRAY_FA);
  tft.setFont(NULL);
  tft.setCursor(tft.width()-20, tft.height()/2 -30);
  tft.print(F("iaq"));
  tft.drawRGBBitmap(10, tft.height()/2, (const uint16_t *)termo3.data, termo3.width, termo3.height); // Copy to screen
  tft.drawRGBBitmap(10, tft.height()/2+30, (const uint16_t *)humidity.data, humidity.width, humidity.height); // Copy to screen
  updateValuesDisplay();

}

void updateValuesDisplay(){
  BHY2Host.update(100);      
  if (millis() - printTime > 1000) {
    printTime = millis();
    updateSensorData();
    iaqdisplayValues.display((int)iaq);
    tempdisplayValues.display((int) niclaTemperature);
    humiditydisplayValues.display((int) niclaHumidity);
    displayAccuracy((int) accuracy);
  }
}





void initIAQDisplay() {
  tft.drawRGBBitmap(3,135, (const uint16_t *)termo3.data, termo3.width, termo3.height); // Copy to screen
  tft.drawRGBBitmap( tft.width()/2+15,135, (const uint16_t *)humidity.data, humidity.width, humidity.height); // Copy to screen
}
void updateIAQDisplay() {
    if (niclaEnabled) {
      BHY2Host.update(100);      
      if (millis() - printTime > 1000) {
        printTime = millis();
        niclaSeconds = millis()/1000;
        updateSensorData();
        displayLevel((int)iaq);
        displayTemperature((int) niclaTemperature);
        displayHumidity((int) niclaHumidity);
        displayAccuracy((int) accuracy);
      }      
    }
}



void initGraphDisplay() {
  int o1x = 10;
  int o2x = o1x;
  int o3x = o2x;
  int o1y = 15;
  int marginY = 8;
  int h1 = (tft.height() -o1y) /3 - marginY;
  int o2y = o1y + h1 + marginY ;
  int o3y = o2y + h1 + marginY ;

  int w1 = tft.width() - 2*o1x;
  
  tft.drawRect(o1x, o1y,w1 ,  h1, WHITE);
  tft.drawRect(o2x, o2y, w1,  h1, WHITE);
  tft.drawRect(o3x, o3y,w1,  h1, WHITE);
  int length = h1 /2;

  tft.setTextSize(0);
  tft.setTextColor(ST77XX_GRAY_FA);
  tft.setFont(NULL);
  tft.setCursor(0,o1y+h1);
  tft.print(F("          iaq"));
  tft.setCursor(0,o2y+h1);
  tft.print(F("         temp"));
  tft.setCursor(0,o3y+h1);
  tft.print(F("     % humidity"));

  char fileName[10];
  char title[10];
 
  sprintf(title, "%02d.%02d.%02d",  rtc.getYear(), rtc.getMonth(),rtc.getDay());
  sprintf(fileName, "%02d%02d%02d.txt",  rtc.getYear(), rtc.getMonth(),rtc.getDay()); 
  tft.setCursor(3,3);
  tft.print(title);

  dataLogggerFile = SD.open(fileName, FILE_READ);


  int count = 0;
  if (dataLogggerFile) {
    // read header
    String header = dataLogggerFile.readStringUntil( '\n');
    Serial.println(header);
    while (dataLogggerFile.available()) {
      // 19:33:09,205,3,26.76,35.12,2932,28.85,293

     String found = dataLogggerFile.readStringUntil( ',');
     int fiaq = dataLogggerFile.parseInt(); // co2Sensor.iaq());

     int faccuracy = dataLogggerFile.parseInt();// co2Sensor.accuracy());

     float ftemp = dataLogggerFile.parseFloat();//( co2Sensor.comp_t());
     float fhumidity = dataLogggerFile.parseFloat(); //( co2Sensor.comp_h());
     int fco2eq = dataLogggerFile.parseInt(); // co2Sensor.co2_eq());
     float fvoceq = dataLogggerFile.parseFloat(); // co2Sensor.b_voc_eq());
     int fiaqs = dataLogggerFile.parseInt(); //( co2Sensor.iaq_s());

     int niaq = map(fiaq, 0, 500, 0, h1);
     int ntemp = map(ftemp, -30, 50, 0, h1);
     int nhumidity = map(fhumidity, 0, 100, 0, h1);
     if (count< w1-2) {
        tft.drawFastVLine(o1x+count+1, o1y+ h1 - niaq -1 , niaq -2, GREEN);
        tft.drawFastVLine(o2x+count+1, o2y+ h1 - ntemp -1 , ntemp -2, YELLOW);
        tft.drawFastVLine(o3x+count+1, o3y+ h1 - nhumidity -1 , nhumidity -2, BLUE);
    } else {
        dataLogggerFile.close();
        break;
    }
     ++count;


    }
    // close the file:
    dataLogggerFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }


}

void updateGraphDisplay() {



}

void updateSensorData() {
#if DEBUG
  Serial.println(co2Sensor.toString());
#endif          
  niclaTemperature = co2Sensor.comp_t();
  iaq = co2Sensor.iaq();
  accuracy =  co2Sensor.accuracy();
  niclaHumidity = co2Sensor.comp_h();
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


  rtc.setEpoch(ntpClient.getEpochTime());
  Serial.println(ntpClient.getEpochTime() );

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

void initDisplay(Layouts layout) {
  Serial.println(F("Display Initialized"));
  tft.fillScreen(ST77XX_BLACK);
  switch (layout) {
    case IAQDISPLAY : initIAQDisplay(); break;
    case VALUESDISPLAY : initValuesDisplay(); break;
    case GRAPHDISPLAY : initGraphDisplay(); break;
    default: initIAQDisplay();
  }    

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

void displayLevel(const int level){
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
    drawGauge(level, 10, 600);
    tft.setFont(NULL);
    iaqdisplay.display(level);
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
    sprintf(buf, "%.2d:%.2d", rtc.getHours() % 24, rtc.getMinutes());
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

void showLogs() {
  File root;
  root = SD.open("/");

  printDirectory(root, 0);
  root.close();
}
void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
