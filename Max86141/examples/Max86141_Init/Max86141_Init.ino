
/*
 * Intensity LED  : 0 = 0mA | 255 = 124mA (max)
 * PGG Sample Rate : 0x00 = 24.995 samples per second | 0x13 = 4096 samples per second (max)
 * Sample Average : 2, 4, 8, 16, 32, 64, 128 samples (max)
 * Led Sequence Control : LED1 (1) // IR LED, LED2 (2) //red LED, LED1 and LED2 pulsed simultaneously (4), LED2 and LED3 pulsed simultaneously (5),
 * LED1 and LED3 pulsed simultaneously (5), LED2 and LED3 pulsed simultaneously (6), LED1, LED2, and LED3 pulsed simultaneously (7),
 * Pilot on LED1 (8), DIRECT AMBIENT (9), LED4 [external mux control] (10), LED5 [external mux control] (11), LED6 [external mux control] (12)
 * DIRECT AMBIENT (i.e. normal photodiode measurements)
 */

#include <SPI.h>
#include "MAX86141.h"

//PD: PhotoDiode
#define PDsLED; // 2 PD - 1 LED
//#define PDLEDs; // 1 PD - 2 LEDs
//#define PDsLEDs; // 2 PDs - 3 LEDs


static int spiClk = 1000000; // 8 MHz Maximum

int fifo_size = 0;

bool RED_ON = false;
bool IR_ON = false;
bool AMBIENT = true;
float RED_AVG, IR_AVG, AMBIENT_AVG;

//
// Pin Definitions.
//*6
// #define MISO_PIN              19
// #define MOSI_PIN              18
// #define SCK_PIN               5
#define SS_PIN                19
//  #define SS_PIN                5

MAX86141 pulseOx1;


void setup() {
  Serial.begin(115200);
  while ( !Serial ) delay(10);

  // Configure IO.
  pinMode(SS_PIN, OUTPUT);

  digitalWrite(SS_PIN, HIGH);


  //initialise SPI
  pulseOx1.spi = new SPIClass(SPI);

  pulseOx1.SS = SS_PIN;

  Serial.println("Init Device");
  pulseOx1.spi->begin();
  delay(100);

  pulseOx1.setDebug(true);
  #ifdef PDsLED
    int LedMode[]= {1/*LED1*/, 9/*DIRECT AMBIENT*/};
    pulseOx1.initialisation(2/*nb_ppg*/,LedMode/*LedMode*/,2/*Number of sequences*/,0x10/*intensity_LEDs*/,4/*sample_average*/, 0x0B/*sample_rate*/,0x3/*pulse width*/,0x2/*ADC Range= 16uA*/,spiClk);
  #endif

  #ifdef PDLEDs
   int LedMode[]={4/*LED1 and LED2 simultaneously*/, 9/*DIRECT AMBIENT*/};
   pulseOx1.initialisation(1/*nb_ppg*/,LedMode/*LedMode*/,2/*Number of sequences*/,0x7/*intensity_LEDs*/,1/*sample_average*/, 0x0E/*sample_rate*/,0x3/*pulse width*/,0x3/*ADC Range= 16uA*/,spiClk);
  #endif

  #ifdef PDsLEDs
  int LedMode[]={1/*LED1*/, 2/*LED2*/,5/*LED2 and LED3 simultaneously*/, 9/*DIRECT AMBIENT*/};
  pulseOx1.initialisation(2/*nb_ppg*/,LedMode/*LedMode*/,4/*Number of sequences*/,0x4/*intensity_LEDs*/,8/*sample_average*/, 0x0E/*sample_rate*/,3/*pulse width*/,0x3/*ADC Range= 16uA*/,spiClk);
  #endif
  
  Serial.println("--Read Register-- System Control");
  Serial.println(pulseOx1.read_reg(0x0D));

  Serial.println("--Read Register-- PART_ID");
  Serial.println(pulseOx1.read_reg(0xFF));

  Serial.println("--Read Temp-- 0x40");
  pulseOx1.write_reg(0x40, 0xFF);

  Serial.println(pulseOx1.read_reg(0x41));
  Serial.println(pulseOx1.read_reg(0x41));
  delay(1000);

  pulseOx1.setDebug(false);
}


void loop() {
  fifo_size = pulseOx1.read_reg(REG_FIFO_DATA_COUNT);
  if(fifo_size >= 6){
    
   
  #ifdef PDsLED
  uint8_t dataBuf[3072]; ///128 FIFO samples, 2 channels PPG, 3 bytes/channel, 2 number of sequences control (128*3*2*4)*2 = 3072 byte buffer
  pulseOx1.device_data_read(dataBuf);
    Serial.println("LED 1 - PPG1 channel : ");
    int led1 = pulseOx1.getLED1()[0];
    Serial.println(led1);
    Serial.println("TAG of LED 1 - PPG1 channel : ");
    int tag1 = pulseOx1.getTag1()[0];
    Serial.println(tag1);
    Serial.println("LED 1 - PPG2 channel : ");
    int led2 = pulseOx1.getLED2()[0];
    Serial.println(led2);
    Serial.println("TAG of LED 1 - PPG2  channel : ");
    int tag2 = pulseOx1.getTag2()[0];
    Serial.println(tag2);
    Serial.println("Direct ambient - PPG1 channel : ");
    int led3 = pulseOx1.getLED3()[0];
    Serial.println(led3);
    Serial.println("TAG Direct ambient - PPG1 channel : ");
    int tag3 = pulseOx1.getTag3()[0];
    Serial.println(tag3);
    Serial.println("Direct ambient - PPG2 channel : ");
    int led4 = pulseOx1.getLED4()[0];
    Serial.println(led4);
    Serial.println("TAG Direct ambient - PPG2 channel : ");
    int tag4 = pulseOx1.getTag4()[0];
    Serial.println(tag4);

    Serial.println("LED 1 AVERAGE");
    int led_avg = (led1+led2)*0.5;
    Serial.println(led_avg);
    Serial.println("Direct ambient AVERAGE");
    int ambient_avg = (led3+led4)*0.5;
    Serial.println(ambient_avg);
    
  #endif

  #ifdef PDLEDs
  uint8_t dataBuf[3072]; ///128 FIFO samples, 2 channels PPG, 3 bytes/channel, 2 number of sequences control (128*3*2*4)*2 = 3072 byte buffer
  pulseOx1.device_data_read(dataBuf);
    Serial.println("LED 1 & LED 2 - PPG1 channel : ");
    int led1 = pulseOx1.getLED1()[0];
    Serial.println(led1);
    Serial.println("TAG of LED 1 & LED 2 - PPG1 channel : ");
    int tag1 = pulseOx1.getTag1()[0];
    Serial.println(tag1);
    Serial.println("Direct ambient - PPG1 channel : ");
    int led2 = pulseOx1.getLED2()[0];
    Serial.println(led2);
    Serial.println("TAG Direct ambient - PPG1 channel : ");
    int tag2 = pulseOx1.getTag2()[0];
    Serial.println(tag3);
   
  #endif
  
  #ifdef PDsLEDs
  uint8_t dataBuf[6144]; ///128 FIFO samples, 2 channels PPG, 3 bytes/channel, 4 number of sequences control (128*3*2*4)*4 = 6144 byte buffer
  pulseOx1.device_data_read(dataBuf);   
    Serial.println("LED 1 - PPG1 channel : ");
    int led1 = pulseOx1.getLED1()[0];
    Serial.println(led1);
    Serial.println("TAG 1 - PPG1 channel : ");
    int tag1 = pulseOx1.getTag1()[0];
    Serial.println(tag1);
    Serial.println("LED 1 - PPG2 channel : ");
    int led2 = pulseOx1.getLED2()[0];
    Serial.println(led2);
    Serial.println("TAG 2 - PPG2  channel : ");
    int tag2 = pulseOx1.getTag2()[0];
    Serial.println(tag2);
    
    Serial.println("LED 2 - PPG1 channel : ");
    int led3 = pulseOx1.getLED3()[0];
    Serial.println(led3);
    Serial.println("TAG of LED 2 - PPG1 channel : ");
    int tag3 = pulseOx1.getTag3()[0];
    Serial.println(tag3);
    Serial.println("LED 2 - PPG2 channel : ");
    int led4 = pulseOx1.getLED4()[0];
    Serial.println(led4);
    Serial.println("TAG of LED 2 - PPG2  channel : ");
    int tag4 = pulseOx1.getTag4()[0];
    Serial.println(tag4);
    
    Serial.println("LED 2 & LED 3 - PPG1 channel : ");
    int led5 = pulseOx1.getLED5()[0];
    Serial.println(led5);
    Serial.println("TAG of LED 2 & LED 3 - PPG1 channel : ");
    int tag5 = pulseOx1.getTag5()[0];
    Serial.println(tag5);
    Serial.println("LED 2 & LED 3 - PPG2 channel : ");
    int led6 = pulseOx1.getLED6()[0];
    Serial.println(led6);
    Serial.println("TAG of LED 2 & LED 3 - PPG2  channel : ");
    int tag6 = pulseOx1.getTag6()[0];
    Serial.println(tag6);
    Serial.println("Direct ambient - PPG1 channel : ");
    int led7 = pulseOx1.getLED7()[0];
    Serial.println(led7);
    Serial.println("TAG Direct ambient - PPG1 channel : ");
    int tag7 = pulseOx1.getTag7()[0];
    Serial.println(tag7);
    Serial.println("Direct ambient - PPG2 channel : ");
    int led8 = pulseOx1.getLED8()[0];
    Serial.println(led8);
    Serial.println("TAG Direct ambient - PPG2 channel : ");
    int tag8 = pulseOx1.getTag8()[0];
    Serial.println(tag4);

    Serial.println("LED 1 AVERAGE");
    int led1_avg = (led1+led2)*0.5;
    Serial.println(led1_avg);
    Serial.println("LED 2 AVERAGE");
    int led2_avg = (led3+led4)*0.5;
    Serial.println(led2_avg);
    Serial.println("LED 2 & LED 3 AVERAGE");
    int led_avg = (led5+led6)*0.5;
    Serial.println(led_avg);
    Serial.println("Direct ambient AVERAGE");
    int ambient_avg = (led7+led8)*0.5;
    Serial.println(ambient_avg);
  #endif

    delayMicroseconds(2500);
  }
}
