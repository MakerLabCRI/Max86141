/*
 * Intensity LED  : 0 = 0mA | 255 = 124mA (max)
 * PGG Sample Rate : 0x00 = 24.995 samples per second | 0x13 = 4096 samples per second (max)
 * Sample Average : 2, 4, 8, 16, 32, 64, 128 samples (max)
 * Led Sequence Control : LED1 (1) // IR LED, LED2 (2) //red LED, LED1 and LED2 pulsed simultaneously (4), LED2 and LED3 pulsed simultaneously (5),
 * LED1 and LED3 pulsed simultaneously (5), LED2 and LED3 pulsed simultaneously (6), LED1, LED2, and LED3 pulsed simultaneously (7),
 * Pilot on LED1 (8), DIRECT AMBIENT (9), LED4 [external mux control] (10), LED5 [external mux control] (11), LED6 [external mux control] (12)
 * DIRECT AMBIENT (i.e. normal photodiode measurements)
 * Sequence Control page 14-15 datasheet
 */

#include "MAX86141.h"
#include <SPI.h>

//PD: PhotoDiode
#define PDsLED; // 2 PDs - 1 LED
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

long startTime;
long samplesTaken = 0; //Counter for calculating the Hz or read rate

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
    int LedMode[]= {1/*LED1 (Sequence 1, 0-3)*/, 9/*DIRECT AMBIENT (Sequence 1, 4-7)*/};
 pulseOx1.initialisation(2/*nb_pds*/,LedMode/*LedMode*/,2/*Number of sequences*/,1/*Number of LEDs*/,10/*intensity_LEDs*/,8/*sample_average*/, 0x0E/*sample_rate*/,0x3/*pulse width*/,0x2/*ADC Range= 16uA*/,spiClk);
  #endif

  #ifdef PDLEDs
   int LedMode[]={4/*LED1 and LED2 simultaneously (Sequence 1, 0-3)*/, 9/*DIRECT AMBIENT (Sequence 1, 4-7)*/};
   pulseOx1.initialisation(1/*nb_ppg*/,LedMode/*LedMode*/,2/*Number of sequences*/,2/*Number of LEDs*/,15/*intensity_LEDs*/,1/*sample_average*/, 0x0E/*sample_rate*/,0x3/*pulse width*/,0x3/*ADC Range= 16uA*/,spiClk);
  #endif

  #ifdef PDsLEDs
  int LedMode[]={1/*LED1 (Sequence 1, 0-3)*/, 2/*LED2 (Sequence 1, 4-7)*/,5/*LED2 and LED3 simultaneously (Sequence 2, 0-3)*/, 9/*DIRECT AMBIENT (Sequence 2, 4-7)*/};
  pulseOx1.initialisation(2/*nb_ppg*/,LedMode/*LedMode*/,4/*Number of sequences*/,3/*Number of LEDs*/,0x4/*intensity_LEDs*/,8/*sample_average*/, 0x0E/*sample_rate*/,3/*pulse width*/,0x3/*ADC Range= 16uA*/,spiClk);
  #endif
  
  Serial.println("--Read Register-- System Control");
  Serial.println(pulseOx1.read_reg(0x0D));

  Serial.println("--Read Register-- PART_ID");
  Serial.println(pulseOx1.read_reg(0xFF));

  Serial.println("--Read Temp-- 0x40");
  pulseOx1.write_reg(0x40, 0xFF);

  Serial.println(pulseOx1.read_reg(0x41));
  //Serial.println(pulseOx1.read_reg(0x41));
  delay(1000);

  pulseOx1.setDebug(false);
  
  // Set signal bufer size
  pulseOx1.signalData_ledSeq1A_PPG1.setStorage(pulseOx1.storage_array);
}


void loop() {
    uint8_t intStatus;
    //Read Status
    intStatus = pulseOx1.read_reg(REG_INT_STAT_1);
      
if ( intStatus & 0x80 ) { //A FULL RDY
  fifo_size = pulseOx1.read_reg(REG_FIFO_DATA_COUNT);
  //Serial.println("fifo-size :"+String(fifo_size));
  
  #ifdef PDsLED
  uint8_t dataBuf[3072]={0}; ///128 FIFO samples, 2 channels PPG, 3 bytes/channel, 2 number of sequences control (128*3*2*4)*2 = 3072 byte buffer
  pulseOx1.device_data_read(dataBuf,fifo_size);
    Serial.println("LED 1 Seq 1 A - PPG1 channel : ");
    int ledSeq1A_PPG1 = pulseOx1.get_ledSeq1A_PPG1()[0];
    Serial.println(ledSeq1A_PPG1);
    Serial.println("TAG of LED 1 Seq 1 A - PPG1 channel : ");
    int tagSeq1A_PPG1 = pulseOx1.get_tagSeq1A_PPG1()[0];
    Serial.println(tagSeq1A_PPG1);
    Serial.println("LED 1 Seq 1 A - PPG2 channel : ");
    int ledSeq1A_PPG2 = pulseOx1.get_ledSeq1A_PPG2()[0];
    Serial.println(ledSeq1A_PPG2);
    Serial.println("TAG of LED 1 Seq 1 A - PPG2  channel : ");
    int tagSeq1A_PPG2 = pulseOx1.get_tagSeq1A_PPG2()[0];
    Serial.println(tagSeq1A_PPG2);
    Serial.println("Direct ambient - PPG1 channel : ");
    int ledSeq1B_PPG1 = pulseOx1.get_ledSeq1B_PPG1()[0];
    Serial.println(ledSeq1B_PPG1);
    Serial.println("TAG Direct ambient - PPG1 channel : ");
    int tagSeq1B_PPG1 = pulseOx1.get_tagSeq1B_PPG1()[0];
    Serial.println(tagSeq1B_PPG1);
    Serial.println("Direct ambient - PPG2 channel : ");
    int ledSeq1B_PPG2 = pulseOx1.get_ledSeq1B_PPG2()[0];
    Serial.println(ledSeq1B_PPG2);
    Serial.println("TAG Direct ambient - PPG2 channel : ");
    int tagSeq1B_PPG2 = pulseOx1.get_tagSeq1B_PPG2()[0];
    Serial.println(tagSeq1B_PPG2);

    Serial.println("LED AVERAGE");
    int led_avg = (ledSeq1A_PPG1+ledSeq1A_PPG2)*0.5;
    Serial.println(led_avg);
    Serial.println("Direct ambient AVERAGE");
    int ambient_avg = (ledSeq1B_PPG1+ledSeq1B_PPG2)*0.5;
    Serial.println(ambient_avg);

  #endif

  #ifdef PDLEDs
    uint8_t dataBuf[3072]={ 0 }; ///128 FIFO samples, 2 channels PPG, 3 bytes/channel, 2 number of sequences control (128*3*2*4)*2 = 3072 byte buffer
    pulseOx1.device_data_read(dataBuf,fifo_size);
   
    Serial.println("LED 1 & LED 2 Seq 1 A - PPG1 channel : ");
    int ledSeq1A_PPG1 = pulseOx1.get_ledSeq1A_PPG1()[0];
    Serial.println(ledSeq1A_PPG1);
    Serial.println("TAG of LED 1 & LED 2 Seq 1 A - PPG1 channel : ");
    int tagSeq1A_PPG1 = pulseOx1.get_tagSeq1A_PPG1()[0];
    Serial.println(tagSeq1A_PPG1);
    Serial.println("Direct ambient - PPG1 channel : ");
    int ledSeq1B_PPG1 = pulseOx1.get_ledSeq1B_PPG1()[0];
    Serial.println(ledSeq1B_PPG1);
    Serial.println("TAG Direct ambient - PPG1 channel : ");
    int tagSeq1B_PPG1 = pulseOx1.get_tagSeq1B_PPG1()[0];
    Serial.println(tagSeq1B_PPG1);

    ///////////// Add in buffer for SNR //////////
    pulseOx1.signalData_ledSeq1A_PPG1.push_back(ledSeq1A_PPG1);
    if(pulseOx1.signalData_ledSeq1A_PPG1.size()==SIZE){
    //Serial.println("SNR (dB): "+String(pulseOx1.signaltonoise(pulseOx1.signalData_ledSeq1A_PPG1, SIZE)));
     Serial.println(pulseOx1.signaltonoise(pulseOx1.signalData_ledSeq1A_PPG1, SIZE));

    pulseOx1.signalData_ledSeq1A_PPG1.clear();
    }
  #endif
  
  #ifdef PDsLEDs
  uint8_t dataBuf[6144]={0}; ///128 FIFO samples, 2 channels PPG, 3 bytes/channel, 4 number of sequences control (128*3*2*4)*4 = 6144 byte buffer
  pulseOx1.device_data_read(dataBuf,fifo_size);   
    Serial.println("LED 1 Seq 1 A - PPG1 channel : ");
    int ledSeq1A_PPG1 = pulseOx1.get_ledSeq1A_PPG1()[0];
    Serial.println(ledSeq1A_PPG1);
    Serial.println("TAG Seq 1 A - PPG1 channel : ");
    int tagSeq1A_PPG1 = pulseOx1.get_tagSeq1A_PPG1()[0];
    Serial.println(tagSeq1A_PPG1);
    Serial.println("LED 1 Seq 1 A - PPG2 channel : ");
    int ledSeq1A_PPG2 = pulseOx1.get_ledSeq1A_PPG2()[0];
    Serial.println(ledSeq1A_PPG2);
    Serial.println("TAG Seq 1 A - PPG2  channel : ");
    int tagSeq1A_PPG2 = pulseOx1.get_tagSeq1A_PPG2()[0];
    Serial.println(tagSeq1A_PPG2);
    
    Serial.println("LED 2 Seq 1 B - PPG1 channel : ");
    int ledSeq1B_PPG1 = pulseOx1.get_ledSeq1B_PPG1()[0];
    Serial.println(ledSeq1B_PPG1);
    Serial.println("TAG of LED 2 Seq 1 B - PPG1 channel : ");
    int tagSeq1B_PPG1 = pulseOx1.get_tagSeq1B_PPG1()[0];
    Serial.println(tagSeq1B_PPG1);
    Serial.println("LED 2 Seq 1 B - PPG2 channel : ");
    int ledSeq1B_PPG2 = pulseOx1.get_ledSeq1B_PPG2()[0];
    Serial.println(ledSeq1B_PPG2);
    Serial.println("TAG of LED 2 Seq 1 B - PPG2  channel : ");
    int tagSeq1B_PPG2 = pulseOx1.get_tagSeq1A_PPG2()[0];
    Serial.println(tagSeq1A_PPG2);
    
    Serial.println("LED 2 & LED 3 Seq 2 A - PPG1 channel : ");
    int ledSeq2A_PPG1 = pulseOx1.get_ledSeq2A_PPG1()[0];
    Serial.println(ledSeq2A_PPG1);
    Serial.println("TAG of LED 2 & LED 3 Seq 2 A - PPG1 channel : ");
    int tagSeq2A_PPG1 = pulseOx1.get_tagSeq2A_PPG1()[0];
    Serial.println(tagSeq2A_PPG1);
    Serial.println("LED 2 & LED 3 Seq 2 A - PPG2 channel : ");
    int ledSeq2A_PPG2 = pulseOx1.get_ledSeq2A_PPG2()[0];
    Serial.println(ledSeq2A_PPG2);
    Serial.println("TAG of LED 2 & LED 3 Seq 2 A - PPG2  channel : ");
    int tagSeq2A_PPG2 = pulseOx1.get_tagSeq2A_PPG2()[0];
    Serial.println(tagSeq2A_PPG2);
    
    Serial.println("Direct ambient Seq 2 B - PPG1 channel : ");
    int ledSeq2B_PPG1 = pulseOx1.get_ledSeq2B_PPG1()[0];
    Serial.println(ledSeq2B_PPG1);
    Serial.println("TAG Direct ambient Seq 2 B - PPG1 channel : ");
    int tagSeq2B_PPG1 = pulseOx1.get_tagSeq2B_PPG1()[0];
    Serial.println(tagSeq2B_PPG1);
    Serial.println("Direct ambient Seq 2 B - PPG2 channel : ");
    int ledSeq2B_PPG2 = pulseOx1.get_ledSeq2B_PPG2()[0];
    Serial.println(ledSeq2B_PPG2);
    Serial.println("TAG Direct ambient Seq 2 B - PPG2 channel : ");
    int tagSeq2B_PPG2 = pulseOx1.get_tagSeq2B_PPG2()[0];
    Serial.println(tagSeq2B_PPG2);

    Serial.println("LED 1 AVERAGE");
    int led1_avg = (ledSeq1A_PPG1+ledSeq1A_PPG2)*0.5;
    Serial.println(led1_avg);
    Serial.println("LED 2 AVERAGE");
    int led2_avg = (ledSeq1B_PPG1+ledSeq1B_PPG2)*0.5;
    Serial.println(led2_avg);
    Serial.println("LED 2 & LED 3 AVERAGE");
    int led_avg = (ledSeq2A_PPG1+ledSeq2A_PPG2)*0.5;
    Serial.println(led_avg);
    Serial.println("Direct ambient AVERAGE");
    int ambient_avg = (ledSeq2B_PPG1+ledSeq2B_PPG2)*0.5;
    Serial.println(ambient_avg);
    
    //delayMicroseconds(2500);
  #endif
}
else {
  //Serial.println("FIFO not A_full");
}
  }