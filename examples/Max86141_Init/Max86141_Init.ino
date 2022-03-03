
/*
 * Intensity LED  : 0 = 0mA | 255 = 124mA (max). Be careful, data from PD can saturate at 524287 if Intensity LED is too high
 * PGG Sample Rate : 0x00 = 24.995 samples per second | 0x13 = 4096 samples per second (max)
 * Sample Average : 2, 4, 8, 16, 32, 64, 128 samples (max)
 * Led Sequence Control : LED1 (1), LED2 (2), LED3 (3), LED1 and LED2 pulsed simultaneously (4),
   LED1 and LED3 pulsed simultaneously (5), LED2 and LED3 pulsed simultaneously (6), LED1, LED2 and LED3 pulsed simultaneously (7),
   Pilot on LED1 (8), DIRECT AMBIENT (9), LED4 [external mux control] (10), LED5 [external mux control] (11), LED6 [external mux control] (12)
 * DIRECT AMBIENT : DA (i.e. normal photodiode measurements)
 * Sequence Control is up to the configuration you wish (page 14-15 datasheet)
 * PD: PhotoDiode
 * 1 LED is RGB or just 1 color  
*/

#include <MAX86141.h>
#include <SPI.h>

/* Sensor Configurations */
// Sensor composed with 2 PDs and 1 LED //
#define PDsLED

// Sensor composed with 1 PD and 2 LEDs //
//#define PDLEDs

// Sensor composed with 2 PDs and 3 LEDs //
//#define PDsLEDs


/* Inculde LED configuration */
int ledMode[10];

#include "Configuration_Sensor.h"

/* Sample Rate taken */
#define Sample_Rate


/* Pin Definitions */
//#define MISO_PIN              19
//#define MOSI_PIN              18
//#define SCK_PIN               5
//#define SS_PIN                19 //(Adafruit)
#define SS_PIN                10 //(Movuino)

/* Global variables */
MAX86141 pulseOx1;
static int spiClk = 1000000; // 8 MHz Maximum
int cpt1 = 0, cpt2 = 0;
long startTime;
long samplesTaken = 0;
int sequences_size = 0;

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
  sequences_size = config(rgbLED_G /*Green LED selected (Sequence 1A, 0-3)*/ | rgbLED_IR /*IR LED (Sequence 1B, 4-9)*/ | rgbLED_R/*RED LED (Sequence 2A, 0-3)*/ | DA /*Direct Ambient (Sequence 2B, 4-9)*/);
  pulseOx1.initialisation(2/*nb_pds*/, ledMode/*LedMode*/, sequences_size/*Number of sequences*/, 10/*intensity_LEDs*/, 0x00/*sample_average*/, 0xE/*sample_rate*/, 0x3/*pulse width*/, 0x2/*ADC Range= 16uA*/, spiClk);
#endif

#ifdef PDLEDs
  sequences_size = config(LED_G/*Green LED (Sequence 1A, 0-3)*/ | LED_G/*Green LED (Sequence 1B, 4-9)*/ | DA /*Direct Ambient (Sequence 22A, 0-3)*/);
  pulseOx1.initialisation(1/*nb_pds*/, ledMode/*LedMode*/, sequences_size/*Number of sequences*/, 10/*intensity_LEDs*/, 1/*sample_average*/, 0x0E/*sample_rate*/, 0x3/*pulse width*/, 0x3/*ADC Range= 16uA*/, spiClk);
#endif

#ifdef PDsLEDs
  sequences_size = config(rgbLED_G_R_IR /*Green LED, RED LED and IR LED selected simultaneously (Sequence 1A, 0-3)*/ | DA /*Direct Ambient (Sequence 1B, 4-9)*/);
  pulseOx1.initialisation(2/*nb_ppg*/, ledMode/*LedMode*/, sequences_size/*Number of sequences*/, 10/*intensity_LEDs*/, 0x3/*sample_average*/, 0x0E/*sample_rate*/, 0x3/*pulse width*/, 0x2/*ADC Range= 16uA*/, spiClk);
#endif

  Serial.println("--Read Register-- System Control");
  Serial.println(pulseOx1.read_reg(0x0D));

  Serial.println("--Read Register-- PART_ID");
  int part_ID = pulseOx1.read_reg(0xFF);
  Serial.println(part_ID);

  Serial.println("--Read Temp-- 0x40");
  pulseOx1.write_reg(0x40, 0xFF);

  Serial.println(pulseOx1.read_reg(0x41));

  if (part_ID == 36) {
    Serial.println("MAX86140 connection succeeded !");
  }
  else if (part_ID == 37) {
    Serial.println("MAX86141 connection succeeded !");
  }
  else {
    Serial.println("Connection failed !");
  }

  delay(1000);

  pulseOx1.setDebug(false);

  startTime = millis();
}


void loop() {

  uint8_t intStatus;
  //Read Status
  intStatus = pulseOx1.read_reg(REG_INT_STAT_1);
  bool flagA_full = (intStatus & 0x80) >> 7;

  /////// if there is 8 samples in the FIFO ///////
  if (flagA_full) {
    int fifo_size = pulseOx1.device_data_read1();

#ifdef PDsLED
    samplesTaken += 1;

    Serial.println("##########################################");
    Serial.println("LEDC1 Seq 1 A - PD1 channel : ");
    int ledSeq1A_PD1 = pulseOx1.get_ledSeq1A_PD1();
    Serial.println(ledSeq1A_PD1);
    Serial.println("TAG of LEDC1 Seq 1 A - PD1 channel : ");
    int tagSeq1A_PD1 = pulseOx1.get_tagSeq1A_PD1();
    Serial.println(tagSeq1A_PD1);
    Serial.println("LEDC1 Seq 1 A - PD2 channel : ");
    int ledSeq1A_PD2 = pulseOx1.get_ledSeq1A_PD2();
    Serial.println(ledSeq1A_PD2);
    Serial.println("TAG of LEDC1 Seq 1 A - PD2  channel : ");
    int tagSeq1A_PD2 = pulseOx1.get_tagSeq1A_PD2();
    Serial.println(tagSeq1A_PD2);

    
    Serial.println("LEDC2 Seq 1 B - PD1 channel : ");
    int ledSeq1B_PD1 = pulseOx1.get_ledSeq1B_PD1();
    Serial.println(ledSeq1B_PD1);
    Serial.println("TAG LEDC2 Seq 1 B - PD1 channel : ");
    int tagSeq1B_PD1 = pulseOx1.get_tagSeq1B_PD1();
    Serial.println(tagSeq1B_PD1);
    Serial.println("LEDC2 Seq 1 B - PD2 channel : ");
    int ledSeq1B_PD2 = pulseOx1.get_ledSeq1B_PD2();
    Serial.println(ledSeq1B_PD2);
    Serial.println("TAG LEDC2 Seq 1 B - PD2 channel : ");
    int tagSeq1B_PD2 = pulseOx1.get_tagSeq1B_PD2();
    Serial.println(tagSeq1B_PD2);

    Serial.println("LEDC3 Seq 2 A - PD1 channel : ");
    int ledSeq2A_PD1 = pulseOx1.get_ledSeq2A_PD1();
    Serial.println(ledSeq2A_PD1);
    Serial.println("TAG LEDC3 Seq 2 A - PD1 channel : ");
    int tagSeq2A_PD1 = pulseOx1.get_tagSeq2A_PD1();
    Serial.println(tagSeq2A_PD1);
    Serial.println("LEDC3 Seq 2 A - PD2 channel : ");
    int ledSeq2A_PD2 = pulseOx1.get_ledSeq2A_PD2();
    Serial.println(ledSeq2A_PD2);
    Serial.println("TAG LEDC3 Seq 2 A - PD2 channel : ");
    int tagSeq2A_PD2 = pulseOx1.get_tagSeq2A_PD2();
    Serial.println(tagSeq2A_PD2);

    Serial.println("LEDC4 Seq 2 B - PD1 channel : ");
    int ledSeq2B_PD1 = pulseOx1.get_ledSeq2B_PD1();
    Serial.println(ledSeq2B_PD1);
    Serial.println("TAG LEDC4 Seq 2 B - PD1 channel : ");
    int tagSeq2B_PD1 = pulseOx1.get_tagSeq2B_PD1();
    Serial.println(tagSeq2B_PD1);
    Serial.println("LEDC4 Seq 2 B - PD2 channel : ");
    int ledSeq2B_PD2 = pulseOx1.get_ledSeq2B_PD2();
    Serial.println(ledSeq2B_PD2);
    Serial.println("TAG LEDC4 Seq 2 B - PD2 channel : ");
    int tagSeq2B_PD2 = pulseOx1.get_tagSeq2B_PD2();
    Serial.println(tagSeq2B_PD2);
    Serial.println("##########################################");

    
    ///////////// Addition data of PD1 in buffer to measure SNR (Signal Noise Ratio) //////////
    pulseOx1.signalData_ledSeq1A_PD1[cpt1] = ledSeq1A_PD1;
    cpt1 += 1;
    if (cpt1 == SIZE) {
      Serial.println("SNR (dB): " + String(pulseOx1.signaltonoise(pulseOx1.signalData_ledSeq1A_PD1, SIZE)));
      cpt1 = 0;
    }

    ///////////// Addition data of PD2 in buffer to measure SNR (Signal Noise Ratio) //////////
    pulseOx1.signalData_ledSeq1A_PD2[cpt2] = ledSeq1A_PD2;
    cpt2 += 1;
    if (cpt2 == SIZE) {
      Serial.println("SNR (dB): " + String(pulseOx1.signaltonoise(pulseOx1.signalData_ledSeq1A_PD2, SIZE)));
      cpt2 = 0;
    }


#ifdef Sample_Rate
    Serial.print("Sample Rate : Hz[");
    Serial.print((float)(samplesTaken) / ((millis() - startTime) / 1000.0), 2);
    Serial.print("]");
    Serial.println();
    Serial.println();
#endif


#endif

#ifdef PDLEDs

    Serial.println("##########################################");
    Serial.println("LEDC1 Seq 1 A - PD1 channel : ");
    int ledSeq1A_PD1 = pulseOx1.get_ledSeq1A_PD1();
    Serial.println(ledSeq1A_PD1);
    Serial.println("TAG of LEDC1 Seq 1 A - PD1 channel : ");
    int tagSeq1A_PD1 = pulseOx1.get_tagSeq1A_PD1();
    Serial.println(tagSeq1A_PD1);
    
    Serial.println("LEDC2 Seq 1 B - PD1 channel : ");
    int ledSeq1B_PD1 = pulseOx1.get_ledSeq1B_PD1();
    Serial.println(ledSeq1B_PD1);
    Serial.println("TAG LEDC2 Seq 1 B - PD1 channel : ");
    int tagSeq1B_PD1 = pulseOx1.get_tagSeq1B_PD1();
    Serial.println(tagSeq1B_PD1);

    Serial.println("LEDC3 Seq 2 A - PD1 channel : ");
    int ledSeq2A_PD1 = pulseOx1.get_ledSeq2A_PD1();
    Serial.println(ledSeq2A_PD1);
    Serial.println("TAG LEDC3 Seq 2 A - PD1 channel : ");
    int tagSeq2A_PD1 = pulseOx1.get_tagSeq2A_PD1();
    Serial.println(tagSeq2A_PD1);
    Serial.println("##########################################");

#endif

#ifdef PDsLEDs

    Serial.println("##########################################");
    Serial.println("LEDC1 Seq 1 A - PD1 channel : ");
    int ledSeq1A_PD1 = pulseOx1.get_ledSeq1A_PD1();
    Serial.println(ledSeq1A_PD1);
    Serial.println("TAG of LEDC1 Seq 1 A - PD1 channel : ");
    int tagSeq1A_PD1 = pulseOx1.get_tagSeq1A_PD1();
    Serial.println(tagSeq1A_PD1);
    Serial.println("LEDC1 Seq 1 A - PD2 channel : ");
    int ledSeq1A_PD2 = pulseOx1.get_ledSeq1A_PD2();
    Serial.println(ledSeq1A_PD2);
    Serial.println("TAG of LEDC1 Seq 1 A - PD2  channel : ");
    int tagSeq1A_PD2 = pulseOx1.get_tagSeq1A_PD2();
    Serial.println(tagSeq1A_PD2);

    
    Serial.println("LEDC2 Seq 1 B - PD1 channel : ");
    int ledSeq1B_PD1 = pulseOx1.get_ledSeq1B_PD1();
    Serial.println(ledSeq1B_PD1);
    Serial.println("TAG LEDC2 Seq 1 B - PD1 channel : ");
    int tagSeq1B_PD1 = pulseOx1.get_tagSeq1B_PD1();
    Serial.println(tagSeq1B_PD1);
    Serial.println("LEDC2 Seq 1 B - PD2 channel : ");
    int ledSeq1B_PD2 = pulseOx1.get_ledSeq1B_PD2();
    Serial.println(ledSeq1B_PD2);
    Serial.println("TAG LEDC2 Seq 1 B - PD2 channel : ");
    int tagSeq1B_PD2 = pulseOx1.get_tagSeq1B_PD2();
    Serial.println(tagSeq1B_PD2);

    Serial.println("LEDC3 Seq 2 A - PD1 channel : ");
    int ledSeq2A_PD1 = pulseOx1.get_ledSeq2A_PD1();
    Serial.println(ledSeq2A_PD1);
    Serial.println("TAG LEDC3 Seq 2 A - PD1 channel : ");
    int tagSeq2A_PD1 = pulseOx1.get_tagSeq2A_PD1();
    Serial.println(tagSeq2A_PD1);
    Serial.println("LEDC3 Seq 2 A - PD2 channel : ");
    int ledSeq2A_PD2 = pulseOx1.get_ledSeq2A_PD2();
    Serial.println(ledSeq2A_PD2);
    Serial.println("TAG LEDC3 Seq 2 A - PD2 channel : ");
    int tagSeq2A_PD2 = pulseOx1.get_tagSeq2A_PD2();
    Serial.println(tagSeq2A_PD2);
    Serial.println("##########################################");

#endif
  }
  else {
    //Serial.println("FIFO not A_full");
  }
}
