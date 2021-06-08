#include "MAX86141.h"

/*write to register function*/
void MAX86141::write_reg(uint8_t address, uint8_t data_in) {

    /**< A buffer with data to transfer. */
    m_tx_buf[0] = address;  //Target Register
    m_tx_buf[1] = WRITE_EN; //Set Write mode
    m_tx_buf[2] = data_in;  //Byte to Write

    if(debug == true) {
        Serial.print("W_TX ");
        Serial.print(m_tx_buf[0], HEX);
        Serial.print("|");
        Serial.print(m_tx_buf[1], HEX);
        Serial.print("|");
        Serial.println(m_tx_buf[2], HEX);
    }

    /**< A buffer for incoming data. */

    m_rx_buf[0] = 0;
    m_rx_buf[1] = 0;
    m_rx_buf[2] = 0;

    // digitalWrite(SS, HIGH);
    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(SS, LOW);

    m_rx_buf[0] = spi->transfer(address);
    m_rx_buf[1] = spi->transfer(WRITE_EN);
    m_rx_buf[2] = spi->transfer(data_in);

    digitalWrite(SS, HIGH);
    spi->endTransaction();

    if(debug == true) {
        Serial.print("W_RX ");
        Serial.print(m_rx_buf[0], HEX);
        Serial.print("|");
        Serial.print(m_rx_buf[1], HEX);
        Serial.print("|");
        Serial.println(m_rx_buf[2], HEX);
    }
}

/*read register function*/
uint8_t MAX86141::read_reg(uint8_t address) {

    /**< A buffer with data to transfer. */
    m_tx_buf[0] = address;  //Target Address
    m_tx_buf[1] = READ_EN;  //Set Read mode
    m_tx_buf[2] = 0x00;     //

    /**< A buffer for incoming data. */
    m_rx_buf[0] = 0;
    m_rx_buf[1] = 0;
    m_rx_buf[2] = 0;

    // if(debug == true) {
    //     Serial.print("R_TX ");
    //     Serial.print(m_tx_buf[0], HEX);
    //     Serial.print("|");
    //     Serial.print(m_tx_buf[1], HEX);
    //     Serial.print("|");
    //     Serial.println(m_tx_buf[2], HEX);
    // }

    // digitalWrite(SS, HIGH);
    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(SS, LOW);

    spi->transfer(m_tx_buf,3);

    digitalWrite(SS, HIGH);

    spi->endTransaction();

    m_rx_buf[0] = m_tx_buf[0];
    m_rx_buf[1] = m_tx_buf[1];
    m_rx_buf[2] = m_tx_buf[2];

    if(debug == true){
        Serial.print("R_RX ");
        Serial.print(m_rx_buf[0], HEX);
        Serial.print("|");
        Serial.print(m_rx_buf[1], HEX);
        Serial.print("|");
        Serial.println(m_rx_buf[2], HEX);
    }

    return m_rx_buf[2];
}

//////////////// GETs ////////////////////
    int MAX86141::getledModeSize(){
      return ledModeSize;
     }

    int MAX86141::getNbPPG(){
      return nb_ppg;
    }
    int* MAX86141::getLedMode(){
      return ledMode;
    }
    int MAX86141::getIntensityLed(){
      return intensity_led;
    }
    int MAX86141::getSample_average(){
      return sample_average;
    }
    int MAX86141::getSample_rate(){
      return sample_rate;
    }
    int MAX86141::getPulse_width(){
      return pulse_width;
    }
    int MAX86141::getADCRange(){
      return adcRange;
    }
    int* MAX86141::getLED1(){
      return led1;
    }
    int* MAX86141::getLED2(){
      return led2;
    }
    int* MAX86141::getLED3(){
      return led3;
    }
    int* MAX86141::getLED4(){
      return led4;
    }
    int* MAX86141::getLED5(){
      return led5;
    }
    int* MAX86141::getLED6(){
      return led6;
    }
    int* MAX86141::getLED7(){
      return led7;
    }
    int* MAX86141::getLED8(){
      return led8;
    }
    int* MAX86141::getTag1(){
      return tag1;
    }
    int* MAX86141::getTag2(){
      return tag2;
    }
    int* MAX86141::getTag3(){
      return tag3;
    }
    int* MAX86141::getTag4(){
      return tag4;
    }
    int* MAX86141::getTag5(){
      return tag5;
    }
    int* MAX86141::getTag6(){
      return tag6;
    }
    int* MAX86141::getTag7(){
      return tag7;
    }
    int* MAX86141::getTag8(){
      return tag8;
    }
////////////////////////////////////////

////////////// SETs ///////////////////////////
    void MAX86141::setNumbPPG(int ppg){
      nb_ppg= ppg;
      if(nb_ppg==1){
          write_reg(REG_MODE_CONFIG, 0xA);
      }
      else
          write_reg(REG_MODE_CONFIG, 0x2);

    }
    void MAX86141::setLedModeSize( int size_led){
      ledModeSize= size_led;
    }
    void MAX86141::setLedMode(int *ledMd){
      ledMode= ledMd;
      if(ledModeSize==1){
          Serial.print("1 LED CONTROL SEQ1");
          Serial.println(ledMode[0],BIN);
          write_reg(REG_LED_SEQ_1, ledMode[0]);
      }
      else if(ledModeSize==2){
        ledMode[1]= ledMode[1]<<4; 
        int SEQ1= ledMode[1]+ ledMode[0];
        Serial.print("2 LED CONTROL SEQ1");
          Serial.println(SEQ1,BIN);
        write_reg(REG_LED_SEQ_1, SEQ1);
      }
      else if(ledModeSize==3){
        ledMode[1]= ledMode[1]<<4; 
        int SEQ1= ledMode[1]+ ledMode[0];
        Serial.println("3 LED CONTROL ");
        write_reg(REG_LED_SEQ_1, SEQ1);
        write_reg(REG_LED_SEQ_2, ledMode[2]);

      }
      else if(ledModeSize==4){
          Serial.print("4 LED CONTROL ");

        ledMode[1]= ledMode[1]<<4; 
        int SEQ1= ledMode[1]+ ledMode[0];
                  Serial.println(SEQ1,BIN);
        write_reg(REG_LED_SEQ_1, SEQ1);
        ledMode[3]= ledMode[3]<<4;
        int SEQ2= ledMode[3]+ ledMode[2];
                  Serial.println(SEQ2,BIN);
        write_reg(REG_LED_SEQ_2, SEQ2);
      }
      else if(ledModeSize==5){
         Serial.print("5 LED CONTROL ");
          
        ledMode[1]= ledMode[1]<<4;
        int SEQ1= ledMode[1]+ ledMode[0];
        write_reg(REG_LED_SEQ_1, SEQ1);
        ledMode[3]= ledMode[3]<<4;
        int SEQ2= ledMode[3]+ ledMode[2];
        write_reg(REG_LED_SEQ_2, SEQ2);
        write_reg(REG_LED_SEQ_3, ledMode[4]);
      }
      else {
        ledMode[1]= ledMode[1]<<4;
        int SEQ1= ledMode[1]+ ledMode[0];
        write_reg(REG_LED_SEQ_1, SEQ1);
        ledMode[3]= ledMode[3]<<4;
        int SEQ2= ledMode[3] + ledMode[2];
        write_reg(REG_LED_SEQ_2, SEQ2);
        ledMode[5]= ledMode[5]<<4;
        int SEQ3= ledMode[5]+ ledMode[4];
        write_reg(REG_LED_SEQ_3, SEQ3);
      }
      
    }
    void MAX86141::setIntensityLed(int intens_led){
      intensity_led= intens_led;
      
      if(ledModeSize==1){
      write_reg(REG_LED_RANGE_1, 0b00000011); // xx,LED3,LED2,LED1. 00,01,10,11 low to high
      write_reg(REG_LED1_PA, intensity_led);}
      
      else if(ledModeSize==2){
      write_reg(REG_LED_RANGE_1, 0b00001111); // xx,LED3,LED2,LED1. 00,01,10,11 low to high
      write_reg(REG_LED1_PA, intensity_led); // 0 = 0mA, 255 = Max mA
    write_reg(REG_LED2_PA, intensity_led);}
    
    else if(ledModeSize==3){
    write_reg(REG_LED_RANGE_1, 0b00111111); // xx,LED3,LED2,LED1. 00,01,10,11 low to high
    write_reg(REG_LED1_PA, intensity_led); // 0 = 0mA, 255 = Max mA
    write_reg(REG_LED2_PA, intensity_led);
    write_reg(REG_LED3_PA, intensity_led);
    }
    
    else if(ledModeSize==4){
    write_reg(REG_LED_RANGE_1, 0b00111111); // xx,LED3,LED2,LED1. 00,01,10,11 low to high
    write_reg(REG_LED_RANGE_2, 0b00000011); // xx,LED6,LED5,LED4. 00,01,10,11 low to high
    write_reg(REG_LED1_PA, intensity_led); // 0 = 0mA, 255 = Max mA
    write_reg(REG_LED2_PA, intensity_led);
    write_reg(REG_LED3_PA, intensity_led);
    write_reg(REG_LED4_PA, intensity_led);
    }

    else if(ledModeSize==5){
    write_reg(REG_LED_RANGE_1, 0b00111111); // xx,LED3,LED2,LED1. 00,01,10,11 low to high
    write_reg(REG_LED_RANGE_2, 0b00001111); // xx,LED6,LED5,LED4. 00,01,10,11 low to high
    write_reg(REG_LED1_PA, intensity_led); // 0 = 0mA, 255 = Max mA
    write_reg(REG_LED2_PA, intensity_led);
    write_reg(REG_LED3_PA, intensity_led);
    write_reg(REG_LED4_PA, intensity_led);
    write_reg(REG_LED5_PA, intensity_led);
    }
    else{
    write_reg(REG_LED_RANGE_1, 0b00111111); // xx,LED3,LED2,LED1. 00,01,10,11 low to high
    write_reg(REG_LED_RANGE_2, 0b00111111); // xx,LED6,LED5,LED4. 00,01,10,11 low to high
    write_reg(REG_LED1_PA, intensity_led); // 0 = 0mA, 255 = Max mA
    write_reg(REG_LED2_PA, intensity_led);
    write_reg(REG_LED3_PA, intensity_led);
    write_reg(REG_LED4_PA, intensity_led);
    write_reg(REG_LED5_PA, intensity_led);
    write_reg(REG_LED6_PA, intensity_led);
    }
}
    void MAX86141::setPulseWidth(int pulse){
  pulse_width= pulse;
    }
    void MAX86141::setADCrange(int adc){
      adcRange= adc;
    }

     void MAX86141::setSample(int smpl_avr, int smpl_rate){
      sample_average= smpl_avr;
      sample_rate= smpl_rate;
     }
///////////////////////////////////////////////

/*inspired by pseudo-code available on MAX86141 datasheet for initialisation*/
 void MAX86141::initialisation(int ppg, int *ledMd, int size_led, int intens_led, int smpl_avr, int smpl_rate, int pulse, int adc, int newSpiClk=10000)
 {
      setSpiClk(newSpiClk);
          uint8_t temp;
      write_reg(REG_MODE_CONFIG, 0x01);
      delay(10);
      setNumbPPG(ppg);
      setLedModeSize(size_led);
     // write_reg(REG_PICKET_FENCE,0b11000000);

      
      //
    // Clear interrupts.
    //
    read_reg(REG_INT_STAT_1);
    read_reg(REG_INT_STAT_2);

    //
    // PPG1 & 2 & 3
    //
    write_reg(REG_PPG_SYNC_CTRL, 0b000000000);
    setPulseWidth( pulse);
       setADCrange( adc);
    if(nb_ppg==1){
       
       adcRange= adcRange<<2;
       int8_t adcRange_pulseWidth=  adcRange + pulse_width;
       Serial.print("ADC Range - PulseWidth");
          Serial.println(adcRange_pulseWidth,BIN);
       write_reg(REG_PPG_CONFIG_1, adcRange_pulseWidth); //ALC_DIS,ADD_OFF,PPG2_RGE,PPG1_RGE,PPG_TINT
    }
    else{
      int8_t a = adcRange<<4;
      int8_t b = adcRange<<2;
  
      adcRange = a+b;
      //Serial.println(adcRange,BIN);
       int8_t adcRange_pulseWidth= adcRange + pulse_width;
       Serial.print("ADC Range - PulseWidth");
          Serial.println(adcRange_pulseWidth,BIN);
       write_reg(REG_PPG_CONFIG_1, adcRange_pulseWidth); //ALC_DIS,ADD_OFF,PPG2_RGE,PPG1_RGE,PPG_TINT
    }

    setSample(smpl_avr, smpl_rate);
        Serial.println(sample_rate,BIN);

    int sample1= sample_rate<<3;
    Serial.println(sample1,BIN);
    int sample2= sample_average>>2;
    Serial.println(sample2,BIN);
    int sample= sample1 + sample2;
    Serial.print("Samlple");
          Serial.println(sample,BIN);
    write_reg(REG_PPG_CONFIG_2, sample); //SPS (0-5), SMP_AVE (6-8)
    write_reg(REG_PPG_CONFIG_3, 0b11000110); //LED_SETLNG, DIG_FILT_SEL, BURST_EN
        ////
    write_reg(REG_PD_BIAS, 0b00010001);
    ////
      setIntensityLed( intens_led);

      // Configure FIFO.
       write_reg(REG_FIFO_CONFIG_1, 128 - 6);
    write_reg(REG_FIFO_CONFIG_2, 0b00001101);

          setLedMode(ledMd);

//
    // Configure interrupt.
    //
    write_reg(REG_INT_EN_1, 0x80);

    //
    // exit shutdown mode.
    //
    read_reg(REG_MODE_CONFIG);
    temp &= ~0x02;
    write_reg(REG_MODE_CONFIG, temp);

    //
    // Clear interrupts.
    //
    read_reg(REG_INT_STAT_1);
    read_reg(REG_INT_STAT_2);

    //
    // Return REG_SYS_CNTRL which should be 0x00.
    //
    temp = read_reg(REG_MODE_CONFIG);
    //Thank you Michael Lyons!
 }
 


/* inspired by pseudo-code available on MAX86141 datasheet */
void MAX86141::device_data_read(uint8_t *dataBuf)
{
    uint8_t sample_count;
    uint8_t reg_val;
    // uint8_t dataBuf[6144]; ///128 FIFO samples, 2 channels, 2 PDs, 3 bytes/channel, 4 Leds sequence control (128*3*2*4)*2 = 6144 byte buffer
    //uint8_t dataBuf[1152]; ///128 FIFO samples, 3 channels, 1 PDs, 3 bytes/channel 128*3*1*3 = 1152 byte buffer
    // uint8_t dataBuf[768]; ///128 FIFO samples, 2 channels, 1 PDs, 3 bytes/channel 128*3*2*3 = 2304 byte buffer
    sample_count = read_reg(REG_FIFO_DATA_COUNT); //number of items available in FIFO to read

    if(debug == true){
        Serial.print("Sample Count: ");
        Serial.println(sample_count);
    }

    read_fifo(dataBuf, sample_count);

    //refer to MAX86140-MAX86141 datasheet - page 15
    if(nb_ppg==1){
        /*suitable formatting of data for 1 LED control*/
   if(ledModeSize==1){
     int i = 0;

    for (i = 0; i < sample_count; i++)
    {
        tag1[i] = (dataBuf[i*3+0] >> 3) & 0x1f;
        led1[i] = ((dataBuf[i*3+0] << 16) | (dataBuf[i*3+1] << 8) | (dataBuf[i*3+2])) & 0x7ffff;
    }
   }

    /*suitable formatting of data for 2 LEDs control*/
   if(ledModeSize==2){
     int i = 0;

    for (i = 0; i < sample_count; i++)
    {
        tag1[i] = (dataBuf[i*6+0] >> 3) & 0x1f;
        led1[i] = ((dataBuf[i*6+0] << 16) | (dataBuf[i*6+1] << 8) | (dataBuf[i*6+2])) & 0x7ffff;

        tag2[i] = (dataBuf[i*6+3] >> 3) & 0x1f;
        led2[i] = ((dataBuf[i*6+3] << 16) | (dataBuf[i*6+4] << 8) | (dataBuf[i*6+5])) & 0x7ffff;
        
    }
   }
   
    /*suitable formatting of data for 3 LEDs control*/
       if(ledModeSize==3){

    int i = 0;

    for (i = 0; i < sample_count; i++)
    {
        tag1[i] = (dataBuf[i*9+0] >> 3) & 0x1f;
        led1[i] = ((dataBuf[i*9+0] << 16) | (dataBuf[i*9+1] << 8) | (dataBuf[i*9+2])) & 0x7ffff;

        tag2[i] = (dataBuf[i*9+3] >> 3) & 0x1f;
        led2[i] = ((dataBuf[i*9+3] << 16) | (dataBuf[i*9+4] << 8) | (dataBuf[i*9+5])) & 0x7ffff;

        tag3[i] = (dataBuf[i*9+6] >> 3) & 0x1f;
        led3[i] = ((dataBuf[i*9+6] << 16) | (dataBuf[i*9+7] << 8) | (dataBuf[i*9+8])) & 0x7ffff;
    }
       }

        /*suitable formatting of data for 4 LEDs control*/
       if(ledModeSize==4){

    int i = 0;

    for (i = 0; i < sample_count; i++)
    {
        tag1[i] = (dataBuf[i*12+0] >> 3) & 0x1f;
        led1[i] = ((dataBuf[i*12+0] << 16) | (dataBuf[i*12+1] << 8) | (dataBuf[i*12+2])) & 0x7ffff;

        tag2[i] = (dataBuf[i*12+3] >> 3) & 0x1f;
        led2[i] = ((dataBuf[i*12+3] << 16) | (dataBuf[i*12+4] << 8) | (dataBuf[i*12+5])) & 0x7ffff;

        tag3[i] = (dataBuf[i*12+6] >> 3) & 0x1f;
        led3[i] = ((dataBuf[i*12+6] << 16) | (dataBuf[i*12+7] << 8) | (dataBuf[i*12+8])) & 0x7ffff;

        tag4[i] = (dataBuf[i*12+9] >> 3) & 0x1f;
        led4[i] = ((dataBuf[i*12+9] << 16) | (dataBuf[i*12+10] << 8) | (dataBuf[i*12+11])) & 0x7ffff;
    }
       }

        /*suitable formatting of data for 5 LEDs control*/
       if(ledModeSize==5){

    int i = 0;

    for (i = 0; i < sample_count; i++)
    {
        tag1[i] = (dataBuf[i*15+0] >> 3) & 0x1f;
        led1[i] = ((dataBuf[i*15+0] << 16) | (dataBuf[i*15+1] << 8) | (dataBuf[i*15+2])) & 0x7ffff;

        tag2[i] = (dataBuf[i*15+3] >> 3) & 0x1f;
        led2[i] = ((dataBuf[i*15+3] << 16) | (dataBuf[i*15+4] << 8) | (dataBuf[i*15+5])) & 0x7ffff;

        tag3[i] = (dataBuf[i*15+6] >> 3) & 0x1f;
        led3[i] = ((dataBuf[i*15+6] << 16) | (dataBuf[i*15+7] << 8) | (dataBuf[i*15+8])) & 0x7ffff;

        tag4[i] = (dataBuf[i*15+9] >> 3) & 0x1f;
        led4[i] = ((dataBuf[i*15+9] << 16) | (dataBuf[i*15+10] << 8) | (dataBuf[i*15+11])) & 0x7ffff;

        tag5[i] = (dataBuf[i*15+12] >> 3) & 0x1f;
        led5[i] = ((dataBuf[i*15+12] << 16) | (dataBuf[i*15+13] << 8) | (dataBuf[i*15+14])) & 0x7ffff;
        
    }
       }

       /*suitable formatting of data for 6 LEDs control*/
       if(ledModeSize==6){

    int i = 0;

    for (i = 0; i < sample_count; i++)
    {
        tag1[i] = (dataBuf[i*18+0] >> 3) & 0x1f;
        led1[i] = ((dataBuf[i*18+0] << 16) | (dataBuf[i*18+1] << 8) | (dataBuf[i*18+2])) & 0x7ffff;

        tag2[i] = (dataBuf[i*18+3] >> 3) & 0x1f;
        led2[i] = ((dataBuf[i*18+3] << 16) | (dataBuf[i*18+4] << 8) | (dataBuf[i*18+5])) & 0x7ffff;

        tag3[i] = (dataBuf[i*18+6] >> 3) & 0x1f;
        led3[i] = ((dataBuf[i*18+6] << 16) | (dataBuf[i*18+7] << 8) | (dataBuf[i*18+8])) & 0x7ffff;

        tag4[i] = (dataBuf[i*18+9] >> 3) & 0x1f;
        led4[i] = ((dataBuf[i*18+9] << 16) | (dataBuf[i*18+10] << 8) | (dataBuf[i*18+11])) & 0x7ffff;

        tag5[i] = (dataBuf[i*18+12] >> 3) & 0x1f;
        led5[i] = ((dataBuf[i*18+12] << 16) | (dataBuf[i*18+13] << 8) | (dataBuf[i*18+14])) & 0x7ffff;

        tag6[i] = (dataBuf[i*18+15] >> 3) & 0x1f;
        led6[i] = ((dataBuf[i*18+15] << 16) | (dataBuf[i*18+16] << 8) | (dataBuf[i*18+17])) & 0x7ffff;
        
    }
       }}
       else {
        if(ledModeSize==1){
     int i = 0;

    for (i = 0; i < sample_count; i++)
    {
        tag1[i] = (dataBuf[i*6+0] >> 3) & 0x1f;
        led1[i] = ((dataBuf[i*6+0] << 16) | (dataBuf[i*6+1] << 8) | (dataBuf[i*6+2])) & 0x7ffff;

        tag2[i] = (dataBuf[i*6+3] >> 3) & 0x1f;
        led2[i] = ((dataBuf[i*6+3] << 16) | (dataBuf[i*6+4] << 8) | (dataBuf[i*6+5])) & 0x7ffff;
        
    }
   }

    /*suitable formatting of data for 2 LEDs control*/
   if(ledModeSize==2){
     int i = 0;

    for (i = 0; i < sample_count; i++)
    {
        tag1[i] = (dataBuf[i*12+0] >> 3) & 0x1f;
        led1[i] = ((dataBuf[i*12+0] << 16) | (dataBuf[i*12+1] << 8) | (dataBuf[i*12+2])) & 0x7ffff;

        tag2[i] = (dataBuf[i*12+3] >> 3) & 0x1f;
        led2[i] = ((dataBuf[i*12+3] << 16) | (dataBuf[i*12+4] << 8) | (dataBuf[i*12+5])) & 0x7ffff;

        tag3[i] = (dataBuf[i*12+6] >> 3) & 0x1f;
        led3[i] = ((dataBuf[i*12+6] << 16) | (dataBuf[i*12+7] << 8) | (dataBuf[i*12+8])) & 0x7ffff;

        tag4[i] = (dataBuf[i*12+9] >> 3) & 0x1f;
        led4[i] = ((dataBuf[i*12+9] << 16) | (dataBuf[i*12+10] << 8) | (dataBuf[i*12+11])) & 0x7ffff;
        
    }
   }
   
    /*suitable formatting of data for 3 LEDs control*/
       if(ledModeSize==3){

    int i = 0;

    for (i = 0; i < sample_count; i++)
    {
        tag1[i] = (dataBuf[i*18+0] >> 3) & 0x1f;
        led1[i] = ((dataBuf[i*18+0] << 16) | (dataBuf[i*18+1] << 8) | (dataBuf[i*18+2])) & 0x7ffff;

        tag2[i] = (dataBuf[i*18+3] >> 3) & 0x1f;
        led2[i] = ((dataBuf[i*18+3] << 16) | (dataBuf[i*18+4] << 8) | (dataBuf[i*18+5])) & 0x7ffff;

        tag3[i] = (dataBuf[i*18+6] >> 3) & 0x1f;
        led3[i] = ((dataBuf[i*18+6] << 16) | (dataBuf[i*18+7] << 8) | (dataBuf[i*18+8])) & 0x7ffff;

        tag4[i] = (dataBuf[i*18+9] >> 3) & 0x1f;
        led4[i] = ((dataBuf[i*18+9] << 16) | (dataBuf[i*18+10] << 8) | (dataBuf[i*18+11])) & 0x7ffff;

        tag5[i] = (dataBuf[i*18+12] >> 3) & 0x1f;
        led5[i] = ((dataBuf[i*18+12] << 16) | (dataBuf[i*18+13] << 8) | (dataBuf[i*18+14])) & 0x7ffff;

        tag6[i] = (dataBuf[i*18+15] >> 3) & 0x1f;
        led6[i] = ((dataBuf[i*18+15] << 16) | (dataBuf[i*18+16] << 8) | (dataBuf[i*18+17])) & 0x7ffff;
    }
       }
           if(ledModeSize==4){

    int i = 0;

    for (i = 0; i < sample_count; i++)
    {
        tag1[i] = (dataBuf[i*24+0] >> 3) & 0x1f;
        led1[i] = ((dataBuf[i*24+0] << 16) | (dataBuf[i*24+1] << 8) | (dataBuf[i*24+2])) & 0x7ffff;

        tag2[i] = (dataBuf[i*24+3] >> 3) & 0x1f;
        led2[i] = ((dataBuf[i*24+3] << 16) | (dataBuf[i*24+4] << 8) | (dataBuf[i*24+5])) & 0x7ffff;

        tag3[i] = (dataBuf[i*24+6] >> 3) & 0x1f;
        led3[i] = ((dataBuf[i*24+6] << 16) | (dataBuf[i*24+7] << 8) | (dataBuf[i*24+8])) & 0x7ffff;

        tag4[i] = (dataBuf[i*24+9] >> 3) & 0x1f;
        led4[i] = ((dataBuf[i*24+9] << 16) | (dataBuf[i*24+10] << 8) | (dataBuf[i*24+11])) & 0x7ffff;

        tag5[i] = (dataBuf[i*24+12] >> 3) & 0x1f;
        led5[i] = ((dataBuf[i*24+12] << 16) | (dataBuf[i*24+13] << 8) | (dataBuf[i*24+14])) & 0x7ffff;

        tag6[i] = (dataBuf[i*24+15] >> 3) & 0x1f;
        led6[i] = ((dataBuf[i*24+15] << 16) | (dataBuf[i*24+16] << 8) | (dataBuf[i*24+17])) & 0x7ffff;

        tag7[i] = (dataBuf[i*24+18] >> 3) & 0x1f;
        led7[i] = ((dataBuf[i*24+18] << 16) | (dataBuf[i*24+19] << 8) | (dataBuf[i*24+20])) & 0x7ffff;

        tag8[i] = (dataBuf[i*24+21] >> 3) & 0x1f;
        led8[i] = ((dataBuf[i*24+21] << 16) | (dataBuf[i*24+22] << 8) | (dataBuf[i*24+23])) & 0x7ffff;
    }
       }
}
    clearInt();
}

/*void MAX86141::fifo_intr()
{
    uint8_t count;
    count = read_reg(REG_FIFO_DATA_COUNT);

    if (count == 0x80) //indicates full FIFO
    {
        device_data_read();
    }
 }*/


/*read FIFO*/
void MAX86141::read_fifo(uint8_t data_buffer[], uint8_t count)
{
    data_buffer[0] = REG_FIFO_DATA;
    data_buffer[1] = READ_EN;
    digitalWrite(SS, HIGH);
    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(SS, LOW);
    spi->transfer(data_buffer, 2);
    spi->transfer(data_buffer, count * 3);
    digitalWrite(SS, HIGH);
    spi->endTransaction();
    digitalWrite(SS, LOW);

    if(debug == true){
        Serial.println("Data buffer");
        Serial.println(data_buffer[0]);
    }

}

void MAX86141::setSS(int pin){
    SS = pin;
}

void MAX86141::setSPI(SPIClass * newspi){
    spi = newspi;
}

void MAX86141::setSpiClk(int newSpiClk) {
    spiClk = newSpiClk;
}

void MAX86141::setDebug(bool setdebug) {
    debug = setdebug;
}

void MAX86141::clearInt() {
    uint16_t intr = 0x00;
    intr = read_reg(REG_INT_STAT_1) << 8;
    intr |= read_reg(REG_INT_STAT_2);
}
