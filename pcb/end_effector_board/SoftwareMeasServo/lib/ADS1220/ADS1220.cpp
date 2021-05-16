// Library für die Programmierung des ADS1220. Eike Christmann, TU Darmstadt 2016. Umgeschrieben von Sven Suppelt 2021. 

#include <Arduino.h>
#include "ADS1220.h"
#include "SPI.h"

int PGA = 128;

// Konstruktor
ADS1220::ADS1220(void)
{
  return;
}
// Methoden

void ADS1220::begin(int in_DRDY, int in_CS1)
{
  DRDY = in_DRDY; // DRDY ADS1220
  CS1 = in_CS1; // slave select ADS1220
  pinMode(CS1, OUTPUT);
  pinMode(DRDY, INPUT);
  SPI.begin(14, 12, 13, 21); // This is specifically for Sven Suppelts ESP32 boards!
  SPI.setDataMode(SPI_MODE1);
  delayMicroseconds(5);
  SPI_Command(RESET);
  delayMicroseconds(5);
  digitalWrite(CS1, LOW); 
  writeRegister(0x00, config_reg_0);
  writeRegister(0x01, config_reg_1);
  writeRegister(0x02, config_reg_2);
  writeRegister(0x03, config_reg_3);
  
  digitalWrite(CS1, HIGH); 
  delayMicroseconds(5);
  NewDataAvailable = false;
}


void ADS1220::writeRegister(byte address, byte value)
{
  digitalWrite(CS1, LOW);
  delayMicroseconds(2);
  SPI.transfer(WREG | (address << 2));
  SPI.transfer(value);
  delayMicroseconds(2);
  digitalWrite(CS1, HIGH);
}


byte ADS1220::readRegister(byte address)
{
  byte data;

  digitalWrite(CS1, LOW);
  delayMicroseconds(2);
  SPI.transfer(RREG | (address << 2));
  data = SPI.transfer(SPI_MASTER_DUMMY);
  delayMicroseconds(2);
  digitalWrite(CS1, HIGH);

  return data;
}

void ADS1220::check_Register()
{
	byte test0 = 0x00;
	byte test1 = 0x00;
	byte test2 = 0x00;
	byte test3 = 0x00;
	
	test0 = readRegister(0x00);
    test1 = readRegister(0x01);
    test2 = readRegister(0x02);
    test3 = readRegister(0x03);
	
	Serial.println("Config_Reg : ");
    Serial.println(test0, HEX);
    Serial.println(test1, HEX);
    Serial.println(test2, HEX);
    Serial.println(test3, HEX);
    Serial.println(" ");
}

void ADS1220::setGain(int value)
{
	byte gain_reg;
	switch(value) {
		case 1:
		  // Gain = 1
		  gain_reg = 0x00;
		  PGA = 1;
		  break;
		case 2:
		  // Gain = 2
		  gain_reg = 0x01;
		  PGA = 2;
		  break;
		case 4:
		  // Gain = 4
		  gain_reg = 0x02;
		  PGA = 4;
		  break;
		case 8:
		  // Gain = 8
		  gain_reg = 0x03;
		  PGA = 8;
		  break;
		case 16:
		  // Gain = 16
		  gain_reg = 0x04;
		  PGA = 16;
		  break;
		case 32:
		  // Gain = 32
		  gain_reg = 0x05;
		  PGA = 32;
		  break;
		case 64:
		  // Gain = 64
		  gain_reg = 0x06;
		  PGA = 64;
		  break;
		case 128:
		  // Gain = 128
		  gain_reg = 0x07;
		  PGA = 128;
		  break;
			
	}	
		
	byte reg_0 = 0x00;
	reg_0 = readRegister(0x00);
	// relevante Stellen löschen:
	reg_0 = reg_0 & 0b11110001;
    // relevante Stellen mit gain_reg ersetzen:
	gain_reg = gain_reg << 1;
    reg_0 = reg_0 | gain_reg;
    delayMicroseconds(5);
    writeRegister(0x00, reg_0);
	
}

void ADS1220::setMUX(int mux)
{
	byte mux_reg;
	switch(mux) {
		case 0:
		  // AINP = AIN0, AINN = AIN1 
		  mux_reg = 0x00;
		break;
		case 1:
		  // AINP = AIN0, AINN = AIN2
		  mux_reg = 0x01;
		break;
		case 2:
		  // AINP = AIN0, AINN = AIN3
		  mux_reg = 0x02;
		break;
		case 3:
		  // AINP = AIN1, AINN = AIN2
		  mux_reg = 0x03;
		break;
		
		case 4:
		  // AINP = AIN1, AINN = AIN3
		  mux_reg = 0x04;
		break;
		case 5:
		  // AINP = AIN2, AINN = AIN3
		  mux_reg = 0x05;
		break;
		case 6:
		  // AINP = AIN1, AINN = AIN0
		  mux_reg = 0x06;
		break;
		case 7:
		  // AINP = AIN3, AINN = AIN2
		  mux_reg = 0x07;
		break;
		case 8:
		  // AINP = AIN0, AINN = AVSS
		  mux_reg = 0x08;
		break;
		case 9:
		  // AINP = AIN1, AINN = AVSS
		  mux_reg = 0x09;
		break;
		case 10:
		  // AINP = AIN2, AINN = AVSS
		  mux_reg = 0x0A;
		break;
		case 11:
		  // AINP = AIN3, AINN = AVSS
		  mux_reg = 0x0B;
		break;
		case 12:
		  // (V(REFPx) – V(REFNx)) / 4 monitor (PGA bypassed)
		  mux_reg = 0x0C;
		break;
		case 13:
		  // (AVDD – AVSS) / 4 monitor (PGA bypassed)
		  mux_reg = 0x0D;
		break;
		case 14:
		  // AINP and AINN shorted to (AVDD + AVSS) / 2
		  mux_reg = 0x0E;
		break;
		
			
	}	
		
	byte reg_0 = 0x00;
	reg_0 = readRegister(0x00);
	// relevante Stellen löschen:
	reg_0 = reg_0 & 0b00001111;
	// relevante Stellen mit mux_reg ersetzen:
	mux_reg = mux_reg << 4;
	reg_0 = reg_0 | mux_reg;
	delayMicroseconds(2);
    writeRegister(0x00, reg_0);
}

void ADS1220::setDatarate(int datarate)
{
	
	byte rate_reg;
	byte reg_1 = 0x00;
	reg_1 = readRegister(0x01);
	if((reg_1 & 0b00011000) == 0x00)
	{
		switch(datarate) {
		case 20:
		  // 20 SPS
		  rate_reg = 0x00;
		break;
		case 45:
		  // 45 SPS
		  rate_reg = 0x01;
		break;
		case 90:
		  // 90 SPS
		  rate_reg = 0x02;
		break;
		case 175:
		  // 175 SPS
		  rate_reg = 0x03;
		break;
		case 330:
		  // 330 SPS
		  rate_reg = 0x04;
		break;
		case 600:
		  // 600 SPS
		  rate_reg = 0x05;
		break;
		case 1000:
		  // 1000 SPS
		  rate_reg = 0x06;
		  Serial.println("1000");
		break;
		default:
		Serial.print("Es wurde keine gültige Datarate ausgewählt. Eingestellter Modus: Normal");
		break;	
	}
	}	
	
	else if((reg_1 & 0b00011000) == 0x10)
	{
		switch(datarate) {
		case 40:
		  // 40 SPS
		  rate_reg = 0x00;
		break;
		case 90:
		  // 90 SPS
		  rate_reg = 0x01;
		break;
		default:
		case 180:
		  // 180 SPS
		  rate_reg = 0x02;
		break;
		case 350:
		  // 350 SPS
		  rate_reg = 0x03;
		break;
		case 660:
		  // 660 SPS
		  rate_reg = 0x04;
		break;
		case 1200:
		  // 1200 SPS
		  rate_reg = 0x05;
		break;
		case 2000:
		  // 2000 SPS
		  rate_reg = 0x06;
		break;
		Serial.print("Es wurde keine gültige Datarate ausgewählt. Eingestellter Modus: Turbo");
		break;	
	}
	}
	else Serial.print("Es wurde kein gültiger Modus ausgewählt. Funktion: setDatarate");
	
	reg_1 = reg_1 & 0b00011111;
	// relevante Stellen mit mux_reg ersetzen:
	rate_reg = rate_reg << 5;
	reg_1 = reg_1 | rate_reg;
	delayMicroseconds(5);
    writeRegister(0x01, reg_1);
}

void ADS1220::setAnalogReference(char mode)
{
	byte mode_reg;
	switch(mode) {
		case 'I':
		  // Internal 2.048V reference
		  mode_reg = 0x00;
		  VREF_ADS1220 = 2.048;
		break;
		case 'E':
		  // External reference using AVDD-AVSS
		  mode_reg = 0x03;
		  VREF_ADS1220 = 1;
		break;
		default:
		Serial.print("Es wurde kein gültiger Modus ausgewählt");
		break;	
	}
	byte reg_1 = 0x00;
	reg_1 = readRegister(0x02);
	//Serial.println("Vorher: " + String(reg_1));
	// relevante Stellen löschen:
	reg_1 = reg_1 & 0b00111111;
	// relevante Stellen mit mode_reg ersetzen:
	mode_reg = mode_reg << 6;
	reg_1 = reg_1 | mode_reg;
	delayMicroseconds(5);
    writeRegister(0x02, reg_1);
	//Serial.println("Nachher: " + String(reg_1));
}

void ADS1220::setOperatingMode(char mode)
{
	byte mode_reg;
	switch(mode) {
		case 'N':
		  // Normal mode
		  mode_reg = 0x00;
		break;
		case 'T':
		  // Turbo mode
		  mode_reg = 0x02;
		break;
		default:
		Serial.print("Es wurde kein gültiger Modus ausgewählt");
		break;	
	}
	byte reg_1 = 0x00;
	reg_1 = readRegister(0x01);
	// relevante Stellen löschen:
	reg_1 = reg_1 & 0b11100111;
	// relevante Stellen mit mode_reg ersetzen:
	mode_reg = mode_reg << 3;
	reg_1 = reg_1 | mode_reg;
	delayMicroseconds(5);
    writeRegister(0x01, reg_1);
}

void ADS1220::startContinuousMeas(boolean enable)
{
	
   byte cont_reg;
   if(enable == true)
   {
	// Continuous mode aktivieren
	cont_reg = 0x01;
   }
   else
   {
	// Single-shot 
    cont_reg = 0x00;
   }
   byte reg_1 = 0x00;
   reg_1 = readRegister(0x01);
   // relevante Stellen löschen:
   reg_1 = reg_1 & 0b11111011;
   // relevante Stellen mit cont_reg ersetzen:
   cont_reg = cont_reg << 2;
   reg_1 = reg_1 | cont_reg;
   delayMicroseconds(5);
   writeRegister(0x01, reg_1);
   delayMicroseconds(5); 
   SPI_Command(START);
}


void ADS1220::SPI_Command(unsigned char data_in)
{
  digitalWrite(CS1, LOW);
  delayMicroseconds(5);
  digitalWrite(CS1, HIGH);
  delayMicroseconds(5);
  digitalWrite(CS1, LOW);
  delayMicroseconds(5);
  SPI.transfer(data_in);
  delayMicroseconds(5);
  digitalWrite(CS1, HIGH);
}


float ADS1220::Read_Data()
{
  static byte SPI_Buff[3];
  
  byte MSB;
  byte data;
  byte LSB;

  digitalWrite(CS1, LOW);                     
  delayMicroseconds(2);
  for (int i = 0; i < 3; i++)
  {
    SPI_Buff[i] = SPI.transfer(SPI_MASTER_DUMMY);
  }
  delayMicroseconds(2);
  digitalWrite(CS1, HIGH); 
	
  MSB = SPI_Buff[0];
  data = SPI_Buff[1];
  LSB = SPI_Buff[2];

  long int bit24;
  long int bit32;

  bit24 = MSB;
  bit24 = (bit24 << 8) | data;
  bit24 = (bit24 << 8) | LSB;
 
  bit24= ( bit24 << 8 );
  bit32 = ( bit24 >> 8 );                    
  
  float Vout = (float)((bit32*VREF_ADS1220*1000)/(PGA*8388607));     // in mV
  //Serial.print("Vout in mV : ");  
  //Serial.println(Vout);
  //Serial.print("  32bit HEX : ");
  //Serial.println(bit32,HEX);
  return Vout;
}