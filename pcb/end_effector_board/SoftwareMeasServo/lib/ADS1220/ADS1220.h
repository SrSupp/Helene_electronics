// Library für die Programmierung des ADS1220. Eike Christmann, TU Darmstadt 2016

#ifndef ADS1220_h
#define ADS1220_h

#include <Arduino.h>

#define RESET 0x06
#define START 0x08

// Folgende Einstellungen werden als default für den Versuchsaufbau Praktikum Messtechnik als default eingestellt:
// Gain 128, siehe Seite 40
// MUX AIN1-AIN2, siehe Seite 40
// 0b00111110
#define config_reg_0 0x3E
// Default
#define config_reg_1 0x00
// Default
#define config_reg_2 0x00
// Default
#define config_reg_3 0x00

// Diese Werten dienen nur zur Berechnung der Eingangsspannung.
// PGA etc. lässt sich über config_reg_0 ändern
// Muss zur korrekten Eingangsspannung auch hier geändert werden!
extern int PGA;        
#define VFSR VREF_ADS1220/PGA             
#define FSR (((long int)1<<23)-1)        

// 4 Bits für den Schreibbefehl, es folgen 2 Bit für die Adresse des Regsiters, 
// anschließend 2 Bit für die Menge der zu übertragenden Bits. 
// Erklärung Seite 36 
#define WREG 0x40
// 4 Bits für den Lesebbefehl, es folgen 2 Bit für die Adresse des Regsiters, 
// anschließend 2 Bit für die Menge der zu übertragenden Bits. 
#define RREG 0x20

#define SPI_MASTER_DUMMY 0xFF

class ADS1220
{
  public:
    boolean NewDataAvailable;
    ADS1220(); // Konstruktor
    void begin(int in_DRDY, int in_CS1);
	void writeRegister(byte address, byte value);
    uint8_t readRegister(byte address);
	void check_Register();
	void setGain(int value);
	void setMUX(int mux);
	void setDatarate(int datarate);
	void setOperatingMode(char mode);
  void setAnalogReference(char mode);
	void startContinuousMeas(boolean enable);
    void SPI_Command(unsigned char data_in);
	float Read_Data();
  private:
    int DRDY; // DRDY ADS1220
    int CS1; // slave select ADS1220
    float VREF_ADS1220 = 2.048;
};

#endif