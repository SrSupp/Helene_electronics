#include "Motor.h"

void Motor::send_Data(unsigned long address, unsigned long datagram)
{
  uint8_t stat;
  unsigned long i_datagram;
  SPI.beginTransaction(tmc_spi_settings);

  digitalWrite(g_chipCS, LOW);
  delayMicroseconds(10);

  stat = SPI.transfer(address);

  i_datagram |= SPI.transfer((datagram >> 24) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 16) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 8) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram)&0xff);
  digitalWrite(g_chipCS, HIGH);
  SPI.endTransaction();
}

Motor::Motor(int CS, int EN, long u_maxspeed, long u_acceleration, boolean u_highcurrent)
{
  g_chipCS = CS;
  g_max_speed = u_maxspeed;
  g_acceleration = u_acceleration;
  pinMode(g_chipCS, OUTPUT);
  pinMode(EN, OUTPUT);
  g_pin_en = EN;
  digitalWrite(g_pin_en,HIGH);
  tmc_spi_settings = SPISettings(4000000, MSBFIRST, SPI_MODE3);
  SPI.begin(14, 12, 13, 9);
  send_Data(0xA0, 0);
  send_Data(0x80, 0x00000000);     //GCONF
  send_Data(0xEC, 0x000101D5);     //CHOPCONF: TOFF=5, HSTRT=5, HEND=3, TBL=2, CHM=0 (spreadcycle)
  if(u_highcurrent==true){
      send_Data(0x90,0x0006070A);
  }else{
      send_Data(0x90,0x00060304);
  }
  send_Data(0x91, 0x0000000A);     //TPOWERDOWN=10
  send_Data(0xF0, 0x00000000);     // PWMCONF
  send_Data(0xA4, g_acceleration); //A1=1000
  send_Data(0xA5, g_max_speed);    //V1=100000
  send_Data(0xA6, g_acceleration); //AMAX=50000
  send_Data(0xA7, g_max_speed);    //VMAX=500000
  send_Data(0xAA, 100000);         //D1=1400
  send_Data(0xA8, 100000);         //DMAX=50000
  send_Data(0xAB, 0x0000000A);     //VSTOP=10
  send_Data(0xA0, 0);              //RAMPMODE=0 - bedeutet position-interface. Bei 1 velocity Interface. Die Geschwindigkeit wird dann über VMax gesteuert
  send_Data(0xA1, 0x00000000);     //XACTUAL=0
  send_Data(0xAD, 0x00000000);     //XTARGET=0
  delay(20);
}

long Motor::get_vactual()
{
  uint8_t stat;
  long i_datagram;
  SPI.beginTransaction(tmc_spi_settings);
  digitalWrite(g_chipCS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x22);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(g_chipCS, HIGH);
  SPI.endTransaction();

  delayMicroseconds(1);

  SPI.beginTransaction(tmc_spi_settings);
  digitalWrite(g_chipCS, LOW);
  delayMicroseconds(1);
  stat = SPI.transfer(0x22);
  i_datagram |= SPI.transfer(0x00);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer(0x00);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer(0x00);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer(0x00);
  digitalWrite(g_chipCS, HIGH);
  SPI.endTransaction();

  if (i_datagram > 8387800)
  { //8387800 ist die häflte einer 3byte Long -> Das vorzeichen wird so hingetrickst!
    i_datagram = i_datagram - 16775600 - 1616;
  }

  return i_datagram;
}

long Motor::get_vtarget()
{
  return g_target_velocity;
}

long Motor::get_xactual()
{
  uint8_t stat;
  long i_datagram;
  SPI.beginTransaction(tmc_spi_settings);
  digitalWrite(g_chipCS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x21);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(g_chipCS, HIGH);
  SPI.endTransaction();

  delayMicroseconds(1);

  SPI.beginTransaction(tmc_spi_settings);
  digitalWrite(g_chipCS, LOW);
  delayMicroseconds(1);
  stat = SPI.transfer(0x21);
  i_datagram |= SPI.transfer(0x00);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer(0x00);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer(0x00);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer(0x00);
  digitalWrite(g_chipCS, HIGH);
  SPI.endTransaction();

  if (i_datagram > 8387800)
  { //8387800 ist die häflte einer 3byte Long -> Das vorzeichen wird so hingetrickst!
    i_datagram = i_datagram - 16775600 - 1616;
  }

  return i_datagram;
}

void Motor::set_velocity(long datagram)
{
  g_lastcall = millis();
  g_target_velocity = datagram;
  if (abs(datagram) < 1)
  {
    send_Data(0xA7, 0);
  }
  else if (datagram < 0)
  {
    send_Data(0xA0, 2);
    send_Data(0xA7, -datagram); 
  }
  else
  {
    send_Data(0xA0, 1);
    send_Data(0xA7, datagram);  
  }
}

void Motor::set_position(long datagram)
{
  send_Data(0xA0, 0);
  send_Data(0xAD, datagram * 256); 
}

void Motor::set_maxspeed(long datagram)
{
  g_max_speed = datagram;
  send_Data(0xA5, g_max_speed); 
  send_Data(0xA7, g_max_speed); 
}

void Motor::set_acceleration(long datagram)
{
  g_acceleration = datagram;
  send_Data(0xA4, g_acceleration); 
  send_Data(0xA6, g_acceleration); 
}

long Motor::get_last_time_of_set_velocity(){
  return g_lastcall;
}

void Motor::set_current(boolean u_highcurrent)
{
  if(u_highcurrent==true){
      send_Data(0x90,0x0006070A);
  }else{
      send_Data(0x90,0x00060304);
  }
}

void Motor::enable_motor()
{
  digitalWrite(g_pin_en,LOW);
}

void Motor::disable_motor()
{
  digitalWrite(g_pin_en,HIGH);
}