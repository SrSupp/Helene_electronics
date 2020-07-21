#include <Arduino.h>
#include <SPI.h>

class Motor
{
private:
    int g_chipCS;
    int g_pin_en;
    SPISettings tmc_spi_settings;
    long g_max_speed;
    long g_acceleration;
    long g_target_velocity;
    long g_lastcall;
    void send_Data(unsigned long address, unsigned long datagram);
public:
    Motor(int CS,int EN, long u_maxspeed, long u_acceleration,boolean u_highcurrent);

    long get_vactual();
    long get_vtarget();
    long get_xactual();
    long get_last_time_of_set_velocity();
    void set_velocity(long datagram);
    void set_position(long datagram);
    void set_maxspeed(long datagram);
    void set_acceleration(long datagram);
    void set_current(boolean u_highcurrent);
    void enable_motor();
    void disable_motor();
};