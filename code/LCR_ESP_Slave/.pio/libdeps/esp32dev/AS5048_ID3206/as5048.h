#ifndef _AS5048_H_
#define _AS5048_H_

#include "mbed.h"
/**
 * Interfacing with the AMS AS5048A magnetic rotary sensor using SPI protocol
 * AS5048 uses 16-bit transfer;
 * We use two 8-bit transfers for compatibility with 8-bit SPI master devices
 * SPI protocol:
 *   Mode = 1: 
 *   clock polarity = 0 --> clock pulse is high
 *   clock phase = 1 --> sample on falling edge of clock pulse
 * Code was succesfully tested on the FRDM KL25Z and K22F. The same code fails
 * on the K64F for some reason. Sampling using a logic analyzer does however
 * show the same results for al three boards.
 */
class As5048 {


public:

    static const int kNumSensorBits        = 14;      // 14-bits sensor
    static const uint16_t kCountsPerRev    = 0x4000;  // 2**NUM_SENSOR_BITS
    static const uint16_t kMask            = 0x3FFF;  // 2**NUM_SENSOR_BITS - 1
    static const int kParity               = 1;       // even parity
    
    static const int kSpiFrequency         = 1000000; // AS5048 max 10 MHz
    static const int kSpiBitsPerTransfer   = 8;
    static const int kSpiMode              = 1;
    
    static const float kDegPerRev          = 360.0f;  // 360 degrees/rev
    static const float kRadPerRev          = 6.28318530718f; // 2*pi rad/rev

    // AS5048 flags
    typedef enum {
        AS_FLAG_PARITY          = 0x8000,
        AS_FLAG_READ            = 0x4000,
    } As5048Flag;
    
    // AS5048 commands
    typedef enum {
        AS_CMD_NOP              = 0x0000,
        AS_CMD_ERROR            = 0x0001 | AS_FLAG_READ,   // Reads error register of sensor and clear error flags
        AS_CMD_DIAGNOSTICS      = 0x3FFD | AS_FLAG_READ,   // Reads automatic gain control and diagnostics info
        AS_CMD_MAGNITUDE        = 0x3FFE | AS_FLAG_READ,
        AS_CMD_ANGLE            = 0x3FFF | AS_FLAG_PARITY | AS_FLAG_READ,
    } As5048Command;
    
    // AS5048 diagnostics
    typedef enum {
        AS_DIAG_CORDIC_OVERFLOW = 0x0200,
        AS_DIAG_HIGH_MAGNETIC   = 0x0400,
        AS_DIAG_LOW_MAGNETIC    = 0x0800,
    } As5048Diagnostics;

    /**
     * Creates an object of num_sensors daisy chained AS5048 sensors;
     * default number of sensors in chain is 1
     * @param mosi: pinname of the mosi pin of the spi communication
     * @param miso: pinname of the miso pin of the spi communication
     * @param sck: pinname of the clock pin of the spi communication
     * @param cs: pinname of the chip select pin of the spi communication
     * @param num_sensors = 1: number of sensors in daisy chain
     */
    As5048(PinName mosi, PinName miso, PinName sck, PinName cs, int num_sensors = 1):
        kNumSensors_(num_sensors),
        chip_(cs),
        spi_(mosi, miso, sck) 
    {
        DeselectChip();
        
        spi_.format(kSpiBitsPerTransfer, kSpiMode);
        spi_.frequency(kSpiFrequency);
        
        read_buffer_  = new uint16_t[kNumSensors_];
        angle_buffer_ = new uint16_t[kNumSensors_];
        angle_offset_ = new uint16_t[kNumSensors_];
        directions_ = new bool[kNumSensors_];
        
        for (int i=0; i<kNumSensors_; ++i) {
            read_buffer_[i] = 0;
            angle_buffer_[i] = 0;
            angle_offset_[i] = 0;
            directions_[i] = true;
        }
        
        last_command_ = AS_CMD_NOP;
    }

    
    /**
     * Destructor, memory deallocation
     */
    ~As5048() 
    {
        delete [] read_buffer_;
        delete [] angle_buffer_;
        delete [] angle_offset_;
        delete [] directions_;
    }
        
    /**
     * Parity check
     * @param n: integer to check
     * @return: true if ok
     */
    static bool CheckParity(int n) 
    {
        int parity = n;
        for(int i=1; i <= kNumSensorBits+1; ++i) {
            n >>= 1;
            parity ^= n;
        }
        return (parity & kParity) == 0;
    }
    
    /**
     * Update the buffer with angular measurements
     * NOTE 1:
     *  If the last command sent through Transfer was *not* AS_CMD_ANGLE
     *  then we need an additional Transfer; this takes more time!
     *  This should not occur, since Transfer is not *yet* used elsewhere.
     * NOTE 2:
     *  We run a parity check on the results from the transfer. We only 
     *  update the angle_buffer_ with values that pass the parity check.
     * Measurement using Timer on K64F  for last_command_ == AS_CMD_ANGLE
     * shows this function takes 87 or 88 us.
     */
    void UpdateAngleBuffer() 
    {
        // ensure that the new results indeed will be angles
        if (last_command_ != AS_CMD_ANGLE) {
            Transfer(AS_CMD_ANGLE);
        }
        
        // update the read buffer
        Transfer(AS_CMD_ANGLE); 
        
        // update the angle buffer with parity checked values
        for (int i=0; i<kNumSensors_; ++i) {
            if (CheckParity(read_buffer_[i])) {
                // only update angles when parity is correct
                angle_buffer_[i] = read_buffer_[i];
            }
        }
    }
    
    /**
     * @return: pointer to read_buffer_
     */
    const uint16_t* get_read_buffer()  { return read_buffer_; }
    
    /**
     * @return: pointer to angle_buffer_
     */
    const uint16_t* get_angle_buffer() { return angle_buffer_; }
    
    /**
     * @return: pointer to angle_offet_
     */
    const uint16_t* get_angle_offset() { return angle_offset_; }
    
    /**
     * @return: pointer to directions_
     */
    const bool * get_directions_() { return directions_;}
    
    /**
     * You get the angles from two UpdateAngleBuffer() calls before
     * @return: 14 bits absolute position
     */
    int getAngle(int i_sensor=0)
    { 
        int ans = ((int) (angle_buffer_[i_sensor] & kMask)) - angle_offset_[i_sensor];
        return directions_[i_sensor]?ans:-ans;
    }
    
    /**
     * You get the angles from two UpdateAngleBuffer() calls before
     * @return: revolution ratio in [0,1]
     */
    float getAngleRatio(int i_sensor=0)      { return (float) getAngle(i_sensor) / kCountsPerRev; }
    
    /**
     * You get the angles from two UpdateAngleBuffer() calls before
     * @return: angle in degrees
     */
    float getAngleDegrees(int i_sensor=0)    { return getAngleRatio(i_sensor) * kDegPerRev; }
    
    /**
     * You get the angles from two UpdateAngleBuffer() calls before
     * @return: angle in radians
     */
    float getAngleRadians(int i_sensor=0)    { return getAngleRatio(i_sensor) * kRadPerRev; }
    
    /**
     * Set direction for a sensor
     * @param i_sensor: id of sensor for which the offset is to be set
     * @param dir: true positive, false negative
     * @return: true if i_sensor in [0,kNumSensor_)
     */
    bool setDirection(int i_sensor, bool dir) 
    {
        if (i_sensor>-1 and i_sensor<kNumSensors_) {
            directions_[i_sensor] = dir;
            return true;
        }
        return false;
    }
    
    /**
     * Set direction for the first sensor
     * @param dir: true positive, false negative
     * @return: true if i_sensor in [0,kNumSensor_)
     */
    bool setDirection(bool dir) 
    {
        return setDirection(0,dir);
    }
    
    
    /**
     * Set offset for a sensor
     * @param i_sensor: id of sensor for which the offset is to be set
     * @param offset: offset in counts [0,2**14-1]
     * @return: true if i_sensor in [0,kNumSensor_)
     */
    bool setOffset(int i_sensor, uint16_t offset) 
    {
        if (i_sensor>-1 and i_sensor<kNumSensors_) {
            angle_offset_[i_sensor] = offset;
            return true;
        }
        return false;
    }
    
    /**
     * Set offset for the first sensor
     * @param offset: offset in counts [0,2**14-1]
     * @return: true if i_sensor in [0,kNumSensor_)
     */
    bool setOffset(uint16_t offset) { return setOffset(0,offset); }
    
    /**
     * Set offset for a sensor
     * @param i_sensor: id of sensor for which the offset is to be set
     * @param offset_ratio: offset in ratio in [0,1]
     * @return: true if i_sensor in [0,kNumSensor_)
     */
    bool setOffsetRatio (int i_sensor, float offset_ratio) 
    {
        return setOffset(i_sensor,offset_ratio*kCountsPerRev);
    }
    
    /**
     * Set offset for the first sensor
     * @param offset_ratio: offset in ratio in [0,1]
     * @return: true if i_sensor in [0,kNumSensor_)
     */
    bool setOffsetRatio(float offset_ratio) 
    { 
        return setOffsetRatio(0,offset_ratio); 
    }
    
    /**
     * Set offset for a sensor
     * @param i_sensor: id of sensor for which the offset is to be set
     * @param offset_degrees: offset in degrees in [0,360]
     * @return: true if i_sensor in [0,kNumSensor_)
     */
    bool setOffsetDegrees(int i_sensor, float offset_degrees) 
    {
        return setOffsetRatio(i_sensor,offset_degrees / kDegPerRev);
    }
    
    /**
     * Set offset for the first sensor
     * @param offset_degrees: offset in degrees in [0,360]
     * @return: true if i_sensor in [0,kNumSensor_)
     */
    bool setOffsetDegrees(float offset_degrees) 
    {
        return setOffsetDegrees(0, offset_degrees);
    }
    
    /**
     * Set offset for a sensor
     * @param i_sensor: id of sensor for which the offset is to be set
     * @param offset_radians: offset in radians in [0,2*pi]
     * @return: true if i_sensor in [0,kNumSensor_)
     */
    bool setOffsetRadians(int i_sensor, float offset_radians) 
    {
        return setOffsetRatio(i_sensor, offset_radians / kRadPerRev);
    }
    
    /**
     * Set offset for the first sensor
     * @param offset_radians: offset in radians in [0,2*pi]
     * @return: true if i_sensor in [0,kNumSensor_)
     */
    bool setOffsetRadians(float offset_radians) 
    {
        return setOffsetRadians(0, offset_radians);
    }
    
    
   

protected:



    /**
     * Select (low) chip, and wait 1 us (at least 350 ns)
     */
    void SelectChip()   { chip_.write(0); wait_us(1); }
    
    /**
     * Deselect (high) chip, and wait 1 us (at least 350 ns)
     */
    void DeselectChip() { chip_.write(1); wait_us(1); }

    /**
     * SPI transfer between each of the daisy chained sensors
     * @param cmd: Command to send
     */
    void Transfer(As5048Command cmd) 
    {
        SelectChip();
        for(int i=0; i<kNumSensors_; ++i){
            read_buffer_[i]  = spi_.write(cmd>>8) << 8;
            read_buffer_[i] |= spi_.write(cmd & 0x00FF);
        }
        DeselectChip();
        last_command_ = cmd;
    }

    const int kNumSensors_;     // number of sensors in daisy chain
    DigitalOut chip_;           // chip select port
    SPI spi_;                   // mbed spi communiation object
    
    uint16_t* read_buffer_;     // buffer for results from last transfer
    uint16_t* angle_buffer_;    // buffer for angle results from last transfer
    uint16_t* angle_offset_;    // offset array for each sensor
    bool* directions_;          // direction true positive, false negative
    
    As5048Command last_command_;// command sent during last Transfer
    
};
#endif