#include <Arduino.h>
#include "Motor.h" //Das hier ist der TMC5160
#include <AS5048A.h>
#include <ros.h>
#include <lcr_controller/jointPosition.h>
#include <std_msgs/Empty.h>
#include <ESP32CAN.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include "ws2812.h"
#include <EEPROM.h>
#include <PID_v1.h>

#define PI 3.1415926535897932384626433832795
#define PIN_CS_TMC 15
#define PIN_EN_TMC 16
#define PIN_CS_INT_AMS 17
#define PIN_CS_EXT_AMS 18
#define PIN_NEOPIXEL 32
#define PIN_JOINT_ID_BIT3 33
#define PIN_JOINT_ID_BIT2 25
#define PIN_JOINT_ID_BIT1 26
#define PIN_JOINT_ID_BIT0 27
#define EEPROM_SIZE 64
#define EEPROM_ADDRESS_ENABLE_STARTUP 1    //If the Integer at this Address = 1, the "Nullpunktfahrt" is enabled at startup
#define EEPROM_ADDRESS_AS5048_OFFSET 10    //The magnet Offset is saved at the Adresses 10-13 as long
#define TIME_MS_TIMEOUT_NULLPUNKTFART 7500 //The maximum amount of time per axis for the Nullpunktfahrt.
#define TIME_MS_PUBLISH_FREQUENCY 19 // A bit more than 50Hz
#define TIME_MS_TIMEOUT_STOP 300 //If there is no command for 300ms, the Motor is automatically switched off.

int g_this_joint = 1;
float g_motor_transmission[6] = {3.5, -84.48, -37.77, -4.75, -3, 1};
int g_as_sign[6] = {1, 1, -1, 1, 1, 1};
boolean g_high_motor_current[6] = {1, 1, 1, 0, 1, 0};
int g_enable_Nullpunktfahrt = 0;

long g_target_velocities[6];
long g_actual_velocities[6];
long g_actual_angles[6];
int g_state = 0; //0 = Off+LED_Red; 1 = normal operation; 2 = going to initial position
long g_ms_time_differenze_publish = 0;
uint8_t g_led_rgb[3] = {1, 1, 1};
boolean g_tx_frame_master2slave_send = false; //Is true, when there is a Can-packet, that should be sent.
int g_init_order[6] = {2, 3, 4, 5, 1, 6};
uint8_t g_ma2sl_led_red = 0;

typedef union { // This structure is used to easily convert 2 long values into 8 uint8_t values for CAN communication
  struct
  {
    long angle;
    long velocity;
  };
  uint8_t data[8];
} convert_ll2c;
convert_ll2c g_slave2master;

typedef union { // This structure is used to easily convert 4 int values and 1 long value into 8 uint8_t values for CAN communication
  struct
  {
    uint8_t operation_ident; //0 = off, 1=normal operation, 2=Do Nullpunktfahrt
    uint8_t reserved;
    uint8_t led_green;
    uint8_t led_blue;
    long target_velocity;
  };
  uint8_t data[8];
} convert_iiiil2c;
convert_iiiil2c g_master2slave;

typedef union { // This structure is used to easily convert 2 int values and 3 uint16 value into 8 uint8_t values for CAN communication
  struct
  {
    uint8_t operation; //0 is nothing, 1 is save the tobesaved_offset_value, 2 is to start the calibraton sequence, 3 is to end. In the Calibration Sequence the Motor is deactivated! 4 is to enable the Nullpunktfahrt, 5 is to disable, 6 is to answer with the actual Angle (not RAW!), 7 is to do Nullpunktfahrt
    uint8_t answer_to_this;
    uint16_t current_offset_value;
    uint16_t tobesaved_offset_value;
    uint16_t raw_magnet_angle;
  };
  uint8_t data[8];
} convert_config;
convert_config g_platine2config;

CAN_device_t CAN_cfg;
/*
Can ID List: 
2:Master2Slave for Joint 2
3:Master2Slave for Joint 3
4:Master2Slave for Joint 4
5:Master2Slave for Joint 5
6:Master2Slave for Joint 6
12:Slave2Master from Joint 2
13:Slave2Master from Joint 3
14:Slave2Master from Joint 4
15:Slave2Master from Joint 5
16:Slave2Master from Joint 6
21:platine2config to Joint 1
22:platine2config to Joint 2
23:platine2config to Joint 3
24:platine2config to Joint 4
25:platine2config to Joint 5
26:platine2config to Joint 6
27:platine2config to calibrationMaster from the previously adressed Joint
*/
Motor tmc = Motor(PIN_CS_TMC, PIN_EN_TMC, 10000, 10000, false);
AS5048A obj_angleSensor(PIN_CS_EXT_AMS);
rgbVal *obj_pixels;
double g_Input_obj_init_PID, g_Output_obj_init_PID, g_Setpoint_obj_init_PID;
PID obj_init_PID(&g_Input_obj_init_PID, &g_Output_obj_init_PID, &g_Setpoint_obj_init_PID, 30, 0.05, 0, P_ON_E, DIRECT);

lcr_controller::jointPosition allAngles;
lcr_controller::jointPosition allVelocities;
ros::NodeHandle nh;
ros::Publisher pub_angles("actual_angles", &allAngles);
ros::Publisher pub_velocities("actual_velocities", &allVelocities);

//This method gehts called, if there is a new message in "joint_velocity"
void sub_velocity(const lcr_controller::jointPosition &arm_steps)
{
  g_target_velocities[0] = arm_steps.joint1;
  g_target_velocities[1] = arm_steps.joint2;
  g_target_velocities[2] = arm_steps.joint3;
  g_target_velocities[3] = arm_steps.joint4;
  g_target_velocities[4] = arm_steps.joint5;
  g_target_velocities[5] = arm_steps.joint6;
  g_master2slave.operation_ident = 1; //Normal Operation!
  CAN_frame_t tx_frame;
  for (int joint = 1; joint < 6; joint++)
  {
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = joint + 1;
    tx_frame.FIR.B.DLC = 8;
    g_master2slave.target_velocity = g_target_velocities[joint];
    tx_frame.data.u8[0] = g_master2slave.data[0];
    tx_frame.data.u8[1] = g_master2slave.data[1];
    tx_frame.data.u8[2] = g_master2slave.data[2];
    tx_frame.data.u8[3] = g_master2slave.data[3];
    tx_frame.data.u8[4] = g_master2slave.data[4];
    tx_frame.data.u8[5] = g_master2slave.data[5];
    tx_frame.data.u8[6] = g_master2slave.data[6];
    tx_frame.data.u8[7] = g_master2slave.data[7];
    ESP32Can.CANWriteFrame(&tx_frame);
  }
  g_target_velocities[0] = g_target_velocities[0] * g_motor_transmission[g_this_joint - 1];
  tmc.set_velocity(g_target_velocities[0]);
  g_ma2sl_led_red = 0;
}
ros::Subscriber<lcr_controller::jointPosition> sub_vel("target_velocity", sub_velocity);

void sub_blueCallback(const std_msgs::UInt8 &led_msg)
{
  g_master2slave.led_blue = led_msg.data;
}
ros::Subscriber<std_msgs::UInt8> sub_ledblue("helene_led_blue", sub_blueCallback);

void sub_greenCallback(const std_msgs::UInt8 &led_msg)
{
  g_master2slave.led_green = led_msg.data;
}
ros::Subscriber<std_msgs::UInt8> sub_ledgreen("helene_led_green", sub_greenCallback);

//This axis does a Nullpunktfahrt
void this_axis_do_Nullpunktfahrt(boolean l_thisjointisatgoal, long l_starttime)
{
  obj_init_PID.SetMode(AUTOMATIC);
  obj_init_PID.SetOutputLimits(-5000, 5000);
  g_Setpoint_obj_init_PID = 8192;
  while (l_thisjointisatgoal == false && millis() - l_starttime < TIME_MS_TIMEOUT_NULLPUNKTFART)
  {
    obj_pixels[0] = makeRGBVal(0, 125 + 125 * sin(millis() / 50), 0);
    ws2812_setColors(1, obj_pixels);
    g_Input_obj_init_PID = double((obj_angleSensor.getRotation() * g_as_sign[g_this_joint - 1] + 8192));
    obj_init_PID.Compute();
    tmc.set_velocity(long(g_Output_obj_init_PID * g_motor_transmission[g_this_joint - 1]));
    if (abs(obj_angleSensor.getRotation()) < 5 && abs(tmc.get_vtarget()) < abs(25 * g_motor_transmission[g_this_joint - 1]))
    { //Goal is reached
      tmc.set_velocity(0);
      l_thisjointisatgoal = true;
    }
    delay(10);
  }
  obj_pixels[0] = makeRGBVal(0, 0, 0);
  ws2812_setColors(1, obj_pixels);
}

void setup()
{
  tmc.disable_motor();
  pinMode(PIN_CS_INT_AMS, OUTPUT);
  pinMode(PIN_CS_EXT_AMS, OUTPUT);
  pinMode(PIN_JOINT_ID_BIT3, INPUT_PULLUP);
  pinMode(PIN_JOINT_ID_BIT2, INPUT_PULLUP);
  pinMode(PIN_JOINT_ID_BIT1, INPUT_PULLUP);
  pinMode(PIN_JOINT_ID_BIT0, INPUT_PULLUP);
  digitalWrite(PIN_CS_INT_AMS, HIGH);
  digitalWrite(PIN_CS_EXT_AMS, HIGH);
  delay(1);

  //Decode the Joint-ID
  g_this_joint = !digitalRead(PIN_JOINT_ID_BIT0) * 1 + !digitalRead(PIN_JOINT_ID_BIT1) * 2 + !digitalRead(PIN_JOINT_ID_BIT2) * 4 + !digitalRead(PIN_JOINT_ID_BIT3) * 8;

  //Neopixel initialisation
  ws2812_init(PIN_NEOPIXEL, LED_WS2812B);
  obj_pixels = (rgbVal *)malloc(sizeof(rgbVal) * 1); //1 ist die Anzahl der Neopixel. Wir haben halt nur einen
  obj_pixels[0] = makeRGBVal(0, 0, 0);               // Rot am Anfang
  ws2812_setColors(1, obj_pixels);

  //Check if JointID is valid
  if (g_this_joint != 1)
  {
    while (true)
    {
      obj_pixels[0] = makeRGBVal(255, 0, 0);
      ws2812_setColors(1, obj_pixels);
      delay(200);
      obj_pixels[0] = makeRGBVal(0, 0, 0);
      ws2812_setColors(1, obj_pixels);
      delay(2000);
    }
  }

  //Start the EEprom
  EEPROM.begin(EEPROM_SIZE);

  //Read the important EEprom values
  g_enable_Nullpunktfahrt = EEPROM.read(EEPROM_ADDRESS_ENABLE_STARTUP);
  uint16_t l_ASoffset = EEPROM.readShort(EEPROM_ADDRESS_AS5048_OFFSET);

  //Check if the values are valid. If not overwrite the saved values
  if (g_enable_Nullpunktfahrt != 0 && g_enable_Nullpunktfahrt != 1)
  { //Overwrite if wrong
    g_enable_Nullpunktfahrt = 0;
    EEPROM.write(EEPROM_ADDRESS_ENABLE_STARTUP, 0);
    EEPROM.commit();
  }
  if (l_ASoffset > 16384)
  { // Overwrite if Offset is to high
    l_ASoffset = 0;
    EEPROM.writeShort(EEPROM_ADDRESS_AS5048_OFFSET, 0);
    EEPROM.commit();
  }

  //CAN initialisation. We start the CAN Interface with 500kbps, using GPIOs 4 and 5 and with a que of 100 elements
  CAN_cfg.speed = CAN_SPEED_200KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(100, sizeof(CAN_frame_t));
  ESP32Can.CANInit();

  //Set the motor current and enable motor. However it would not be great, if all motors would start at the same time. So first axis 1, then 2, then 3... are started.
  for (int i = 0; i < g_this_joint; i++)
  {
    obj_pixels[0] = makeRGBVal(0, 0, 255);
    ws2812_setColors(1, obj_pixels);
    delay(200);
    obj_pixels[0] = makeRGBVal(0, 0, 0);
    ws2812_setColors(1, obj_pixels);
    delay(200);
  }
  tmc.enable_motor();
  tmc.set_current(g_high_motor_current[g_this_joint - 1]);
  for (int i = g_this_joint; i < 6; i++)
  {
    delay(400);
  }
  obj_pixels[0] = makeRGBVal(255, 0, 0);
  ws2812_setColors(1, obj_pixels);

  //Start the magnetic rotary encoder and set the zero Position
  obj_angleSensor.init();
  obj_angleSensor.setZeroPosition(l_ASoffset);
    //check if the angle sensor works
  boolean l_boolean_isnotzero = false;
  for(int i=0; i< 20; i++){ //One Reading in 20 should not be 0 due to noise. If not -> Sensor is missing!
    if(obj_angleSensor.getRawRotation() != 0){
      l_boolean_isnotzero = true;
    }
  }
  if(l_boolean_isnotzero == false)//If all values have been 0 -> Error blinking
  {
    while (true)
    {
      obj_pixels[0] = makeRGBVal(255, 0, 0);
      ws2812_setColors(1, obj_pixels);
      delay(200);
      obj_pixels[0] = makeRGBVal(0, 0, 0);
      ws2812_setColors(1, obj_pixels);
      delay(200);
      obj_pixels[0] = makeRGBVal(255, 0, 0);
      ws2812_setColors(1, obj_pixels);
      delay(200);
      obj_pixels[0] = makeRGBVal(0, 0, 0);
      ws2812_setColors(1, obj_pixels);
      delay(2000);
    }
  }

  //Wait a bit to settle
  delay(1500);

  //Now do the Nullpunktfahrt, if enabled
  if (g_enable_Nullpunktfahrt == 1)
  { //Blink green if enabled
    obj_pixels[0] = makeRGBVal(0, 255, 0);
    ws2812_setColors(1, obj_pixels);
  }
  else
  {
    obj_pixels[0] = makeRGBVal(0, 0, 255);
    ws2812_setColors(1, obj_pixels);
  }
  delay(250);
  obj_pixels[0] = makeRGBVal(0, 0, 0);
  ws2812_setColors(1, obj_pixels);
  g_Setpoint_obj_init_PID = 0;

  if (g_enable_Nullpunktfahrt == 1) //Now make the Nullpunkfahrt, if enabled
  {
    boolean l_finished = false;
    int l_nullpunktfahrt_jointid = 1;
    while (l_finished == false)
    { //Hier gehe ich nacheinander die Joints ab. Die Nullpunktfahrt Reihenfolge steht dabei in g_init_order
      int l_actualjoint = g_init_order[l_nullpunktfahrt_jointid - 1];
      if (l_actualjoint == g_this_joint) //This joint has to do the Nullpunktfahrt
      {
        boolean l_thisjointisatgoal = false;
        long l_starttime = millis();
        this_axis_do_Nullpunktfahrt(l_thisjointisatgoal, l_starttime);
      }
      else if (l_actualjoint > 1 || l_actualjoint < 7) //Another Joint has to be driven to 0. I need to communicate via CAN with them
      {
        g_master2slave.operation_ident = 2; //Nullpunktsfahrt!
        g_master2slave.target_velocity = 0;
        CAN_frame_t tx_frame;
        tx_frame.FIR.B.FF = CAN_frame_std;
        tx_frame.MsgID = l_actualjoint;
        tx_frame.FIR.B.DLC = 8;
        tx_frame.data.u8[0] = g_master2slave.data[0];
        tx_frame.data.u8[1] = g_master2slave.data[1];
        tx_frame.data.u8[2] = g_master2slave.data[2];
        tx_frame.data.u8[3] = g_master2slave.data[3];
        tx_frame.data.u8[4] = g_master2slave.data[4];
        tx_frame.data.u8[5] = g_master2slave.data[5];
        tx_frame.data.u8[6] = g_master2slave.data[6];
        tx_frame.data.u8[7] = g_master2slave.data[7];
        ESP32Can.CANWriteFrame(&tx_frame);
        //Jetzt sollte ich noch warten
        xQueueReset(CAN_cfg.rx_queue); //Clear the CAN-Queue
        boolean l_thisjointisatgoal = false;
        long l_starttime = millis(); //Auch immer mit Timeout
        while (l_thisjointisatgoal == false && millis() - l_starttime < TIME_MS_TIMEOUT_NULLPUNKTFART)
        {
          CAN_frame_t rx_frame;
          while (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) //If i am receiving a CAN Massage from this joint again, i know, that the procedure was successfull
          {
            if (rx_frame.FIR.B.RTR != CAN_RTR && rx_frame.MsgID == (10 + l_actualjoint)) //Only let the important massage through
            {
              l_thisjointisatgoal = true;
            }
          }
        }
      }
      l_nullpunktfahrt_jointid++;
      if (l_nullpunktfahrt_jointid > 6)
      {
        l_finished = true;
      }
    }
  }

  //Start the nodeHanlde. We set the BaudRate to 1000000 and initialize the topics.
  nh.getHardware()->setBaud(1000000);
  nh.initNode();
  nh.subscribe(sub_vel);
  nh.subscribe(sub_ledblue);
  nh.subscribe(sub_ledgreen);
  nh.advertise(pub_angles);
  nh.advertise(pub_velocities);
}

void loop()
{
  CAN_frame_t rx_frame;
  //receive next CAN frame from queue
  while (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
  {
    if (rx_frame.FIR.B.RTR != CAN_RTR && rx_frame.MsgID >= 10 && rx_frame.MsgID <= 19) //I receive a can packet. ID-10 = The Joint where it came from. The data includes the angle an the velocity of a single joint.
    {
      g_slave2master.data[0] = rx_frame.data.u8[0];
      g_slave2master.data[1] = rx_frame.data.u8[1];
      g_slave2master.data[2] = rx_frame.data.u8[2];
      g_slave2master.data[3] = rx_frame.data.u8[3];
      g_slave2master.data[4] = rx_frame.data.u8[4];
      g_slave2master.data[5] = rx_frame.data.u8[5];
      g_slave2master.data[6] = rx_frame.data.u8[6];
      g_slave2master.data[7] = rx_frame.data.u8[7];
      g_actual_angles[rx_frame.MsgID - 11] = g_slave2master.angle;
      g_actual_velocities[rx_frame.MsgID - 11] = g_slave2master.velocity;
    }
    else if (rx_frame.FIR.B.RTR != CAN_RTR && rx_frame.MsgID == 20 + g_this_joint) //I received a config packet for this specific axis
    {
      g_platine2config.data[0] = rx_frame.data.u8[0];
      g_platine2config.data[1] = rx_frame.data.u8[1];
      g_platine2config.data[2] = rx_frame.data.u8[2];
      g_platine2config.data[3] = rx_frame.data.u8[3];
      g_platine2config.data[4] = rx_frame.data.u8[4];
      g_platine2config.data[5] = rx_frame.data.u8[5];
      g_platine2config.data[6] = rx_frame.data.u8[6];
      g_platine2config.data[7] = rx_frame.data.u8[7];
      boolean l_thisjointisatgoal = false;
      long l_starttime = millis();
      //g_platine2config.operation: 0 is nothing, 1 is save the tobesaved_offset_value, 2 is to start the calibraton sequence, 3 is to end. In the Calibration Sequence the Motor is deactivated! 4 is to enable the Nullpunktfahrt, 5 is to disable,6 is to answer with actual pos instead of RAW, 7 is to do Nullpunktfahrt
      switch (g_platine2config.operation)
      {
      case 1:
        EEPROM.writeShort(EEPROM_ADDRESS_AS5048_OFFSET, g_platine2config.tobesaved_offset_value);
        EEPROM.commit();
        obj_angleSensor.setZeroPosition(g_platine2config.tobesaved_offset_value);
        break;
      case 2:
        tmc.disable_motor();
        break;
      case 3:
        tmc.enable_motor();
        break;
      case 4:
        g_enable_Nullpunktfahrt = 1;
        EEPROM.write(EEPROM_ADDRESS_ENABLE_STARTUP, 1);
        EEPROM.commit();
        break;
      case 5:
        g_enable_Nullpunktfahrt = 0;
        EEPROM.write(EEPROM_ADDRESS_ENABLE_STARTUP, 0);
        EEPROM.commit();
        break;
      case 7:
        this_axis_do_Nullpunktfahrt(l_thisjointisatgoal, l_starttime);
        break;
      default:
        break;
      }
      if (g_platine2config.answer_to_this == 1)
      {
        CAN_frame_t l_tx_frame;
        g_platine2config.current_offset_value = EEPROM.readShort(EEPROM_ADDRESS_AS5048_OFFSET);
        if (g_platine2config.operation == 6)
        {
          g_platine2config.raw_magnet_angle = obj_angleSensor.getRotation();
        }
        else
        {
          g_platine2config.raw_magnet_angle = obj_angleSensor.getRawRotation();
        }
        l_tx_frame.data.u8[0] = g_platine2config.data[0];
        l_tx_frame.data.u8[1] = g_platine2config.data[1];
        l_tx_frame.data.u8[2] = g_platine2config.data[2];
        l_tx_frame.data.u8[3] = g_platine2config.data[3];
        l_tx_frame.data.u8[4] = g_platine2config.data[4];
        l_tx_frame.data.u8[5] = g_platine2config.data[5];
        l_tx_frame.data.u8[6] = g_platine2config.data[6];
        l_tx_frame.data.u8[7] = g_platine2config.data[7];
        l_tx_frame.FIR.B.FF = CAN_frame_std;
        l_tx_frame.MsgID = 27;
        l_tx_frame.FIR.B.DLC = 8;
        ESP32Can.CANWriteFrame(&l_tx_frame);
      }
    }
  }
  if (millis() - g_ms_time_differenze_publish >= TIME_MS_PUBLISH_FREQUENCY) // Time to publish! Also LED and check Failsafe
  {
    g_ms_time_differenze_publish = millis();
    g_actual_angles[0] = obj_angleSensor.getRotation() * g_as_sign[g_this_joint - 1];
    allAngles.joint1 = g_actual_angles[0];
    allAngles.joint2 = g_actual_angles[1];
    allAngles.joint3 = g_actual_angles[2];
    allAngles.joint4 = g_actual_angles[3];
    allAngles.joint5 = g_actual_angles[4];
    allAngles.joint6 = g_actual_angles[5];
    pub_angles.publish(&allAngles);
    g_actual_velocities[0] = tmc.get_vactual() / g_motor_transmission[g_this_joint - 1];
    allVelocities.joint1 = g_actual_velocities[0];
    allVelocities.joint2 = g_actual_velocities[1];
    allVelocities.joint3 = g_actual_velocities[2];
    allVelocities.joint4 = g_actual_velocities[3];
    allVelocities.joint5 = g_actual_velocities[4];
    allVelocities.joint6 = g_actual_velocities[5];
    pub_velocities.publish(&allVelocities);
    if (millis() - tmc.get_last_time_of_set_velocity() > TIME_MS_TIMEOUT_STOP)
    { //Failsafe!
      g_target_velocities[0] = 0;
      tmc.set_velocity(g_target_velocities[0]);
      g_master2slave.led_green = 0;
      g_master2slave.led_blue = 0;
      g_ma2sl_led_red = 255;
    }
    if (g_led_rgb[0] != g_ma2sl_led_red || g_led_rgb[1] != g_master2slave.led_green || g_led_rgb[2] != g_master2slave.led_blue)
    { //Wenn LED Values anders, dann print
      g_led_rgb[0] = g_ma2sl_led_red;
      g_led_rgb[1] = g_master2slave.led_green;
      g_led_rgb[2] = g_master2slave.led_blue;
      //Print LED Values here!
      obj_pixels[0] = makeRGBVal(g_led_rgb[0], g_led_rgb[1], g_led_rgb[2]);
      ws2812_setColors(1, obj_pixels);
    }
  }
  nh.spinOnce(); //Spin ROS-communication
}