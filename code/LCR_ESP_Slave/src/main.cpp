#include <Arduino.h>
#include <math.h>
#include "Motor.h" //Das hier ist der TMC5160
#include <AS5048A.h>
#include <ESP32CAN.h>
#include "ws2812.h"
#include <EEPROM.h>
#include <PID_v1.h>
#include <scpiparser.h>

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
#define TIME_MS_PUBLISH_FREQUENCY 13
#define TIME_MS_TIMEOUT_STOP 300 //If there is no command for 300ms, the Motor is automatically switched off.

int g_this_joint = 0;

float g_motor_transmission[6] = {3.5, -84.48, -37.77, -4.75, -3, 1};
int g_as_sign[6] = {1, 1, -1, 1, 1, 1};
boolean g_high_motor_current[6] = {1, 1, 1, 0, 1, 0};
int g_enable_Nullpunktfahrt;

long g_target_velocity;
long g_actual_velocity;
long g_actual_angle;
int g_state = 0; //0 = Off+LED_Red; 1 = normal operation; 2 = going to initial position
long g_ms_time_differenze_publish = 0;
uint8_t g_led_rgb[3] = {0, 0, 0};
boolean g_tx_frame_slave2master_send = false; //Is true, when there is a Can-packet, that should be sent.
boolean g_tx_frame_slave2config_send = false; //Is true, when there is a Can-packet, that should be sent.
uint8_t g_ma2sl_led_red;

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
    uint8_t operation_ident;
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
    uint8_t operation; //0 is nothing, 1 is save the tobesaved_offset_value, 2 is to start the calibraton sequence, 3 is to end. In the Calibration Sequence the Motor is deactivated! 4 is to enable the Nullpunktfahrt, 5 is to disable, 6 is to answer with the actual Angle (not RAW!), 7 is to do Nullpunkfahrt
    uint8_t answer_to_this;
    uint16_t current_offset_value;
    uint16_t tobesaved_offset_value;
    uint16_t raw_magnet_angle;
  };
  uint8_t data[8];
} convert_config;
convert_config g_platine2config;

CAN_device_t CAN_cfg;
CAN_frame_t g_rx_frame_master2slave;
CAN_frame_t g_tx_frame_slave2master;
CAN_frame_t g_tx_frame_slave2config;
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
AS5048A obj_angleSensor(PIN_CS_INT_AMS);
rgbVal *obj_pixels;
double g_Input_obj_init_PID, g_Output_obj_init_PID, g_Setpoint_obj_init_PID;
PID obj_init_PID(&g_Input_obj_init_PID, &g_Output_obj_init_PID, &g_Setpoint_obj_init_PID, 30, 0.05, 0, P_ON_E, DIRECT);

//SCPI Stuff
struct scpi_parser_context ctx;

scpi_error_t identify(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t setzenullpunktfahrt(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t get_calvalueofoint(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t get_actlmagvalofjoint(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t get_rawvalofjoint(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t set_calvalueofoint(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t caljoint(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t do_nullpunktfahrt(struct scpi_parser_context *context, struct scpi_token *command);

TaskHandle_t canwriteframe_TaskHnd;

void canwriteframe_Task(void *parameter) //This Task runs on a different core and is only responsible for pushing the can packet
{
  for (;;)
  {
    if (g_tx_frame_slave2master_send == true)
    {
      ESP32Can.CANWriteFrame(&g_tx_frame_slave2master);
      g_tx_frame_slave2master_send = false; // Set it to false afterwards, so that a new message can be sent
    }
    if (g_tx_frame_slave2config_send == true)
    {
      ESP32Can.CANWriteFrame(&g_tx_frame_slave2config);
      g_tx_frame_slave2config_send = false;
    }
    delayMicroseconds(5);
    yield();
  }
}

//This axis does a Nullpunktfahrt
void this_axis_do_Nullpunktfahrt(boolean l_thisjointisatgoal, long l_starttime)
{
  obj_init_PID.SetMode(AUTOMATIC);
  obj_init_PID.SetOutputLimits(-5000, 5000);
  //obj_init_PID.SetSampleTime(9);
  g_Setpoint_obj_init_PID = 8192;
  if (g_this_joint == 3)
  {
    g_Setpoint_obj_init_PID = 8192 + 4096;
  }
  while (l_thisjointisatgoal == false && millis() - l_starttime < TIME_MS_TIMEOUT_NULLPUNKTFART) //PID Regler zum erreichen des Nullpunkts
  {
    obj_pixels[0] = makeRGBVal(0, 125 + 125 * sin(millis() / 50), 0);
    ws2812_setColors(1, obj_pixels);
    g_Input_obj_init_PID = double((obj_angleSensor.getRotation() * g_as_sign[g_this_joint - 1] + 8192));
    obj_init_PID.Compute();
    //Serial.println("Pos: " + String(g_Input_obj_init_PID) + "  I-Summe: " + String(obj_init_PID.GetOutputSum()));
    tmc.set_velocity(long(g_Output_obj_init_PID * g_motor_transmission[g_this_joint - 1]));
    int angle = obj_angleSensor.getRotation();
    if (g_this_joint == 3)
    {
      angle = angle + 4096;
    }
    if (abs(angle) < 10 && abs(tmc.get_vtarget()) < abs(25 * g_motor_transmission[g_this_joint - 1]))
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

  //All slaves are configured, to be able to configure all zeroPositions from all joints. To start the calibration, the serial interface is necessary
  Serial.begin(115200);
  Serial.println(String(F("Started Slave. Joint = ")) + String(g_this_joint));

  //SCPI initialisation
  scpi_init(&ctx);
  /*
  IDN?         -> identify (done)
  CAlsingleJOint 
  SEtNUll
  GEtCAlvalue
  */
  //Allgemein
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "*IDN?", 5, "IDN?", 4, identify);
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "CALSINGLEJOINT", 14, "CAJO", 4, caljoint);
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "SETNULL", 7, "SENU", 4, setzenullpunktfahrt);
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "GETCALVALUE", 11, "GECA", 4, get_calvalueofoint);
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "SETCALVALUE", 11, "SECA", 4, set_calvalueofoint);
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "GETACTUAL", 9, "GEAC", 4, get_actlmagvalofjoint);
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "GETRAW", 6, "GERA", 4, get_rawvalofjoint);
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "DONULL", 6, "DONU", 4, do_nullpunktfahrt);
  

  //Neopixel initialisation
  ws2812_init(PIN_NEOPIXEL, LED_WS2812B);
  obj_pixels = (rgbVal *)malloc(sizeof(rgbVal) * 1); //1 ist die Anzahl der Neopixel. Wir haben halt nur einen
  obj_pixels[0] = makeRGBVal(255, 0, 0);             // Rot am Anfang
  ws2812_setColors(1, obj_pixels);

  //Check if JointID is valid
  if (g_this_joint == 0 || g_this_joint == 1 || g_this_joint > 6)
  {
    Serial.println(String(F("Invalid Joint ID!")));
    while (true)
    {
      obj_pixels[0] = makeRGBVal(255, 0, 0);
      ws2812_setColors(1, obj_pixels);
      delay(200);
      obj_pixels[0] = makeRGBVal(0, 0, 0);
      ws2812_setColors(1, obj_pixels);
      delay(200);
    }
  }

  //CAN initialisation. We start the CAN Interface with 500kbps, using GPIOs 4 and 5 and with a que of 100 elements
  CAN_cfg.rx_queue = xQueueCreate(100, sizeof(CAN_frame_t));
  CAN_cfg.speed = CAN_SPEED_200KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  ESP32Can.CANInit();

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
  if (l_ASoffset > (16384))
  { // Overwrite if Offset is to high
    l_ASoffset = 0;
    EEPROM.writeShort(EEPROM_ADDRESS_AS5048_OFFSET, 0);
    EEPROM.commit();
  }

  //Start the magnetic rotary encoder and set the zero Position
  obj_angleSensor.init();
  obj_angleSensor.setZeroPosition(l_ASoffset);

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
  if (g_this_joint == 2 || g_this_joint == 3)
  { //Higher acceleration if axis 2 or 3. Due to the high transmission!
    tmc.set_acceleration(10000 * abs(g_motor_transmission[g_this_joint - 1]));
  }
  for (int i = g_this_joint; i < 6; i++)
  {
    delay(400);
  }
  obj_pixels[0] = makeRGBVal(255, 0, 0);
  ws2812_setColors(1, obj_pixels);

  //The function ESP32Can.CANWriteFrame is blocking, when an error occours. For that reason, a new task is created, so that the main program still functions.
  xTaskCreatePinnedToCore(canwriteframe_Task, "canwriteframe_Task", 5000, NULL, 1, &canwriteframe_TaskHnd, 0);

  disableCore0WDT();
  //disableCore1WDT();

  //Wait a bit to settle
  delay(100);
}

void loop()
{
  while (xQueueReceive(CAN_cfg.rx_queue, &g_rx_frame_master2slave, 0) == pdTRUE) //I just received a CAN-packet
  {
    if (g_rx_frame_master2slave.FIR.B.RTR != CAN_RTR && g_rx_frame_master2slave.MsgID == g_this_joint) //I received a command package
    {
      g_master2slave.data[0] = g_rx_frame_master2slave.data.u8[0];
      g_master2slave.data[1] = g_rx_frame_master2slave.data.u8[1];
      g_master2slave.data[2] = g_rx_frame_master2slave.data.u8[2];
      g_master2slave.data[3] = g_rx_frame_master2slave.data.u8[3];
      g_master2slave.data[4] = g_rx_frame_master2slave.data.u8[4];
      g_master2slave.data[5] = g_rx_frame_master2slave.data.u8[5];
      g_master2slave.data[6] = g_rx_frame_master2slave.data.u8[6];
      g_master2slave.data[7] = g_rx_frame_master2slave.data.u8[7];
      g_target_velocity = g_master2slave.target_velocity * g_motor_transmission[g_this_joint - 1];
      g_state = g_master2slave.operation_ident;
      g_ma2sl_led_red = 0;
      boolean l_thisjointisatgoal = false; //I cant initialise variables inside the switch-routine (Thanks C)
      long l_starttime = millis();
      switch (g_state)
      {
      case 0:
        tmc.set_velocity(0);
        break;
      case 1:
        tmc.set_velocity(g_target_velocity);
        break;
      case 2:
        if (g_enable_Nullpunktfahrt == 1) //case 2 is to do the Nullpunktfahrt. Only start it, if globally enabled!
        {
          this_axis_do_Nullpunktfahrt(l_thisjointisatgoal,l_starttime);
        }
        else
        {
          tmc.set_velocity(0);
        }
        g_state = 0;
        break;
      default:
        tmc.set_velocity(0);
        break;
      }
    }
    else if (g_rx_frame_master2slave.FIR.B.RTR != CAN_RTR && g_rx_frame_master2slave.MsgID == 20 + g_this_joint) //I received a config packet for this specific axis
    {
      g_platine2config.data[0] = g_rx_frame_master2slave.data.u8[0];
      g_platine2config.data[1] = g_rx_frame_master2slave.data.u8[1];
      g_platine2config.data[2] = g_rx_frame_master2slave.data.u8[2];
      g_platine2config.data[3] = g_rx_frame_master2slave.data.u8[3];
      g_platine2config.data[4] = g_rx_frame_master2slave.data.u8[4];
      g_platine2config.data[5] = g_rx_frame_master2slave.data.u8[5];
      g_platine2config.data[6] = g_rx_frame_master2slave.data.u8[6];
      g_platine2config.data[7] = g_rx_frame_master2slave.data.u8[7];
      boolean l_thisjointisatgoal = false; //I cant initialise variables inside the switch-routine (Thanks C)
      long l_starttime = millis();
      //g_platine2config.operation: 0 is nothing, 1 is save the tobesaved_offset_value, 2 is to start the calibraton sequence, 3 is to end. In the Calibration Sequence the Motor is deactivated! 4 is to enable the Nullpunktfahrt, 5 is to disable, 6 is to send actual Position instead of raw, 7 is to do Nullpunktfahrt.
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
        this_axis_do_Nullpunktfahrt(l_thisjointisatgoal,l_starttime);
        break;
      default:
        break;
      }
      if (g_platine2config.answer_to_this == 1)
      {
        g_platine2config.current_offset_value = EEPROM.readShort(EEPROM_ADDRESS_AS5048_OFFSET);
        if (g_platine2config.operation == 6)
        {
          g_platine2config.raw_magnet_angle = obj_angleSensor.getRotation();
        }
        else
        {
          g_platine2config.raw_magnet_angle = obj_angleSensor.getRawRotation();
        }
        g_tx_frame_slave2config.data.u8[0] = g_platine2config.data[0];
        g_tx_frame_slave2config.data.u8[1] = g_platine2config.data[1];
        g_tx_frame_slave2config.data.u8[2] = g_platine2config.data[2];
        g_tx_frame_slave2config.data.u8[3] = g_platine2config.data[3];
        g_tx_frame_slave2config.data.u8[4] = g_platine2config.data[4];
        g_tx_frame_slave2config.data.u8[5] = g_platine2config.data[5];
        g_tx_frame_slave2config.data.u8[6] = g_platine2config.data[6];
        g_tx_frame_slave2config.data.u8[7] = g_platine2config.data[7];
        g_tx_frame_slave2config.FIR.B.FF = CAN_frame_std;
        g_tx_frame_slave2config.MsgID = 27;
        g_tx_frame_slave2config.FIR.B.DLC = 8;
        g_tx_frame_slave2config_send = true;
      }
    }
  }
  if (millis() - g_ms_time_differenze_publish >= TIME_MS_PUBLISH_FREQUENCY) //Checke if Failsafe notwendig ist und sende einen CAN Frame zum Master mit den aktuellen Daten
  {
    g_ms_time_differenze_publish = millis();
    if (millis() - tmc.get_last_time_of_set_velocity() > TIME_MS_TIMEOUT_STOP && (g_state == 1 || g_state == 0))
    { //Failsafe!
      g_target_velocity = 0;
      if (tmc.get_vtarget() != 0)
      {
        tmc.set_velocity(g_target_velocity);
      }
      g_master2slave.led_green = 0;
      g_master2slave.led_blue = 0;
      g_ma2sl_led_red = 255;
    }

    if (g_tx_frame_slave2master_send == false) //Wenn nix gesendet wird, dann baue ich einen neuen CAN-Frame! Und sende ihn
    {
      g_slave2master.velocity = tmc.get_vactual() / g_motor_transmission[g_this_joint - 1];
      g_slave2master.angle = obj_angleSensor.getRotation() * g_as_sign[g_this_joint - 1];
      g_tx_frame_slave2master.FIR.B.FF = CAN_frame_std;
      g_tx_frame_slave2master.MsgID = g_this_joint + 10;
      g_tx_frame_slave2master.FIR.B.DLC = 8;
      g_tx_frame_slave2master.data.u8[0] = g_slave2master.data[0];
      g_tx_frame_slave2master.data.u8[1] = g_slave2master.data[1];
      g_tx_frame_slave2master.data.u8[2] = g_slave2master.data[2];
      g_tx_frame_slave2master.data.u8[3] = g_slave2master.data[3];
      g_tx_frame_slave2master.data.u8[4] = g_slave2master.data[4];
      g_tx_frame_slave2master.data.u8[5] = g_slave2master.data[5];
      g_tx_frame_slave2master.data.u8[6] = g_slave2master.data[6];
      g_tx_frame_slave2master.data.u8[7] = g_slave2master.data[7];
      g_tx_frame_slave2master_send = true;
    }

    if (g_led_rgb[0] != g_ma2sl_led_red || g_led_rgb[1] != g_master2slave.led_green || g_led_rgb[2] != g_master2slave.led_blue) //Leds setzen
    {
      g_led_rgb[0] = g_ma2sl_led_red;
      g_led_rgb[1] = g_master2slave.led_green;
      g_led_rgb[2] = g_master2slave.led_blue;
      //Print LED Values here!
      obj_pixels[0] = makeRGBVal(g_led_rgb[0], g_led_rgb[1], g_led_rgb[2]);
      ws2812_setColors(1, obj_pixels);
    }
  }
  yield();
  if (Serial.available() > 0) //Hier schaue ich ob Serielle Daten Kommen & wenn ja dann wird der Befehl nach SCPI durchsucht
  {
    char line_buffer[128];
    unsigned char read_length;
    /* Read in a line and execute it. */
    read_length = Serial.readBytesUntil('\n', line_buffer, 128);
    if (read_length > 0)
    {
      scpi_execute_command(&ctx, line_buffer, read_length);
    }
  }
}

int16_t calculatemagnetoffset(int16_t cal1, int16_t cal2) //This function calculates the zeroPosition of the AS5048A magnet encoder. It returns the zeroPosition and it gets the position of the two endstops of one joint
{
  int16_t output = 0;
  output = (cal1 + cal2 - 16384) / 2;
  if (output < 0)
  {
    output = output + 16384;
  }
  return output;
}

void setPowerstateofJoint(int jointid, boolean powerstate) //This function either activates or deactivates the motor of one joint. If powerstate == true the motor gets activated.
{
  long l_starttime = millis();
  boolean l_igottotransmit = false;
  g_platine2config.answer_to_this = 0;
  ;
  // g_platine2config.operation 2 is to start the calibraton sequence, 3 is to end
  if (powerstate == true)
  {
    g_platine2config.operation = 3;
  }
  else
  {
    g_platine2config.operation = 2;
  }
  while (l_igottotransmit == false && millis() - l_starttime < 500)
  {
    if (g_tx_frame_slave2master_send == false) //Wenn nix gesendet wird, dann baue ich einen neuen CAN-Frame! Und sende ihn
    {
      g_tx_frame_slave2master.FIR.B.FF = CAN_frame_std;
      g_tx_frame_slave2master.MsgID = 20 + jointid;
      g_tx_frame_slave2master.FIR.B.DLC = 8;
      g_tx_frame_slave2master.data.u8[0] = g_platine2config.data[0];
      g_tx_frame_slave2master.data.u8[1] = g_platine2config.data[1];
      g_tx_frame_slave2master.data.u8[2] = g_platine2config.data[2];
      g_tx_frame_slave2master.data.u8[3] = g_platine2config.data[3];
      g_tx_frame_slave2master.data.u8[4] = g_platine2config.data[4];
      g_tx_frame_slave2master.data.u8[5] = g_platine2config.data[5];
      g_tx_frame_slave2master.data.u8[6] = g_platine2config.data[6];
      g_tx_frame_slave2master.data.u8[7] = g_platine2config.data[7];
      g_tx_frame_slave2master_send = true;
      l_igottotransmit = true;
    }
  }
  if (l_igottotransmit == false)
  {
    Serial.println("Unable to transmit CAN Package");
  }
}

int16_t getRAWPositionofJoint(int jointid) //This function returns the raw angle of one Joint
{
  g_platine2config.answer_to_this = 1;
  g_platine2config.operation = 0;
  long l_starttime = millis();
  boolean l_gotanresponse = false;
  boolean l_igottotransmit = false;
  int16_t answer = 0;
  while (l_igottotransmit == false && millis() - l_starttime < 2000)
  {
    if (g_tx_frame_slave2master_send == false) //Wenn nix gesendet wird, dann baue ich einen neuen CAN-Frame! Und sende ihn
    {
      g_tx_frame_slave2master.FIR.B.FF = CAN_frame_std;
      g_tx_frame_slave2master.MsgID = 20 + jointid;
      g_tx_frame_slave2master.FIR.B.DLC = 8;
      g_tx_frame_slave2master.data.u8[0] = g_platine2config.data[0];
      g_tx_frame_slave2master.data.u8[1] = g_platine2config.data[1];
      g_tx_frame_slave2master.data.u8[2] = g_platine2config.data[2];
      g_tx_frame_slave2master.data.u8[3] = g_platine2config.data[3];
      g_tx_frame_slave2master.data.u8[4] = g_platine2config.data[4];
      g_tx_frame_slave2master.data.u8[5] = g_platine2config.data[5];
      g_tx_frame_slave2master.data.u8[6] = g_platine2config.data[6];
      g_tx_frame_slave2master.data.u8[7] = g_platine2config.data[7];
      g_tx_frame_slave2master_send = true;
      l_igottotransmit = true;
    }
  }
  while (l_gotanresponse == false && millis() - l_starttime < 2000)
  {                                                                                //Timeout after 1s
    while (xQueueReceive(CAN_cfg.rx_queue, &g_rx_frame_master2slave, 0) == pdTRUE) //I just received a CAN-packet
    {
      if (g_rx_frame_master2slave.FIR.B.RTR != CAN_RTR && g_rx_frame_master2slave.MsgID == 27)
      {
        g_platine2config.data[0] = g_rx_frame_master2slave.data.u8[0];
        g_platine2config.data[1] = g_rx_frame_master2slave.data.u8[1];
        g_platine2config.data[2] = g_rx_frame_master2slave.data.u8[2];
        g_platine2config.data[3] = g_rx_frame_master2slave.data.u8[3];
        g_platine2config.data[4] = g_rx_frame_master2slave.data.u8[4];
        g_platine2config.data[5] = g_rx_frame_master2slave.data.u8[5];
        g_platine2config.data[6] = g_rx_frame_master2slave.data.u8[6];
        g_platine2config.data[7] = g_rx_frame_master2slave.data.u8[7];
        l_gotanresponse = true;
        answer = g_platine2config.current_offset_value;
      }
    }
  }
  if (l_gotanresponse == true && l_igottotransmit == true)
  {
  }
  else
  {
    Serial.println(String(F("Timeout! The Joint did not answer or i could not send the CAN-packet")));
  }
  return g_platine2config.raw_magnet_angle;
}

//Antwortet auf *IDN?
scpi_error_t identify(struct scpi_parser_context *context, struct scpi_token *command)
{
  scpi_free_tokens(command);

  Serial.println(F("LCR Roboter"));
  Serial.println(F("Hardware: Felix Herbst"));
  Serial.println(F("Elektronik: Sven Suppelt"));
  Serial.println(F("Elektronik Softwareversion: V1.1"));
  return SCPI_SUCCESS;
}

//Cal a single Joint
scpi_error_t caljoint(struct scpi_parser_context *context, struct scpi_token *command)
{
  struct scpi_token *args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while (args != NULL && args->type == 0)
  {
    args = args->next;
  }

  output_numeric = scpi_parse_numeric(args->value, args->length, 1e3, 0, 25e6);
  if (output_numeric.length == 0 ||
      (output_numeric.length == 2 && output_numeric.unit[0] == 'H' && output_numeric.unit[1] == 'z'))
  {
    //Here i can do stuff. output_numeric.value is my Variable
    if (output_numeric.value == g_this_joint)
    {
      Serial.println(F("In 5 seconds I will deactivate the selected Joint."));
      Serial.println(F("After that you have 5 seconds until you have to move the joint to one of its endstops."));
      Serial.println(F("After that you will have to move the joint to the other endstop."));
      Serial.println(F("Deactivating in 5s."));
      delay(1000);
      Serial.println(F("Deactivating in 4s."));
      delay(1000);
      Serial.println(F("Deactivating in 3s."));
      delay(1000);
      Serial.println(F("Deactivating in 2s."));
      delay(1000);
      Serial.println(F("Deactivating in 1s."));
      delay(1000);
      tmc.disable_motor();
      Serial.println(F("Deactivated."));
      Serial.println(F("Please move the joint manually to one endstop. You have 5 seconds."));
      delay(5000);
      int16_t l_calvalue1 = obj_angleSensor.getRawRotation();
      Serial.println(String(F("Okay, i measured: ")) + String(l_calvalue1));
      Serial.println(F("Please move the joint manually to the other endstop. You have 5 seconds."));
      delay(5000);
      int16_t l_calvalue2 = obj_angleSensor.getRawRotation();
      Serial.println(String(F("Okay, i measured: ")) + String(l_calvalue2));
      int16_t l_offset = calculatemagnetoffset(l_calvalue1, l_calvalue2);
      Serial.println(String(F("The zeroPosition would be: ")) + String(l_offset));
      Serial.println(F("Can I save this value in the EEProm? Answer with 0/1. 1 = yes; 0=no."));
      Serial.setTimeout(60000);
      long temp;
      temp = (Serial.readStringUntil('\n')).toInt();
      if (temp == 1)
      {
        obj_angleSensor.setZeroPosition(l_offset);
        EEPROM.writeShort(EEPROM_ADDRESS_AS5048_OFFSET, l_offset);
        EEPROM.commit();
        Serial.println(F("I saved the value."));
      }
      Serial.println(F("In 5s i will activate the motor again. Maybe bring axis in zeroposition?"));
      delay(5000);
      tmc.enable_motor();
      Serial.println(F("Ok"));
    }
    else if (output_numeric.value > 0 && output_numeric.value < 7)
    {
      Serial.println(F("In 5 seconds I will deactivate the selected Joint."));
      Serial.println(F("After that you have 5 seconds until you have to move the joint to one of its endstops."));
      Serial.println(F("After that you will have to move the joint to the other endstop."));
      Serial.println(F("Deactivating in 5s."));
      delay(1000);
      Serial.println(F("Deactivating in 4s."));
      delay(1000);
      Serial.println(F("Deactivating in 3s."));
      delay(1000);
      Serial.println(F("Deactivating in 2s."));
      delay(1000);
      Serial.println(F("Deactivating in 1s."));
      delay(1000);
      setPowerstateofJoint(output_numeric.value, false);
      Serial.println(F("Deactivated."));
      Serial.println(F("Please move the joint manually to one endstop. You have 5 seconds."));
      delay(5000);
      int16_t l_calvalue1 = getRAWPositionofJoint(output_numeric.value);
      Serial.println(String(F("Okay, i measured: ")) + String(l_calvalue1));
      Serial.println(F("Please move the joint manually to the other endstop. You have 5 seconds."));
      delay(5000);
      int16_t l_calvalue2 = getRAWPositionofJoint(output_numeric.value);
      Serial.println(String(F("Okay, i measured: ")) + String(l_calvalue2));
      int16_t l_offset = calculatemagnetoffset(l_calvalue1, l_calvalue2);
      Serial.println(String(F("The zeroPosition would be: ")) + String(l_offset));
      Serial.println(F("Can I save this value in the EEProm? Answer with 0/1. 1 = yes; 0=no."));
      Serial.setTimeout(60000);
      long temp;
      temp = (Serial.readStringUntil('\n')).toInt();
      if (temp == 1)
      {
        g_platine2config.answer_to_this = 0;
        g_platine2config.operation = 1;
        g_platine2config.tobesaved_offset_value = l_offset;
        long l_starttime = millis();
        boolean l_igottotransmit = false;
        while (l_igottotransmit == false && millis() - l_starttime < 2000)
        {
          if (g_tx_frame_slave2master_send == false) //Wenn nix gesendet wird, dann baue ich einen neuen CAN-Frame! Und sende ihn
          {
            g_tx_frame_slave2master.FIR.B.FF = CAN_frame_std;
            g_tx_frame_slave2master.MsgID = 20 + output_numeric.value;
            g_tx_frame_slave2master.FIR.B.DLC = 8;
            g_tx_frame_slave2master.data.u8[0] = g_platine2config.data[0];
            g_tx_frame_slave2master.data.u8[1] = g_platine2config.data[1];
            g_tx_frame_slave2master.data.u8[2] = g_platine2config.data[2];
            g_tx_frame_slave2master.data.u8[3] = g_platine2config.data[3];
            g_tx_frame_slave2master.data.u8[4] = g_platine2config.data[4];
            g_tx_frame_slave2master.data.u8[5] = g_platine2config.data[5];
            g_tx_frame_slave2master.data.u8[6] = g_platine2config.data[6];
            g_tx_frame_slave2master.data.u8[7] = g_platine2config.data[7];
            g_tx_frame_slave2master_send = true;
            l_igottotransmit = true;
          }
        }
        if (l_igottotransmit == true)
        {
          Serial.println(F("I saved the value."));
        }
      }
      Serial.println(F("In 5s i will activate the motor again. Maybe bring axis in zeroposition?"));
      delay(5000);
      setPowerstateofJoint(output_numeric.value, true);
      Serial.println(F("Ok"));
    }
    else
    {
      Serial.println(F("Invalid Joint ID"));
    }
  }
  else
  {
    Serial.println(F("Error"));
    scpi_error error;
    error.id = -200;
    error.description = "Invalid unit";
    error.length = 26;

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  scpi_free_tokens(command);

  return SCPI_SUCCESS;
}

//Gets the actual cal. data of one joint
scpi_error_t get_calvalueofoint(struct scpi_parser_context *context, struct scpi_token *command)
{
  struct scpi_token *args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while (args != NULL && args->type == 0)
  {
    args = args->next;
  }

  output_numeric = scpi_parse_numeric(args->value, args->length, 1e3, 0, 25e6);
  if (output_numeric.length == 0 ||
      (output_numeric.length == 2 && output_numeric.unit[0] == 'H' && output_numeric.unit[1] == 'z'))
  {
    //Here i can do stuff. output_numeric.value is my Variable
    if (output_numeric.value == g_this_joint)
    {
      Serial.println(EEPROM.readShort(EEPROM_ADDRESS_AS5048_OFFSET));
    }
    else if (output_numeric.value > 0 && output_numeric.value < 7)
    {
      g_platine2config.answer_to_this = 1;
      g_platine2config.operation = 0;
      long l_starttime = millis();
      boolean l_gotanresponse = false;
      boolean l_igottotransmit = false;
      int16_t answer = 0;
      while (l_igottotransmit == false && millis() - l_starttime < 2000)
      {
        if (g_tx_frame_slave2master_send == false) //Wenn nix gesendet wird, dann baue ich einen neuen CAN-Frame! Und sende ihn
        {
          g_tx_frame_slave2master.FIR.B.FF = CAN_frame_std;
          g_tx_frame_slave2master.MsgID = 20 + output_numeric.value;
          g_tx_frame_slave2master.FIR.B.DLC = 8;
          g_tx_frame_slave2master.data.u8[0] = g_platine2config.data[0];
          g_tx_frame_slave2master.data.u8[1] = g_platine2config.data[1];
          g_tx_frame_slave2master.data.u8[2] = g_platine2config.data[2];
          g_tx_frame_slave2master.data.u8[3] = g_platine2config.data[3];
          g_tx_frame_slave2master.data.u8[4] = g_platine2config.data[4];
          g_tx_frame_slave2master.data.u8[5] = g_platine2config.data[5];
          g_tx_frame_slave2master.data.u8[6] = g_platine2config.data[6];
          g_tx_frame_slave2master.data.u8[7] = g_platine2config.data[7];
          g_tx_frame_slave2master_send = true;
          l_igottotransmit = true;
        }
      }
      while (l_gotanresponse == false && millis() - l_starttime < 2000)
      {                                                                                //Timeout after 1s
        while (xQueueReceive(CAN_cfg.rx_queue, &g_rx_frame_master2slave, 0) == pdTRUE) //I just received a CAN-packet
        {
          if (g_rx_frame_master2slave.FIR.B.RTR != CAN_RTR && g_rx_frame_master2slave.MsgID == 27)
          {
            g_platine2config.data[0] = g_rx_frame_master2slave.data.u8[0];
            g_platine2config.data[1] = g_rx_frame_master2slave.data.u8[1];
            g_platine2config.data[2] = g_rx_frame_master2slave.data.u8[2];
            g_platine2config.data[3] = g_rx_frame_master2slave.data.u8[3];
            g_platine2config.data[4] = g_rx_frame_master2slave.data.u8[4];
            g_platine2config.data[5] = g_rx_frame_master2slave.data.u8[5];
            g_platine2config.data[6] = g_rx_frame_master2slave.data.u8[6];
            g_platine2config.data[7] = g_rx_frame_master2slave.data.u8[7];
            l_gotanresponse = true;
            answer = g_platine2config.current_offset_value;
          }
        }
      }
      if (l_gotanresponse == true && l_igottotransmit == true)
      {
        Serial.println(answer);
      }
      else
      {
        Serial.println(String(F("Timeout! The Joint did not answer or i could not send the CAN-packet")));
        Serial.println("Transmit: " + String(l_igottotransmit) + " Response: " + String(l_gotanresponse));
      }
    }
    else
    {
      Serial.println(F("Invalid Joint number"));
    }
  }
  else
  {
    Serial.println(F("Error"));
    scpi_error error;
    error.id = -200;
    error.description = "Invalid unit";
    error.length = 26;

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  scpi_free_tokens(command);

  return SCPI_SUCCESS;
}

//Sets the actual cal. data of one joint
scpi_error_t set_calvalueofoint(struct scpi_parser_context *context, struct scpi_token *command)
{
  struct scpi_token *args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while (args != NULL && args->type == 0)
  {
    args = args->next;
  }

  output_numeric = scpi_parse_numeric(args->value, args->length, 1e3, 0, 25e6);
  if (output_numeric.length == 0 ||
      (output_numeric.length == 2 && output_numeric.unit[0] == 'H' && output_numeric.unit[1] == 'z'))
  {
    //Here i can do stuff. output_numeric.value is my Variable
    int16_t l_calvaluetobesaved;
    Serial.setTimeout(60000);
    Serial.println(F("Please send the Cal. value"));
    l_calvaluetobesaved = (Serial.readStringUntil('\n')).toInt();
    if (output_numeric.value == g_this_joint)
    {
      EEPROM.writeShort(EEPROM_ADDRESS_AS5048_OFFSET, l_calvaluetobesaved);
      EEPROM.commit();
      obj_angleSensor.setZeroPosition(l_calvaluetobesaved);
      Serial.println("Ok");
    }
    else if (output_numeric.value > 0 && output_numeric.value < 7)
    {
      g_platine2config.answer_to_this = 0;
      g_platine2config.operation = 1;
      g_platine2config.tobesaved_offset_value = l_calvaluetobesaved;
      long l_starttime = millis();
      boolean l_igottotransmit = false;
      while (l_igottotransmit == false && millis() - l_starttime < 2000)
      {
        if (g_tx_frame_slave2master_send == false) //Wenn nix gesendet wird, dann baue ich einen neuen CAN-Frame! Und sende ihn
        {
          g_tx_frame_slave2master.FIR.B.FF = CAN_frame_std;
          g_tx_frame_slave2master.MsgID = 20 + output_numeric.value;
          g_tx_frame_slave2master.FIR.B.DLC = 8;
          g_tx_frame_slave2master.data.u8[0] = g_platine2config.data[0];
          g_tx_frame_slave2master.data.u8[1] = g_platine2config.data[1];
          g_tx_frame_slave2master.data.u8[2] = g_platine2config.data[2];
          g_tx_frame_slave2master.data.u8[3] = g_platine2config.data[3];
          g_tx_frame_slave2master.data.u8[4] = g_platine2config.data[4];
          g_tx_frame_slave2master.data.u8[5] = g_platine2config.data[5];
          g_tx_frame_slave2master.data.u8[6] = g_platine2config.data[6];
          g_tx_frame_slave2master.data.u8[7] = g_platine2config.data[7];
          g_tx_frame_slave2master_send = true;
          l_igottotransmit = true;
        }
      }
      if (l_igottotransmit == true)
      {
        Serial.println("Ok");
      }
      else
      {
        Serial.println(String(F("Timeout! I could not send the CAN-packet")));
      }
    }
    else
    {
      Serial.println(F("Invalid Joint number"));
    }
  }
  else
  {
    Serial.println(F("Error"));
    scpi_error error;
    error.id = -200;
    error.description = "Invalid unit";
    error.length = 26;

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  scpi_free_tokens(command);

  return SCPI_SUCCESS;
}

//Enable Nullpunktfahrt
scpi_error_t setzenullpunktfahrt(struct scpi_parser_context *context, struct scpi_token *command)
{
  struct scpi_token *args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while (args != NULL && args->type == 0)
  {
    args = args->next;
  }

  output_numeric = scpi_parse_numeric(args->value, args->length, 1e3, 0, 25e6);
  if (output_numeric.length == 0 ||
      (output_numeric.length == 2 && output_numeric.unit[0] == 'H' && output_numeric.unit[1] == 'z'))
  {
    if (output_numeric.value == 1 || output_numeric.value == 0)
    {
      int l_actualjoint = 1;
      g_platine2config.answer_to_this = 0;
      if (output_numeric.value == 1)
      {
        g_platine2config.operation = 4; //4 is to enable the Nullpunktfahrt, 5 is to disable
      }
      else
      {
        g_platine2config.operation = 5; //4 is to enable the Nullpunktfahrt, 5 is to disable
      }
      for (l_actualjoint; l_actualjoint < 7; l_actualjoint++)
      {
        if (l_actualjoint != g_this_joint)
        {
          long l_starttime = millis();
          boolean l_igottotransmit = false;
          while (l_igottotransmit == false && millis() - l_starttime < 500)
          {
            if (g_tx_frame_slave2master_send == false) //Wenn nix gesendet wird, dann baue ich einen neuen CAN-Frame! Und sende ihn
            {
              g_tx_frame_slave2master.FIR.B.FF = CAN_frame_std;
              g_tx_frame_slave2master.MsgID = 20 + l_actualjoint;
              g_tx_frame_slave2master.FIR.B.DLC = 8;
              g_tx_frame_slave2master.data.u8[0] = g_platine2config.data[0];
              g_tx_frame_slave2master.data.u8[1] = g_platine2config.data[1];
              g_tx_frame_slave2master.data.u8[2] = g_platine2config.data[2];
              g_tx_frame_slave2master.data.u8[3] = g_platine2config.data[3];
              g_tx_frame_slave2master.data.u8[4] = g_platine2config.data[4];
              g_tx_frame_slave2master.data.u8[5] = g_platine2config.data[5];
              g_tx_frame_slave2master.data.u8[6] = g_platine2config.data[6];
              g_tx_frame_slave2master.data.u8[7] = g_platine2config.data[7];
              g_tx_frame_slave2master_send = true;
              l_igottotransmit = true;
            }
          }
          if (l_igottotransmit == false)
          {
            Serial.println("Unable to transmit CAN Package for joint-id: " + String(l_actualjoint));
          }
        }
        else
        {
          EEPROM.write(EEPROM_ADDRESS_ENABLE_STARTUP, output_numeric.value);
          EEPROM.commit();
        }
      }
      Serial.println(F("Ok"));
    }
    else
    {
      Serial.println(F("Wrong input! I expected a 1 or 0."));
    }
  }
  else
  {
    Serial.println(F("Error"));
    scpi_error error;
    error.id = -200;
    error.description = "Invalid unit";
    error.length = 26;

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  scpi_free_tokens(command);

  return SCPI_SUCCESS;
}

//Get actual position of joint
scpi_error_t get_actlmagvalofjoint(struct scpi_parser_context *context, struct scpi_token *command)
{
  struct scpi_token *args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while (args != NULL && args->type == 0)
  {
    args = args->next;
  }

  output_numeric = scpi_parse_numeric(args->value, args->length, 1e3, 0, 25e6);
  if (output_numeric.length == 0 ||
      (output_numeric.length == 2 && output_numeric.unit[0] == 'H' && output_numeric.unit[1] == 'z'))
  {
    //Here i can do stuff. output_numeric.value is my Variable
    if (output_numeric.value == g_this_joint)
    {
      Serial.println(obj_angleSensor.getRotation());
    }
    else if (output_numeric.value > 0 && output_numeric.value < 7)
    {
      g_platine2config.answer_to_this = 1;
      g_platine2config.operation = 6;
      long l_starttime = millis();
      boolean l_gotanresponse = false;
      boolean l_igottotransmit = false;
      int16_t answer = 0;
      while (l_igottotransmit == false && millis() - l_starttime < 2000)
      {
        if (g_tx_frame_slave2master_send == false) //Wenn nix gesendet wird, dann baue ich einen neuen CAN-Frame! Und sende ihn
        {
          g_tx_frame_slave2master.FIR.B.FF = CAN_frame_std;
          g_tx_frame_slave2master.MsgID = 20 + output_numeric.value;
          g_tx_frame_slave2master.FIR.B.DLC = 8;
          g_tx_frame_slave2master.data.u8[0] = g_platine2config.data[0];
          g_tx_frame_slave2master.data.u8[1] = g_platine2config.data[1];
          g_tx_frame_slave2master.data.u8[2] = g_platine2config.data[2];
          g_tx_frame_slave2master.data.u8[3] = g_platine2config.data[3];
          g_tx_frame_slave2master.data.u8[4] = g_platine2config.data[4];
          g_tx_frame_slave2master.data.u8[5] = g_platine2config.data[5];
          g_tx_frame_slave2master.data.u8[6] = g_platine2config.data[6];
          g_tx_frame_slave2master.data.u8[7] = g_platine2config.data[7];
          g_tx_frame_slave2master_send = true;
          l_igottotransmit = true;
        }
      }
      while (l_gotanresponse == false && millis() - l_starttime < 2000)
      {                                                                                //Timeout after 1s
        while (xQueueReceive(CAN_cfg.rx_queue, &g_rx_frame_master2slave, 0) == pdTRUE) //I just received a CAN-packet
        {
          if (g_rx_frame_master2slave.FIR.B.RTR != CAN_RTR && g_rx_frame_master2slave.MsgID == 27)
          {
            g_platine2config.data[0] = g_rx_frame_master2slave.data.u8[0];
            g_platine2config.data[1] = g_rx_frame_master2slave.data.u8[1];
            g_platine2config.data[2] = g_rx_frame_master2slave.data.u8[2];
            g_platine2config.data[3] = g_rx_frame_master2slave.data.u8[3];
            g_platine2config.data[4] = g_rx_frame_master2slave.data.u8[4];
            g_platine2config.data[5] = g_rx_frame_master2slave.data.u8[5];
            g_platine2config.data[6] = g_rx_frame_master2slave.data.u8[6];
            g_platine2config.data[7] = g_rx_frame_master2slave.data.u8[7];
            l_gotanresponse = true;
            answer = g_platine2config.raw_magnet_angle;
          }
        }
      }
      if (l_gotanresponse == true && l_igottotransmit == true)
      {
        Serial.println(answer);
      }
      else
      {
        Serial.println(String(F("Timeout! The Joint did not answer or i could not send the CAN-packet")));
        Serial.println("Transmit: " + String(l_igottotransmit) + " Response: " + String(l_gotanresponse));
      }
    }
    else
    {
      Serial.println(F("Invalid Joint number"));
    }
  }
  else
  {
    Serial.println(F("Error"));
    scpi_error error;
    error.id = -200;
    error.description = "Invalid unit";
    error.length = 26;

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  scpi_free_tokens(command);

  return SCPI_SUCCESS;
}

//Get Raw position of joint
scpi_error_t get_rawvalofjoint(struct scpi_parser_context *context, struct scpi_token *command)
{
  struct scpi_token *args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while (args != NULL && args->type == 0)
  {
    args = args->next;
  }

  output_numeric = scpi_parse_numeric(args->value, args->length, 1e3, 0, 25e6);
  if (output_numeric.length == 0 ||
      (output_numeric.length == 2 && output_numeric.unit[0] == 'H' && output_numeric.unit[1] == 'z'))
  {
    //Here i can do stuff. output_numeric.value is my Variable
    if (output_numeric.value == g_this_joint)
    {
      Serial.println(obj_angleSensor.getRotation());
    }
    else if (output_numeric.value > 0 && output_numeric.value < 7)
    {
      g_platine2config.answer_to_this = 1;
      g_platine2config.operation = 0;
      long l_starttime = millis();
      boolean l_gotanresponse = false;
      boolean l_igottotransmit = false;
      int16_t answer = 0;
      while (l_igottotransmit == false && millis() - l_starttime < 2000)
      {
        if (g_tx_frame_slave2master_send == false) //Wenn nix gesendet wird, dann baue ich einen neuen CAN-Frame! Und sende ihn
        {
          g_tx_frame_slave2master.FIR.B.FF = CAN_frame_std;
          g_tx_frame_slave2master.MsgID = 20 + output_numeric.value;
          g_tx_frame_slave2master.FIR.B.DLC = 8;
          g_tx_frame_slave2master.data.u8[0] = g_platine2config.data[0];
          g_tx_frame_slave2master.data.u8[1] = g_platine2config.data[1];
          g_tx_frame_slave2master.data.u8[2] = g_platine2config.data[2];
          g_tx_frame_slave2master.data.u8[3] = g_platine2config.data[3];
          g_tx_frame_slave2master.data.u8[4] = g_platine2config.data[4];
          g_tx_frame_slave2master.data.u8[5] = g_platine2config.data[5];
          g_tx_frame_slave2master.data.u8[6] = g_platine2config.data[6];
          g_tx_frame_slave2master.data.u8[7] = g_platine2config.data[7];
          g_tx_frame_slave2master_send = true;
          l_igottotransmit = true;
        }
      }
      while (l_gotanresponse == false && millis() - l_starttime < 2000)
      {                                                                                //Timeout after 1s
        while (xQueueReceive(CAN_cfg.rx_queue, &g_rx_frame_master2slave, 0) == pdTRUE) //I just received a CAN-packet
        {
          if (g_rx_frame_master2slave.FIR.B.RTR != CAN_RTR && g_rx_frame_master2slave.MsgID == 27)
          {
            g_platine2config.data[0] = g_rx_frame_master2slave.data.u8[0];
            g_platine2config.data[1] = g_rx_frame_master2slave.data.u8[1];
            g_platine2config.data[2] = g_rx_frame_master2slave.data.u8[2];
            g_platine2config.data[3] = g_rx_frame_master2slave.data.u8[3];
            g_platine2config.data[4] = g_rx_frame_master2slave.data.u8[4];
            g_platine2config.data[5] = g_rx_frame_master2slave.data.u8[5];
            g_platine2config.data[6] = g_rx_frame_master2slave.data.u8[6];
            g_platine2config.data[7] = g_rx_frame_master2slave.data.u8[7];
            l_gotanresponse = true;
            answer = g_platine2config.raw_magnet_angle;
          }
        }
      }
      if (l_gotanresponse == true && l_igottotransmit == true)
      {
        Serial.println(answer);
      }
      else
      {
        Serial.println(String(F("Timeout! The Joint did not answer or i could not send the CAN-packet")));
        Serial.println("Transmit: " + String(l_igottotransmit) + " Response: " + String(l_gotanresponse));
      }
    }
    else
    {
      Serial.println(F("Invalid Joint number"));
    }
  }
  else
  {
    Serial.println(F("Error"));
    scpi_error error;
    error.id = -200;
    error.description = "Invalid unit";
    error.length = 26;

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  scpi_free_tokens(command);

  return SCPI_SUCCESS;
}

//does a Nullpunktfahrt of one axis
scpi_error_t do_nullpunktfahrt(struct scpi_parser_context *context, struct scpi_token *command)
{
  struct scpi_token *args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while (args != NULL && args->type == 0)
  {
    args = args->next;
  }

  output_numeric = scpi_parse_numeric(args->value, args->length, 1e3, 0, 25e6);
  if (output_numeric.length == 0 ||
      (output_numeric.length == 2 && output_numeric.unit[0] == 'H' && output_numeric.unit[1] == 'z'))
  {
    //Here i can do stuff. output_numeric.value is my Variable
    if (output_numeric.value == g_this_joint)
    {
      boolean l_thisjointisatgoal = false; //I cant initialise variables inside the switch-routine (Thanks C)
      long l_starttime = millis();
      this_axis_do_Nullpunktfahrt(l_thisjointisatgoal,l_starttime);
      Serial.println("Ok");
    }
    else if (output_numeric.value > 0 && output_numeric.value < 7)
    {
      g_platine2config.answer_to_this = 0;
      g_platine2config.operation = 7;
      g_platine2config.tobesaved_offset_value = 0;
      long l_starttime = millis();
      boolean l_igottotransmit = false;
      while (l_igottotransmit == false && millis() - l_starttime < 2000)
      {
        if (g_tx_frame_slave2master_send == false) //Wenn nix gesendet wird, dann baue ich einen neuen CAN-Frame! Und sende ihn
        {
          g_tx_frame_slave2master.FIR.B.FF = CAN_frame_std;
          g_tx_frame_slave2master.MsgID = 20 + output_numeric.value;
          g_tx_frame_slave2master.FIR.B.DLC = 8;
          g_tx_frame_slave2master.data.u8[0] = g_platine2config.data[0];
          g_tx_frame_slave2master.data.u8[1] = g_platine2config.data[1];
          g_tx_frame_slave2master.data.u8[2] = g_platine2config.data[2];
          g_tx_frame_slave2master.data.u8[3] = g_platine2config.data[3];
          g_tx_frame_slave2master.data.u8[4] = g_platine2config.data[4];
          g_tx_frame_slave2master.data.u8[5] = g_platine2config.data[5];
          g_tx_frame_slave2master.data.u8[6] = g_platine2config.data[6];
          g_tx_frame_slave2master.data.u8[7] = g_platine2config.data[7];
          g_tx_frame_slave2master_send = true;
          l_igottotransmit = true;
        }
      }
      if (l_igottotransmit == true)
      {
        Serial.println("Ok");
      }
      else
      {
        Serial.println(String(F("Timeout! I could not send the CAN-packet")));
      }
    }
    else
    {
      Serial.println(F("Invalid Joint number"));
    }
  }
  else
  {
    Serial.println(F("Error"));
    scpi_error error;
    error.id = -200;
    error.description = "Invalid unit";
    error.length = 26;

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  scpi_free_tokens(command);

  return SCPI_SUCCESS;
}