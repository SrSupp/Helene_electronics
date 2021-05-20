//Includes:
#include <Arduino.h>
#include <ADS1220.h>
#include <scpiparser.h>
#include <ESP32CAN.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//defines:
#define PIN_ADC1_DRDY 23
#define PIN_ADC1_CS 18
#define PIN_LED 2
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define TIME_MS_PUBLISH_FREQUENCY 19 // A bit more than 50Hz

//https://diyusthad.com/image2cpp   Bild von Helene
static const unsigned char PROGMEM logo_bmp[] = {
	// 'Helene_oled, 32x32px
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x3f, 0xf0, 
	0x00, 0x0e, 0x67, 0xf0, 0x03, 0xfc, 0x17, 0xf8, 0x0f, 0xb8, 0x3f, 0xf8, 0x0c, 0x1e, 0xff, 0xf8, 
	0x0d, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xe0, 0x30, 0x07, 0xff, 0xe0, 0x00, 0x06, 0xff, 0xf0, 0x00, 
	0x10, 0xc7, 0xfc, 0x00, 0x1b, 0x81, 0xff, 0x00, 0x1f, 0x00, 0x77, 0xc0, 0x0b, 0x00, 0x03, 0xe0, 
	0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x0f, 0xc0, 
	0x00, 0x07, 0xff, 0xc0, 0x00, 0x0f, 0xff, 0xe0, 0x00, 0x0f, 0xff, 0xf0, 0x00, 0x0f, 0xff, 0xf0, 
	0x00, 0x05, 0xff, 0xe0, 0x00, 0x0f, 0xff, 0xe0, 0x00, 0x0f, 0xff, 0xe0, 0x00, 0x0f, 0xff, 0xe0, 
	0x00, 0x0f, 0xff, 0xe0, 0x00, 0x0f, 0xff, 0xf0, 0x00, 0x0f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00
};

static const char* g_names[] =  {"Helene", "FG: MuST","Sven Suppelt","Felix Herbst","Romal Chadda","Jan Hinrichs" ,"Dennis Roth","Esan Sundaralingam","Eric Pohl","Philipp Witulla","Prof. Kupnik","Technische","Universitaet","Darmstadt"};

//Objekte:
ADS1220 obj_ADC1 = ADS1220();
Adafruit_SSD1306 obj_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TaskHandle_t Task1;
CAN_device_t CAN_cfg;
CAN_frame_t g_rx_frame_master2slave;
CAN_frame_t g_tx_frame_meas2master;
TaskHandle_t canwriteframe_TaskHnd;

//Globale Variablen: 
float g_adc_rawvalue = 0.0;
boolean g_adc_newdata = false;
boolean g_tx_frame_meas2master_send = false;

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

typedef union { // This structure is used to easily convert 4 int values and 1 long value into 8 uint8_t values for CAN communication
  struct
  {
    float adc_value;
  };
  uint8_t data[4];
} convert_f2c;
convert_f2c g_meas2master;

//Voids:
void IRAM_ATTR isr();
void Task_Oled( void * parameter);
void canwriteframe_Task(void *parameter); //This Task runs on a different core and is only responsible for pushing the can packet

void setup()
{
  //Serial initialisation
  Serial.begin(115200);

  //Setup the Pins
  pinMode(PIN_ADC1_CS, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  digitalWrite(PIN_ADC1_CS, HIGH);

  //Wait a bit
  delay(2000);

  //Start Oled
    if(!obj_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  //Initialize Screen and show all names inside g_names!
  int l_namesize = 0;
  int l_totalsize = sizeof(g_names);
  Serial.println(F("Helene Roboterarm!"));
  while(l_totalsize > 0){
    l_totalsize -= sizeof(g_names[l_namesize]);
    Serial.println(g_names[l_namesize]);
    l_namesize++;
  }

  boolean l_switchnames = esp_random()%2;
  
  obj_display.display();
  obj_display.clearDisplay();
  obj_display.setTextSize(1); // Draw 1X-scale text
  obj_display.setTextColor(SSD1306_WHITE);
  for(int i = 0; i < (l_namesize)*8-31; i++){
    obj_display.clearDisplay();
    obj_display.setCursor(0, -i);
    for (int ii = 0; ii < l_namesize; ii++){
      if(ii == 2 || ii == 3){
        obj_display.println(g_names[ii+l_switchnames*(5-2*ii)]);
      } else {
        obj_display.println(g_names[ii]);
      }
    }
    obj_display.drawBitmap(127-34,0,logo_bmp,32,32,1);
    obj_display.display();
    if(i == 0){
      delay(1000);
    }
    delay(120);
  }
  delay(1000);

  //Start the ADCs
  obj_ADC1.begin(PIN_ADC1_DRDY, PIN_ADC1_CS);

  //Configure the ADCs
  obj_ADC1.setMUX(0); //Kanäle! 0 ist AIN0/AIN1 - 5 ist AIN2/AIN3
  obj_ADC1.setGain(128);
  obj_ADC1.setOperatingMode('T');
  obj_ADC1.setAnalogReference('E');
  obj_ADC1.setDatarate(180);
  obj_ADC1.startContinuousMeas(true);
  delay(10);
  
  //Attach Tasks/Interrupts
  attachInterrupt(PIN_ADC1_DRDY, isr, FALLING);
  xTaskCreatePinnedToCore(
      Task_Oled, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      1); /* Core where the task should run */

  //CAN initialisation. We start the CAN Interface with 200kbps, using GPIOs 4 and 5 and with a que of 100 elements
  CAN_cfg.rx_queue = xQueueCreate(100, sizeof(CAN_frame_t));
  CAN_cfg.speed = CAN_SPEED_200KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  ESP32Can.CANInit();

  //The function ESP32Can.CANWriteFrame is blocking, when an error occours. For that reason, a new task is created, so that the main program still functions.
  xTaskCreatePinnedToCore(canwriteframe_Task, "canwriteframe_Task", 5000, NULL, 1, &canwriteframe_TaskHnd, 0);


  disableCore0WDT();
  //disableCore1WDT();
}

long g_ms_time_differenze_publish = 0;

void loop()
{
  if(millis() - g_ms_time_differenze_publish >= TIME_MS_PUBLISH_FREQUENCY){
    g_ms_time_differenze_publish = millis();
    if (g_tx_frame_meas2master_send == false) //Wenn nix gesendet wird, dann baue ich einen neuen CAN-Frame! Und sende ihn
    {
      g_meas2master.adc_value = g_adc_rawvalue;
      g_tx_frame_meas2master.FIR.B.FF = CAN_frame_std;
      g_tx_frame_meas2master.MsgID = 42;
      g_tx_frame_meas2master.FIR.B.DLC = 4;
      g_tx_frame_meas2master.data.u8[0] = g_meas2master.data[0];
      g_tx_frame_meas2master.data.u8[1] = g_meas2master.data[1];
      g_tx_frame_meas2master.data.u8[2] = g_meas2master.data[2];
      g_tx_frame_meas2master.data.u8[3] = g_meas2master.data[3];
      g_tx_frame_meas2master_send = true;
    }
  }

  if(g_adc_newdata == true){ //Neue Daten vom ADC sind verfügbar
    g_adc_rawvalue = obj_ADC1.Read_Data()*0.1+0.9*g_adc_rawvalue;
    Serial.println(String(g_adc_rawvalue,3));
    g_adc_newdata = false;
    delay(1);
  }

while (xQueueReceive(CAN_cfg.rx_queue, &g_rx_frame_master2slave, 0) == pdTRUE) //I just received a CAN-packet
  {
    if (g_rx_frame_master2slave.FIR.B.RTR != CAN_RTR && g_rx_frame_master2slave.MsgID > 0 && g_rx_frame_master2slave.MsgID < 10) //I received a command package
    {
      g_master2slave.data[0] = g_rx_frame_master2slave.data.u8[0];
      g_master2slave.data[1] = g_rx_frame_master2slave.data.u8[1];
      g_master2slave.data[2] = g_rx_frame_master2slave.data.u8[2];
      g_master2slave.data[3] = g_rx_frame_master2slave.data.u8[3];
      g_master2slave.data[4] = g_rx_frame_master2slave.data.u8[4];
      g_master2slave.data[5] = g_rx_frame_master2slave.data.u8[5];
      g_master2slave.data[6] = g_rx_frame_master2slave.data.u8[6];
      g_master2slave.data[7] = g_rx_frame_master2slave.data.u8[7];
    }
  }
}

void IRAM_ATTR isr() {
  g_adc_newdata = true;
}

void Task_Oled( void * parameter) {
  long millis_timedisp = 0;
  for(;;) {
  if(millis() - millis_timedisp>100){
    millis_timedisp = millis();
    obj_display.clearDisplay();
    obj_display.setCursor(0, 0);
    obj_display.setTextSize(2);
    obj_display.print(String(g_adc_rawvalue,3));
    obj_display.setCursor(80, 0);
    obj_display.println(String("mV/V"));
    obj_display.setTextSize(1);
    obj_display.setCursor(0, 16);
    obj_display.println(String("Op-ID: "+ String(g_master2slave.operation_ident)));
    obj_display.print(String("Res: "+ String(g_master2slave.reserved)));
    obj_display.setCursor(64, 16);
    obj_display.print(String("Gr: "+ String(g_master2slave.led_green)));
    obj_display.setCursor(64, 24);
    obj_display.print(String("Bl: "+ String(g_master2slave.led_blue)));
    obj_display.display();
  }
  yield();
  }
}

void canwriteframe_Task(void *parameter) //This Task runs on a different core and is only responsible for pushing the can packet
{
  for (;;)
  {
    if (g_tx_frame_meas2master_send == true)
    {
      ESP32Can.CANWriteFrame(&g_tx_frame_meas2master);
      g_tx_frame_meas2master_send = false; // Set it to false afterwards, so that a new message can be sent
    }
    delayMicroseconds(5);
    yield();
  }
}