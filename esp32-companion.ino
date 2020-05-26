#include "HardwareSerial.h"

#include "SBUS.h"
#include <SPI.h>
#include <Wire.h>
 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define RGBLED_DISPLAY
#define OLED_DISPLAY
#define OLED_128x32_DISPLAY
//#define OLED_128*64_DISPLAY

#if defined(OLED_128x32_DISPLAY) && !defined(SSD1306_128_32)
#error You must configure your display nodel in Adafruit_SSD1306.h as well
#endif

#if defined(OLED_128x64_DISPLAY) && !defined(SSD1306_128_64)
#error You must configure your display nodel in Adafruit_SSD1306.h as well
#endif

#ifdef RGBLED_DISPLAY
// RGB Led on original grumpy car to show vehicle status
#include <Adafruit_NeoPixel.h>
#endif

#define ROSSERIAL_BAUD 1500000
#define ROSSERIAL_OVERRIDE_SERIAL_CLASS &Serial1
#define ROSSERIAL_RXD_PIN 18
#define ROSSERIAL_TXD_PIN 19

#include <ros.h>
#include <ros/time.h>
#include <robocars_msgs/robocars_radio_channels.h>
#include <robocars_msgs/robocars_actuator_output.h>
#include <robocars_msgs/robocars_brain_state.h>
#include <robocars_msgs/robocars_debug.h>


// Built in LED 
#define BUILT_IN_LED               2   // built in ESP32 Led


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define I2C_SDA 21
#define I2C_SCL 22

// Screen const and global object
#ifdef OLED_128x32_DISPLAY
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#else
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#endif

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// LED const and global object
#ifdef RGBLED_DISPLAY
#define GPIO_OUTPUT_LED    13
Adafruit_NeoPixel statusLed (1, GPIO_OUTPUT_LED, NEO_GRB + NEO_KHZ800);
#endif

void updateDisplay();

//PWM Const

#define PWM_RC_THROTTLE_OUTUT_PIN 14   //Set GPIO 15 as PWM0A
#define PWM_RC_STEERING_OUTUT_PIN 27   //Set GPIO 15 as PWM1A

#define PWM_FREQ 50
const int throttleChannel = 0;
const int steeringChannel = 2;

// ESP32 status
#define INT_DISCONNECTED  0
#define INT_WAIT_RX       1
#define INT_RXERROR       2
#define INT_RXOK          3

//Host status
#define HOST_INIT           0
#define HOST_MODE_USER      1
#define HOST_MODE_LOCAL     2
#define HOST_MODE_DISARMED  3

// Throttle and steering output order
int cmd_throttle = 1500;
int cmd_steering = 1500;

// a SBUS object, which is on hardware
// serial port 2 is used to connect SBUS
SBUS x8r(Serial2);

// channel, fail safe, and lost frames data
uint16_t channels[16];
bool failSafe;
bool lostFrame;

ros::NodeHandle  nh;
robocars_msgs::robocars_radio_channels channels_msg;  
ros::Publisher pub_channels( "radio_channels", &channels_msg);
const char radio_frameid[] = "radio";

// ------------------------
// PWM Output control logic
// ------------------------

static void pwm_gpio_initialize()
{
    ledcAttachPin(PWM_RC_THROTTLE_OUTUT_PIN, throttleChannel);
    ledcAttachPin(PWM_RC_STEERING_OUTUT_PIN, steeringChannel);

    ledcSetup(throttleChannel, PWM_FREQ, 8);
    ledcSetup(steeringChannel, PWM_FREQ, 8);
}


static void mcpwm_set_throttle_pwm(int pwm_width_in_us)
{
  const int dutyCycle = (255*pwm_width_in_us)/(1000000/PWM_FREQ);
  ledcWrite(throttleChannel, dutyCycle);
}

static void mcpwm_set_steering_pwm(int pwm_width_in_us)
{
  const int dutyCycle = (255*pwm_width_in_us)/(1000000/PWM_FREQ);
  ledcWrite(steeringChannel, dutyCycle);
}

// --------------------------------------
// Status display : global var to hold last reported status
// --------------------------------------

static int last_int_status = -1;
static int last_host_status = -1;

// --------------------------------------
// Status display : LED Part
// --------------------------------------

// Number of steps in animation 
#define TIMESTEPS      16 

struct Led {
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned int timing;
} ;

struct Led ledControl;
uint32_t currentColor;

void led_gpio_initialize (void) {
#ifdef RGBLED_DISPLAY
  pinMode(GPIO_OUTPUT_LED, OUTPUT);
#endif
  pinMode(BUILT_IN_LED, OUTPUT);
}

void updateLed(void * pvParameters ) {
  static int seq = 0;

  while(1) {
    statusLed.clear();
    if ((ledControl.timing>>seq) & 0x01) {
      currentColor=statusLed.Color(ledControl.r,ledControl.g,ledControl.b);
      digitalWrite(BUILT_IN_LED, LOW);  
    } else {
      currentColor=statusLed.Color(0,0,0);
      digitalWrite(BUILT_IN_LED, HIGH);  
    }
#ifndef RGBLED_DISPLAY
    statusLed.setPixelColor(0, currentColor);         
    statusLed.show();
#endif  
    seq=(seq+1)%TIMESTEPS;
    vTaskDelay((1000/TIMESTEPS) / portTICK_PERIOD_MS);
  }
  vTaskDelete( NULL );
}

void setLed (unsigned char r, unsigned char g, unsigned char b, unsigned int timing) {
  ledControl.r=r;
  ledControl.g=g;
  ledControl.b=b;
  ledControl.timing = timing;
}

void displayIntStatusOnLED (int status)
{
  if (status==INT_DISCONNECTED) {
    // Initial state, fast red blink
    setLed (0xff,0x00,0x00,0x5555);
  }
  if (status == INT_WAIT_RX) {
    // Waiting Radio signal white dual flash
    setLed (0xff,0xff,0xff,0x0505);      
  }
  if (status==INT_RXOK) {
    // Slow blink
    //setLed (0xff,0xff,0xff,0x000F);
  }
  if (status==INT_RXERROR) {
    // Lost radio, slow red dual flash
    setLed (0xff,0x00,0x00,0x0505);
  }
}

void displayHostStatusOnLED (int status)
{
  if (status==HOST_INIT) {
    // Do nothing since this is default stes for Host
    //setLed (0xff,0x00,0x0,0x18);
  }
  if (status==HOST_MODE_DISARMED) {
    // slow green blink pulse
    setLed (0x00,0xff,0x00,0x0005);
  }
  if (status==HOST_MODE_USER) {
    // Slow green blink
    setLed (0x00,0xff,0x00,0xF0F0);
  }
  if (status==HOST_MODE_LOCAL) {
    // Very slow blue blink
    setLed (0x00,0x00,0xFF,0xF5F5);
  }
}

// --------------------------------------
// Status display : OLED DIsplay Part
// --------------------------------------

#ifdef OLED_DISPLAY

const char * const intStatus2text[] = {"Waiting for ROS Link","Waiting For Radio", "Radio Rx Error","Radio RX OK"};
const char * const hostStatus2text[] = {"Waiting Host INIT","Autopilot Manual","Autopilot Autonomous","Host Disarmed"};
static char last_host_debug[25];

// display
void updateDisplay() {

  display.clearDisplay();
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("GrumpyCar Dashboard"));
  display.println(F(last_host_debug));
  if (last_int_status>=0) {
    display.println(F(intStatus2text[last_int_status]));    
  }
  if (last_host_status>=0) {
    display.println(F(hostStatus2text[last_host_status]));
  }
  display.display();  
}
#else
void updateDisplay() {
}
#endif

// --------------------------------------
// Status display : Common Part
// --------------------------------------

void displayIntStatus (int status) {
  if (last_int_status != status) {
    displayIntStatusOnLED(status);
    last_int_status = status;
    updateDisplay();
  }
}

void displayHostStatus (int status) {
  if (last_host_status != status) {
    displayHostStatusOnLED(status);
    last_host_status = status;
    updateDisplay();
  }
}

void processStatusFromHost (int status) {

  switch(status) {
    case robocars_msgs::robocars_brain_state::BRAIN_STATE_IDLE:
      displayHostStatus(HOST_MODE_DISARMED);
    break;
    case robocars_msgs::robocars_brain_state::BRAIN_STATE_MANUAL_DRIVING:
      displayHostStatus(HOST_MODE_USER);
    break;
    case robocars_msgs::robocars_brain_state::BRAIN_STATE_AUTONOMOUS_DRIVING:
      displayHostStatus(HOST_MODE_LOCAL);
    break;
  }  
}

void processDebugFromHost (std::string msg) {
  strncpy(last_host_debug, msg.c_str(), 25);
#ifdef OLED_DISPLAY
    updateDisplay();
#endif  
}

// --------------------------------------
// ROS msg subscribers and callback 
// --------------------------------------

void steering_cb( const robocars_msgs::robocars_actuator_output& steering_msg){
  mcpwm_set_steering_pwm (steering_msg.pwm);
}

void throttling_cb( const robocars_msgs::robocars_actuator_output& throttling_msg){
  mcpwm_set_throttle_pwm (throttling_msg.pwm);
}

void state_msg_cb( const robocars_msgs::robocars_brain_state& state_msg){
  processStatusFromHost(state_msg.state);
}

void debug_msg_cb( const robocars_msgs::robocars_debug& debug_msg){
  processDebugFromHost(debug_msg.msg);
}


ros::Subscriber<robocars_msgs::robocars_actuator_output> steering_sub("steering_ctrl/output", steering_cb);
ros::Subscriber<robocars_msgs::robocars_actuator_output> throttling_sub("throttling_ctrl/output", throttling_cb);
ros::Subscriber<robocars_msgs::robocars_brain_state> state_sub("robocars_brain_state", &state_msg_cb );
ros::Subscriber<robocars_msgs::robocars_debug> debug_sub("robocars_debug", &debug_msg_cb );

// -------------------------------
// MAIN
// -------------------------------


void setup() {
  
  // begin the SBUS communication
  x8r.begin();

  // Init debug serial link
  Serial.begin(115200);

  // Init PWM output logic and set to default 1500 us (idle)
  pwm_gpio_initialize();
  led_gpio_initialize();
  mcpwm_set_throttle_pwm (1500);
  mcpwm_set_steering_pwm (1500);

#ifndef RGBLED_DISPLAY
  //init RGB Led driver
  statusLed.begin();
#endif

#ifdef OLED_DISPLAY
  // Init OLED display driver
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(500); // Pause for 2 seconds
  // Clear the buffer
  display.setRotation(2);
  display.clearDisplay();
  display.display();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
#endif

  //Init ROS node, advertise published topicsm and subscribed 
  nh.initNode();
  nh.advertise(pub_channels);
  nh.subscribe(steering_sub);
  nh.subscribe(throttling_sub);
  nh.subscribe(state_sub);
  nh.subscribe(debug_sub);

  channels_msg.header.frame_id =  radio_frameid;

  // Set default status display on display and led
  displayIntStatus(INT_DISCONNECTED);   
  displayHostStatus(HOST_INIT);

  // Log node startup through ros
  nh.logdebug("Starting ESP32 Node");
  
  // Start a task to handle led animation.
  xTaskCreate(
    updateLed
    ,  (const char *)"Led Status"   // A name just for humans
    ,  2048  // Stack size
    ,  NULL
    ,  1  // priority
    ,  NULL );
  
}

void loop() {

  
  static uint8_t ROSStatus = 1;
  static uint8_t RXStatus = 1;
  static uint32_t RXStatus_timer=millis();

  //Wait for ROS to connect to host
  while (!nh.connected()) {
    nh.spinOnce();
    if (ROSStatus == 1) {
      //If disconnected, show status on LED and display
      displayIntStatus(INT_DISCONNECTED);     
    } 
    ROSStatus = 0;
  }
  ROSStatus = 1;

  // look for a good SBUS packet from the receiver
  if(x8r.read(&channels[0], &failSafe, &lostFrame)){
    RXStatus=1;
    RXStatus_timer=millis();
    // write the SBUS packet to an SBUS compatible servo
    if (failSafe == 1) {
      //if failsafe is reported, show status and switch back PWM output to default idle value
      channels_msg.ch1=0;
      channels_msg.ch2=0;
      channels_msg.ch3=0;
      channels_msg.ch4=0;
      channels_msg.ch5=0;
      channels_msg.ch6=0;
      mcpwm_set_throttle_pwm (1500);
      mcpwm_set_steering_pwm (1500);
      displayIntStatus(INT_RXERROR);    
    } else {
      channels_msg.ch1=channels[0];
      channels_msg.ch2=channels[1];
      channels_msg.ch3=channels[2];
      channels_msg.ch4=channels[3];
      channels_msg.ch5=channels[4];
      channels_msg.ch6=channels[5];
      displayIntStatus(INT_RXOK);    
    }
    pub_channels.publish(&channels_msg);
  } else {
    if ((RXStatus==1) && (millis()-RXStatus_timer)>100) {
      //Looks like we did not received anything from Radio in the last 100ms, show it
      displayIntStatus(INT_WAIT_RX);     
      RXStatus=0;      
    }
  }

  //schedule ROS processing
  nh.spinOnce();
}
