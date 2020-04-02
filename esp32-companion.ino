
#include "HardwareSerial.h"
HardwareSerial Serial2(2);

#include <ros.h>
#include <ros/time.h>
#include <robocars_msgs/robocars_radio_channels.h>
#include <robocars_msgs/robocars_tof.h>
#include <robocars_msgs/robocars_actuator_output.h>
#include <robocars_msgs/robocars_debug.h>

#include "SBUS.h"
#include <Adafruit_NeoPixel.h>

// GPIO Assignment
#define BUILT_IN_LED 2
#define GPIO_OUTPUT_LED    13
#define PWM_RC_THROTTLE_OUTUT_PIN 14   //Set GPIO 15 as PWM0A
#define PWM_RC_STEERING_OUTUT_PIN 27   //Set GPIO 15 as PWM1A
#define PWM_FREQ 50
const int throttleChannel = 0;
const int steeringChannel = 2;

// Statuses
#define INT_DISCONNECTED  0
#define INT_RXERROR       1
#define INT_CALIBRATE     2
#define HOST_INIT         3
#define HOST_MODE_USER    4
#define HOST_MODE_LOCAL   5
#define HOST_MODE_DISARMED   6

//Each 50ms, check and output value to serial link
#define OUTPUTLOOP 30
#define INTPUTLOOP 10

int cmd_throttle = 1500;
int cmd_steering = 1500;
// GLobal buffer for serial output
char buff [2048] = {};
char debug [2048] = {};

// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial2);

// channel, fail safe, and lost frames data
uint16_t channels[16];
bool failSafe;
bool lostFrame;

ros::NodeHandle  nh;
robocars_msgs::robocars_radio_channels channels_msg;  
ros::Publisher pub_channels( "radio_channels", &channels_msg);

char radio_frameid[] = "/radio";


//
// Tools
//

uint32_t mapRange(uint32_t a1,uint32_t a2,uint32_t b1,uint32_t b2,uint32_t s)
{
  if (s<a1) {s=a1;}
  if (s>a2) {s=a2;}
  return b1 + ((s-a1)*(b2-b1))/(a2-a1);
}

//
// PWM Output
//

static void pwm_gpio_initialize()
{
    double res;

    ledcAttachPin(PWM_RC_THROTTLE_OUTUT_PIN, throttleChannel);
    ledcAttachPin(PWM_RC_STEERING_OUTUT_PIN, steeringChannel);

    printf("initializing pwm gpio...\n");
    res=ledcSetup(throttleChannel, PWM_FREQ, 8);
    printf("Throttle Channel setup return %ld\n", res); 
    res = ledcSetup(steeringChannel, PWM_FREQ, 8);
    printf("Steering Channel setup return %ld\n", res); 
}


static void mcpwm_set_throttle_pwm(int pwm_width_in_us)
{
  const int dutyCycle = (255*pwm_width_in_us)/(1000000/PWM_FREQ);
  ledcWrite(throttleChannel, dutyCycle);
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void mcpwm_set_steering_pwm(int pwm_width_in_us)
{
  const int dutyCycle = (255*pwm_width_in_us)/(1000000/PWM_FREQ);
  ledcWrite(steeringChannel, dutyCycle);
}

// --------------------------------------
// LED Part
// --------------------------------------

Adafruit_NeoPixel statusLed (1, GPIO_OUTPUT_LED, NEO_GRB + NEO_KHZ800);

#define COMPCOLOR(r, g, b) (g<<16)|(r<<8)|(b)
#define TIMESTEPS      8 

struct Led {
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned char timing;
} ;

struct Led ledControl;
uint32_t currentColor;

void led_gpio_initialize (void) {
  pinMode(GPIO_OUTPUT_LED, OUTPUT);
  pinMode(BUILT_IN_LED, OUTPUT);
}

void switchOffLed() {
    currentColor=statusLed.Color(0,0,0);
    statusLed.setPixelColor(0, currentColor);         
    statusLed.show();  
}

void switchLedRed() {
    currentColor=statusLed.Color(128,0,0);
    statusLed.setPixelColor(0, currentColor);         
    statusLed.show();  
}

void updateLed( void *pvParameters );

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
    statusLed.setPixelColor(0, currentColor);         
    statusLed.show();  
    seq=(seq+1)%TIMESTEPS;
    vTaskDelay((1000/TIMESTEPS) / portTICK_PERIOD_MS);
  }
  vTaskDelete( NULL );
}

void setLed (unsigned char r, unsigned char g, unsigned char b, unsigned char timing) {
  ledControl.r=r;
  ledControl.g=g;
  ledControl.b=b;
  ledControl.timing = timing;
}

void displayStatusOnLED (int status)
{
  static int _last_status = -1;
  if (status != _last_status) {
    _last_status = status;
    if (status==INT_CALIBRATE) {
      // fast white blink
      setLed (0xff,0xff,0xff,0x55);
    }
    if (status==HOST_INIT) {
      // Slow red blink
      setLed (0xff,0x00,0x0,0x18);
    }
    if (status==HOST_MODE_USER) {
      // Slow green blink
      setLed (0x00,0xff,0x00,0x18);
    }
    if (status==HOST_MODE_LOCAL) {
      // Slow blue blink
      setLed (0x00,0x00,0xFF,0x18);
    }
    if (status==INT_DISCONNECTED) {
      // fast red blink
      setLed (0xff,0x00,0x00,0x55);
    }
    if (status==INT_RXERROR) {
      // slow red blink
      setLed (0xff,0x00,0x00,0x50);
    }
    if (status==HOST_MODE_DISARMED) {
      // fast green blink
      setLed (0x00,0xff,0x00,0x54);
    }
  }
}
void processStatusFromHost (const char *status) {

  if (strcmp(status, "init")==0) {
    displayStatusOnLED(HOST_INIT);
  }
  if (strcmp(status, "disarmed")==0) {
    displayStatusOnLED(HOST_MODE_DISARMED);
  }
  if (strcmp(status, "user")==0) {
    displayStatusOnLED(HOST_MODE_USER);
  }
  if (strcmp(status, "local")==0) {
    displayStatusOnLED(HOST_MODE_LOCAL);
  }
}

void steering_cb( const robocars_msgs::robocars_actuator_output& steering_msg){
  mcpwm_set_steering_pwm (steering_msg.pwm);
}

void throttling_cb( const robocars_msgs::robocars_actuator_output& throttling_msg){
  mcpwm_set_throttle_pwm (throttling_msg.pwm);
}


ros::Subscriber<robocars_msgs::robocars_actuator_output> steering_sub("steering", steering_cb);
ros::Subscriber<robocars_msgs::robocars_actuator_output> throttling_sub("throttling", throttling_cb);


//
// MAIN
//


void setup() {
  // begin the SBUS communication
  x8r.begin();
 
  pwm_gpio_initialize();
  led_gpio_initialize();
  mcpwm_set_throttle_pwm (1500);
  mcpwm_set_steering_pwm (1500);
  statusLed.begin();

  nh.initNode();
  nh.advertise(pub_channels);
  nh.subscribe(steering_sub);
  nh.subscribe(throttling_sub);


  channels_msg.header.frame_id =  radio_frameid;

  displayStatusOnLED(INT_DISCONNECTED);   

  
  // Now set up two tasks to run independently.
  
  xTaskCreate(
    updateLed
    ,  (const char *)"Led Status"   // A name just for humans
    ,  2048  // Stack size
    ,  NULL
    ,  1  // priority
    ,  NULL );
  
}

void loop() {
  
  static uint32_t lastTs = 0;
  uint32_t t = (micros()/1000)%50000;

  // look for a good SBUS packet from the receiver
  if(x8r.read(&channels[0], &failSafe, &lostFrame)){
    // write the SBUS packet to an SBUS compatible servo
    //sprintf(output, "Ch1 %d Ch2 %d Ch3 %d Ch4 %d Ch5 %d Ch6 %d failSafe %d lostFrame %d", channels[0], channels[1], channels[2], channels[3], channels[4], channels[5], failSafe, lostFrame);
    if (failSafe == 1) {
      channels_msg.ch1=0;
      channels_msg.ch2=0;
      channels_msg.ch3=0;
      channels_msg.ch4=0;
      channels_msg.ch5=0;
      channels_msg.ch6=0;
      mcpwm_set_throttle_pwm (1500);
      mcpwm_set_steering_pwm (1500);
      displayStatusOnLED(INT_RXERROR);    
    } else {
      channels_msg.ch1=channels[0];
      channels_msg.ch2=channels[1];
      channels_msg.ch3=channels[2];
      channels_msg.ch4=channels[3];
      channels_msg.ch5=channels[4];
      channels_msg.ch6=channels[5];
    }
    pub_channels.publish(&channels_msg);
  }
  nh.spinOnce(); 
}
