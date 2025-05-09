#include <Wire.h>
#include <PCF8574.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "driver/ledc.h"
#include <ESP32Servo.h>
#include <NewPing.h>
#include "BluetoothSerial.h"
#include <math.h>
#include "esp_pm.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
// const char *pin = "1234"; // Change this to more secure PIN.



#define SDA 21
#define SCL 22

// motor_1 rotation direction logic 
#define pcf8574_IO_1 1
#define pcf8574_IO_0 0
// motor_2 rotation direction logic 
#define pcf8574_IO_3 3
#define pcf8574_IO_2 2

// motor_3 rotation direction logic 
#define pcf8574_IO_7 7
#define pcf8574_IO_6 6
// motor_4 rotation direction logic 
#define pcf8574_IO_5 5
#define pcf8574_IO_4 4

#define motorAenpin 4
#define motorBenpin 13


// Joystick active start position=
#define stick_b_active_val (127 - 22)
#define stick_f_active_val (127 + 22)
#define stick_r_active_val (127 + 22)
#define stick_l_active_val (127 - 22)

// pwm details
#define PWM_FREQ 5000
#define pwm_resolution_bit 12
#define pwm_resolution (pow(2, pwm_resolution_bit) - 1)

#define analog_base_speed 2500


// Ultrasonic sensor pin details
#define TRIGGER_PIN  27
#define ECHO_PIN     15
#define MAX_DISTANCE 500

#define GROUND_DIST_ALLW 5
#define RF_SCAN_TIMEOUT 5000
#define POWER_PERIPHERAL_PIN 1

// neck_servo_pos_offset
#define neck_servo_offset -10

#define sonar_default_angle 50
// Data packet struct
typedef struct __attribute__((packed)) rx_data_packet
{
  bool f_btn_state; //forward button state
  bool b_btn_state; //backward button state 
  bool l_btn_state; //left button state 
  bool r_btn_state; //right button state 

  char x_axis_val[5]; 
  char y_axis_val[5];

  bool l_one_state;
  bool r_one_state;
  bool l_two_state;
  bool r_two_state;
} rx_data_packet;

typedef struct _wireless_device_state
{
  bool rf_module_is_active;
  bool bluetooth_is_active;
} wireless_devices_state;

typedef struct _sweeped_data
{
  bool spaceisavailable;
  float obstacle_width;
  float space_width;
  char side;
} sweeped_data;

const uint8_t motorAcontrolPins[3] = {GPIO_NUM_4,  pcf8574_IO_1, pcf8574_IO_0};
const uint8_t motorBcontrolPins[3] = {GPIO_NUM_13, pcf8574_IO_3, GPIO_NUM_14};
const uint8_t motorDcontrolPins[3] = {GPIO_NUM_4,  pcf8574_IO_7, pcf8574_IO_6};
const uint8_t motorCcontrolPins[3] = {GPIO_NUM_13, pcf8574_IO_5, pcf8574_IO_4};

const byte addresses[][6] = {"00001", "00002"};

// safe distance recorded distance from obstacle
float f_sonar_ground_dist = 8.00; //cm
float f_sonar_fall_dist = f_sonar_ground_dist + (GROUND_DIST_ALLW);
uint8_t bot_width = 4;
float distAtninetyDeg;

bool f_sonar_obstacle_flag;
bool f_sonar_fall_flag;

volatile char controlMode = 'N';  
int global_analog_speed = 0;


// peripherals and objects
RF24 radio(2, 5); // CE, CSN
PCF8574 pcf8574(0X38);

BluetoothSerial SerialBT;
String device_name = "C1";
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Servo objects
Servo hindlegs_servo;
Servo forelegs_servo;
Servo neck_rot_servo;
Servo sonar_servo;

// Servo pins
const uint8_t hindlegs_servo_pin = 26;
const uint8_t forelegs_servo_pin = 32;
const uint8_t neck_rot_servo_pin = 33;
const uint8_t sonar_servo_pin = 25;


// Functions initialization
void readModeFromSerial();
void radio_system_control(rx_data_packet received_data);
void sweepandcheck(uint8_t startAngle, uint8_t endAngle);
void sweepandread(uint8_t start_angle, uint8_t end_angle);
float calcSenDistAtAngle(float hyp, uint8_t servo_angle);
void move_f();
void  move_b();
void turn_l();
void  turn_r();
void halt();
void analog_move_f(unsigned int y_stick_val);
void analog_move_b(unsigned int y_stick_val);
void analog_l_drift(unsigned int x_stick_val);
void analog_r_drift(unsigned int x_stick_val);
bool activate_bt();
void deactive_bt();
void printRadiopacket(rx_data_packet structData);
void ease_servo_to_angle(uint8_t angle, Servo easeservo);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(200);
  Serial.println("hello new world");
 
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  hindlegs_servo.setPeriodHertz(50);    // standard 50 hz servo
  hindlegs_servo.attach(hindlegs_servo_pin, 500, 2500); // attaches the servo on servo pin 26 to the servo object
  forelegs_servo.setPeriodHertz(50);    // standard 50 hz servo
  forelegs_servo.attach(forelegs_servo_pin, 500, 2500); // attaches the servo on servo pin 25 to the servo object
  neck_rot_servo.setPeriodHertz(50);    // standard 50 hz servo
  neck_rot_servo.attach(neck_rot_servo_pin, 500, 2500); // attaches the servo on servo pin 33 to the servo object
  sonar_servo.setPeriodHertz(50);    // standard 50 hz servo
  sonar_servo.attach(sonar_servo_pin, 500, 2500); // attaches the servo on servo pin 32 to the servo object


  hindlegs_servo.write(90);
  forelegs_servo.write(90);
  neck_rot_servo.write(90 + neck_servo_offset);
  sonar_servo.write(sonar_default_angle);
  // Attach pins to channels
  ledcAttach(motorAcontrolPins[0], PWM_FREQ, pwm_resolution_bit);
  ledcAttach(motorBcontrolPins[0], PWM_FREQ, pwm_resolution_bit);

  // Initialize Radio Object
  if(!radio.begin())
  {
    Serial.println("Radio Hardware not responding");
    while(1)
    {

    }
  }

  // Initialize I2C and I2C gpio expander
  Wire.begin(SDA, SCL);
  Serial.print("pcf initialization is "); Serial.println(pcf8574.begin());  
  pcf8574.begin();
  if(pcf8574.begin() < 1)
  {
    Serial.println("Failed to initialize PCF8574. Check connections! ");  
    while(1)
    {

    }
  }
  halt();
  //one of the pins on the I/O expander got damaged, so i substituted the last pin with GPIO_14
  pinMode(GPIO_NUM_14, OUTPUT);
  digitalWrite(GPIO_NUM_14, LOW);
  // radio.openWritingPipe(addresses[1]);  //For some reason the esp32 isn't able to function as a transmitter
  radio.openReadingPipe(1, addresses[0]);

  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();
  Serial.println("Esp starts as receiver");


  // sweepandcheck(30, 150);

}

void loop() {
  // put your main code here, to run repeatedly:

  if (radio.available())
  {
    rx_data_packet received_data;
    radio.read(&received_data, sizeof(received_data));
    // printRadiopacket(received_data);
    radio_system_control(received_data);

  }
  else{

    halt();
  }
  Serial.print("average distance is "); Serial.println(averageSonarDist());

}





void radio_system_control(rx_data_packet received_data)
{

  bool analog_shifted_f = (atoi(received_data.y_axis_val) > stick_f_active_val) && ((atoi(received_data.x_axis_val) < stick_r_active_val) && (atoi(received_data.x_axis_val) > stick_l_active_val));
  bool analog_shifted_b = (atoi(received_data.y_axis_val) < stick_b_active_val) && ((atoi(received_data.x_axis_val) < stick_r_active_val) && (atoi(received_data.x_axis_val) > stick_l_active_val));
  bool analog_shifted_l = (atoi(received_data.x_axis_val) < stick_l_active_val) && ((atoi(received_data.y_axis_val) > stick_b_active_val) && (atoi(received_data.y_axis_val) < stick_f_active_val));
  bool analog_shifted_r = (atoi(received_data.x_axis_val) > stick_r_active_val) && ((atoi(received_data.y_axis_val) > stick_b_active_val) && (atoi(received_data.y_axis_val) < stick_f_active_val));
  bool analog_shifted = analog_shifted_f || analog_shifted_r || analog_shifted_b || analog_shifted_l;

  bool f_btn_state = received_data.f_btn_state;
  bool b_btn_state = received_data.b_btn_state;
  bool l_btn_state = received_data.l_btn_state;
  bool r_btn_state = received_data.r_btn_state;

  bool l_one_btn_state = received_data.l_one_state;
  bool l_two_btn_state = received_data.l_two_state;
  bool r_one_btn_state = received_data.r_one_state;
  bool r_two_btn_state = received_data.r_two_state;

  bool dir_btn_pressed = (f_btn_state == 1 || b_btn_state == 1 || l_btn_state == 1 || r_btn_state == 1 ); 
  //l_one is exempted for now, the button isn't functioning well on my RC pad   
  bool fun_btn_pressed = (r_one_btn_state == 1 || l_two_btn_state == 1 || r_two_btn_state == 1);
 
  if (dir_btn_pressed && !analog_shifted)
  {

    if (f_btn_state) {
      move_f();
      Serial.println("move_f ran");
      
    }

    else if (b_btn_state) {
      move_b();
      Serial.println("move_b ran");
      
    }

    else if (l_btn_state) {
      turn_l();
      Serial.println("turn_l ran");
      
    }

    else if (r_btn_state) {
      turn_r();
      Serial.println("turn_r ran");
      
    }
  }

  else if (analog_shifted && fun_btn_pressed == false)
  {
    if (analog_shifted_f) {
      analog_move_f(atoi(received_data.y_axis_val));
    }

    else if (analog_shifted_b) {
      analog_move_b(atoi(received_data.y_axis_val));
    }

    else if (analog_shifted_r) {
      analog_l_drift(atoi(received_data.x_axis_val));
    }

    else if (analog_shifted_l) {
      analog_r_drift(atoi(received_data.x_axis_val));
    }

  }

  else if(analog_shifted && fun_btn_pressed)
  {
    if((r_one_btn_state == true && analog_shifted_f == true))
    {
      uint8_t sonar_servo_angle = map(atoi(received_data.y_axis_val), stick_f_active_val, 255, 90, 130);
      Serial.print("Mapped angle is "); Serial.println(sonar_servo_angle);
      sonar_servo.write(sonar_servo_angle);
      delay(30);
    }
    else if ((r_one_btn_state == true && analog_shifted_b == true))
    {
      uint8_t sonar_servo_angle = map(atoi(received_data.y_axis_val), stick_b_active_val, 0, 90, 50);
      Serial.print("Mapped angle is "); Serial.println(sonar_servo_angle);
      ease_servo_to_angle(sonar_servo_angle, sonar_servo);
      delay(30);
    }

    else if((r_one_btn_state == true && analog_shifted_r == true))
    {

      Serial.println("neck rot servo activated to turn right"); 
      Serial.print("x axis val is "); Serial.println(atoi(received_data.x_axis_val));
      uint8_t neck_servo_angle = map(atoi(received_data.x_axis_val), stick_r_active_val, 255, (90 + neck_servo_offset), (30 + neck_servo_offset));
      Serial.print("Mapped angle is "); Serial.println(neck_servo_angle);
      neck_rot_servo.write(neck_servo_angle);
      delay(30);
    }

    else if((r_one_btn_state == true && analog_shifted_l == true))
    {

      Serial.println("neck rot servo activated to turn left"); 
      Serial.print("x axis val is "); Serial.println(atoi(received_data.x_axis_val));
      uint8_t neck_servo_angle = map(atoi(received_data.x_axis_val), stick_l_active_val, 0, (90 + neck_servo_offset), (150 + neck_servo_offset));
      Serial.print("Mapped angle is "); Serial.println(neck_servo_angle);
      neck_rot_servo.write(neck_servo_angle);
      delay(30);
    }
  }
  
  else if(fun_btn_pressed && analog_shifted == false)
  {
    if(l_two_btn_state)
    {
      neck_rot_servo.write(90 + neck_servo_offset);
      sonar_servo.write(sonar_default_angle);
    }
  }

  else
  {
    halt();
  }
  delay(70);
  // delay(140);
}

// Motion Functions
void analog_move_f(unsigned int y_stick_val)
{
  unsigned int mapped_speed = map(y_stick_val, stick_f_active_val, 255, analog_base_speed, pwm_resolution);
  if(mapped_speed >= 4000)
  {
    mapped_speed = pwm_resolution;
  }
  ledcWrite(motorAcontrolPins[0], mapped_speed);
  ledcWrite(motorBcontrolPins[0], mapped_speed);

  pcf8574.write(motorAcontrolPins[1], HIGH);
  pcf8574.write(motorAcontrolPins[2], LOW); 
  pcf8574.write(motorBcontrolPins[1], LOW);
  digitalWrite(motorBcontrolPins[2], HIGH);    

  pcf8574.write(motorCcontrolPins[1], HIGH);
  pcf8574.write(motorCcontrolPins[2], LOW); 
  pcf8574.write(motorDcontrolPins[1], HIGH);
  pcf8574.write(motorDcontrolPins[2], LOW);
}

void analog_move_b(unsigned int y_stick_val)
{
  unsigned int mapped_speed = map(y_stick_val, stick_b_active_val, 0, analog_base_speed, pwm_resolution);
  if(mapped_speed >= 4000)
  {
    mapped_speed = pwm_resolution;
  }
  ledcWrite(motorAcontrolPins[0], mapped_speed);
  ledcWrite(motorBcontrolPins[0], mapped_speed);

  pcf8574.write(motorAcontrolPins[1], LOW);
  pcf8574.write(motorAcontrolPins[2], HIGH); 

  pcf8574.write(motorBcontrolPins[1], HIGH);
  digitalWrite(motorBcontrolPins[2], LOW);   

  pcf8574.write(motorCcontrolPins[1], LOW);
  pcf8574.write(motorCcontrolPins[2], HIGH); 
  pcf8574.write(motorDcontrolPins[1], LOW);
  pcf8574.write(motorDcontrolPins[2], HIGH);  


}

void analog_l_drift(unsigned int x_stick_val)
{

  unsigned int drift_difference = map(x_stick_val, stick_r_active_val, 0, 100, 4094 - analog_base_speed);

  ledcWrite(motorAcontrolPins[0], analog_base_speed);
  ledcWrite(motorBcontrolPins[0], (analog_base_speed + drift_difference));

  pcf8574.write(motorAcontrolPins[1], LOW);
  pcf8574.write(motorAcontrolPins[2], HIGH); 
  pcf8574.write(motorBcontrolPins[1], LOW);
  digitalWrite(motorBcontrolPins[2], HIGH);

  pcf8574.write(motorCcontrolPins[1], LOW);
  pcf8574.write(motorCcontrolPins[2], HIGH); 
  pcf8574.write(motorDcontrolPins[1], HIGH);
  pcf8574.write(motorDcontrolPins[2], LOW);
}

void analog_r_drift(unsigned int x_stick_val)
{
  unsigned int drift_difference = map(x_stick_val, stick_l_active_val, 255, 100, 4094 - analog_base_speed);

  ledcWrite(motorBcontrolPins[0], analog_base_speed);
  ledcWrite(motorAcontrolPins[0], (analog_base_speed + drift_difference));

  pcf8574.write(motorAcontrolPins[1], HIGH);
  pcf8574.write(motorAcontrolPins[2], LOW); 
  pcf8574.write(motorBcontrolPins[1], LOW);
  digitalWrite(motorBcontrolPins[2], HIGH);

  pcf8574.write(motorCcontrolPins[1], HIGH);
  pcf8574.write(motorCcontrolPins[2], LOW); 
  pcf8574.write(motorDcontrolPins[1], HIGH);
  pcf8574.write(motorDcontrolPins[2], LOW);
}

void move_f()
{
  ledcWrite(motorAcontrolPins[0], pwm_resolution);
  ledcWrite(motorBcontrolPins[0], pwm_resolution);

  pcf8574.write(motorAcontrolPins[1], HIGH);
  pcf8574.write(motorAcontrolPins[2], LOW); 
  pcf8574.write(motorBcontrolPins[1], LOW);
  digitalWrite(motorBcontrolPins[2], HIGH);    

  pcf8574.write(motorCcontrolPins[1], HIGH);
  pcf8574.write(motorCcontrolPins[2], LOW); 
  pcf8574.write(motorDcontrolPins[1], HIGH);
  pcf8574.write(motorDcontrolPins[2], LOW);
}

void move_b()
{

  ledcWrite(motorAcontrolPins[0], pwm_resolution);
  ledcWrite(motorBcontrolPins[0], pwm_resolution);

  pcf8574.write(motorAcontrolPins[1], LOW);
  pcf8574.write(motorAcontrolPins[2], HIGH); 

  pcf8574.write(motorBcontrolPins[1], HIGH);
  digitalWrite(motorBcontrolPins[2], LOW);   

  pcf8574.write(motorCcontrolPins[1], LOW);
  pcf8574.write(motorCcontrolPins[2], HIGH); 
  pcf8574.write(motorDcontrolPins[1], LOW);
  pcf8574.write(motorDcontrolPins[2], HIGH);  
}

void turn_l()
{
  ledcWrite(motorAcontrolPins[0], pwm_resolution);
  ledcWrite(motorBcontrolPins[0], pwm_resolution);


  pcf8574.write(motorAcontrolPins[1], LOW);
  pcf8574.write(motorAcontrolPins[2], HIGH); 
  pcf8574.write(motorCcontrolPins[1], LOW);
  pcf8574.write(motorCcontrolPins[2], HIGH);   

  pcf8574.write(motorBcontrolPins[1], LOW);
  digitalWrite(motorBcontrolPins[2], HIGH); 
  pcf8574.write(motorDcontrolPins[1], HIGH);
  pcf8574.write(motorDcontrolPins[2], LOW);
}

void turn_r()
{
  ledcWrite(motorAcontrolPins[0], pwm_resolution);
  ledcWrite(motorBcontrolPins[0], pwm_resolution);


  pcf8574.write(motorAcontrolPins[1], HIGH);
  pcf8574.write(motorAcontrolPins[2], LOW); 
  pcf8574.write(motorCcontrolPins[1], HIGH);
  pcf8574.write(motorCcontrolPins[2], LOW);   

  pcf8574.write(motorBcontrolPins[1], HIGH);
  digitalWrite(motorBcontrolPins[2], LOW); 
  pcf8574.write(motorDcontrolPins[1], LOW);
  pcf8574.write(motorDcontrolPins[2], HIGH);
}

void halt()
{
  ledcWrite(motorAcontrolPins[0], 0);
  ledcWrite(motorBcontrolPins[0], 0);

  pcf8574.write(motorAcontrolPins[1], LOW);
  pcf8574.write(motorBcontrolPins[1], LOW); 
  pcf8574.write(motorAcontrolPins[2], LOW);
  digitalWrite(motorBcontrolPins[2], LOW);

  pcf8574.write(motorCcontrolPins[1], LOW);
  pcf8574.write(motorDcontrolPins[1], LOW); 
  pcf8574.write(motorCcontrolPins[2], LOW);
  pcf8574.write(motorDcontrolPins[2], LOW);
}




// BT functions
bool activate_bt()
{

  if(SerialBT.begin(device_name))
  {
    Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());    
    return true;    
  }
  return false;

}

void deactive_bt()
{
  SerialBT.end();
}


void printRadiopacket(rx_data_packet structData)
{

  Serial.print("f switch "); Serial.print(" is "); Serial.println(structData.f_btn_state);
  Serial.print("b switch "); Serial.print(" is "); Serial.println(structData.b_btn_state);
  Serial.print("l switch "); Serial.print(" is "); Serial.println(structData.l_btn_state);
  Serial.print("r switch "); Serial.print(" is "); Serial.println(structData.r_btn_state);

  Serial.println();
  Serial.println();

  Serial.print("L1 switch "); Serial.print(" is "); Serial.println(structData.l_one_state);
  Serial.print("R1 switch "); Serial.print(" is "); Serial.println(structData.r_one_state);
  Serial.print("L2 switch "); Serial.print(" is "); Serial.println(structData.l_two_state);
  Serial.print("R2 switch "); Serial.print(" is "); Serial.println(structData.r_two_state);

  Serial.println();
  Serial.println();

  Serial.print("X axis is "); Serial.println(structData.x_axis_val);
  Serial.print("Y axis is "); Serial.println(structData.y_axis_val);

}


// Sonar Functions
float dist_mm()
{
  unsigned long ping_time = sonar.ping();
  float distance_mm = (ping_time * 0.0343) / 2;
  return distance_mm;
}
// so far I've noticed that the averagedist function runs 10 to max of 18 times for each duration of 1000ms,
// which means for each 1000ms we get a minimum of 50 pings, which is plenty. 
float averageSonarDist()
{
  float average_dist = 0;
  float cummulative_dist = 0;
  long start_time = millis();
  for(uint8_t i = 0; i < 3; i++)
  {
    float present_dst = dist_mm();
    cummulative_dist += present_dst;
    // Serial.print("present distance is "); Serial.println(present_dst);
    delay(7);
  }
  average_dist = cummulative_dist / 3;
  // Serial.print("average dist took "); Serial.print(millis() - start_time); Serial.println(" milliseconds");
  return average_dist;
}


void sweepandcheck(uint8_t startAngle, uint8_t endAngle)
{
  // let's  start with finding the length of one obstacle

  // the current version of calcsendistatangle assumes the hyp dist and adj dist gotten from the sensor is that of a right-angled triangle
  // which returns a wrong value if hyp and adj isn't  that of a right angle triangle
  // to fix this, we would use the cosing rule to find the true length at the servo angle which would mean we need the sensor distance at
  // 90 degree 
  sweeped_data returned_data;
  float hyp_dist_from_obstacle = 0;
  float adj_dist_from_obstacle = 0;
  bool obstacleIsPresent = false;
  float prev_hyp_dist_from_obstacle = 0;
  float space_width = 0;
  uint8_t obstacle_start_angle = 0;
  uint8_t obstacle_end_angle = 0;

  neck_rot_servo.write(90 + neck_servo_offset); // im using 90 + neck_servo_offset because the midpoint of my hardware setup(ultrasonic sensor and servo) is at 90 + neck_servo_offset not 90
  delay(400);
  distAtninetyDeg = averageSonarDist();
  Serial.print("distAtninetyDeg  is "); Serial.println(distAtninetyDeg);
  f_sonar_ground_dist = distAtninetyDeg + 1.0;
  distAtninetyDeg = distAtninetyDeg - 1.0;
  neck_rot_servo.write(90 + neck_servo_offset);
  float distAtMidpoint = averageSonarDist(); //This distance is recorded at angle 90 which would be our opposite reference angle
  for (uint8_t pos = startAngle; pos <= endAngle; pos++) 
  {
    neck_rot_servo.write(pos);              // tell servo to go to position in variable 'pos'
    // Serial.print("servo pos is "); Serial.println(neck_rot_servo.read());
    float average_dist = averageSonarDist();
    // Serial.print("average distance is "); Serial.println(average_dist);
    // Serial.println();

    float distAtAngle = calcSenDistAtAngle(average_dist, neck_rot_servo.read() - 3);
    // Serial.print("average distance is "); Serial.println(average_dist);
    // Serial.print("calcsendist is "); Serial.println(distAtAngle);
    if(distAtAngle >= distAtninetyDeg && distAtAngle < f_sonar_ground_dist && obstacleIsPresent == false)
    {
      hyp_dist_from_obstacle = average_dist;
      prev_hyp_dist_from_obstacle = hyp_dist_from_obstacle;
      obstacleIsPresent = true;
      obstacle_start_angle = neck_rot_servo.read() - 3;
      Serial.print("start angle for "); Serial.println(obstacle_start_angle);
      Serial.print("average distance is "); Serial.println(average_dist);
      Serial.print("servo pos is "); Serial.println(neck_rot_servo.read());
      Serial.print("calcsendist is "); Serial.println(distAtAngle);
      Serial.println();
      // delay(2500);

    }

    else if(distAtAngle >= distAtninetyDeg && distAtAngle < f_sonar_ground_dist && obstacleIsPresent == true)
    {
      prev_hyp_dist_from_obstacle = average_dist;
      // Serial.print("average distance is "); Serial.println(average_dist);
      // Serial.print("servo pos is "); Serial.println(neck_rot_servo.read());
    }

    else if(distAtAngle >= f_sonar_ground_dist && obstacleIsPresent == true) // 
    {
      adj_dist_from_obstacle = prev_hyp_dist_from_obstacle;
      obstacleIsPresent = false;
      obstacle_end_angle = neck_rot_servo.read() - 3;
      space_width = cosine_rule(hyp_dist_from_obstacle, adj_dist_from_obstacle, (obstacle_end_angle - obstacle_start_angle));
      Serial.print("obstacle width is "); Serial.println(space_width);
      Serial.print("end angle for "); Serial.println(obstacle_end_angle);
      Serial.print("average distance is "); Serial.println(average_dist);
      Serial.print("calcsendist is "); Serial.println(distAtAngle);
      Serial.print("calcsendist at prev angle is "); Serial.println(adj_dist_from_obstacle);
      Serial.print("servo pos is "); Serial.println(neck_rot_servo.read());
      Serial.println("first loop");
      Serial.println();
      Serial.println();
      Serial.println();
      hyp_dist_from_obstacle = 0;
      adj_dist_from_obstacle = 0;
      obstacle_end_angle = 0;
      obstacle_start_angle = 0;
      prev_hyp_dist_from_obstacle = 0;
      // delay(2500);
    }

    delayMicroseconds(10);                      // waits 15 ms for the servo to reach the position

  }

  // obstacleIsPresent = false;
  Serial.println("second loop ");
  for (uint8_t pos = endAngle; pos >= startAngle; pos--) 
  {
    neck_rot_servo.write(pos);              // tell servo to go to position in variable 'pos'
    // Serial.print("servo pos is "); Serial.println(neck_rot_servo.read());
    float average_dist = averageSonarDist();
    // Serial.print("average distance is "); Serial.println(average_dist);
    // Serial.println();
    float distAtAngle = calcSenDistAtAngle(average_dist, neck_rot_servo.read() - 3);
    // Serial.print("average distance is "); Serial.println(average_dist);
    // Serial.print("calcsendist is "); Serial.println(distAtAngle);
    if(distAtAngle >= distAtninetyDeg && distAtAngle < f_sonar_ground_dist && obstacleIsPresent == false)
    {
      hyp_dist_from_obstacle = average_dist;
      prev_hyp_dist_from_obstacle = hyp_dist_from_obstacle;
      obstacle_start_angle = neck_rot_servo.read() - 3;
      obstacleIsPresent = true;
      Serial.print("servo pos is "); Serial.println(neck_rot_servo.read());
      Serial.print("start angle for "); Serial.println(obstacle_start_angle);
      Serial.print("average distance is "); Serial.println(average_dist);
      Serial.print("calcsendist is "); Serial.println(distAtAngle);
      // delay(2500);
    }

    else if(distAtAngle >= distAtninetyDeg && distAtAngle < f_sonar_ground_dist && obstacleIsPresent == true)
    {
      prev_hyp_dist_from_obstacle = average_dist;
      // Serial.print("servo pos is "); Serial.println(neck_rot_servo.read());
      // Serial.print("average distance is "); Serial.println(average_dist);
    }

    else if(distAtAngle >= f_sonar_ground_dist && obstacleIsPresent == true)
    {
      adj_dist_from_obstacle = prev_hyp_dist_from_obstacle;
      obstacleIsPresent = false;
      obstacle_end_angle = neck_rot_servo.read() - 3;
      space_width = cosine_rule(hyp_dist_from_obstacle, adj_dist_from_obstacle, (obstacle_end_angle - obstacle_start_angle));
      Serial.print("obstacle width is "); Serial.println(space_width);
      Serial.print("servo pos is "); Serial.println(neck_rot_servo.read());
      Serial.print("end angle is "); Serial.println(obstacle_end_angle);
      Serial.print("average distance is "); Serial.println(average_dist);
      Serial.print("calcsendist is "); Serial.println(distAtAngle);
      Serial.println();
      Serial.println();
      Serial.println();
      hyp_dist_from_obstacle = 0;
      adj_dist_from_obstacle = 0;
      prev_hyp_dist_from_obstacle = 0;
    }
    delayMicroseconds(10);                      // waits 15 ms for the servo to reach the position    
  }

}

float cosine_rule(float hyp, float opp, float angle)
{
  return sqrt(pow(hyp, 2) + pow(opp, 2) - (2 * hyp * opp) * cos(angle * DEG_TO_RAD));
}


void sweepandread(uint8_t start_angle, uint8_t end_angle)
{
  long loop_start_time = millis();
  for (uint8_t pos = start_angle; pos <= end_angle; pos++) 
  {   
    // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
    neck_rot_servo.write(pos);              // tell servo to go to position in variable 'pos'

    float average_dist = averageSonarDist();
    Serial.print("servo pos is "); Serial.println(neck_rot_servo.read());      
    Serial.print("average distance is "); Serial.println(average_dist);
    Serial.print("calcSendistAtAngle is "); Serial.println(calcSenDistAtAngle(average_dist, neck_rot_servo.read()));  
    Serial.println();        
      delayMicroseconds(10);                       // waits 15 ms for the servo to reach the position
  }
  for (uint8_t pos = end_angle; pos >= start_angle; pos--)
  { // goes from 180 degrees to 0 degrees
    neck_rot_servo.write(pos);              // tell servo to go to position in variable 'pos'

    float average_dist = averageSonarDist();
    // if(calcSenDistAtAngle(average_dist, neck_rot_servo.read()) >= 6.20)
    // {
      Serial.print("servo pos is "); Serial.println(neck_rot_servo.read());      
      Serial.print("average distance is "); Serial.println(average_dist);
      Serial.print("calcSendistAtAngle is "); Serial.println(calcSenDistAtAngle(average_dist, neck_rot_servo.read()));  
      Serial.println();        
    // }
    delayMicroseconds(10);                       // waits 15 ms for the servo to reach the position
  }
  Serial.print("sweepandread back and forth took "); Serial.print(millis() - loop_start_time); Serial.println(" milliseconds");

}



float calcSenDistAtAngle(float hyp, uint8_t servo_angle)
{
  if(servo_angle <= 90)
  {
    return (cos((90 - servo_angle) * DEG_TO_RAD) * hyp);    
  }

  else if(servo_angle > 90)
  {
    return (cos((90 - (180 - servo_angle)) * DEG_TO_RAD) * hyp);
  }
}


void readModeFromSerial()
{
  if(Serial.available() > 0)
  {
    String serialInput = Serial.readStringUntil('\n');
    char CHAR = serialInput[0];
    CHAR = toupper(CHAR);
    if(CHAR == 'F' || CHAR == 'B' || CHAR == 'L' || CHAR == 'R' || CHAR == 'N' || CHAR == 'A')
    {
      controlMode = CHAR;   
      if(CHAR == 'A')
      {
        int analog_speed = (serialInput.substring(1)).toInt();
        Serial.println(analog_speed);
        if(analog_speed >= 0 || analog_speed <= 255)
        {
          Serial.println("analog mode");
          controlMode = 'A';
          global_analog_speed = analog_speed;
        }
        else
        {
          Serial.println("Enter a number between 0 and 255");
        }
      }
    }
    else
    {
      Serial.println("Please Enter the Character F or B or L or R to control the motors.");        
    }
  }
}


void ease_neck_servo_to_angle(uint8_t angle)
{
  uint8_t start_pos = neck_rot_servo.read();  
  Serial.print("neck servo present angle is "); Serial.println(start_pos);
  Serial.print("set angle is "); Serial.println(angle);
  if(start_pos - angle < 0)
  {
    for(uint8_t i = 0; i <= angle - start_pos; i++)
    {
      neck_rot_servo.write(start_pos + i); 
      Serial.print("neck Servo pos is "); Serial.println(neck_rot_servo.read());
      delay(10);  
    }
    if(neck_rot_servo.read() != angle)
    {
      neck_rot_servo.write(angle);
      Serial.println("final angle was forcefully written");
      Serial.print("servo pos is now "); Serial.println(neck_rot_servo.read());
    }
    return;
  }

  else if(start_pos - angle > 0)
  {
    for(uint8_t i = 0; i <= start_pos - angle; i++)
    {
      neck_rot_servo.write(start_pos - i);
      Serial.print("neck Servo pos is "); Serial.println(neck_rot_servo.read());
      delay(10);
    }
    if(neck_rot_servo.read() != angle)
    {
      neck_rot_servo.write(angle);
      Serial.println("final angle was forcefully written");
      Serial.print("servo pos is now "); Serial.println(neck_rot_servo.read());
    }
    return;
  }
}


void ease_servo_to_angle(uint8_t angle, Servo easeservo)
{
  uint8_t start_pos = easeservo.read();  
  if(start_pos - angle < 0)
  {

    for(uint8_t i = 0; i <= angle - start_pos; i++)
    {
      easeservo.write(start_pos + i); 
      Serial.print("Sonar servo pos is "); Serial.println(easeservo.read());
      delay(10);
    }
    if(easeservo.read() != angle)
    {
      neck_rot_servo.write(angle);
      Serial.println("final angle was forcefully written");
      Serial.print("servo pos is now "); Serial.println(easeservo.read());
    }
    return;
  }

  else if(start_pos - angle > 0)
  {
    for(uint8_t i = 0; i <= start_pos - angle; i++)
    {
      easeservo.write(start_pos - i);
      Serial.print("Sonar servo pos is "); Serial.println(easeservo.read());
      delay(10);
    }
    if(easeservo.read() != angle)
    {
      neck_rot_servo.write(angle);
      Serial.println("final angle was forcefully written");
      Serial.print("servo pos is now "); Serial.println(easeservo.read());
    }
    return;
  }
}
