#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "driver/ledc.h"
#include <ESP32Servo.h>
#include <NewPing.h>
#include <NimBLEDevice.h>
#include <math.h>
#include "esp_pm.h"

// Global Constants
#define SVC_UUID "9caacf7b-2da1-413a-b00a-5f02d98c5c89"
#define CHAR_UUID "a409f8e5-016c-4320-ad4f-35011362984c"

static NimBLEServer* pServer;
NimBLECharacteristic* pFoodCharacteristic;
bool isAdvertising = false;

// BLE Data packet struct
typedef struct __attribute__((packed)) BLE_rx_data_packet {
  bool d_pad_f_btn_state;  //forward button state
  bool d_pad_b_btn_state;  //backward button state
  bool d_pad_l_btn_state;  //left button state
  bool d_pad_r_btn_state;  //right button state
} BLE_rx_data_packet;


BLE_rx_data_packet BLE_data_packet;

// Function to set up and start advertising
void setupAdvertising() {
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();

  // Stop any ongoing advertising first
  pAdvertising->stop();
  delay(100);  // Short delay

  // Set device name and service UUID
  pAdvertising->setName("C1 Robot");
  pAdvertising->addServiceUUID(SVC_UUID);

  // Set more aggressive advertising parameters
  pAdvertising->setMinInterval(100);  // 100ms (in units of 0.625ms)
  pAdvertising->setMaxInterval(200);  // 200ms (in units of 0.625ms)

  // Enable scan response
  pAdvertising->enableScanResponse(true);

  // Start advertising
  pAdvertising->start();
  isAdvertising = true;

  Serial.println("Advertising started");
}


class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    Serial.printf("Client connected: %s\n", connInfo.getAddress().toString().c_str());
    pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
    isAdvertising = false;
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    Serial.printf("Client disconnected - restarting advertising\n");

    // Add a small delay before restarting advertising
    delay(100);

    // Restart advertising
    setupAdvertising();
  }

  void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
    Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
  }

  void onConfirmPassKey(NimBLEConnInfo& connInfo, uint32_t pass_key) override {
    Serial.printf("Confirming passkey: %06d\n", pass_key);
    // We'll automatically confirm any passkey
    NimBLEDevice::injectConfirmPasskey(connInfo, true);
  }

  void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
    if (!connInfo.isEncrypted()) {
      NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
      Serial.println("Encryption failed - disconnecting client");
      return;
    }
    Serial.printf("Secured connection to: %s\n", connInfo.getAddress().toString().c_str());
  }
} serverCallbacks;

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string value = pCharacteristic->getValue();
    Serial.printf("Received command: ");
    for (auto& c : value) {
      Serial.printf("%02X ", (uint8_t)c);
    }
    Serial.println();

    if (value.length() >= 1) {
      uint8_t command = value[0];
      BLE_data_packet.d_pad_f_btn_state = command & 0x01;
      BLE_data_packet.d_pad_b_btn_state = command & 0x02;
      BLE_data_packet.d_pad_l_btn_state = command & 0x04;
      BLE_data_packet.d_pad_r_btn_state = command & 0x08;
      Serial.printf("Command: Fwd:%d Back:%d Left:%d Right:%d\n",
                    BLE_data_packet.d_pad_f_btn_state, BLE_data_packet.d_pad_b_btn_state, BLE_data_packet.d_pad_l_btn_state, BLE_data_packet.d_pad_r_btn_state);
    }
  }
} chrCallbacks;

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
#define TRIGGER_PIN 27
#define ECHO_PIN 15
#define MAX_DISTANCE 500

#define GROUND_DIST_ALLW 5
#define RF_SCAN_TIMEOUT 5000
#define POWER_PERIPHERAL_PIN 1

// neck_servo_pos_offset
#define neck_servo_offset -10
#define sonar_default_angle 50

// SPI pins
#define MOSI 23
#define MISO 19
#define SCK 18
#define CE 2
#define CSN 5


// Radio Data packet struct
typedef struct __attribute__((packed)) nrf_rx_data_packet {
  bool f_btn_state;  //forward button state
  bool b_btn_state;  //backward button state
  bool l_btn_state;  //left button state
  bool r_btn_state;  //right button state

  char x_axis_val[5];
  char y_axis_val[5];

  bool l_one_state;
  bool r_one_state;
  bool l_two_state;
  bool r_two_state;
} nrf_rx_data_packet;




// Radio state struct
typedef enum wireless_conn_mode {
  BLE,
  NRF,
  ESPNOW
} wireless_conn_mode;



typedef struct _sweeped_data {
  bool spaceisavailable;
  float obstacle_width;
  float space_width;
  char side;
} sweeped_data;

const uint8_t motorAcontrolPins[3] = { GPIO_NUM_4, pcf8574_IO_1, pcf8574_IO_0 };
const uint8_t motorBcontrolPins[3] = { GPIO_NUM_13, pcf8574_IO_3, GPIO_NUM_14 };
const uint8_t motorDcontrolPins[3] = { GPIO_NUM_4, pcf8574_IO_7, pcf8574_IO_6 };
const uint8_t motorCcontrolPins[3] = { GPIO_NUM_13, pcf8574_IO_5, pcf8574_IO_4 };

const byte addresses[][6] = { "00001", "00002" };

// safe distance recorded distance from obstacle
float f_sonar_ground_dist = 8.00;  //cm
float f_sonar_fall_dist = f_sonar_ground_dist + (GROUND_DIST_ALLW);
uint8_t bot_width = 4;
float distAtninetyDeg;

bool f_sonar_obstacle_flag;
bool f_sonar_fall_flag;

volatile char controlMode = 'N';
int global_analog_speed = 0;


// peripherals and objects
RF24 radio(CE, CSN);  // CE, CSN
PCF8574 pcf8574(0X38);
String device_name = "C1";
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);


// Servo objects
Servo hindlegs_servo;
Servo forelegs_servo;
Servo neck_rot_servo;
Servo sonar_servo;

// Servo pins
const uint8_t hindlegs_servo_pin = 33;
const uint8_t forelegs_servo_pin = 32;
const uint8_t neck_rot_servo_pin = 26;
const uint8_t sonar_servo_pin = 25;


// Functions initialization

// Motion Functions
void readModeFromSerial();

void sweepandcheck(uint8_t startAngle, uint8_t endAngle);
void sweepandread(uint8_t start_angle, uint8_t end_angle);
float calcSenDistAtAngle(float hyp, uint8_t servo_angle);
void move_f();
void move_b();
void turn_l();
void turn_r();
void halt();
void analog_move_f(unsigned int y_stick_val);
void analog_move_b(unsigned int y_stick_val);
void analog_l_drift(unsigned int x_stick_val);
void analog_r_drift(unsigned int x_stick_val);
void ease_servo_to_angle(uint8_t angle, Servo easeservo);

// Wireless connection Function

// BLE
void setupBLE();
void deactivateBLE();

// NRF24L01
void setupNRF();
void deactivateNRF();
void nrf_system_control(nrf_rx_data_packet received_data);
void printRadiopacket(nrf_rx_data_packet structData);

// ESPNOW
void setupESPNOW();
void deactivateESPNOW();


wireless_conn_mode wireless_mode = NRF;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(200);

  initPCF8574();
  Serial.println("initialized i2xc expander");
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  hindlegs_servo.setPeriodHertz(50);                     // standard 50 hz servo
  hindlegs_servo.attach(hindlegs_servo_pin, 500, 2500);  // attaches the servo on servo pin 26 to the servo object
  forelegs_servo.setPeriodHertz(50);                     // standard 50 hz servo
  forelegs_servo.attach(forelegs_servo_pin, 500, 2500);  // attaches the servo on servo pin 25 to the servo object
  neck_rot_servo.setPeriodHertz(50);                     // standard 50 hz servo
  neck_rot_servo.attach(neck_rot_servo_pin, 500, 2500);  // attaches the servo on servo pin 33 to the servo object
  sonar_servo.setPeriodHertz(50);                        // standard 50 hz servo
  sonar_servo.attach(sonar_servo_pin, 500, 2500);        // attaches the servo on servo pin 32 to the servo object


  // Attach pins to channels
  ledcAttach(motorAcontrolPins[0], PWM_FREQ, pwm_resolution_bit);
  ledcAttach(motorBcontrolPins[0], PWM_FREQ, pwm_resolution_bit);




  // Initialize Radio Object

  // if(!radio.begin())
  // {
  //   Serial.println("Radio Hardware not responding");
  //   while(1)
  //   {

  //   }
  // }

  // Initialize I2C and I2C gpio expander
  // Wire.begin(SDA, SCL);
  // Serial.print("pcf initialization is "); Serial.println(pcf8574.begin());
  // pcf8574.begin();
  // if(pcf8574.begin() < 1)
  // {
  //   Serial.println("Failed to initialize PCF8574. Check connections! ");
  //   while(1)
  //   {

  //   }
  // }
  // halt();

  //one of the pins on the I/O expander got damaged, so i substituted the last pin with GPIO_14
  pinMode(GPIO_NUM_14, OUTPUT);
  digitalWrite(GPIO_NUM_14, LOW);
  // // radio.openWritingPipe(addresses[1]);  //For some reason the esp32 isn't able to function as a transmitter
  // radio.openReadingPipe(1, addresses[0]);

  // radio.setPALevel(RF24_PA_HIGH);
  // radio.startListening();
  hindlegs_servo.write(90);
  forelegs_servo.write(90);
  neck_rot_servo.write(90 + neck_servo_offset);
  sonar_servo.write(sonar_default_angle);
  // Serial.println("Esp starts as receiver");
  // delay(3000);
  // Set and configure wireless connectivity mode.

  sweepandcheck(45, 135);
  delay(100);
  neck_rot_servo.write(90 + neck_servo_offset);
  delay(500);
  // Serial.print("average distance is "); Serial.println(averageSonarDist());
  Serial.println("Hello World");
  switch (wireless_mode) {
    case BLE:
      setupBLE();
      break;
    case NRF:
      setupNRF();
      break;

    case ESPNOW:
      setupESPNOW();
      break;
    default:
      setupBLE();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (wireless_mode) {
    case BLE:
      if (!isAdvertising && pServer->getConnectedCount() == 0) {
        setupAdvertising();
      }
      BLE_system_control();
      break;
    case NRF:
      if (radio.available()) {
        nrf_rx_data_packet received_data;
        radio.read(&received_data, sizeof(received_data));
        // printRadiopacket(received_data);
        nrf_system_control(received_data);
      } else {
        halt();
      }
      break;
    case ESPNOW:
      break;
    default:
      halt();
      break;
  }
  // Serial.print("average distance is "); Serial.println(averageSonarDist());
}

void BLE_system_control() {

  if (BLE_data_packet.d_pad_f_btn_state) {
    move_f();
    Serial.println("move_f");
  } else if (BLE_data_packet.d_pad_b_btn_state) {
    move_b();
    Serial.println("move_b");
  } else if (BLE_data_packet.d_pad_l_btn_state) {
    turn_l();
    Serial.println("turn_l");
  } else if (BLE_data_packet.d_pad_r_btn_state) {
    turn_r();
    Serial.println("turn_r");
  } else {
    halt();
  }
  delay(50);
}

void initPCF8574() {
  Wire.begin(SDA, SCL);
  Serial.print("pcf initialization is ");
  Serial.println(pcf8574.begin());
  pcf8574.begin();
  if (pcf8574.begin() < 1) {
    Serial.println("Failed to initialize PCF8574. Check connections! ");
    while (1) {
    }
  }
  Serial.println("PCF8574. successfully initialize");
  halt();
  return;
}

void nrf_system_control(nrf_rx_data_packet received_data) {

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

  bool dir_btn_pressed = (f_btn_state == 1 || b_btn_state == 1 || l_btn_state == 1 || r_btn_state == 1);
  //l_one is exempted for now, the button isn't functioning well on my RC pad
  bool fun_btn_pressed = (r_one_btn_state == 1 || l_two_btn_state == 1 || r_two_btn_state == 1);

  if (dir_btn_pressed && !analog_shifted) {

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

  else if (analog_shifted && fun_btn_pressed == false) {
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

  else if (analog_shifted && fun_btn_pressed) {
    if ((r_one_btn_state == true && analog_shifted_f == true)) {
      uint8_t sonar_servo_angle = map(atoi(received_data.y_axis_val), stick_f_active_val, 255, 90, 130);
      Serial.print("Mapped angle is ");
      Serial.println(sonar_servo_angle);
      sonar_servo.write(sonar_servo_angle);
      delay(30);
    } else if ((r_one_btn_state == true && analog_shifted_b == true)) {
      uint8_t sonar_servo_angle = map(atoi(received_data.y_axis_val), stick_b_active_val, 0, 90, 50);
      Serial.print("Mapped angle is ");
      Serial.println(sonar_servo_angle);
      ease_servo_to_angle(sonar_servo_angle, sonar_servo);
      delay(30);
    }

    else if ((r_one_btn_state == true && analog_shifted_r == true)) {

      Serial.println("neck rot servo activated to turn right");
      Serial.print("x axis val is ");
      Serial.println(atoi(received_data.x_axis_val));
      uint8_t neck_servo_angle = map(atoi(received_data.x_axis_val), stick_r_active_val, 255, (90 + neck_servo_offset), (30 + neck_servo_offset));
      Serial.print("Mapped angle is ");
      Serial.println(neck_servo_angle);
      neck_rot_servo.write(neck_servo_angle);
      delay(30);
    }

    else if ((r_one_btn_state == true && analog_shifted_l == true)) {

      Serial.println("neck rot servo activated to turn left");
      Serial.print("x axis val is ");
      Serial.println(atoi(received_data.x_axis_val));
      uint8_t neck_servo_angle = map(atoi(received_data.x_axis_val), stick_l_active_val, 0, (90 + neck_servo_offset), (150 + neck_servo_offset));
      Serial.print("Mapped angle is ");
      Serial.println(neck_servo_angle);
      neck_rot_servo.write(neck_servo_angle);
      delay(30);
    }
  }

  else if (fun_btn_pressed && analog_shifted == false) {
    if (l_two_btn_state) {
      neck_rot_servo.write(90 + neck_servo_offset);
      sonar_servo.write(sonar_default_angle);
    } else if (l_two_btn_state) {
      sweepandcheck(45, 135);
      delay(100);
      neck_rot_servo.write(90 + neck_servo_offset);
      delay(500);
      Serial.print("average distance is ");
      Serial.println(averageSonarDist());
    }
  }

  else {
    halt();
  }
  delay(50);
}

void printMotorPinsState()
{
  Serial.printf("motorAcontrolPins[0] is %i\nmotorBcontrolPins[0] is %i\nmotorAcontrolPins[1] is %i\nmotorAcontrolPins[2] is %i\nmotorBcontrolPins[1] is %i\nmotorBcontrolPins[2] is %i\nmotorCcontrolPins[1] is %i\nmotorCcontrolPins[2] is %i\nmotorDcontrolPins[1] is %i\nmotorDcontrolPins[2] is %i\n",
                ledcRead(motorAcontrolPins[0]),
                ledcRead(motorBcontrolPins[0]),

                pcf8574.read(motorAcontrolPins[1]),
                pcf8574.read(motorAcontrolPins[2]),
                pcf8574.read(motorBcontrolPins[1]),
                digitalRead(motorBcontrolPins[2]),
                pcf8574.read(motorCcontrolPins[1]),
                pcf8574.read(motorCcontrolPins[2]),
                pcf8574.read(motorDcontrolPins[1]),
                pcf8574.read(motorDcontrolPins[2]));
}

// Motion Functions
void analog_move_f(unsigned int y_stick_val) {
  unsigned int mapped_speed = map(y_stick_val, stick_f_active_val, 255, analog_base_speed, pwm_resolution);

  if (mapped_speed >= 4000) {
    mapped_speed = pwm_resolution;
  }
  Serial.printf("Analog Stick value is %i\nMapped speed is %i\n", y_stick_val, mapped_speed);
  ledcWrite(motorAcontrolPins[0], mapped_speed);
  ledcWrite(motorBcontrolPins[0], mapped_speed);

  pcf8574.write(motorAcontrolPins[1], HIGH);
  pcf8574.write(motorAcontrolPins[2], LOW);
  pcf8574.write(motorCcontrolPins[1], HIGH);
  pcf8574.write(motorCcontrolPins[2], LOW);

  pcf8574.write(motorBcontrolPins[1], HIGH);
  digitalWrite(motorBcontrolPins[2], LOW);
  pcf8574.write(motorDcontrolPins[1], HIGH);
  pcf8574.write(motorDcontrolPins[2], LOW);
}

void analog_move_b(unsigned int y_stick_val) {
  unsigned int mapped_speed = map(y_stick_val, stick_b_active_val, 0, analog_base_speed, pwm_resolution);
  if (mapped_speed >= 4000) {
    mapped_speed = pwm_resolution;
  }
  Serial.printf("Analog Stick value is %i\nMapped speed is %i\n", y_stick_val, mapped_speed);
  ledcWrite(motorAcontrolPins[0], mapped_speed);
  ledcWrite(motorBcontrolPins[0], mapped_speed);

  pcf8574.write(motorAcontrolPins[1], LOW);
  pcf8574.write(motorAcontrolPins[2], HIGH);
  pcf8574.write(motorCcontrolPins[1], LOW);
  pcf8574.write(motorCcontrolPins[2], HIGH);

  pcf8574.write(motorBcontrolPins[1], LOW);
  digitalWrite(motorBcontrolPins[2], HIGH);
  pcf8574.write(motorDcontrolPins[1], LOW);
  pcf8574.write(motorDcontrolPins[2], HIGH);
}

void analog_l_drift(unsigned int x_stick_val) {

  unsigned int drift_difference = map(x_stick_val, stick_r_active_val, 0, 100, 4000 - analog_base_speed);

  ledcWrite(motorAcontrolPins[0], analog_base_speed);
  ledcWrite(motorBcontrolPins[0], (analog_base_speed + drift_difference));

  pcf8574.write(motorAcontrolPins[1], HIGH);
  pcf8574.write(motorAcontrolPins[2], LOW);
  pcf8574.write(motorCcontrolPins[1], HIGH);
  pcf8574.write(motorCcontrolPins[2], LOW);

  pcf8574.write(motorBcontrolPins[1], HIGH);
  digitalWrite(motorBcontrolPins[2], LOW);
  pcf8574.write(motorDcontrolPins[1], HIGH);
  pcf8574.write(motorDcontrolPins[2], LOW);
}

void analog_r_drift(unsigned int x_stick_val) {
  unsigned int drift_difference = map(x_stick_val, stick_l_active_val, 255, 100, 4000 - analog_base_speed);

  ledcWrite(motorBcontrolPins[0], analog_base_speed);
  ledcWrite(motorAcontrolPins[0], (analog_base_speed + drift_difference));

  pcf8574.write(motorAcontrolPins[1], HIGH);
  pcf8574.write(motorAcontrolPins[2], LOW);
  pcf8574.write(motorCcontrolPins[1], HIGH);
  pcf8574.write(motorCcontrolPins[2], LOW);

  pcf8574.write(motorBcontrolPins[1], HIGH);
  digitalWrite(motorBcontrolPins[2], LOW);
  pcf8574.write(motorDcontrolPins[1], HIGH);
  pcf8574.write(motorDcontrolPins[2], LOW);
}

void move_f() {
  ledcWrite(motorAcontrolPins[0], pwm_resolution);
  ledcWrite(motorBcontrolPins[0], pwm_resolution);

  pcf8574.write(motorAcontrolPins[1], HIGH);
  pcf8574.write(motorAcontrolPins[2], LOW);
  pcf8574.write(motorBcontrolPins[1], HIGH);
  digitalWrite(motorBcontrolPins[2], LOW);

  pcf8574.write(motorCcontrolPins[1], HIGH);
  pcf8574.write(motorCcontrolPins[2], LOW);
  pcf8574.write(motorDcontrolPins[1], HIGH);
  pcf8574.write(motorDcontrolPins[2], LOW);
}

void move_b() {

  ledcWrite(motorAcontrolPins[0], pwm_resolution);
  ledcWrite(motorBcontrolPins[0], pwm_resolution);

  pcf8574.write(motorAcontrolPins[1], LOW);
  pcf8574.write(motorAcontrolPins[2], HIGH);
  pcf8574.write(motorBcontrolPins[1], LOW);
  digitalWrite(motorBcontrolPins[2], HIGH);

  pcf8574.write(motorCcontrolPins[1], LOW);
  pcf8574.write(motorCcontrolPins[2], HIGH);
  pcf8574.write(motorDcontrolPins[1], LOW);
  pcf8574.write(motorDcontrolPins[2], HIGH);
}

void turn_l() {
  ledcWrite(motorAcontrolPins[0], pwm_resolution);
  ledcWrite(motorBcontrolPins[0], pwm_resolution);


  pcf8574.write(motorAcontrolPins[1], HIGH);
  pcf8574.write(motorAcontrolPins[2], LOW);
  pcf8574.write(motorCcontrolPins[1], HIGH);
  pcf8574.write(motorCcontrolPins[2], LOW);

  pcf8574.write(motorBcontrolPins[1], LOW);
  digitalWrite(motorBcontrolPins[2], HIGH);
  pcf8574.write(motorDcontrolPins[1], LOW);
  pcf8574.write(motorDcontrolPins[2], HIGH);
}

void turn_r() {
  ledcWrite(motorAcontrolPins[0], pwm_resolution);
  ledcWrite(motorBcontrolPins[0], pwm_resolution);


  pcf8574.write(motorAcontrolPins[1], LOW);
  pcf8574.write(motorAcontrolPins[2], HIGH);
  pcf8574.write(motorCcontrolPins[1], LOW);
  pcf8574.write(motorCcontrolPins[2], HIGH);

  pcf8574.write(motorBcontrolPins[1], HIGH);
  digitalWrite(motorBcontrolPins[2], LOW);
  pcf8574.write(motorDcontrolPins[1], HIGH);
  pcf8574.write(motorDcontrolPins[2], LOW);
}



void halt() {
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



void printRadiopacket(nrf_rx_data_packet structData) {

  Serial.print("f switch ");
  Serial.print(" is ");
  Serial.println(structData.f_btn_state);
  Serial.print("b switch ");
  Serial.print(" is ");
  Serial.println(structData.b_btn_state);
  Serial.print("l switch ");
  Serial.print(" is ");
  Serial.println(structData.l_btn_state);
  Serial.print("r switch ");
  Serial.print(" is ");
  Serial.println(structData.r_btn_state);

  Serial.println();
  Serial.println();

  Serial.print("L1 switch ");
  Serial.print(" is ");
  Serial.println(structData.l_one_state);
  Serial.print("R1 switch ");
  Serial.print(" is ");
  Serial.println(structData.r_one_state);
  Serial.print("L2 switch ");
  Serial.print(" is ");
  Serial.println(structData.l_two_state);
  Serial.print("R2 switch ");
  Serial.print(" is ");
  Serial.println(structData.r_two_state);

  Serial.println();
  Serial.println();

  Serial.print("X axis is ");
  Serial.println(structData.x_axis_val);
  Serial.print("Y axis is ");
  Serial.println(structData.y_axis_val);
}


// Sonar Functions
float dist_mm() {
  unsigned long ping_time = sonar.ping();
  float distance_mm = (ping_time * 0.0343) / 2;
  return distance_mm;
}
// so far I've noticed that the averagedist function runs 10 to max of 18 times for each duration of 1000ms,
// which means for each 1000ms we get a minimum of 50 pings, which is plenty.
float averageSonarDist() {
  float average_dist = 0;
  float cummulative_dist = 0;
  long start_time = millis();
  for (uint8_t i = 0; i < 3; i++) {
    float present_dst = dist_mm();
    cummulative_dist += present_dst;
    // Serial.print("present distance is "); Serial.println(present_dst);
    delay(7);
  }
  average_dist = cummulative_dist / 3;
  // Serial.print("average dist took "); Serial.print(millis() - start_time); Serial.println(" milliseconds");
  return average_dist;
}


void sweepandcheck(uint8_t startAngle, uint8_t endAngle) {
  // let's  start with finding the length of one obstacle
  // the current version of calcsendistatangle assumes the hyp dist and adj dist gotten from the sensor is that of a right-angled triangle
  // which returns a wrong value if hyp and adj isn't  that of a right angle triangle
  // to fix this, we would use the cosing rule to find the true length at the servo angle which would mean we need the sensor distance at
  // 90 degree
  sweeped_data returned_data;
  float hyp_dist_from_obstacle = 0;
  float adj_dist_from_obstacle = 0;
  bool thresholdDistanceIsMaintained;
  bool obstacleIsPresent = false;
  float prev_hyp_dist_from_obstacle = 0;
  float space_width = 0;
  uint8_t obstacle_start_angle = 0;
  uint8_t obstacle_end_angle = 0;

  neck_rot_servo.write(90 + neck_servo_offset);  // im using 90 + neck_servo_offset because the midpoint of my hardware setup(ultrasonic sensor and servo) is at 90 + neck_servo_offset not 90
  distAtninetyDeg = averageSonarDist();
  Serial.print("distAtninetyDeg  is ");
  Serial.println(distAtninetyDeg);
  // f_sonar_ground_dist = distAtninetyDeg + 1.0;
  neck_rot_servo.write(90 + neck_servo_offset);
  float distAtMidpoint = averageSonarDist();  //This distance is recorded at angle 90 which would be our opposite reference angle
  startAngle = startAngle + neck_servo_offset;
  endAngle = endAngle + neck_servo_offset;



  delay(100);
  for (uint8_t pos = startAngle; pos <= endAngle; pos++) {
    neck_rot_servo.write(pos);  // tell servo to go to position in variable 'pos'
    // Serial.printf("Servo offset angular pos is %i\n", neck_rot_servo.read() - neck_servo_offset);
    float average_dist = averageSonarDist();
    float distAtAngle = calcSenDistAtAngle(average_dist, neck_rot_servo.read() - neck_servo_offset);
    // Serial.print("average distance is "); Serial.println(average_dist);
    // Serial.print("calcsendist is "); Serial.println(distAtAngle);

    if ((distAtAngle >= distAtninetyDeg - 3) && (distAtAngle <= distAtninetyDeg + 3)) {
      Serial.println("within calculated boundary");
      if (obstacleIsPresent == false) {
        hyp_dist_from_obstacle = average_dist;
        obstacle_start_angle = neck_rot_servo.read() - neck_servo_offset;
        Serial.print("start angle is ");
        Serial.println(obstacle_start_angle);
        obstacleIsPresent = true;
      }
      prev_hyp_dist_from_obstacle = average_dist;
      obstacle_end_angle = neck_rot_servo.read() - neck_servo_offset;
      Serial.println();
    } else {
      if (obstacleIsPresent == true) {
        adj_dist_from_obstacle = prev_hyp_dist_from_obstacle;
        space_width = cosine_rule(hyp_dist_from_obstacle, adj_dist_from_obstacle, (obstacle_end_angle - obstacle_start_angle));
        Serial.print("obstacle width is ");
        Serial.println(space_width);
        Serial.print("end angle for ");
        Serial.println(obstacle_end_angle);
        Serial.print("average distance is ");
        Serial.println(average_dist);
        Serial.println();
        Serial.println();
        Serial.println();
        // Zero all variables
        hyp_dist_from_obstacle = 0;
        adj_dist_from_obstacle = 0;
        obstacle_end_angle = 0;
        obstacle_start_angle = 0;
        prev_hyp_dist_from_obstacle = 0;
        obstacleIsPresent = false;
      }
      Serial.printf("not within boundary\ncalcsendist is %f\n", distAtAngle);
    }
    // Serial.printf("\n\n");
    delayMicroseconds(1000);  // waits 15 ms for the servo to reach the position
  }

  Serial.printf("Servo offset angular pos is %i\nServo angular pos is %i\n", neck_rot_servo.read() + neck_servo_offset, neck_rot_servo.read());

  Serial.println("second loop ");
  for (uint8_t pos = endAngle; pos >= startAngle; pos--) {
    neck_rot_servo.write(pos);  // tell servo to go to position in variable 'pos'
    // Serial.printf("Servo offset angular pos is %i\n", neck_rot_servo.read() - neck_servo_offset);
    float average_dist = averageSonarDist();
    float distAtAngle = calcSenDistAtAngle(average_dist, neck_rot_servo.read() - neck_servo_offset);
    if ((distAtAngle >= distAtninetyDeg - 3) && (distAtAngle <= distAtninetyDeg + 3)) {
      Serial.println("within calculated boundary");
      if (obstacleIsPresent == false) {
        hyp_dist_from_obstacle = average_dist;
        obstacle_start_angle = neck_rot_servo.read() - neck_servo_offset;
        Serial.print("start angle is ");
        Serial.println(obstacle_start_angle);
        obstacleIsPresent = true;
      }
      prev_hyp_dist_from_obstacle = average_dist;
      obstacle_end_angle = neck_rot_servo.read() - neck_servo_offset;
      Serial.println();
    } else {
      if (obstacleIsPresent == true) {
        adj_dist_from_obstacle = prev_hyp_dist_from_obstacle;
        space_width = cosine_rule(hyp_dist_from_obstacle, adj_dist_from_obstacle, (obstacle_end_angle - obstacle_start_angle));
        Serial.print("obstacle width is ");
        Serial.println(space_width);
        Serial.print("end angle for ");
        Serial.println(obstacle_end_angle);
        Serial.print("average distance is ");
        Serial.println(average_dist);
        Serial.println();
        Serial.println();
        Serial.println();
        // Zero all variables
        hyp_dist_from_obstacle = 0;
        adj_dist_from_obstacle = 0;
        obstacle_end_angle = 0;
        obstacle_start_angle = 0;
        prev_hyp_dist_from_obstacle = 0;
        obstacleIsPresent = false;
      }
      Serial.printf("not within boundary\ncalcsendist is %f\n", distAtAngle);
    }
    // Serial.printf("\n\n");
    delayMicroseconds(1000);  // waits 15 ms for the servo to reach the position
  }
}

float cosine_rule(float hyp, float opp, float angle) {
  if (angle < 0) {
    angle = (angle * -1);
    Serial.println("angle was changed from negative to positive");
  }
  return sqrt(pow(hyp, 2) + pow(opp, 2) - (2 * hyp * opp) * cos(angle * DEG_TO_RAD));
}


void sweepandread(uint8_t start_angle, uint8_t end_angle) {
  long loop_start_time = millis();
  for (uint8_t pos = start_angle; pos <= end_angle; pos++) {
    // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    neck_rot_servo.write(pos);  // tell servo to go to position in variable 'pos'

    float average_dist = averageSonarDist();
    // Serial.print("servo pos is "); Serial.println(neck_rot_servo.read());
    // Serial.print("average distance is "); Serial.println(average_dist);
    Serial.print("calcSendistAtAngle is ");
    Serial.println(calcSenDistAtAngle(average_dist, neck_rot_servo.read()));
    Serial.println();
    delayMicroseconds(1000);  // waits 15 ms for the servo to reach the position
  }
  for (uint8_t pos = end_angle; pos >= start_angle; pos--) {  // goes from 180 degrees to 0 degrees
    neck_rot_servo.write(pos);                                // tell servo to go to position in variable 'pos'

    float average_dist = averageSonarDist();
    // if(calcSenDistAtAngle(average_dist, neck_rot_servo.read()) >= 6.20)
    // {
    // Serial.print("servo pos is "); Serial.println(neck_rot_servo.read());
    // Serial.print("average distance is "); Serial.println(average_dist);
    Serial.print("calcSendistAtAngle is ");
    Serial.println(calcSenDistAtAngle(average_dist, neck_rot_servo.read()));
    Serial.println();
    // }
    delayMicroseconds(1000);  // waits 15 ms for the servo to reach the position
  }
  Serial.print("sweepandread back and forth took ");
  Serial.print(millis() - loop_start_time);
  Serial.println(" milliseconds");
}



float calcSenDistAtAngle(float hyp, uint8_t servo_angle) {
  if (servo_angle <= 90) {
    return (cos((90 - servo_angle) * DEG_TO_RAD) * hyp);
  }

  else if (servo_angle > 90) {
    return (cos((90 - (180 - servo_angle)) * DEG_TO_RAD) * hyp);
  }
}


void readModeFromSerial() {
  if (Serial.available() > 0) {
    String serialInput = Serial.readStringUntil('\n');
    serialInput.toUpperCase();
    char CHAR = serialInput[0];
    if (CHAR == 'F' || CHAR == 'B' || CHAR == 'L' || CHAR == 'R' || CHAR == 'N' || CHAR == 'A' || CHAR == 'S') {
      controlMode = CHAR;
      Serial.printf("Control Mode is %c\n", CHAR);
      if (CHAR == 'A') {
        int analog_speed = (serialInput.substring(1)).toInt();
        Serial.println(analog_speed);
        if (analog_speed >= 0 || analog_speed <= 255) {
          Serial.println("analog mode");
          controlMode = 'A';
          global_analog_speed = analog_speed;
        } else {
          Serial.println("Enter a number between 0 and 255");
        }
      } else if (CHAR == 'S') {
        Serial.println("true true true");
        if ((char)(serialInput[1]) == '1') {
          // Serial.printf("neck servo pos is %i\n", serialInput);
          uint8_t neck_angle = (serialInput.substring(2)).toInt();
          Serial.printf("neck_rot servo angle is %i\n", neck_angle);
          neck_rot_servo.write(neck_angle);

        } else if ((char)(serialInput[1]) == '2') {
          uint8_t sonar_angle = (serialInput.substring(2)).toInt();
          Serial.printf("sonar_rot servo angle is %i\n", sonar_angle);
          sonar_servo.write(sonar_angle);
        }
      }
    } else {
      Serial.println("Please Enter the Character F or B or L or R to control the motors or S1angle S2angle to set servo angles");
    }
  }
}


void ease_neck_servo_to_angle(uint8_t angle) {
  uint8_t start_pos = neck_rot_servo.read();
  Serial.print("neck servo present angle is ");
  Serial.println(start_pos);
  Serial.print("set angle is ");
  Serial.println(angle);
  if (start_pos - angle < 0) {
    for (uint8_t i = 0; i <= angle - start_pos; i++) {
      neck_rot_servo.write(start_pos + i);
      Serial.print("neck Servo pos is ");
      Serial.println(neck_rot_servo.read());
      delay(10);
    }
    if (neck_rot_servo.read() != angle) {
      neck_rot_servo.write(angle);
      Serial.println("final angle was forcefully written");
      Serial.print("servo pos is now ");
      Serial.println(neck_rot_servo.read());
    }
    return;
  }

  else if (start_pos - angle > 0) {
    for (uint8_t i = 0; i <= start_pos - angle; i++) {
      neck_rot_servo.write(start_pos - i);
      Serial.print("neck Servo pos is ");
      Serial.println(neck_rot_servo.read());
      delay(10);
    }
    if (neck_rot_servo.read() != angle) {
      neck_rot_servo.write(angle);
      Serial.println("final angle was forcefully written");
      Serial.print("servo pos is now ");
      Serial.println(neck_rot_servo.read());
    }
    return;
  }
}

void ease_servo_to_angle(uint8_t angle, Servo easeservo) {
  uint8_t start_pos = easeservo.read();
  if (start_pos - angle < 0) {

    for (uint8_t i = 0; i <= angle - start_pos; i++) {
      easeservo.write(start_pos + i);
      Serial.print("Sonar servo pos is ");
      Serial.println(easeservo.read());
      delay(10);
    }
    if (easeservo.read() != angle) {
      neck_rot_servo.write(angle);
      Serial.println("final angle was forcefully written");
      Serial.print("servo pos is now ");
      Serial.println(easeservo.read());
    }
    return;
  }

  else if (start_pos - angle > 0) {
    for (uint8_t i = 0; i <= start_pos - angle; i++) {
      easeservo.write(start_pos - i);
      Serial.print("Sonar servo pos is ");
      Serial.println(easeservo.read());
      delay(10);
    }
    if (easeservo.read() != angle) {
      neck_rot_servo.write(angle);
      Serial.println("final angle was forcefully written");
      Serial.print("servo pos is now ");
      Serial.println(easeservo.read());
    }
    return;
  }
}



// wireless connectivity functions
void setupBLE() {
  // Initialize BLE
  NimBLEDevice::init("C1 Robot");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // Maximum transmission power

  // Set security requirements
  NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM | BLE_SM_PAIR_AUTHREQ_SC);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO);

  // Create BLE server
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks);

  // Create telemetry service
  NimBLEService* pBaadService = pServer->createService(SVC_UUID);

  // Create telemetry characteristic
  pFoodCharacteristic = pBaadService->createCharacteristic(
    CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::WRITE_ENC);

  // Set initial value and callbacks
  uint8_t initValue[3] = { 0, 0, 0 };
  pFoodCharacteristic->setValue(initValue, sizeof(initValue));
  pFoodCharacteristic->setCallbacks(&chrCallbacks);

  // Create descriptor
  NimBLEDescriptor* pDesc = pFoodCharacteristic->createDescriptor(
    "2901",
    NIMBLE_PROPERTY::READ);
  pDesc->setValue("Robot Telemetry");

  // Start service
  pBaadService->start();

  // Start advertising
  setupAdvertising();
}

void deactiveBLE() {
  NimBLEDevice::deinit("C1 Robot");
}

void setupNRF() {
  // Initialize Radio Object
  if (!radio.begin()) {
    Serial.println("Radio Hardware not responding");
    while (1) {
    }
  }

  // radio.openWritingPipe(addresses[1]);  //For some reason the esp32 isn't able to function as a transmitter
  radio.openReadingPipe(1, addresses[0]);

  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();
}

void deactivateNRF() {
  radio.powerDown();
}


void setupESPNOW() {
  return;
}

void deactivateESPNOW() {
  return;
}
