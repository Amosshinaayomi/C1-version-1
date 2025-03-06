#include <Servo.h>
#include <math.h>
#include <stdlib.h>
Servo neck;
Servo forearms;
Servo hind_legs;


typedef struct coordinate
{
  float x_axis;
  float y_axis;
  int angle;
  float xy_ref;

};

typedef struct sensor_data
{
  uint8_t space_start_angle;
  uint8_t space_end_angle;
  uint8_t space_counter;
  float first_len;
  float last_len;
  bool nodeisnew;
} sen_data;

typedef struct obstacle
{
  float width;
  char end;
} _obstacle;


typedef struct _sweeped_data
{
  bool spaceisavailable;
  float obstacle_width;
  float space_width;
  char side;
} sweeped_data;

typedef struct _node
{
  sen_data data_node;
  struct _node *forw;
  struct _node *prev;
} node;


typedef struct queue
{
  node *head;
  node *tail;
};



int8_t neck_pin = 9;
int8_t forearms_pin = 6;
int8_t hind_legs_pin = 10;

float sec_taken_for_full_rot;

const int n_size = 3;
const int dig_pins = 4;
int controlPinsA[n_size] = { 2, 4, 3 };
//define motorB control pins in an array where elements 0, 1, 2 are in1, in2 and enB respectfully
int controlPinsB[n_size] = { 7, 8, 5 };
// Define the pins the switches are connected to:
int r_btn = A5;
int l_btn = 12;
int b_btn = 13;
int f_btn = 11;

int angle_btn = A2;
//initialize their state to zero:
uint8_t b_btn_state = 0;
uint8_t f_btn_state = 0;
uint8_t l_btn_state = 0;
uint8_t r_btn_state = 0;
// Define the  pins the joystick axis are connected to:
int x_Axis = A1;
int y_Axis = A0;
int joy_switch = 29;
// initialize their state value to 0:
int x_Axis_val = 0;
int y_Axis_val = 0;

// Define the limit or deadspot for the joystick:
int x_limit[2] = { 436, 586 };
int y_limit[2] = { 436, 586 };

// Define the minimum and maximum range on the x-axis for drifting:
int xd_limit[2] = { 150, 800};
//Define the minimum range for drifting on the y_axis
int yd_limit = 40;
int8_t speedA = 0;
int8_t speedB = 0;

int shifted = 0;
int shifted_f = 0;
int shifted_b = 0;
int shifted_l = 0;
int shifted_r = 0;

int  f_sen_safe_dist = 7;
int  b_sen_safe_dist = 3;
int  l_sen_safe_dist = 3;
int r_sen_safe_dist = 3;

int pressed = 0;
// 1 sec constant 
const float sec_constant = 15;
int  f_sen_dist = 8;
int  b_sen_dist = 3;
int  l_sen_dist = 3;
int r_sen_dist = 3;

int sweep_btn = A3;

float body_angle = 0;
int pulse_duration = 0;
uint8_t angle_distance = 1;
int volt_reader = A4;

// the speed unit is m/sec:
const float average_speed_per_sec = 0.7;
float incre_val = 8;
coordinate motion_coord;
int8_t bot_width = 4;

// functions prototypes
void rc_motion_control(float duration);
void analog_move_f(float duration, int speed, int *sensor_distance);
void analog_move_b(float duration,  int speed, int *sensor_distance);
void analog_drift_l(float duration, int speedA, int speedB, int *sensor_distance);
void analog_drift_lb(float duration, int speedA, int speedB, int *sensor_distance);
void analog_drift_r(float duration, int speedA, int speedB, int *sensor_distance);
void analog_drift_rb(float duration, int speedA, int speedB, int *sensor_distance);
void halt(float duration);
void move_f(float duration, int *sensor_distance);
void move_b(float duration, int *sensor_distance);
void tight_turn_l(float duration, int *sensor_distance);
void tight_turn_r(float duration, int *sensor_distance);
void set_servo_pos(int pos1, int pos2, int pos3);
void center(void);
void blink_led(int led, int sec, int blink_rate);
bool Pressed(uint8_t pin);
void angle_turn(float angle, int8_t rot_direction);
void set_body_angle(uint8_t angle);
int power_percent();
float calc_sen_dist(uint8_t servoAngle);
float get_sen_dist();

node *sweepandread(uint8_t l_edge_angle, uint8_t r_edge_angle);
sweeped_data sweepandcheck(uint8_t left_bound, uint8_t right_bound);
void print_sensor_distance(queue* pointer);


sensor_data find_free_space (queue* pointer);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (int i = 0; i < n_size; i++) {
    pinMode(controlPinsA[i], OUTPUT);
    pinMode(controlPinsB[i], OUTPUT);
  }

  analogWrite(controlPinsA[n_size - 1], 0);
  analogWrite(controlPinsB[n_size - 1], 0);

  pinMode(f_btn, INPUT_PULLUP);
  pinMode(b_btn, INPUT_PULLUP);
  pinMode(l_btn, INPUT_PULLUP);
  pinMode(r_btn, INPUT_PULLUP);
  pinMode(angle_btn, INPUT_PULLUP);
  pinMode(volt_reader, INPUT);
  neck.attach(neck_pin);
  forearms.attach(forearms_pin);
  hind_legs.attach(hind_legs_pin);


  pinMode(sweep_btn, INPUT_PULLUP);
  set_servo_pos(90, 180, 180);

  body_angle = 0;
  motion_coord.x_axis = 0;
  motion_coord.y_axis = 0;

}

void loop() {
  // put your main code here, to run repeatedly:
  sec_taken_for_full_rot =  4.00 * (1000 / power_percent());
  node *list_head = NULL;
  sen_data *free_space = NULL;

  sweeped_data data;
  
  // let's assume our machine covers 0.7 meter per second at max speed or at max  battery percent:
  rc_motion_control(0.1);

  if (digitalRead(angle_btn) == 0)
  {
    center();
    // set_body_angle(30);
  }
  
  if (digitalRead(sweep_btn) == 0)
  {
    // list_head = sweepandread(45, 135);
    data = sweepandcheck(45, 135);
    Serial.print("obstacle width is "); Serial.println(data.obstacle_width);
    Serial.print("Space width is "); Serial.println(data.space_width);
    Serial.print("Space is available returns "); Serial.println(data.spaceisavailable);
    Serial.print("Space is available at "); Serial.print(data.side); Serial.println(" side ");
    Serial.println();
  }

  // if(list_head != NULL)
  // {
  //   print_sensor_distance(list_head);
  //   free_space = find_free_space(list_head);
  //   Serial.print("space length is "); Serial.print(free_space->space_width);
  // }

  // free_list(list_head);
}


void rc_motion_control(float duration) {
  x_Axis_val = analogRead(x_Axis);
  y_Axis_val = analogRead(y_Axis);


  shifted_f = (y_Axis_val < y_limit[0]) && ((x_Axis_val > x_limit[0]) && (x_Axis_val < x_limit[1]));
  shifted_b = (y_Axis_val > y_limit[1]) && ((x_Axis_val > x_limit[0]) && (x_Axis_val < x_limit[1]));
  shifted_l = (x_Axis_val < x_limit[0]) && ((y_Axis_val > y_limit[0]) && (y_Axis_val < y_limit[1]));
  shifted_r = (x_Axis_val > x_limit[1]) && ((y_Axis_val > y_limit[0]) && (y_Axis_val < y_limit[1]));
  shifted = shifted_f || shifted_l || shifted_b || shifted_r;

  f_btn_state = digitalRead(f_btn);
  b_btn_state = digitalRead(b_btn);
  l_btn_state = digitalRead(l_btn);
  r_btn_state = digitalRead(r_btn);

  pressed = (f_btn_state == 0 || b_btn_state == 0 || l_btn_state == 0 || r_btn_state == 0);

  // pressed = Pressed(f_btn) || Pressed(b_btn) || Pressed(l_btn) || Pressed(r_btn); 

  int speed = 0;
  int speedA = 0;
  int speedB = 0;

  if (shifted) {
    if (shifted_f) {
      speed = map(y_Axis_val, y_limit[0], 0, 0, 1023);
      analog_move_f(duration, speed, &f_sen_dist);
      return;
    }

    else if (shifted_b) {
      speed = map(y_Axis_val, y_limit[1], 1023, 0, 1023);
      analog_move_b(duration, speed, &b_sen_dist);
      return;
    }

    else if (shifted_l) {
      speedA = map(x_Axis_val, 0, (x_limit[0] - 1), 111, 511);
      speedB = map(x_Axis_val, (x_limit[0] - 1), 0, 611, 1023);
      analog_drift_l(duration, speedA, speedB, &f_sen_dist);
      return;
    }

    else if (shifted_r) {
      speedA = map(x_Axis_val, (x_limit[1] + 1), 1023, 611, 1023);
      speedB = map(x_Axis_val, 1023, (x_limit[1] + 1), 111, 511);
      analog_drift_r(duration,  speedA, speedB, &f_sen_dist);
      return;
    }

  }

  else if (pressed) {

    if (Pressed(f_btn)) {
      move_f(duration, &f_sen_dist);
      return;
    }

    else if (Pressed(b_btn)) {
      move_b(duration, &b_sen_dist);
      return;
    }

    else if (Pressed(l_btn)) {
      angle_turn(15, -1);
      return;
    }

    else if (Pressed(r_btn)) {
      angle_turn(15, 1);
      return;
    }
  }

  else {
    halt(0);
    return;
  }
}

void analog_move_b(float duration, int speed, int *sensor_distance) {
  if (*sensor_distance < b_sen_safe_dist)
  {
    halt(0.01);
    while(*sensor_distance < b_sen_safe_dist)
    {
      move_f(0.01, &f_sen_dist);
      *sensor_distance = *sensor_distance + 1;
    }
    halt(0.01);
    return;
  }
  int count = duration * 1000;
  while(count > 0)
  {
    analogWrite(controlPinsA[n_size - 1], speed);
    analogWrite(controlPinsB[n_size - 1], speed);
    digitalWrite(controlPinsA[0], HIGH);
    digitalWrite(controlPinsA[1], LOW);
    digitalWrite(controlPinsB[0], HIGH);
    digitalWrite(controlPinsB[1], LOW); 
    delay(1);
    count -= 1;
  }
  float dist_covered = average_speed_per_sec * duration / ((float) power_percent() / 1000) * ((float)speed / 1023);

  motion_coord.xy_ref = dist_covered;   
  motion_coord.y_axis -= cos(motion_coord.angle * DEG_TO_RAD) * motion_coord.xy_ref;
  motion_coord.x_axis -= sin(motion_coord.angle * DEG_TO_RAD) * motion_coord.xy_ref;

  Serial.print("y coordinate is "); Serial.println(motion_coord.y_axis);
  Serial.print("x coordinate is "); Serial.println(motion_coord.x_axis);
  Serial.print("coordinate angle is "); Serial.println(motion_coord.angle);
  Serial.println();

  return;
}

void analog_move_f(float duration, int speed, int *sensor_distance) {

  if (*sensor_distance < f_sen_safe_dist)
  {
    // Serial.println("body is close to an obstacle");
    while(*sensor_distance < f_sen_safe_dist)
    {
      // Serial.print("distance from _obstacle is "); Serial.println(*sensor_distance);
      move_b(0.01, &b_sen_dist);
      *sensor_distance = *sensor_distance + 1;
    }
    halt(0.01);
    return;
  }
  int count = duration * 1000;

  while(count > 0)
  {
    analogWrite(controlPinsA[n_size - 1], speed);
    analogWrite(controlPinsB[n_size - 1], speed);
    digitalWrite(controlPinsA[0], LOW);
    digitalWrite(controlPinsA[1], HIGH);
    digitalWrite(controlPinsB[0], LOW);
    digitalWrite(controlPinsB[1], HIGH);  
    delay(1);

    count -= 1;
  }

  float dist_covered = average_speed_per_sec * duration / ((float) power_percent() / 1000) * ((float)speed / 1023);

  motion_coord.xy_ref = dist_covered;   
  motion_coord.y_axis += cos(motion_coord.angle * DEG_TO_RAD) * motion_coord.xy_ref;
  motion_coord.x_axis += sin(motion_coord.angle * DEG_TO_RAD) * motion_coord.xy_ref;

  Serial.print("y coordinate is "); Serial.println(motion_coord.y_axis);
  Serial.print("x coordinate is "); Serial.println(motion_coord.x_axis);
  Serial.print("coordinate angle is "); Serial.println(motion_coord.angle);
  Serial.println();

  return;
}

void analog_drift_l(float duration, int speedA, int speedB, int *sensor_distance) {
  if (*sensor_distance < f_sen_safe_dist)
  {
    // Serial.println("body is close to an obstacle");
    while(*sensor_distance < f_sen_safe_dist)
    {
      // Serial.print("distance from _obstacle is "); Serial.println(*sensor_distance);
      move_b(0.01, &b_sen_dist);
      *sensor_distance = *sensor_distance + 1;
    }
    halt(0.01);
    return;
  }  
  int count = duration * 1000;
  while(count > 0)
  {
  analogWrite(controlPinsA[n_size - 1], speedA);
  analogWrite(controlPinsB[n_size - 1], speedB);
  digitalWrite(controlPinsA[0], HIGH);
  digitalWrite(controlPinsA[1], LOW);
  digitalWrite(controlPinsB[0], HIGH);
  digitalWrite(controlPinsB[1], LOW);
  delay(1);
  count -= 1;
 }
  return;
}


void analog_drift_r(float duration, int speedA, int speedB, int *sensor_distance) {
  if (*sensor_distance < f_sen_safe_dist)
  {

    while(*sensor_distance < f_sen_safe_dist)
    {
      move_b(0.01, &b_sen_dist);
      *sensor_distance = *sensor_distance + 1;
    }
    halt(0.01);
    return;
  }  
  int count = duration * 1000;
  while(count > 0)
  {
    analogWrite(controlPinsA[n_size - 1], speedA);
    analogWrite(controlPinsB[n_size - 1], speedB);
    digitalWrite(controlPinsA[0], HIGH);
    digitalWrite(controlPinsA[1], LOW);
    digitalWrite(controlPinsB[0], HIGH);
    digitalWrite(controlPinsB[1], LOW);
    delay(1);

    count -= 1;
  }
  return;
}

void move_b(float duration, int *sensor_distance) {
  if (*sensor_distance < b_sen_safe_dist)
  {
    while(*sensor_distance < b_sen_safe_dist)
    {
      move_f(0.01, &f_sen_dist);
      *sensor_distance = *sensor_distance + 1;
    }
    halt(0.01);
    return;
  }  
  int count = duration * 1000;
  while(count > 0)
  {
    digitalWrite(controlPinsA[n_size - 1], HIGH);
    digitalWrite(controlPinsB[n_size - 1], HIGH);
    digitalWrite(controlPinsA[0], HIGH);
    digitalWrite(controlPinsA[1], LOW);
    digitalWrite(controlPinsB[0], HIGH);
    digitalWrite(controlPinsB[1], LOW);
    delay(1);

    count -= 1;
  }
  float dist_covered = average_speed_per_sec * duration / ((float) power_percent() / 1000);

  motion_coord.xy_ref = dist_covered;   
  motion_coord.y_axis -= cos(motion_coord.angle * DEG_TO_RAD) * motion_coord.xy_ref;
  motion_coord.x_axis -= sin(motion_coord.angle * DEG_TO_RAD) * motion_coord.xy_ref;

  Serial.print("y coordinate is "); Serial.println(motion_coord.y_axis);
  Serial.print("x coordinate is "); Serial.println(motion_coord.x_axis);
  Serial.print("coordinate angle is "); Serial.println(motion_coord.angle);
  Serial.println();
  return;
}

void move_f(float duration, int *sensor_distance) {

  if (*sensor_distance < f_sen_safe_dist)
  {
    while(*sensor_distance < f_sen_safe_dist)
    {
      move_b(0.01, &b_sen_dist);
      *sensor_distance = *sensor_distance + 1;
    }
    halt(0.01);
    return;
  }

  int count = duration * 1000;
  while(count > 0)
  {
    digitalWrite(controlPinsA[n_size - 1], HIGH);
    digitalWrite(controlPinsB[n_size - 1], HIGH);
    digitalWrite(controlPinsA[0], LOW);
    digitalWrite(controlPinsA[1], HIGH);
    digitalWrite(controlPinsB[0], LOW);
    digitalWrite(controlPinsB[1], HIGH);
    delay(1);
    count -= 1;
  }

  float dist_covered = average_speed_per_sec * duration / ((float) power_percent() / 1000);

  motion_coord.xy_ref = dist_covered;   
  motion_coord.y_axis += cos(motion_coord.angle * DEG_TO_RAD) * motion_coord.xy_ref;
  motion_coord.x_axis += sin(motion_coord.angle * DEG_TO_RAD) * motion_coord.xy_ref;

  Serial.print("y coordinate is "); Serial.println(motion_coord.y_axis);
  Serial.print("x coordinate is "); Serial.println(motion_coord.x_axis);
  Serial.print("coordinate angle is "); Serial.println(motion_coord.angle);
  Serial.println();
  return;
}

void tight_turn_l(float duration, int *sensor_distance) {
  if (*sensor_distance < l_sen_safe_dist)
  {
    halt(0.01);
    while(*sensor_distance < l_sen_safe_dist)
    {
      tight_turn_r(0.01, &r_sen_dist);
      *sensor_distance = *sensor_distance + 1;
    }
    halt(0.01);
    return;
  }
  int count = duration * 1000;

  while(count > 0)
  {
    digitalWrite(controlPinsA[n_size - 1], HIGH);
    digitalWrite(controlPinsB[n_size - 1], HIGH);
    digitalWrite(controlPinsA[0], HIGH);
    digitalWrite(controlPinsA[1], LOW);
    digitalWrite(controlPinsB[0], LOW);
    digitalWrite(controlPinsB[1], HIGH);
    delay(1);
    count -= 1;
  }

  body_angle -= (duration * 360 / sec_taken_for_full_rot);
  
  if (body_angle >= 0)
  {
    body_angle = fmod(body_angle, 360);
  }

  else if (body_angle < 0)
  {
    body_angle = fmod(body_angle, -360);
  }

  Serial.println("angle is decremented to "); Serial.println(body_angle);
  motion_coord.angle = body_angle;
  motion_coord.xy_ref = 0;
  return;
}

void tight_turn_r(float duration, int *sensor_distance) {
  if (*sensor_distance < r_sen_safe_dist)
  {
    halt(0.01);
    while(*sensor_distance < r_sen_safe_dist)
    {
      tight_turn_l(0.01, &l_sen_dist);
      *sensor_distance = *sensor_distance + 1;
    }
    return;
  } 

  int count = duration * 1000;
  while(count > 0)
  {
    digitalWrite(controlPinsA[n_size - 1], HIGH);
    digitalWrite(controlPinsB[n_size - 1], HIGH);
    digitalWrite(controlPinsA[0], LOW);
    digitalWrite(controlPinsA[1], HIGH);
    digitalWrite(controlPinsB[0], HIGH);
    digitalWrite(controlPinsB[1], LOW);
    delay(1);
    count -= 1;
  }

  body_angle += (duration * 360 / sec_taken_for_full_rot);

  if (body_angle >= 0)
  {
    body_angle = fmod(body_angle, 360);
  }

  else if (body_angle < 0)
  {
    body_angle = fmod(body_angle, -360);
  }

  Serial.println("angle is incremented to "); Serial.println(body_angle);
  motion_coord.angle = body_angle;
  motion_coord.xy_ref = 0;
  return;
}

void halt(float duration) {
  digitalWrite(controlPinsA[0], LOW);
  digitalWrite(controlPinsA[1], LOW);
  digitalWrite(controlPinsB[0], LOW);
  digitalWrite(controlPinsB[1], LOW);
  analogWrite(controlPinsA[n_size - 1], 0);
  analogWrite(controlPinsB[n_size - 1], 0);
  delay((duration * 1000));
  return;
}

void set_servo_pos(int pos1, int pos2, int pos3) {
  if (neck.read() == pos1 && forearms.read() == pos2 && hind_legs.read() == pos3) {
    return;
  }
  neck.write(pos1);
  delay(180);
  forearms.write(pos2);
  delay(180);
  hind_legs.write(pos3);
  delay(180);
  return;
}

void angle_turn(float angle, int8_t rot_direction)
{

  float angle_in_sec;
  angle_in_sec = (sec_taken_for_full_rot * angle) / 360;
  
  Serial.print("Battery percent is "); Serial.print(power_percent()); Serial.print(" it will take "); Serial.print(angle_in_sec);
  Serial.print(" seconds to turn "); Serial.print(angle); Serial.println(" degree ");

  if (rot_direction == 1)
  {
    tight_turn_r(angle_in_sec, &r_sen_dist);
  }
  
 else if (rot_direction == -1)
 {
  tight_turn_l(angle_in_sec, &l_sen_dist);
 }
 return;
}

void set_body_angle(uint8_t angle)
{
  if (body_angle > angle)
  {
    angle = body_angle - angle;
    Serial.print("angle to cover is"); Serial.println(angle);
    angle_turn(angle, -1);
  }

  else if (body_angle < angle)
  {
    angle = angle - body_angle;
    Serial.print("angle to cover is "); Serial.println(angle);
    angle_turn(angle, 1);
    Serial.print("present angle is "); Serial.println( body_angle);
  }

  else
  {
    Serial.println("body angle is at the angle to cover");
    Serial.print("present angle is "); Serial.println( body_angle);
    return;
  }
} 


void center(void)
{
    if (body_angle == 0)
    {
      return;
    }
    else if (body_angle < 0)
    {
      Serial.print("Body angle is "); Serial.println(body_angle);
      angle_turn(-1 * body_angle, 1);
      return;
    }

    else if (body_angle > 0)
    {
      Serial.print("Body angle is "); Serial.println(body_angle);
      angle_turn(body_angle, -1);
      return;      
    }
  
}

int power_percent()
{
  int val = analogRead(volt_reader);
  float volt = map(val, 0, 1023, 0, 1000);
  return volt;
}

uint8_t click_counter(uint8_t pin)
{
  long listening_dur = millis() + 200;
  uint8_t clicks = 0;
  while(listening_dur > 0)
  {
    if (Pressed(pin))
    {
      clicks++;
      for(uint8_t i = 0; i < 50; i++)
      {
        if (Pressed(pin) == true)
        {
        return clicks;
        }
        delay(1);
      }
    } 
  }

}

bool Pressed(uint8_t pin)
{
  bool state;
  bool pressedState;
  uint8_t duration = 20; 
  // the pressed state is set to low because we are using pullup switches
  pressedState = LOW;

  for (uint8_t i; i < duration; i++)
  {
    state = digitalRead(pin);
    if (state != pressedState)
    {
      return false;
    }
    delay(1);
  }
  return true;
}

node *sweepandread(uint8_t l_edge_angle, uint8_t r_edge_angle)
{
  uint8_t start_pos = neck.read();
  uint8_t i;

  neck.write(l_edge_angle);
  delay(300);
  i = 1;

  node *headnode = malloc(sizeof(node));
  queue *QUEUE = malloc(sizeof(queue));
  int space_counter = 0;
  QUEUE->head = headnode;
  QUEUE->tail = headnode;
  QUEUE->head->forw = NULL;
  QUEUE->head->prev = NULL;
  space_counter = 0;
  QUEUE->head->data_node.space_end_angle = neck.read();

  QUEUE->head->data_node.first_len = 0;
  QUEUE->head->data_node.last_len = 0;
  QUEUE->head->data_node.space_start_angle = NULL;
  QUEUE->head->data_node.nodeisnew = true;


  while(neck.read() <= r_edge_angle)
  {
    if (calc_sen_dist(neck.read()) >= f_sen_safe_dist)
    {
      space_counter = space_counter + 1;
      QUEUE->tail->data_node.space_end_angle = neck.read();
      if (QUEUE->tail->data_node.nodeisnew == true)
      {
        QUEUE->tail->data_node.first_len = get_sen_dist();
        QUEUE->tail->data_node.nodeisnew = false;
      }
      QUEUE->tail->data_node.last_len = get_sen_dist();
    }

    else if (calc_sen_dist(neck.read()) < f_sen_safe_dist && QUEUE->tail->data_node.nodeisnew == true)
    {
      
    }

    else if (calc_sen_dist(neck.read()) < f_sen_safe_dist && QUEUE->tail->data_node.nodeisnew == false)
    {
      QUEUE->tail->data_node.space_start_angle = QUEUE->tail->data_node.space_end_angle - space_counter;
      node *new_node = malloc(sizeof(node));
      QUEUE->tail->forw = new_node;
      new_node->prev = NULL;
      new_node->prev = QUEUE->tail;
      new_node->data_node.nodeisnew = true;
      space_counter = 0;  
      new_node->data_node.space_start_angle = NULL;
      QUEUE->tail = new_node;
    }

    if (neck.read() == r_edge_angle && QUEUE->tail->data_node.nodeisnew == false)
    {
      QUEUE->tail->data_node.space_end_angle = neck.read();
      QUEUE->tail->data_node.space_start_angle = QUEUE->tail->data_node.space_end_angle - space_counter;
      // Serial.print("start angle is "); Serial.println(QUEUE->tail->data_node.space_start_angle);
      // Serial.println("nodeisnew is false");
      break;
    }
    
    if (neck.read() == r_edge_angle && QUEUE->tail->data_node.nodeisnew == true) 
    {  
      node *tmp = QUEUE->tail;
      tmp = tmp->prev;
      tmp->forw = NULL;
      break;
    }

    neck.write(neck.read() + i);
    delay(10);
  }

  i = 1;
  while (neck.read() > start_pos)
  {
    neck.write(neck.read() - 1);
    delay(10); 
  }
  return QUEUE->head;
}

sweeped_data sweepandcheck(uint8_t left_bound, uint8_t right_bound)
{
  sweeped_data return_data;
  float prev_hyp_len_from_obstacle = get_sen_dist();
  uint8_t obstacle_end_angle = neck.read();
  float obstacle_width = 0;

  float prev_space_raw_len = 0;
  uint8_t space_end_angle = 0;
  float space_width = 0;
  // note the members of the object_data while sweeping do not store the data directly rather indirectly
  // the first_len and last_lens tores the length of the the first and last free point found. 
  uint8_t start_pos = neck.read();

  while (neck.read() >= left_bound)
  {
    if (calc_sen_dist(neck.read()) >= f_sen_safe_dist)
    {
      Serial.print("finally free space at "); Serial.print(neck.read()); Serial.println(" degrees ");
      obstacle_width = sin((90 - obstacle_end_angle) * DEG_TO_RAD) * prev_hyp_len_from_obstacle;
      Serial.print("obstacle width here in left free space is ");  Serial.println(obstacle_width);
      space_end_angle = neck.read();
      space_width = (sin((90 - neck.read()) * DEG_TO_RAD) * get_sen_dist()) - obstacle_width;
      while (neck.read() >= left_bound - 20 || (space_width - obstacle_width) < bot_width) 
      {
        if (calc_sen_dist(neck.read()) < f_sen_safe_dist)
        {
          space_width = (sin((90 - space_end_angle) * DEG_TO_RAD) * prev_space_raw_len) - obstacle_width;
          break;
        }
       
        if (neck.read() == left_bound - 20)
        {
          break;
        }
       
        prev_space_raw_len = get_sen_dist();
        space_end_angle = neck.read();
        space_width = (sin((90 - space_end_angle) * DEG_TO_RAD) * prev_space_raw_len) - obstacle_width;
        neck.write(neck.read() - 1);
        delay(20);
      }

      if (space_width >= bot_width)
      {
        return_data.obstacle_width = obstacle_width;
        return_data.space_width = space_width;
        return_data.spaceisavailable = true;
        return_data.side = 'l';
        return return_data;
      }

      return_data.spaceisavailable = false;
      break;
    }

    if (neck.read() == left_bound)
    {
      return_data.obstacle_width = obstacle_width;
      return_data.space_width = space_width;
      break;
    }

    prev_hyp_len_from_obstacle = get_sen_dist();
    obstacle_end_angle = neck.read();
    obstacle_width = sin((90 - obstacle_end_angle) * DEG_TO_RAD) * prev_hyp_len_from_obstacle;
    return_data.obstacle_width = obstacle_width;    

  
    neck.write(neck.read() - 1);
    delay(20);    
  }

  Serial.print("obstacle width here at the end of left while loop is "); Serial.println(return_data.obstacle_width);
  if (neck.read() == left_bound && obstacle_width == 0)
  {
    obstacle_width = sin((90 - obstacle_end_angle) * DEG_TO_RAD)  * prev_hyp_len_from_obstacle;
    return_data.obstacle_width = obstacle_width;
    Serial.print("obstacle width here too after left loop is "); Serial.println(return_data.obstacle_width);
  }

  neck.write(start_pos);
  delay(400);

  prev_hyp_len_from_obstacle = get_sen_dist();
  obstacle_end_angle = neck.read();
  obstacle_width = 0;
  while (neck.read() <= right_bound)
  {

    if (calc_sen_dist(neck.read()) >= f_sen_safe_dist)
    {
      Serial.print("finally free space at "); Serial.print(neck.read()); Serial.println(" degrees ");
      obstacle_width = sin((obstacle_end_angle - 90) * DEG_TO_RAD) * prev_hyp_len_from_obstacle;
      Serial.print("obstacle width here in right loop is ");  Serial.println(obstacle_width);
      space_end_angle = neck.read();
      space_width = (sin((space_end_angle - 90) * DEG_TO_RAD) * prev_space_raw_len) - obstacle_width;
      while (neck.read() <= right_bound + 20 || space_width < bot_width) 
      {
        if (calc_sen_dist(neck.read()) < f_sen_safe_dist)
        {
          space_width = (sin((space_end_angle - 90) * DEG_TO_RAD) * prev_space_raw_len) - obstacle_width;
          break;
        }

        if (neck.read() == right_bound + 20)
        {
          break;
        }

        prev_space_raw_len = get_sen_dist();
        space_end_angle = neck.read();
        space_width = (sin((space_end_angle - 90) * DEG_TO_RAD) * prev_space_raw_len) - obstacle_width;
        neck.write(neck.read() + 1);
        delay(20);
      }

        return_data.side = 'r';

      if (space_width >= bot_width)
      {
        return_data.spaceisavailable = true;
        return_data.space_width = return_data.space_width + space_width;
        return_data.obstacle_width = return_data.obstacle_width + obstacle_width;

      }
      else
      {
        return_data.space_width = return_data.space_width + space_width;
        return_data.obstacle_width = return_data.obstacle_width + obstacle_width;
        return_data.spaceisavailable = false;
      }
      neck.write(start_pos);
      return return_data;
    }

    if (neck.read() == right_bound)
    {
      break;
    }

    prev_hyp_len_from_obstacle = get_sen_dist();
    obstacle_end_angle = neck.read();
    obstacle_width =  sin((obstacle_end_angle - 90) * DEG_TO_RAD) * prev_hyp_len_from_obstacle;
    // Serial.print("obstacle width in right side while loop is "); Serial.println(obstacle_width);
    neck.write(neck.read() + 1);
    delay(20);    
  }

  return_data.obstacle_width = return_data.obstacle_width + obstacle_width;
  return_data.side = 'n';
  Serial.print("obstacle width after right side while loop finally is "); Serial.println(obstacle_width);
  Serial.print("total obstacle width at the end is "); Serial.println(return_data.obstacle_width);
  neck.write(start_pos);
  delay(400);

  return return_data;
}

float calc_sen_dist(uint8_t servoAngle)
{
  // Serial.print("angle is ");Serial.println(servoAngle);
  return (sin(servoAngle * DEG_TO_RAD) * get_sen_dist());
}

float get_sen_dist()
{
  if (neck.read() < 90)
  {
    incre_val = 5;
  }

  else if (neck.read() >=90 && neck.read() < 100)
  {
    incre_val = 5;
  }

  else if (neck.read() > 100)
  {
    incre_val = 9;
  }
  return incre_val;
}