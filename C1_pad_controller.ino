#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


/*************************************************
   Note Constants for buzzer from arduino pitches library
 *************************************************/

#define NOTE_B0 31
#define NOTE_C1 33
#define NOTE_CS1 35
#define NOTE_D1 37
#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_FS7 2960
#define NOTE_G7 3136
#define NOTE_GS7 3322
#define NOTE_A7 3520
#define NOTE_AS7 3729
#define NOTE_B7 3951
#define NOTE_C8 4186
#define NOTE_CS8 4435
#define NOTE_D8 4699
#define NOTE_DS8 4978


// Joystick active start position
#define stick_f_active_val (127 - 32)
#define stick_b_active_val (127 + 32)
#define stick_r_active_val (127 + 32)
#define stick_l_active_val (127 - 32)


RF24 radio(7, 8); // CE, CSN
// Data packet struct
typedef struct __attribute__((packed)) tx_data_packet
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
} tx_data_packet;

const byte addresses[][6] = {"00001", "00002"};

const uint8_t directional_btn[4] = {A4, A0, A3, A5}; //forward, left, bottom, right
const uint8_t other_four_btn[4] = {4, 2, 9, 5}; //the first two pins represents the two buttons at the top(L1 and R1) while the other two represents the other two pins L2, R2 at the front, same order.
const uint8_t analogInputpins[2] = {A2, A1};

const uint8_t led_pins[] = {3, 6, 10}; //green, buzzer
const uint8_t radio_led = 3;
const uint8_t passive_buzzer = 6;
bool radio_power_state = false;
int radio_power_toggle_timeout = 2000;
bool radio_power_btn_is_pressed;
// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};



bool listen_for_togglepress_on_all_fun_btn();

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 4; i++)
  {
    if(i < 2)
    {
      pinMode(analogInputpins[i], INPUT);
    }
    pinMode(directional_btn[i], INPUT_PULLUP);
    pinMode(other_four_btn[i], INPUT);
  }


  pinMode(radio_led, OUTPUT);
  // pinMode(led_pins[1], OUTPUT);
  // turn red led on to indicate the device is powered on
  // digitalWrite(led_pins[1], HIGH);
  if(!radio.begin())
  {
    Serial.println("Radio Hardware not responding");
    while(1)
    {
      analogWrite(radio_led, 20);
      delay(500);
      analogWrite(radio_led, 0);
      delay(500);
    }
  }

  radio.openWritingPipe(addresses[0]);  // Use addresses[0] instead of addresses[1]
  // radio.openReadingPipe(1, addresses[1]);

  radio.setPALevel(RF24_PA_HIGH);
  radio.stopListening();
  Serial.print("Nano start as transmitter");


  activate_radio_led();
  for (int thisNote = 0; thisNote < 8; thisNote++)
  {
    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(passive_buzzer, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }
  analogWrite(radio_led, 0);
  radio_power_state = true;
}

void loop() {

  // Load all buttons state into data tx_data_packet struct 
  tx_data_packet data;
  data.f_btn_state = (digitalRead(directional_btn[0]) == false);
  data.l_btn_state = (digitalRead(directional_btn[1]) == false);
  data.b_btn_state = (digitalRead(directional_btn[2]) == false);
  data.r_btn_state = (digitalRead(directional_btn[3]) == false);

  data.l_one_state = (digitalRead(other_four_btn[0]) == false);
  data.r_one_state = (digitalRead(other_four_btn[1]) == false); 

  data.l_two_state = (digitalRead(other_four_btn[2]));
  data.r_two_state = (digitalRead(other_four_btn[3]));

  sprintf(data.x_axis_val, "%d", map(analogRead(analogInputpins[0]), 0, 1023, 0, 255));
  sprintf(data.y_axis_val, "%d", map(analogRead(analogInputpins[1]), 0, 1023, 0, 255));

  int received_x_axis_val = atoi(data.x_axis_val);
  int received_y_axis_val = atoi(data.y_axis_val);


  // General input states 
  bool shifted_f = (received_y_axis_val < stick_f_active_val) && ((received_x_axis_val > stick_l_active_val) && (received_x_axis_val < stick_r_active_val));
  bool shifted_b = (received_y_axis_val > stick_b_active_val) && ((received_x_axis_val > stick_l_active_val) && (received_x_axis_val < stick_r_active_val));
  bool shifted_l = (received_x_axis_val < stick_l_active_val) && ((received_y_axis_val > stick_f_active_val) && (received_y_axis_val < stick_b_active_val));
  bool shifted_r = (received_x_axis_val > stick_r_active_val) && ((received_y_axis_val > stick_f_active_val) && (received_y_axis_val < stick_b_active_val));
  bool shifted = shifted_f || shifted_l || shifted_b || shifted_r;

  bool f_btn_state = data.f_btn_state;
  bool b_btn_state = data.b_btn_state;
  bool l_btn_state = data.l_btn_state;
  bool r_btn_state = data.r_btn_state;

  bool l_one_btn_state = data.l_one_state;
  bool l_two_btn_state = data.l_two_state;
  bool r_one_btn_state = data.r_one_state;
  bool r_two_btn_state = data.r_two_state;
  bool pressed = (f_btn_state == 1 || b_btn_state == 1 || l_btn_state == 1 || r_btn_state == 1 || r_one_btn_state == 1 || l_two_btn_state == 1 || r_two_btn_state == 1);

  // if the four functional buttons are pressed at once the state of the radio is toggled
  // which in turn connect and disconnect the robot from the pad controller
  if(listen_for_togglepress_on_all_fun_btn())
  {
    toggle_radio_power_state();
  }

  // Check if radio  is activated; start transmitting else don't transmit
  if(check_radio_power_state())
  {
    // Transmit message and store transmitted message acknowledgement in a bool
    bool tx_result = false;
    if(radio.write(&data, sizeof(data)))
    {
      tx_result = true;
      printInputstates(data);
    }

    // if struct packet was transmitted successfully turn on radio LED, else turn it off
    if(tx_result == true)
    {
      Serial.println("message received successfully");
      analogWrite(radio_led, 10);
    }
    else
    {
      Serial.println("message transmition failure");
      analogWrite(radio_led, 0); 

    }
    // Delay for 40 ms to listen for another radio transmission
    delay(40);
  }
  // if radio isn't active, play a note on the buzzer
  else
  {
    if(shifted || pressed)
    {
      noTone(6);
      // play a note on pin 6 for 200 ms:
      tone(6, 440, 200);
      noTone(6);
    }
  }


}

bool check_radio_power_state()
{
  return radio_power_state;
}

void toggle_radio_power_state()
{
  if(radio_power_state == true)
  {
    radio_power_state = false;

    Serial.println("radio is off");
    deactivate_radio_led();
    radio.powerDown();    
    return;
  }

  else if(radio_power_state == false)
  {
    radio_power_state = true;
    
    Serial.println("radio is on");
    activate_radio_led();
    radio.powerUp();
    for (int thisNote = 0; thisNote < 8; thisNote++)
    {

      // to calculate the note duration, take one second divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / noteDurations[thisNote];
      tone(passive_buzzer, melody[thisNote], noteDuration);

      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      // stop the tone playing:
      noTone(8);
    }
    analogWrite(radio_led, 0);    
    return;   
  }
}

bool listen_for_togglepress_on_all_fun_btn()
{
  if((digitalRead(other_four_btn[0]) == false && digitalRead(other_four_btn[1]) == false && digitalRead(other_four_btn[2]) == true && digitalRead(other_four_btn[3]) == true)) 
  {
    Serial.println("A press was detected");
    radio_power_btn_is_pressed = true;
  }

  if(digitalRead(other_four_btn[1]) == true && digitalRead(other_four_btn[2]) == false && digitalRead(other_four_btn[3]) == false && radio_power_btn_is_pressed == true)
  {
    Serial.println("pressing force was lifted, toggle registered");
    radio_power_btn_is_pressed = false;  
    return true;
  }
  else
  {
    // Serial.println("no toggle was registered");
    return false;
  }
}

void printInputstates(tx_data_packet data)
{
  Serial.print("f_btn state is "); Serial.println(data.f_btn_state);
  Serial.print("b_btn state is "); Serial.println(data.b_btn_state);
  Serial.print("l_btn state is "); Serial.println(data.l_btn_state);
  Serial.print("r_btn state is "); Serial.println(data.r_btn_state);

  Serial.println();
  Serial.print("l_one state is "); Serial.println(data.l_one_state);
  Serial.print("r_one state is "); Serial.println(data.r_one_state);
  Serial.print("l_two state is "); Serial.println(data.l_two_state);
  Serial.print("r_two state is "); Serial.println(data.r_two_state);
  Serial.println();
  Serial.print("x axis is "); Serial.println(data.x_axis_val);
  Serial.print("y state is "); Serial.println(data.y_axis_val);
}

void activate_radio_led()
{
  for(uint8_t i = 0; i < 21; i++)
  {
    // Serial.println(i);
    analogWrite(radio_led, i);
    delay(20);
  } 
  return;
}

void deactivate_radio_led()
{
  for(uint8_t i = 20; i > 0; i--)
  {
    if(i == 1)
    {
      analogWrite(radio_led, 0);
    }
    // Serial.println(i);
    else
    {
      analogWrite(radio_led, i);        
    }

    delay(20);
  }
  return;
}





