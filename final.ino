//cpe301 final project, group 48
//team members: Langdon Isaacson, Matthew Patterson

#include <RTClib.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <Stepper.h>

// pin definitions
#define WATER_SENSOR_PIN A0
#define FAN_ENABLE_PIN 9
#define FAN_IN1_PIN 7
#define FAN_IN2_PIN 8
#define START_BUTTON_PIN 2
#define STOP_BUTTON_PIN 3
#define RESET_BUTTON_PIN 4
#define YELLOW_LED_PIN PL6
#define GREEN_LED_PIN PL7
#define RED_LED_PIN PG0
#define BLUE_LED_PIN PG1
#define DHT_PIN 52
#define DHT_TYPE DHT11
#define STEPPER_IN1 13
#define STEPPER_IN2 12
#define STEPPER_IN3 11
#define STEPPER_IN4 10
#define POTENTIOMETER_PIN A1
#define RDA 0x80
#define TBE 0x20  
#define TEMP_THRESHOLD 22
#define WATER_THRESHOLD 100
#define STEPS_PER_REV 2048
#define UPDATE_INTERVAL 60000

enum State {DISABLED, IDLE, RUNNING, ERROR};
volatile State currentState = DISABLED;
volatile State prevState = DISABLED;

//variables
bool isFanOn = false;
RTC_DS1307 rtc;
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal lcd(24, 25, 26, 27, 28, 29);
Stepper ventMotor(STEPS_PER_REV, STEPPER_IN1, STEPPER_IN3, STEPPER_IN2, STEPPER_IN4);

float temperature = 0;
float humidity = 0;
volatile bool startPressed = false;
volatile bool stopPressed = false;
bool resetPressed = false;
unsigned long lastUpdateTime = 0;

// functions
unsigned int readADC(unsigned char adc_channel_num);
void setLED(uint8_t ledPin);
void startButtonISR();
void stopButtonISR();
void updateSensors();
void monitorConditions();
void handleStateChange(State state);
void controlFanMotor(bool);
void controlStepperWithPotentiometer();
void stopStepperMotor();
void adc_init();
const char* stateToString(State state);
void outputMotorChange(const char*);

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0 = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;


void setup() {
  
  adc_init();

  U0init(9600);

  if (!rtc.begin()) {
    lcd.print("RTC Error");
    while (1);
  }
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  lcd.begin(16, 2);
  dht.begin();
  ventMotor.setSpeed(20);

  DDRD |= (1 << 7); //set fan
  DDRL |= (1 << PL6) | (1 << PL7); // yellow and green
  DDRG |= (1 << PG0) | (1 << PG1); // red and blue

  DDRD &= ~((1 << PD2) | (1 << PD3) | (1 << PD4)); // buttons as inputs
  PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4);  // resistors

  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), startButtonISR, FALLING); //ISR monitor
  attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), stopButtonISR, FALLING);  //ISR monitor


  setLED(YELLOW_LED_PIN); //start disabled
  lcd.print("System Disabled");
}
unsigned long prevMillis = 0;
const long interval = 100; //delay interval


void loop() {
  unsigned long currMillis = millis();


  if(currMillis - prevMillis >= interval){  //delay
    prevMillis = currMillis;
          controlStepperWithPotentiometer();
      //reset button
      if (!(PIND & (1 << PD4))) { // RESET_BUTTON_PIN active low
        resetPressed = true;
      }

      if (resetPressed) {
        resetPressed = false;
        if (currentState == ERROR && readADC(WATER_SENSOR_PIN) >= WATER_THRESHOLD) {
          currentState = IDLE;
          setLED(GREEN_LED_PIN);
        }
      }

      //handle start button
      if (startPressed) {
        startPressed = false;
        if (currentState == DISABLED) {
          currentState = IDLE;
          setLED(GREEN_LED_PIN);
        }
      }

      //handle stop button
      if (stopPressed) {
        stopPressed = false;
        if (currentState != DISABLED) {
          currentState = DISABLED;
          setLED(YELLOW_LED_PIN);
        }
      }

      // update sensors and monitor conditions
      if (currentState != DISABLED) {
        if (millis() - lastUpdateTime >= UPDATE_INTERVAL) { //update sensor once every minute
          updateSensors();
          lastUpdateTime = millis();
        }
        monitorConditions();
      }

      //handle state transitions
      if (currentState != prevState) {
        handleStateChange(currentState);
        prevState = currentState;
      }

      if (currentState == ERROR && readADC(WATER_SENSOR_PIN) >= WATER_THRESHOLD) {
        currentState = IDLE;
        setLED(GREEN_LED_PIN);
      }
  }
}

void startButtonISR() {
  startPressed = true;
}

void stopButtonISR() {
  stopPressed = true;
}

void setLED(uint8_t ledPin) { //turns on and off leds
  PORTL &= ~((1 << PL6) | (1 << PL7)); // turn off yellow and green LEDs
  PORTG &= ~((1 << PG0) | (1 << PG1)); // turn off red and blue LEDs

  if (ledPin == YELLOW_LED_PIN) PORTL |= (1 << PL6);
  else if (ledPin == GREEN_LED_PIN) PORTL |= (1 << PL7);
  else if (ledPin == RED_LED_PIN) PORTG |= (1 << PG0);
  else if (ledPin == BLUE_LED_PIN) PORTG |= (1 << PG1);
}

void updateSensors() { //reads sensors and outputs data
  temperature = dht.readTemperature(); //reading DHT11 and output to LCD
  humidity = dht.readHumidity();
  if(isnan(temperature) || isnan(humidity)) { //check for invalid sensor readings and show error
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error");
    return;
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("Hum: ");
  lcd.print(humidity);
}

void monitorConditions() { //check DHT temp and water level
  if (currentState == IDLE && temperature > TEMP_THRESHOLD) {
    currentState = RUNNING;
    setLED(BLUE_LED_PIN);
  } else if (currentState == RUNNING && temperature <= TEMP_THRESHOLD) {
    currentState = IDLE;
    setLED(GREEN_LED_PIN);
  } else if (readADC(WATER_SENSOR_PIN) < WATER_THRESHOLD) {
    currentState = ERROR;
    setLED(RED_LED_PIN);
  }
}

void handleStateChange(State state) { //changes the state of the system and updates accordingly
  lcd.clear();
  String logMessage;

  switch (state) {
    case DISABLED:
      lcd.print("System Disabled");
      controlFanMotor(false);
      break;
    case IDLE:
      lcd.print("Idle Mode");
      controlFanMotor(false);
      break;
    case RUNNING:
      controlFanMotor(true);
      lcd.print("Cooling...");
      break;
    case ERROR:
      lcd.print("Error: Low Water");
      controlFanMotor(false);
      break;
  }
}

void outputMotorChange(const char* motorState) { //logs the date and time motor changes to monitor
  if (!rtc.begin()) { //RTC not avaliable exit function
      return;
  }
  DateTime now = rtc.now(); //current time

  char logMessage[100]; //message
  snprintf(logMessage, sizeof(logMessage), "Motor %s at %02d/%02d/%04d %02d:%02d:%02d\r\n", motorState, now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
  for (int i = 0; logMessage[i] != '\0'; i++) { //output message using UART
    U0putchar(logMessage[i]);
  }
}

void controlStepperWithPotentiometer() { //control the stepper motor
  int potValue = analogRead(POTENTIOMETER_PIN);
  int motorSpeed = map(potValue, 0, 1023, -10, 10); // map potentiometer to speed/direction

  if (motorSpeed != 0) {
    ventMotor.setSpeed(abs(motorSpeed));            // set motor speed
    ventMotor.step(motorSpeed > 0 ? 10 : -10);     // move in direction based on sign
  }
}


const char* stateToString(State state) { //convert enum val to string
  switch (state) {
    case DISABLED: return "DISABLED";
    case IDLE: return "IDLE";
    case RUNNING: return "RUNNING";
    case ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

void controlFanMotor(bool turnOn){ //function to turn fan on and off and report it to the monitor if the state of the motor changes
  if(turnOn) {
    if(!isFanOn) {  // check if the fan is not already on
      PORTD |= (1 << FAN_ENABLE_PIN); 
      analogWrite(FAN_ENABLE_PIN, 255);  //turn fan on
      isFanOn = true;
      outputMotorChange("ON");
    }
  }else {
    if (isFanOn) {  //only act if the fan is currently on
      //
      PORTD &= ~(1 << FAN_ENABLE_PIN); //turn fan off
      isFanOn = false;
      outputMotorChange("OFF");
    }
  }
}

//adc functions
void adc_init(){ //initialize ADC
  // adc controls and registers
  *my_ADCSRA |= 0b10000000; // enable adc
  *my_ADCSRA &= 0b11011111; // disable adc
  *my_ADCSRA &= 0b11110111;
  *my_ADCSRA &= 0b11111000; // clear prescaler bits (prescaler not set here)
  
  // setup adc control and register b
  *my_ADCSRB &= 0b11110111; // clear MUX5
  *my_ADCSRB &= 0b11111000; // clear bits
  
  // setup adc multiplexer selection register
  *my_ADMUX  &= 0b01111111;
  *my_ADMUX  |= 0b01000000;
  *my_ADMUX  &= 0b11011111;
  *my_ADMUX  &= 0b11100000;
}
unsigned int readADC(unsigned char adc_channel_num){ //return ADC 
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

//uart functions
void U0init(int U0baud){
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

void U0putchar(unsigned char U0pdata){
  while((TBE & *myUCSR0A)==0);
  *myUDR0 = U0pdata;
}

unsigned char U0getchar(){
  return *myUDR0;
}

unsigned char U0kbhit(){
  return (RDA & *myUCSR0A);
}