#include <L293D.h>
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <Adafruit_TCS34725.h>
#include <HX711.h>

// Define constants for the pin connections
const int IR_LEFT_PIN = 2;
const int IR_RIGHT_PIN = 3;
const int RGB_SENSOR_SDA_PIN = A4;
const int RGB_SENSOR_SCL_PIN = A5;
const int LOAD_CELL_DT_PIN = A0;
const int LOAD_CELL_SCK_PIN = A1;
const int MOTOR_LEFT_PWM_PIN = 6;
const int MOTOR_LEFT_DIR_PIN = 7;
const int MOTOR_RIGHT_PWM_PIN = 5;
const int MOTOR_RIGHT_DIR_PIN = 4;
const int BUTTON_PIN = 8;

// Define color thresholds
const int RED_THRESHOLD = 0xFF0000;
const int BLUE_THRESHOLD = 0x0000FF;
const int GREEN_THRESHOLD = 0x00FF00;

// Define global variables
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
HX711 loadCell;
bool buttonPressed = false;
bool atStationA = false;
bool atStationB = false;
bool atStationC = false;

void setup() {
  // Initialize the LCD
  lcd.begin(16, 2);
  
  // Initialize the RGB sensor
  tcs.begin();   // Initialize the load cell
  loadCell.begin(LOAD_CELL_DT_PIN, LOAD_CELL_SCK_PIN);
  loadCell.set_scale();
  
  // Set the pin modes for the IR sensors and push button
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Set the pin modes for the motors
  pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);

  // Start the motors
  digitalWrite(MOTOR_LEFT_DIR_PIN, LOW);
  analogWrite(MOTOR_LEFT_PWM_PIN, 0);
  digitalWrite(MOTOR_RIGHT_DIR_PIN, LOW);
  analogWrite(MOTOR_RIGHT_PWM_PIN, 0);
}

void followLine() {
  while (1) {
    //Read the sensor values
    int leftSensor = digitalRead(IR_LEFT_PIN);
    int rightSensor = digitalRead(IR_RIGHT_PIN);

    //Check the sensor values
    if (leftSensor == 0 && rightSensor == 1) {
        //left sensor on black and right sensor on white
        //take left turn
        digitalWrite(MOTOR_LEFT_DIR_PIN, HIGH);
        digitalWrite(MOTOR_RIGHT_DIR_PIN, LOW);
        analogWrite(MOTOR_LEFT_PWM_PIN, 220);
        analogWrite(MOTOR_RIGHT_PWM_PIN, 220);
    } else if (leftSensor == 0 && rightSensor == 0) {
        //both sensors are on black
        //move forward
        digitalWrite(MOTOR_LEFT_DIR_PIN, LOW);
        digitalWrite(MOTOR_RIGHT_DIR_PIN, LOW);
        analogWrite(MOTOR_LEFT_PWM_PIN, 255);
        analogWrite(MOTOR_RIGHT_PWM_PIN, 255);
    } else if (leftSensor == 1 && rightSensor == 0) {
        //left sensor on white and right sensor on black
        //take right turn
        digitalWrite(MOTOR_LEFT_DIR_PIN, LOW);
        digitalWrite(MOTOR_RIGHT_DIR_PIN, HIGH);
        analogWrite(MOTOR_LEFT_PWM_PIN, 220);
        analogWrite(MOTOR_RIGHT_PWM_PIN, 220);
    }
    if(digitalRead(IR_LEFT_PIN) == 1 && digitalRead(IR_RIGHT_PIN) == 1)
    {
        break;
    }
  }
}

void turnAround(){
  digitalWrite(MOTOR_LEFT_DIR_PIN, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR_PIN, LOW);
  analogWrite(MOTOR_LEFT_PWM_PIN,255);
  analogWrite(MOTOR_RIGHT_PWM_PIN, 255);
  delay(5000); // adjust delay time to make the robot turn around 
  digitalWrite(MOTOR_LEFT_PWM_PIN, 0);
  digitalWrite(MOTOR_RIGHT_PWM_PIN, 0);
}

void moveToStartingPoint(){
  turnAround();
  followLine();
  digitalWrite(MOTOR_LEFT_PWM_PIN, 0);
  digitalWrite(MOTOR_RIGHT_PWM_PIN, 0);
}

void stop() {   
  digitalWrite(MOTOR_LEFT_DIR_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_DIR_PIN, LOW);
  analogWrite(MOTOR_LEFT_PWM_PIN,0);
  analogWrite(MOTOR_RIGHT_PWM_PIN, 0);
  }

void movesforward() {
   digitalWrite(MOTOR_LEFT_DIR_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_DIR_PIN, LOW);
  analogWrite(MOTOR_LEFT_PWM_PIN,255);
  analogWrite(MOTOR_RIGHT_PWM_PIN, 255); 
}

void loop() {
  // Read the current color
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  uint32_t currentColor = (r << 16) | (g << 8) | b;
  
  // determine the current station using color threshold
  if (currentColor > RED_THRESHOLD) {
    atStationA = true;
  } else if (currentColor > BLUE_THRESHOLD) {
    atStationB = true;
  } else if (currentColor > GREEN_THRESHOLD) {
    atStationC = true;
  }

  

  // Read the current weight
  float weight = loadCell.get_units();

  // check if the button is pressed
  buttonPressed = digitalRead(BUTTON_PIN) == LOW;

if(weight < 1) {
    if (atStationA) {
        stop();
        if (buttonPressed) {
            moveToStartingPoint();
        }
    } else {
        moveToStationA();
        stop();
        while(!buttonPressed) {
            // wait for button press
        }
        moveToStartingPoint();
    }
} else if (weight >= 1 && weight < 2) {
    if (atStationA) {
        stop();
        if (buttonPressed) {
            moveToStationB();
        }
    } else if (atStationB) {
        stop();
        if (buttonPressed) {
            moveToStartingPoint();
        }
    } else {
        moveToStationA();
        stop();
        while(!buttonPressed) {
            // wait for button press
        }
        moveToStationB();
        stop();
        while(!buttonPressed) {
            // wait for button press
        }
        moveToStartingPoint();
    }
} else if (weight >= 2 && weight < 3) {
    if (atStationA) {
        stop();
        if (buttonPressed) {
            moveToStationB();
        }
    } else if (atStationB) {
        stop();
        if (buttonPressed) {
            moveToStationC();
        }
    } else if (atStationC) {
        stop();
        if (buttonPressed) {
            moveToStartingPoint();
        }
    } else {
        moveToStationA();
        stop();
        while(!buttonPressed) {
            // wait for button press
        }
        moveToStationB();
        stop();
        while(!buttonPressed) {
            // waitfor button press
}
moveToStationC();
stop();
while(!buttonPressed) {
// wait for button press
}
moveToStartingPoint();
}
} else if (weight >= 3) {
// Do not move
}