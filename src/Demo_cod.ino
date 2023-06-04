#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

#include "printf.h"
#include "nrf_to_nrf.h"

nrf_to_nrf radio;
Servo myServo;
unsigned long motor_timer;
unsigned long motor_end_timer;
const unsigned long duration = 60000;
int pos = 0;

float acce_x, acce_y, acce_z;
float gyro_x, gyro_y, gyro_z;
float mage_x, mage_y, mage_z;


struct IMU_data {
  float Acce_x;
  float Acce_y;
  float Acce_z;
};

uint8_t address[][6] = { "1Node", "2Node" };

bool radioNumber = 1;

enum Command {IMUSENSOR, ERROR};

struct Command_history {
  Command command;
};

bool role = false;
bool rx_mode = 0;
bool new_role = false;
bool new_rx = false;
unsigned long start_timer;
unsigned long end_timer;
bool tx_flag = false;
bool rx_flag = false;
unsigned long start_timerA; // test the overall time from sending data to receiving data
unsigned long end_timerA;
unsigned long start_timerB; // test the time from sending data to setting role switch flag
unsigned long end_timerB;
unsigned long start_timerC; // test the time from role switch flag being set until it being acknowledged
unsigned long end_timerC;
unsigned long start_timerD; // test time from being acknowledged to detect something from pipe
unsigned long end_timerD;
unsigned long start_timerE; // test time from detect something from pipe to read from pipe
unsigned long end_timerE;
unsigned long start_timerF; // test time from sending data to finally motor being actuated (just for testing)
unsigned long end_timerF;

const unsigned long timeout = 1000;
unsigned long start_timeout_count;
unsigned long start_timeout_slave;


int plusThreshold = 30, minusThreshold = -30;
int degreesX = 0;
int degreesY = 0;
float acceleration = 0.0;

// Prototypes
void run_motor(int angle);
void switch_transceiver(void);
Command getCommand(char input);
void printData(Command command, float* data);
void acce_read(float x, float y, float z);
void gyro_read(float x, float y, float z);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myServo.attach(9);
  myServo.write(0);
  motor_timer = millis();
  while (!Serial) {
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  // Set up the radio
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  char input = '0';
  radioNumber = input == 1;
  Serial.print(F("radioNumber = "));
  Serial.println((int)radioNumber);

  Serial.println(F("Choose whether this board is TX or RX by typing 'T' or 'R'"));
  while (!Serial.available()) {

  }
  char c = toupper(Serial.read());
  radio.setPALevel(NRF_PA_LOW);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setAutoAck(1);
  radio.openWritingPipe(address[radioNumber]);
  radio.openReadingPipe(2, address[!radioNumber]);
  
  if (c == 'T') {
    role = true;
    Serial.println("TX!");
    radio.stopListening();
    
  } else if (c == 'R') {
    role = false;
    Serial.println("RX!");
    radio.startListening();
  }

}

void loop() {
  // Starting from here will be the master part of code
  if (Serial.available()) {
    
    if (role == true && new_role == false) {
      
      Command_history comm_his;
      bool report;
      char c = Serial.read();
      Serial.println(c);
      Command command = getCommand(c);
      Serial.println(command);
      comm_his.command = command;
      if (c == 's') {
        start_timerA = micros();
        start_timerB = micros();
        start_timerF = micros();
        if (radio.writeFast(&c, sizeof(char))) {
          Serial.println(c);
        } else {
          Serial.println("Writing failed");
        }
        new_role = true;
        switch_transceiver();
        end_timerB = micros();
        start_timerC = micros();
        start_timeout_count = millis();
      }
      
    } 
  } 
  
  // When the role switch flag was set and acknowledged
  if (role == true && new_role == true) {
    end_timerC = micros();
    start_timerD = micros();
    Command_history comm_his;
    tx_flag = true; // set the acknowledgement flag
    uint8_t new_pipe = 1;
    IMU_data sensordata;
    if (radio.available(&new_pipe)) {
      rx_mode = !rx_mode;
      digitalWrite(LED_BUILTIN, rx_mode);
      end_timerD = micros();
      start_timerE = micros();
      uint8_t bytes = radio.getDynamicPayloadSize();
      char v;
      
      float data[4] = {};
      radio.read(&data, bytes);
      end_timerE = micros();
      end_timerA = micros();
      printData(comm_his.command, data);
      acce_read(data[0], data[1], data[2]);
      run_motor(degreesX);
      new_pipe = 0;
      end_timerF = micros();
      switch_transceiver();

      Serial.print(end_timerA - start_timerA);
      Serial.println(" us.");
      Serial.print(end_timerB - start_timerB);
      Serial.println(" us.");
      Serial.print(end_timerC - start_timerC);
      Serial.println(" us.");
      Serial.print(end_timerD - start_timerD);
      Serial.println(" us.");
      Serial.print(end_timerE - start_timerE);
      Serial.println(" us.");
      Serial.print(end_timerF - start_timerF);
      Serial.println(" us.");
    } else {
      if (millis() - start_timeout_count > timeout) {
        new_role = false;
      }
    }
    
  } 
  // starting from here until the end of the loop() function will be the slave part of code
  if (role == false) {
    char c;
    uint8_t pipe = 1;
    uint8_t pipe2 = 1;
    int count = 0;
    if (new_rx == false) {
      start_timeout_slave = millis();
      if (radio.available(&pipe)) {
        uint8_t bytes = sizeof(char);
        uint8_t command;
        radio.read(&c, bytes);
        if (c == 's') {
          Serial.print("Command received: ");
          Serial.println(c);
          new_rx = true;
          switch_transceiver();
        } 

        pipe = 0;
        count += 1;
      } else {
        if (millis() - start_timeout_slave > timeout) {
          if (radio.available(&pipe2)) {
            uint8_t bytes = sizeof(char);
            uint8_t command;
            radio.read(&c, bytes);
            if (c == 's') {
              Serial.print("Command received: ");
              Serial.println(c);
              new_rx = true;
              switch_transceiver();
            } 

            pipe2 = 0;
          }
        }
      }
     
    }
    if (new_rx == true) {
      
      rx_flag = true;
      bool new_report;
      IMU_data data;
      char v;
      float sensorSet[10] = {};
      switch (c) {
        case 's':
          if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
            IMU.readAcceleration(acce_x, acce_y, acce_z);
            IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
            IMU.readMagneticField(mage_x, mage_y, mage_z);
            Serial.println(mage_z);
            data = {acce_x, acce_y, acce_z};
            acceleration = sqrt(pow(acce_x, 2)+pow(acce_y, 2)+pow(acce_z, 2));
            Serial.print("Acceleration: ");
            Serial.println(acceleration);
            
            radio.write(&data.Acce_x, sizeof(struct IMU_data));
            radio.write(&data.Acce_x, sizeof(struct IMU_data));
            radio.write(&data.Acce_y, sizeof(struct IMU_data));
            radio.write(&data.Acce_z, sizeof(struct IMU_data));


            switch_transceiver();
          }
          break;
        
      }
    }
  }
}

// Detect if the flags were switched and switch the original role
void switch_transceiver(void) {
  if (new_role == true && tx_flag == false && role == true) {
    radio.openReadingPipe(2, address[!radioNumber]);
    radio.startListening();
  } else if (new_rx == true && rx_flag == false && role == false) {
    radio.openWritingPipe(address[radioNumber]);
    radio.stopListening();
  }
  if (new_role == true && tx_flag == true && role == true) {
    new_role = false;
    tx_flag = false;
    radio.openWritingPipe(address[radioNumber]);
    radio.stopListening();
  } else if (new_rx == true && rx_flag == true && role == false) {
    new_rx = false;
    rx_flag = false;
    radio.openReadingPipe(2, address[!radioNumber]);
    radio.startListening();
  }
}

// Identify if the command is valid
Command getCommand(char input) {
  if (input == 's') {
    return IMUSENSOR;
  } else {
    return ERROR;
  }
}

// Print out the data read from pipe
void printData(Command command, float* data) {
  if (command == IMUSENSOR) {
    Serial.print("Acceleration: ");
    Serial.print(data[0]);
    Serial.print(", ");
    Serial.print(data[1]);
    Serial.print(", ");
    Serial.println(data[2]);
    
  } 
}

// Process Accelerometer data
void acce_read(float x, float y, float z) {
  if (x > 0.1) {
    x = 100 * x;
    degreesX = map(x, 0, 97, 0, 90);
    Serial.print("Tilting up ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }
  if (x < -0.1) {
    x = 100 * x;
    degreesX = -map(x, 0, -100, 0, 90);
    Serial.print("Tilting down ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }
  if (x < 0.1 && x >= 0.0) {
    degreesX = 0;
    Serial.print("Flat: ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }
  if (x < 0.0 && x > -0.1) {
    degreesX = 180;
    Serial.print("Filpped: ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }
  if (y > 0.1) {
    y = 100 * y;
    degreesY = map(y, 0, 97, 0, 90);
    Serial.print("Tilting left ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }
  if (y < -0.1) {
    y = 100 * y;
    degreesY = map(y, 0, -100, 0, 90);
    Serial.print("Tilting right ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }
  
}

// Process Gyrometer data
void gyro_read(float x, float y, float z) {
  if(y > plusThreshold) {
    Serial.println("Collision front");
  }
  if(y < minusThreshold) {
    Serial.println("Collision back");
  }
  if(x < minusThreshold) {
    Serial.println("Collision right");
  }
    if(x > plusThreshold) {
    Serial.println("Collision left");
  }
}

// Drive servo motor actuator
/*
This function takes in the angle input return by acce_read() and drive the motor.
In order to allow the motor to spin without getting clogged, a 15ms delay was applied
to every loop until the angle was the target angle.
*/
void run_motor(int angle) {
  if (angle >= 0) {
    if (pos < angle) {
      for (pos = pos; pos <= angle; pos += 1) {
        myServo.write(pos);
        delay(15);
      }
    } else if (pos > angle) {
      for (pos = pos; pos >= angle; pos -= 1) {
        myServo.write(pos);
        delay(15);
      }
    }
  } else if (angle < 0) {
    angle = 180 + angle;
    if (pos < angle) {
      for (pos = pos; pos < angle; pos += 1) {
        myServo.write(pos);
        delay(15);
      }
    } else if (pos > angle) {
      for (pos = pos; pos > angle; pos -= 1) {
        myServo.write(pos);
        delay(15);
      }
    }
  }
  
  // simulate the MOTIMOVE module - automatically turn off after the system ran for one minute
  if (millis() - motor_timer >= duration) {
    myServo.detach();
  }
}

