#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "DisplayModule.h"
#include "Frame.h"
#include "Gesture.h"

// Morgans code

Gesture gesture = Gesture();
  // oled setup
SSD1306 display(0x3c, 23, 22);
byte wifiStat = 0;
byte blutoothStat = 0;
byte batteryStat = 0;

// declare all frame object and link them together here
Frame homeFrame = Frame("home");

Frame hue = Frame("Hue");
Frame light1 = Frame("Light1");
Frame light2 = Frame("Light2");

DisplayModule DM = DisplayModule(&display, &wifiStat, &blutoothStat, &batteryStat, &homeFrame, &gesture);

// Annos code

#define GEST_TILT_L   1<<0
#define GEST_TILT_R   1<<1
#define GEST_TILT_U   1<<2
#define GEST_TILT_D   1<<3
#define GEST_TAP_0    1<<4
#define GEST_TAP_1    1<<5
#define GEST_TAP_2    1<<6

int loopTime = 20;  // Loop time in milliseconds
long lastLoop = 0;  // Last loop time in milliseconds
long tempLoop = 0;  // Temp loop to reduce overhead
long deadTime = 500; // Dead time between gestures
long retTime = 0;

long lastGest = 0;

typedef struct
{
  // Central hand kinematics
  imu::Vector<3> grav;
  imu::Vector<3> normal_hand;

  imu::Vector<3> ang_pos_hand;
  imu::Vector<3> ang_vel_hand;

  imu::Vector<3> lin_acc_hand;
  imu::Vector<3> lin_vel_hand;
  imu::Vector<3> lin_pos_hand;

  // Finger kinematics
  imu::Vector<3> ang_pos_fing_0;
  imu::Vector<3> ang_vel_fing_0;

} r_data;

TwoWire bus0 = TwoWire(1);
//TwoWire bus1 = TwoWire(1);

int r_buffer_depth = 30;
r_data* r_data_buffer; // Data buffer, n samples deep
int data_buffer_index = 0; // Signifies the newest data index

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &bus0);
Adafruit_BNO055 bno_0 = Adafruit_BNO055(55, 0x28, &bus0);

bool out = false;
int lastBtn = 1;

// Defaults: SDA - Pin 21, SCL - Pin 22
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

//***********************************************************************

void setup()
{
  // link all frames
  linkFrames();
  pinMode(17,INPUT);
  // put your setup code here, to run once:
  bus0.begin(21, 22, 400000);
  //bus1.begin(18, 19, 400000);
  r_data_buffer = new r_data[r_buffer_depth];
  Serial.begin(115200);

  int max_attempts = 10;
  int attempts = 0;
  // Initialise the sensor
  while(!bno.begin())
  {
    bus0.reset();
    attempts++;
    Serial.println("Could not initilize hand BNO055, trying again");
    if(attempts >= max_attempts) {
      Serial.println("Could not initialize hand BNO055, give up");
      break;
    }
  }
  attempts = 0;
//  while(!bno_0.begin())
//  {
//    attempts++;
//    if(attempts >= max_attempts) {
//      Serial.println("Could not initialize finger 1 BNO055");
//      break;
//    }
//  }

  // Use external crystal for better accuracy
  bno.setExtCrystalUse(true);

  // Display some basic information on this sensor
  displaySensorDetails();

  // Hit that low pass
}

void loop()
{
  // Display code
  DM.updateDisplay();
  gestureTest();

  tempLoop = millis();
  if(tempLoop - lastLoop >= loopTime)
  {
    // Update data
    update_hand();
    if (tempLoop >= retTime) {
      long gests = calc_gestures();
      if(gests != lastGest && gests != 0) {
        Serial.println("Gesture: " + String(gests));
        retTime = tempLoop + deadTime;
      }
      lastGest = gests;
    }
    //Display data
    if(lastBtn == 1 && digitalRead(17) == 0) {
      lastBtn = 0;
      out = !out;
    } else if(digitalRead(17) == 1) {
      lastBtn = 1;
    }

    if(out) {
      Serial.print(lastLoop + loopTime);
      Serial.print(", ");
      Serial.print(fromVector(r_data_buffer[data_buffer_index].ang_vel_hand) + ", ");
      Serial.print(fromVector(r_data_buffer[data_buffer_index].ang_pos_hand) + ", ");
      Serial.print(fromVector(r_data_buffer[data_buffer_index].grav) + ", ");
      Serial.print(fromVector(r_data_buffer[data_buffer_index].lin_acc_hand) + ", ");
      Serial.print(fromVector(r_data_buffer[data_buffer_index].lin_vel_hand) + ", ");
      Serial.print(fromVector(r_data_buffer[data_buffer_index].lin_pos_hand) + "\n");
    }

    lastLoop = lastLoop + loopTime;
  }
}

String fromVector(imu::Vector<3> vector) {
  String out = String(vector.x()) + ", " + String(vector.y()) + ", " + String(vector.z());
  return out;
}

void update_hand()
{
  int next_index = (data_buffer_index + 1)%r_buffer_depth; // Calculate the next index

  r_data_buffer[next_index].normal_hand = bno.getQuat().toMatrix().col_to_vector(2);
  r_data_buffer[next_index].ang_vel_hand = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  r_data_buffer[next_index].lin_acc_hand = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  r_data_buffer[next_index].grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  r_data_buffer[next_index].grav.normalize();
  r_data_buffer[next_index].lin_acc_hand = r_data_buffer[next_index].lin_acc_hand/10.19*9.81 - r_data_buffer[next_index].grav;

  // Integration
  r_data_buffer[next_index].lin_vel_hand = r_data_buffer[next_index].lin_vel_hand*0; // Reset lin vel
  r_data_buffer[next_index].lin_pos_hand = r_data_buffer[next_index].lin_pos_hand*0; // Reset lin pos
  r_data_buffer[next_index].ang_pos_hand = r_data_buffer[next_index].ang_pos_hand*0; // Reset ang pos
  for (int i = 0; i < r_buffer_depth; i++) {
    r_data_buffer[next_index].lin_vel_hand = r_data_buffer[next_index].lin_vel_hand + r_data_buffer[i].lin_acc_hand; // Integrate for lin vel
    r_data_buffer[next_index].ang_pos_hand = r_data_buffer[next_index].ang_pos_hand + r_data_buffer[i].ang_vel_hand; // Integrate for ang pos
    if (i != next_index) {
      r_data_buffer[next_index].lin_pos_hand = r_data_buffer[next_index].lin_pos_hand + r_data_buffer[i].lin_vel_hand; // Integrate for lin pos (exclude current index)
    }
  }
  r_data_buffer[next_index].lin_vel_hand = r_data_buffer[next_index].lin_vel_hand * (0.001 * loopTime); // Multiply by dt
  r_data_buffer[next_index].lin_pos_hand = r_data_buffer[next_index].lin_pos_hand + r_data_buffer[next_index].lin_vel_hand; // Final step integration for lin pos
  r_data_buffer[next_index].lin_pos_hand = r_data_buffer[next_index].lin_pos_hand * (0.001 * loopTime); // Multiply by dt
  r_data_buffer[next_index].ang_pos_hand = r_data_buffer[next_index].ang_pos_hand * (0.001 * loopTime); // Multiply by dt

  //r_data_buffer[next_index].ang_pos_fing_0 = bno_0.getVector(Adafruit_BNO055::VECTOR_EULER);
  //r_data_buffer[next_index].ang_vel_fing_0 = bno_0.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  data_buffer_index = next_index; // Increment the current index
}

long calc_gestures() {
  long out = 0;
  r_data* data = &r_data_buffer[data_buffer_index];
  if(data->ang_pos_hand.x() <= -40 && data->grav.dot(imu::Vector<3>(0,-1,0)) >= 0.6) { // right
    out = out | GEST_TILT_R;
    gesture.right = 1;
    gesture.flag = 1;
  } else if(data->ang_pos_hand.x() >= 40 && data->grav.dot(imu::Vector<3>(0,1,0)) >= 0.5) { // left
    out = out | GEST_TILT_L;
    gesture.left = 1;
    gesture.flag = 1;
  } else if(data->ang_pos_hand.y() >= 40 && data->grav.dot(imu::Vector<3>(-1,0,0)) >= 0.5) { // up
    out = out | GEST_TILT_U;
    gesture.up = 1;
    gesture.flag = 1;
  } else if(data->ang_pos_hand.y() <= -40 && data->grav.dot(imu::Vector<3>(1,0,0)) >= 0.5) { // down
    out = out | GEST_TILT_D;
    gesture.down = 1;
    gesture.flag = 1;
  }

  return out;
}

void linkFrames(){
  // link frames here
  homeFrame.right = &hue;
  hue.left = &homeFrame;

  hue.up = &light1;
  light1.down = &hue;

  light1.up = &light2;
  light2.down = &light1;
}
