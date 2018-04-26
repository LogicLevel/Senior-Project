#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "DisplayModule.h"
#include "Frame.h"
#include "Gesture.h"
#include "Haptic.h"


TwoWire bus0 = TwoWire(1);
TwoWire bus1 = TwoWire(0);

Gesture gesture = Gesture();
Haptic haptic = Haptic(22);
  // oled setup
SSD1306 display(0x3d, &bus1);
byte wifiStat = 0;
byte blutoothStat = 0;
byte batteryStat = 0;

// declare all frame object and link them together here
Frame homeFrame = Frame("home");

Frame hue = Frame("Hue");
Frame light1 = Frame("Light1");
Frame light2 = Frame("Light2");

DisplayModule DM = DisplayModule(&display, &wifiStat, &blutoothStat, &batteryStat, &homeFrame, &gesture, &haptic);

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

int batLoopTime = 2000;
int lastBatLoop = 0;

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
  imu::Vector<3> grav_fing_0;
  imu::Vector<3> ang_pos_fing_0;
  imu::Vector<3> ang_vel_fing_0;

  imu::Vector<3> grav_fing_1;
  imu::Vector<3> ang_pos_fing_1;
  imu::Vector<3> ang_vel_fing_1;

  imu::Vector<3> grav_fing_2;
  imu::Vector<3> ang_pos_fing_2;
  imu::Vector<3> ang_vel_fing_2;
} r_data;



int r_buffer_depth = 30;
r_data* r_data_buffer; // Data buffer, n samples deep
int data_buffer_index = 0; // Signifies the newest data index

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &bus0);
Adafruit_BNO055 bno_0 = Adafruit_BNO055(55, 0x28, &bus0);
Adafruit_BNO055 bno_1 = Adafruit_BNO055(55, 0x29, &bus1);
Adafruit_BNO055 bno_2 = Adafruit_BNO055(55, 0x28, &bus1);

bool out = false;
bool lastBtn = false;
bool displayState = true;

int lastTouch = 0;
int currTouch = 0;
float alphTouch = 0.2;

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
  Serial.begin(115200);
  // link all frames
  pinMode(23,OUTPUT);
  digitalWrite(23,HIGH);
  // put your setup code here, to run once:
  bus0.begin(18, 5, 400000);
  delay(100);
  scanI2C(&bus0);
  delay(100);
  bus1.begin(17, 16, 400000);
  delay(100);
  bus0.reset();
  delay(100);
  bus1.reset();
  delay(100);
  r_data_buffer = new r_data[r_buffer_depth];
  if(initBNO(&bno, 5)) {
    Serial.println("Initialized hand IMU");
  } else {
    Serial.println("Could not initialize hand IMU, give up");
  }
  if(initBNO(&bno_0, 5)) {
    Serial.println("Initialized finger 0 IMU");
  } else {
    Serial.println("Could not initialize finger 0 IMU, give up");
  }
//  if(initBNO(&bno_1, 5)) {
//    Serial.println("Initialized finger 1 IMU");
//  } else {
//    Serial.println("Could not initialize finger 1 IMU, give up");
//  }
//    if(initBNO(&bno_2, 5)) {
//    Serial.println("Initialized finger 2 IMU");
//  } else {
//    Serial.println("Could not initialize finger 2 IMU, give up");
//  }

  linkFrames();
  DM.setup();
  haptic.setup();
  // Hit that low pass
}

bool initBNO(Adafruit_BNO055* bno, int max_attempts) {
  int attempts = 0;
  // Initialise the sensor
  while(!bno->begin())
  {
    attempts++;
    if(attempts >= max_attempts) {
      return false;
      break;
    }
  }
  if(attempts != max_attempts) {
    //bno->setExtCrystalUse(true);
    return true;
  } else {
    return false;
  }
}

void loop()
{
  haptic.compute();
  tempLoop = millis();
  if(tempLoop - lastBatLoop >= batLoopTime)
  {
    float vbat = 0.0006495*analogRead(36) + 1.6452;
    Serial.println(vbat);
    batteryStat = (int)(4*(vbat-3.1)/(4.3-3.1)); // Not really 3.1 volts, ADC is kinda nonlinear
    if(batteryStat > 3) {
      batteryStat = 3;
    } else if(batteryStat < 0) {
      batteryStat = 0;
    }
    if(displayState) {
      DM.displayClear();
      DM.drawDisplay();
    }
    lastBatLoop = lastBatLoop + batLoopTime;
  }
  if(tempLoop - lastLoop >= loopTime)
  {
    // Touch sensor
    int tempTouch = touchRead(T7);
    lastTouch = currTouch;
    currTouch = (int)(alphTouch * tempTouch + (1-alphTouch) * lastTouch);
    bool btn = (currTouch < 20);
    if(btn == true && lastBtn == false) {
      displayState = !displayState;
      if(displayState) {
        DM.displayOn();
      } else {
        DM.displayOff();
      }
    }
    lastBtn = btn;
    if(displayState) {
      DM.updateDisplay();
    }

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

    if(false) {
      Serial.print(lastLoop + loopTime);
      Serial.print(", ");
      Serial.print(fromVector(r_data_buffer[data_buffer_index].ang_vel_hand) + ", ");
      Serial.print(fromVector(r_data_buffer[data_buffer_index].ang_vel_fing_0) + ", ");
      Serial.print(fromVector(r_data_buffer[data_buffer_index].ang_vel_fing_1) + ", ");
      Serial.print(fromVector(r_data_buffer[data_buffer_index].ang_vel_fing_2) + "\n");
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

  r_data_buffer[next_index].ang_vel_fing_0 = bno_0.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  r_data_buffer[next_index].ang_vel_fing_0 = imu::Vector<3>(-1*r_data_buffer[next_index].ang_vel_fing_0.x(), -1*r_data_buffer[next_index].ang_vel_fing_0.y(), r_data_buffer[next_index].ang_vel_fing_0.z());
  //r_data_buffer[next_index].ang_vel_fing_0 = r_data_buffer[next_index].ang_vel_fing_0 - r_data_buffer[next_index].ang_vel_hand;
  r_data_buffer[next_index].grav_fing_0 = bno_0.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  r_data_buffer[next_index].grav_fing_0.normalize();
  r_data_buffer[next_index].grav_fing_0 = imu::Vector<3>(r_data_buffer[next_index].grav_fing_0.x(), -1*r_data_buffer[next_index].grav_fing_0.y(), r_data_buffer[next_index].grav_fing_0.z());
//
//  r_data_buffer[next_index].ang_vel_fing_1 = bno_1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//  r_data_buffer[next_index].ang_vel_fing_1 = imu::Vector<3>(-1*r_data_buffer[next_index].ang_vel_fing_1.x(), -1*r_data_buffer[next_index].ang_vel_fing_1.y(), r_data_buffer[next_index].ang_vel_fing_1.z());
//  //r_data_buffer[next_index].ang_vel_fing_1 = r_data_buffer[next_index].ang_vel_fing_1 - r_data_buffer[next_index].ang_vel_hand;
//  r_data_buffer[next_index].grav_fing_1 = bno_1.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
//  r_data_buffer[next_index].grav_fing_1.normalize();
//  r_data_buffer[next_index].grav_fing_1 = imu::Vector<3>(r_data_buffer[next_index].grav_fing_1.x(), -1*r_data_buffer[next_index].grav_fing_1.y(), r_data_buffer[next_index].grav_fing_1.z());
//
//  r_data_buffer[next_index].ang_vel_fing_2 = bno_2.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//  r_data_buffer[next_index].ang_vel_fing_2 = imu::Vector<3>(-1*r_data_buffer[next_index].ang_vel_fing_2.x(), -1*r_data_buffer[next_index].ang_vel_fing_2.y(), r_data_buffer[next_index].ang_vel_fing_2.z());
//  r_data_buffer[next_index].ang_vel_fing_2 = r_data_buffer[next_index].ang_vel_fing_2 - r_data_buffer[next_index].ang_vel_hand;
//  r_data_buffer[next_index].grav_fing_2 = bno_2.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
//  r_data_buffer[next_index].grav_fing_2.normalize();
//  r_data_buffer[next_index].grav_fing_2 = imu::Vector<3>(r_data_buffer[next_index].grav_fing_2.x(), -1*r_data_buffer[next_index].grav_fing_2.y(), r_data_buffer[next_index].grav_fing_2.z());

  // Integration
  r_data_buffer[next_index].lin_vel_hand = r_data_buffer[next_index].lin_vel_hand*0; // Reset lin vel
  r_data_buffer[next_index].lin_pos_hand = r_data_buffer[next_index].lin_pos_hand*0; // Reset lin pos
  r_data_buffer[next_index].ang_pos_hand = r_data_buffer[next_index].ang_pos_hand*0; // Reset ang pos

  r_data_buffer[next_index].ang_pos_fing_0 = r_data_buffer[next_index].ang_pos_fing_0*0;
  for (int i = 0; i < r_buffer_depth; i++) {
    r_data_buffer[next_index].lin_vel_hand = r_data_buffer[next_index].lin_vel_hand + r_data_buffer[i].lin_acc_hand; // Integrate for lin vel
    r_data_buffer[next_index].ang_pos_hand = r_data_buffer[next_index].ang_pos_hand + r_data_buffer[i].ang_vel_hand; // Integrate for ang pos
    if (i != next_index) {
      r_data_buffer[next_index].lin_pos_hand = r_data_buffer[next_index].lin_pos_hand + r_data_buffer[i].lin_vel_hand; // Integrate for lin pos (exclude current index)
    }

    r_data_buffer[next_index].ang_pos_fing_0 = r_data_buffer[next_index].ang_pos_fing_0 + r_data_buffer[i].ang_vel_fing_0;
    r_data_buffer[next_index].ang_pos_fing_1 = r_data_buffer[next_index].ang_pos_fing_1 + r_data_buffer[i].ang_vel_fing_1;
    r_data_buffer[next_index].ang_pos_fing_2 = r_data_buffer[next_index].ang_pos_fing_2 + r_data_buffer[i].ang_vel_fing_2;
  }
  r_data_buffer[next_index].lin_vel_hand = r_data_buffer[next_index].lin_vel_hand * (0.001 * loopTime); // Multiply by dt
  r_data_buffer[next_index].lin_pos_hand = r_data_buffer[next_index].lin_pos_hand + r_data_buffer[next_index].lin_vel_hand; // Final step integration for lin pos
  r_data_buffer[next_index].lin_pos_hand = r_data_buffer[next_index].lin_pos_hand * (0.001 * loopTime); // Multiply by dt
  r_data_buffer[next_index].ang_pos_hand = r_data_buffer[next_index].ang_pos_hand * (0.001 * loopTime); // Multiply by dt

  r_data_buffer[next_index].ang_pos_fing_0 = r_data_buffer[next_index].ang_pos_fing_0 * (0.001 * loopTime);
  r_data_buffer[next_index].ang_pos_fing_1 = r_data_buffer[next_index].ang_pos_fing_1 * (0.001 * loopTime);
  r_data_buffer[next_index].ang_pos_fing_2 = r_data_buffer[next_index].ang_pos_fing_2 * (0.001 * loopTime);

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
  } else if(data->ang_vel_fing_0.x() >= 220 && (data->ang_vel_fing_0.x() - data->ang_vel_hand.x()) >= 100) {
    out = out | GEST_TAP_0;
    gesture.select = 1;
    gesture.flag = 1;
  } else if(data->ang_vel_fing_1.x() >= 220 && (data->ang_vel_fing_1.x() - data->ang_vel_hand.x()) >= 100) {
    out = out | GEST_TAP_1;
    gesture.select = 1;
    gesture.flag = 1;
  } else if(data->ang_vel_fing_2.x() >= 220 && (data->ang_vel_fing_2.x() - data->ang_vel_hand.x()) >= 100) {
    out = out | GEST_TAP_2;
    gesture.select = 1;
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

void scanI2C(TwoWire* twi) {
  int nDevices = 0;
  Serial.println("Scanning for I2C devices");
  for(int address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    twi->beginTransmission(address);
    int error = twi->endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
}
