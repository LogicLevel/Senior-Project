#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"

int loopTime = 20;  // Loop time in milliseconds
long lastLoop = 0;  // Last loop time in milliseconds
long tempLoop = 0;  // Temp loop to reduce overhead

typedef struct 
{
  // Central hand kinematics
  imu::Vector<3> grav;
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
    attempts++;
    if(attempts >= max_attempts) {
      Serial.println("Could not initialize hand BNO055");
      break;
    }
  }
  attempts = 0;
  while(!bno_0.begin())
  {
    attempts++;
    if(attempts >= max_attempts) {
      Serial.println("Could not initialize finger 1 BNO055");
      break;
    }
  }

  // Use external crystal for better accuracy
  bno.setExtCrystalUse(true);
   
  // Display some basic information on this sensor
  displaySensorDetails();

  // Hit that low pass
}

void loop() 
{
  if((tempLoop = millis()) - lastLoop >= loopTime) 
  {         
    // Update data
    update_hand();

    //Display data 
    Serial.print(fromVector(r_data_buffer[data_buffer_index].lin_acc_hand) + ", ");
    Serial.print(fromVector(r_data_buffer[data_buffer_index].lin_vel_hand) + "\n");
    // Serial.print("Hand Position: "); Serial.println();
    
    lastLoop = tempLoop;
  }
}

String fromVector(imu::Vector<3> vector) {
  String out = String(vector.x()) + ", " + String(vector.y()) + ", " + String(vector.z());
  return out;
}

void update_hand()
{
  int next_index = (data_buffer_index + 1)%r_buffer_depth; // Calculate the next index

  r_data_buffer[next_index].ang_pos_hand = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  r_data_buffer[next_index].ang_vel_hand = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  r_data_buffer[next_index].lin_acc_hand = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  r_data_buffer[next_index].grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  r_data_buffer[next_index].lin_acc_hand = r_data_buffer[next_index].lin_acc_hand/10.19*9.81 - r_data_buffer[next_index].grav;
//  if(fabs(r_data_buffer[next_index].lin_acc_hand.x()) < 0.05) {
//    r_data_buffer[next_index].lin_acc_hand = r_data_buffer[next_index].lin_acc_hand.dot(imu::Vector<3>(0,1,1));
//  }
//  if(fabs(r_data_buffer[next_index].lin_acc_hand.y()) < 0.05) {
//    r_data_buffer[next_index].lin_acc_hand = r_data_buffer[next_index].lin_acc_hand.dot(imu::Vector<3>(1,0,1));
//  }
//  if(fabs(r_data_buffer[next_index].lin_acc_hand.z()) < 0.05) {
//    r_data_buffer[next_index].lin_acc_hand = r_data_buffer[next_index].lin_acc_hand.dot(imu::Vector<3>(1,1,0));
//  }
  
  // Integrate acceleration for velocity

  r_data_buffer[next_index].lin_vel_hand = r_data_buffer[next_index].lin_vel_hand*0;
  for (int i = 0; i < r_buffer_depth; i++) {
    if (i != next_index) {
      r_data_buffer[next_index].lin_vel_hand = r_data_buffer[next_index].lin_vel_hand + r_data_buffer[i].lin_acc_hand;
    }
  }
  r_data_buffer[next_index].lin_vel_hand = r_data_buffer[next_index].lin_vel_hand * (0.001 * loopTime);

  r_data_buffer[next_index].lin_pos_hand = r_data_buffer[next_index].lin_pos_hand*0;
  for (int i = 0; i < r_buffer_depth; i++) {
    if (i != next_index) {
      r_data_buffer[next_index].lin_pos_hand = r_data_buffer[next_index].lin_pos_hand + r_data_buffer[i].lin_vel_hand;
    }
  }
  r_data_buffer[next_index].lin_pos_hand = r_data_buffer[next_index].lin_pos_hand * (0.001 * loopTime);

  r_data_buffer[next_index].ang_pos_fing_0 = bno_0.getVector(Adafruit_BNO055::VECTOR_EULER);
  r_data_buffer[next_index].ang_vel_fing_0 = bno_0.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  data_buffer_index = next_index; // Increment the current index
}

