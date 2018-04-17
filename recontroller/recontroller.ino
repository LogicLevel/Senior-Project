#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"

int loopTime = 100;  // Loop time in milliseconds
long lastLoop = 0;  // Last loop time in milliseconds
long tempLoop = 0;  // Temp loop to reduce overhead

typedef struct 
{
  // Central hand kinematics
  float ang_pos_hand[3];
  float ang_vel_hand[3];

  float lin_acc_hand[3];
  float lin_vel_hand[3];
  float lin_pos_hand[3];

  // Finger kinematics
  float ang_pos_fing[3][3];
  float ang_vel_fing[3][3];

} r_data;

TwoWire bus0 = TwoWire(1);
TwoWire bus1 = TwoWire(1);

int r_buffer_depth = 10;
r_data* r_data_buffer; // Data buffer, n samples deep
int data_buffer_index = 0; // Signifies the newest data index

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &bus0);
Adafruit_BNO055 bno_1 = Adafruit_BNO055(55, 0x29, &bus0);


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
  Wire.begin(21, 22, 400000);
  r_data_buffer = new r_data[r_buffer_depth];
  Serial.begin(115200);

    // Initialise the sensor
  while(!bno.begin())
  {}

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
    // Poll IMUs
    sensors_event_t event; 
    bno.getEvent(&event);
          
    // Update data
    new_r_data(&event);

    //Display data 
    Serial.print(fromVector(r_data_buffer[data_buffer_index].lin_acc_hand) + "\n");
    // Serial.print("Hand Position: "); Serial.println();
    
    lastLoop = tempLoop;
  }
}

String fromVector(float* vector) {
  String out = String(vector[0]) + ", " + String(vector[1]) + ", " + String(vector[2]);
  return out;
}

void new_r_data(sensors_event_t* event)
{
  int next_index = (data_buffer_index + 1)%r_buffer_depth; // Calculate the next index
  memcpy(r_data_buffer[next_index].ang_pos_hand, event->orientation.v, 3*sizeof(float)); // Copy in the orientation vector
  float x_accel = (int16_t) ((bno.read8(Adafruit_BNO055::BNO055_ACCEL_DATA_X_MSB_ADDR) << 8) | (bno.read8(Adafruit_BNO055::BNO055_ACCEL_DATA_X_LSB_ADDR)));
  float y_accel = (int16_t) ((bno.read8(Adafruit_BNO055::BNO055_ACCEL_DATA_Y_MSB_ADDR) << 8) | (bno.read8(Adafruit_BNO055::BNO055_ACCEL_DATA_Y_LSB_ADDR)));
  float z_accel = (int16_t) ((bno.read8(Adafruit_BNO055::BNO055_ACCEL_DATA_Z_MSB_ADDR) << 8) | (bno.read8(Adafruit_BNO055::BNO055_ACCEL_DATA_Z_LSB_ADDR)));

  float x_gyro = (int16_t) ((bno.read8(Adafruit_BNO055::BNO055_GYRO_DATA_X_MSB_ADDR) << 8) | (bno.read8(Adafruit_BNO055::BNO055_GYRO_DATA_X_LSB_ADDR)));
  float y_gyro = (int16_t) ((bno.read8(Adafruit_BNO055::BNO055_GYRO_DATA_Y_MSB_ADDR) << 8) | (bno.read8(Adafruit_BNO055::BNO055_GYRO_DATA_Y_LSB_ADDR)));
  float z_gyro = (int16_t) ((bno.read8(Adafruit_BNO055::BNO055_GYRO_DATA_Z_MSB_ADDR) << 8) | (bno.read8(Adafruit_BNO055::BNO055_GYRO_DATA_Z_LSB_ADDR)));

  r_data_buffer[next_index].lin_acc_hand[0] = x_accel;
  r_data_buffer[next_index].lin_acc_hand[1] = y_accel;
  r_data_buffer[next_index].lin_acc_hand[2] = z_accel;

  r_data_buffer[next_index].ang_vel_hand[0] = x_gyro;
  r_data_buffer[next_index].ang_vel_hand[1] = y_gyro;
  r_data_buffer[next_index].ang_vel_hand[2] = z_gyro;
  
  // Integrate acceleration for velocity
  for( int j = 0; j < 3; j++ )
  {
    r_data_buffer[next_index].lin_vel_hand[j] = 0; // Reset velocity vector
    for( int i = 0; i < r_buffer_depth; i++ )
    {
      r_data_buffer[next_index].lin_vel_hand[j] += r_data_buffer[i].lin_acc_hand[j]; // Integrate
    }
    r_data_buffer[next_index].lin_vel_hand[j] *= (0.001 * loopTime); // Multiple by dt
  }

  // Integrate velocity for position
  for( int j = 0; j < 3; j++ )
  {
    r_data_buffer[next_index].lin_pos_hand[j] = 0; // Reset position vector
    for( int i = 0; i < r_buffer_depth; i++ )
    {
      r_data_buffer[next_index].lin_pos_hand[j] += r_data_buffer[i].lin_vel_hand[j]; // Integrate
    }
    r_data_buffer[next_index].lin_pos_hand[j] *= (0.001 * loopTime); // Multiply by dt
  }
  
  data_buffer_index = next_index; // Increment the current index
}

