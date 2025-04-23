#include <Wire.h>
#define Addr 0x7B
#define LED 13
#define I2C_DEVICEID 0x00
#define I2C_POWERMODE 0x01
#define I2C_FREQUENCY 0x02
#define I2C_SHAPE 0x03
#define I2C_BOOST 0x04
#define I2C_PVOLTAGE 0x05
#define I2C_P1VOLTAGE 0x06
#define I2C_P2VOLTAGE 0x07
#define I2C_P3VOLTAGE 0x08
#define I2C_P4VOLTAGE 0x09
#define I2C_UPDATEVOLTAGE 0x0A


// Define PID variables for pump 4
double setpoint4 = 2.0;                    // Desired flow rate (ml/min) entered by the user
double integral4 = 0.0;                    // Accumulated error for integral term
double prevError4 = 0;                     // Previous error for derivative term
double Kp = 4.1;                           // PID proportional constant
double Ki = 0.505;                         // PID integral constant
double Kd = 0.00005;                       // PID derivative constant
const double derivativeFilterCoeff = 0.5;  // Adjust this value to control the filtering effect (0.0 to 1.0)
const double smoothingFactor = 0.5;

double prevPumpVoltage4 = 0.0;   // Previous pump voltage value
double maxVoltageChange = 31.0;  // Maximum allowed voltage change per iteration

const int recVin = 7;


int pumpVoltage4 = 0.0;

// Define PID tuning constants
double sampleTime = 0.5;  // Sample time in seconds
double outMin = 0;        // Minimum output value (0V)
double outMax = 250;      // Maximum output value (250V)

//int userAmplitudeInput = 240;
int workingTime_seconds = 0;

int temp = (512 * 100) / 800;  // set the working Hz by changing the value. Set to 100Hz.
int pumpBytes[4] = { 0 };      // Stores the voltages of the pumps in bytes

//int pumpVoltage = map(output, 0, 255, 0, 32); // Modify the min and max values (0 and 255) to the appropriate voltage range

const int ADDRESS = 0x08;               // Sensor I2C Address
const float SCALE_FACTOR_FLOW = 500.0;  // Scale Factor for flow rate measurement
const float SCALE_FACTOR_TEMP = 200.0;  // Scale Factor for temperature measurement
const char *UNIT_FLOW = " ml/min";      //physical unit of the flow rate measurement
const char *UNIT_TEMP = " deg C";       //physical unit of the temperature measurement

unsigned long previousTime = 0;
const unsigned long interval = 100;  // adjusting how often the sensor will read

const int valve1 = 5;
const int valve2 = 4;
const int valve3 = 3;
const int valve4 = 2;
double pumpOutput4 = 0;

int userInput = 0;
int previousInput = 0;

double input4;  // Current flow rate read from the sensor (ml/min)
double output4;




void setup() {

  int ret;



  Serial.begin(115200);
  Serial.println();





  /// pump section

  Wire.begin();
  Wire.beginTransmission(Addr);
  Wire.write(I2C_POWERMODE);
  Wire.write(0x00);  // Disables the pumps
  Wire.endTransmission();

  do {
    // Soft reset the sensor
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while sending soft reset command, retrying...");
      delay(50);  // wait long enough for chip reset to complete
    }
  } while (ret != 0);

  delay(50);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(valve1, OUTPUT);
  pinMode(valve2, OUTPUT);
  pinMode(valve3, OUTPUT);
  pinMode(valve4, OUTPUT);

  digitalWrite(recVin, HIGH);
  digitalWrite(valve1, HIGH);
  digitalWrite(valve2, HIGH);
  digitalWrite(valve3, HIGH);
  digitalWrite(valve4, LOW);
}


void loop() {
  int ret;
  uint16_t aux_value;
  uint16_t sensor_flow_value;
  uint16_t sensor_temp_value;
  int16_t signed_flow_value;
  int16_t signed_temp_value;
  float scaled_flow_value;
  float scaled_temp_value;
  byte aux_crc;
  byte sensor_flow_crc;
  byte sensor_temp_crc;

  if (Serial.available()) {
    userInput = Serial.parseInt();
    workingTime_seconds = Serial.parseInt();
  }

  int workingTime = workingTime_seconds * 1000;

  unsigned long currentTime = millis();
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;

    Wire.beginTransmission(ADDRESS);
    Wire.write(0x36);
    Wire.write(0x08);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write measurement mode command");
    } else {
      delay(100);

      for (int i = 0; i < 10; ++i) {
        delay(100);
        Wire.requestFrom(ADDRESS, 9);
        if (Wire.available() < 9) {
          Serial.println("Error while reading flow measurement");
          continue;
        }

        sensor_flow_value = Wire.read() << 8;  // read the MSB from the sensor
        sensor_flow_value |= Wire.read();      // read the LSB from the sensor
        sensor_flow_crc = Wire.read();
        aux_value = Wire.read() << 8;  // read the MSB from the sensor
        aux_value |= Wire.read();      // read the LSB from the sensor
        aux_crc = Wire.read();

        //        Serial.print("Flow value from Sensor: ");
        //        Serial.print(sensor_flow_value);

        signed_flow_value = (int16_t)sensor_flow_value;
        scaled_flow_value = ((float)signed_flow_value) / SCALE_FACTOR_FLOW;
        //        Serial.print(", scaled value: ");
        //        Serial.print(scaled_flow_value);
        //        Serial.print(UNIT_FLOW);

        input4 = scaled_flow_value;

        Serial.println("");
      }

      Wire.beginTransmission(ADDRESS);
      Wire.write(0x3F);
      Wire.write(0xF9);
      ret = Wire.endTransmission();
      if (ret != 0) {
        Serial.println("Error during write measurement mode command");
      }
    }




    Wire.beginTransmission(Addr);
    Wire.write(I2C_POWERMODE);
    Wire.write(0x01);  // Enable pumps
    Wire.write(0x40);  /// setting the pumps at 100 hz.
    Wire.endTransmission();


    calculatePID4(setpoint4, pumpOutput4, scaled_flow_value);
    pumpVoltage4 = pumpOutput4;  // Round to the nearest integer
    if (pumpVoltage4 > 31) {
      pumpVoltage4 = 31;
    }
    if (userInput == 5) {
      Wire.beginTransmission(Addr);
      Wire.write(I2C_POWERMODE);
      Wire.write(0x00);  // Disables the pumps
      Wire.endTransmission();
      digitalWrite(valve1, HIGH);
      digitalWrite(valve2, HIGH);
      digitalWrite(valve4, LOW);
      digitalWrite(valve3, HIGH);
      Serial.println("Pumps are set to off");
      Wire.endTransmission();
    }
    Wire.beginTransmission(Addr);
    Wire.write(I2C_P1VOLTAGE);
    if (userInput == 1) {
      Wire.write(pumpVoltage4);
      digitalWrite(valve1, LOW);  /// valve1 normalde değiştirmeyi unutma
      digitalWrite(valve2, HIGH);
      digitalWrite(valve3, HIGH);
      digitalWrite(valve4, HIGH);
      Wire.endTransmission();


      Serial.println("First pump is open");
    } else {
      Wire.write(0x00);
      digitalWrite(valve1, HIGH);
      Wire.endTransmission();
    }

    Wire.beginTransmission(Addr);
    Wire.write(I2C_P2VOLTAGE);
    if (userInput == 2) {
      Wire.write(pumpVoltage4);
      digitalWrite(valve2, LOW);
      digitalWrite(valve1, HIGH);
      digitalWrite(valve3, HIGH);
      digitalWrite(valve4, HIGH);
      Serial.println("2nd pump is open");
    } else {
      Wire.write(0x00);
      digitalWrite(valve2, HIGH);
    }
    Wire.endTransmission();


    Wire.beginTransmission(Addr);
    Wire.write(I2C_P2VOLTAGE);
    if (userInput == 3) {
      Wire.write(pumpVoltage4);
      digitalWrite(valve3, LOW);
      digitalWrite(valve1, HIGH);
      digitalWrite(valve2, HIGH);
      digitalWrite(valve4, HIGH);
      Serial.println("2nd pump is open");
    } else {
      Wire.write(0x00);
      digitalWrite(valve3, HIGH);
    }
    Wire.endTransmission();

    Wire.beginTransmission(Addr);
    Wire.write(I2C_P2VOLTAGE);
    if (userInput == 4) {
      Wire.write(pumpVoltage4);
      digitalWrite(valve4, LOW);
      digitalWrite(valve1, HIGH);
      digitalWrite(valve2, HIGH);
      digitalWrite(valve4, HIGH);
      Serial.println("2nd pump is open");
    } else {
      Wire.write(0x00);
      digitalWrite(valve4, HIGH);
    }
    Wire.endTransmission();

    // Serial.println(pumpVoltage4);

    Wire.beginTransmission(Addr);
    Wire.write(I2C_UPDATEVOLTAGE);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(workingTime);  // Wait for the specified working time
    //    Serial.print("Setpoint: ");
    //    Serial.print(setpoint4);
    //    Serial.print(", Sensor Flow Rate: ");
    Serial.print(scaled_flow_value);
    Serial.println(UNIT_FLOW);
    Serial.println(pumpVoltage4);
  }
}

void calculatePID4(double setpoint4, double &desiredPumpVoltage4, double scaled_flow_value) {
  unsigned long now = millis();
  static unsigned long lastTime = now;
  double timeChange = (double)(now - lastTime) / 1000.0;  // Time in seconds
  if (timeChange >= sampleTime) {
    // Calculate error
    double error = setpoint4 - scaled_flow_value;  // scaled_flow_value is the current flow rate

    // Calculate integral term
    integral4 += (Ki * error * timeChange);
    if (integral4 > outMax) integral4 = outMax;
    else if (integral4 < outMin) integral4 = outMin;

    // Calculate derivative term
    double dError = (error - prevError4) / timeChange;

    // Apply derivative filter
    static double filteredDError = dError;
    filteredDError = filteredDError * (1.0 - derivativeFilterCoeff) + dError * derivativeFilterCoeff;

    // Calculate PID output
    double proportional = error;

    // Calculate the desired pump voltage without filtering
    double desiredPumpVoltageWithoutFilter = (Kp * proportional) + (Ki * integral4) + (Kd * filteredDError);

    // Apply low-pass filtering to the pump output
    static double filteredPumpVoltage = desiredPumpVoltageWithoutFilter;
    filteredPumpVoltage = filteredPumpVoltage + smoothingFactor * (desiredPumpVoltageWithoutFilter - filteredPumpVoltage);

    // Apply rate limiting to the pump voltage
    double voltageChange = filteredPumpVoltage - prevPumpVoltage4;

    // Limit the voltage change to the maximum allowed rate
    if (voltageChange > maxVoltageChange) {
      filteredPumpVoltage = prevPumpVoltage4 + maxVoltageChange;
    } else if (voltageChange < -maxVoltageChange) {
      filteredPumpVoltage = prevPumpVoltage4 - maxVoltageChange;
    }

    // Limit the output based on output limits to prevent saturation
    if (filteredPumpVoltage > outMax) {
      filteredPumpVoltage = outMax;
    } else if (filteredPumpVoltage < outMin) {
      filteredPumpVoltage = outMin;
    }

    // Update the desired pump voltage with the filtered value
    desiredPumpVoltage4 = filteredPumpVoltage;

    prevPumpVoltage4 = desiredPumpVoltage4;
    prevError4 = error;
    lastTime = now;
  }
}
