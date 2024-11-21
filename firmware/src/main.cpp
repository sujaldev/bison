#include <WiFi.h>
WiFiClient client;
WiFiServer server(80);

//DHT11 Temperature and Humidity
#include <DHT.h>
#include <DHT_U.h>
int DHTPIN=2;
#define DHTTYPE DHT11
DHT dht(DHTPIN,DHTTYPE);

int h,t,p,alt;

//BMP280 Altitude and Pressure
#include  <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C Interface

//MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;



// Constants for Calculations
  #define METABOLIC_EFFICIENCY 0.2  // Efficiency for cycling (20%)
  #define TIME_STEP 0.1             // Sampling time step in seconds (100 ms)
  #define PEDAL_ARM_LENGTH 0.175    // Pedal arm length in meters
 // Global Variables
 float totalEnergy = 0.0; // Accumulated energy in joules
float caloriesBurned = 0;
unsigned long previousTime = 0;

//HX711+LOAD CELL
#include "HX711.h"
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 14;
HX711 scale;

//GPS MODULE
#include <TinyGPS++.h>
static const int RXPin = 35, TXPin = 34;
static const uint32_t GPSBaud = 9600;
// The TinyGPS++ object
TinyGPSPlus gps;
#include <SoftwareSerial.h>
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
double previousLat = 0.0;
double previousLng = 0.0;
double totalDistance = 0.0;

void setup() 
{
  Serial.begin(9600);
  WiFi.begin("Ocean134", "ocean123");
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(200);
    Serial.print("..");
  }
  Serial.println();
  Serial.println("NodeMCU is connected!");
  Serial.println(WiFi.localIP());
  server.begin();
  
  //BMP
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 21 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
 // Initialize the HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(230); // Replace 230 with your calibrated scale factor
  scale.tare();         // Reset the scale to 0
  Serial.println("Scale is ready. Tared."); 

  //GPS MODULE
  ss.begin(GPSBaud);      
  } 

// Function to calculate distance between two points using Haversine formula
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000; // Radius of Earth in meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);

  lat1 = radians(lat1);
  lat2 = radians(lat2);

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c; // Distance in meters
  }
  
void loop() 
{
  //Temperature and altitude
   h=dht.readHumidity();
   t=dht.readTemperature();

   Serial.print("Humidity:");
   Serial.print(h);
   Serial.print("%,Temp:");
   Serial.print(t);
   Serial.println(" Celsius");

   //Pressure and altitude
   p=bmp.readPressure()/10;
   alt=bmp.readAltitude(1019.66);
   Serial.print("Pressure:");
   Serial.print(p);
   Serial.println("  hPa");

   Serial.print("Altitude:");
   Serial.print(alt);
   Serial.println("  m");   

  //LOAD CELL
  float reading = scale.get_units(10); // Average 10 readings for better stability
  if (!scale.is_ready()) {
    Serial.println("HX711 not ready. Check wiring.");
    delay(500);
    return;
  }

   //MPU6050 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

	/* Print out the values */
	Serial.print("Acceleration X: ");
	Serial.print(a.acceleration.x);
	Serial.print(", Y: ");
	Serial.print(a.acceleration.y);
	Serial.print(", Z: ");
	Serial.print(a.acceleration.z);
	Serial.println(" m/s^2");

	Serial.print("Rotation X: ");
	Serial.print(g.gyro.x);
	Serial.print(", Y: ");
	Serial.print(g.gyro.y);
	Serial.print(", Z: ");
	Serial.print(g.gyro.z);
	Serial.println(" rad/s"); 

   // Acceleration and Angular Velocity
  float acceleration = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));
  float angularVelocity = sqrt(pow(g.gyro.x, 2) + pow(g.gyro.y, 2) + pow(g.gyro.z, 2));

  // Calculate RPM
  int rpm = (angularVelocity * 60.0) / (2.0 * M_PI);

  // Calculate Torque
  float torque = reading * PEDAL_ARM_LENGTH;

  // Calculate Power
  float Power = torque * angularVelocity;

  // CalculateEnergy and Calories Burned
  float energy = Power * TIME_STEP; // Joules
  totalEnergy += energy;
  float caloriesBurned = METABOLIC_EFFICIENCY * (totalEnergy / 4184.0);

  // Display the results
  Serial.print("RPM: ");
  Serial.println(rpm);

  Serial.print("Torque (Nm): ");
  Serial.println(torque);

  Serial.print("Power (Watts): ");
  Serial.println(Power);

  Serial.print("Calories Burned: ");
  Serial.println(caloriesBurned); 



  //GPS MODULE
    while (ss.available() > 0) {
    gps.encode(ss.read());

    if (gps.location.isUpdated()) {
      double currentLat = gps.location.lat();
      double currentLng = gps.location.lng();

      // Check if there is a valid previous point to calculate distance
      if (previousLat != 0.0 && previousLng != 0.0) {
        double distance = calculateDistance(previousLat, previousLng, currentLat, currentLng);
        totalDistance += distance; // Accumulate total distance
      }

      // Update previous coordinates
      previousLat = currentLat;
      previousLng = currentLng;

      // Print GPS coordinates and total distance
      Serial.print("Latitude= ");
      Serial.print(currentLat, 6);
      Serial.print(" Longitude= ");
      Serial.print(currentLng, 6);
      Serial.print(" Distance Travelled= ");
      Serial.print(totalDistance, 2); // Print distance in meters
      Serial.println(" m");
    }
  }
  //WEBSITE
   WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client Connected");
    // Wait until the client sends a request
    while (client.connected() && !client.available()) {
      delay(1);
    }

    // Read and ignore the client request (we are not parsing it here)
    client.readStringUntil('\r');

    // Send HTTP response headers
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();

    // Start of the HTML document
    client.println("<!DOCTYPE HTML>");
    client.println("<html>");
    client.println("<head><title>Sensor Data</title></head>");
    client.println("<body>");

    // Display welcome message and sensor data
    client.println("<h1>Welcome to the MPU6050 Sensor Data Page</h1>");
    client.println("<h3>LED Controls</h3>");
    client.println("<br>");
    client.println("<p><b>TEMPERATURE AND HUMIDITY Data:</b></p>");
    client.println("<p>" + String(h) + "</p>");
    client.println("<p>" + String(t) + "</p>");
    client.println("<p><b>PRESSURE AND ALTITUDE Data:</b></p>");
    client.println("<p>" + String(p) + "</p>"); 
    client.println("<p>" + String(alt) + "</p>");
    client.println("<p><b>IMU DATA:</b></p>");
    client.println("<p>" + String(rpm)+ "</p>");
    client.println("<p>" + String(torque) + "</p>");
    client.println("<p>" + String(Power) + "</p>");
    client.println("<p>" + String(caloriesBurned) + "</p>");
    client.println("<p><b>FORCE APPLIED ON PEDAL:</b></p>");
    long reading = scale.get_units(10);
    client.println("<p>Current Scale Reading: " + String(reading) + " grams</p>");
    client.println("<p><b>DISTANCE TRAVELLED:</b></p>"); //totalDistance
    client.println("<p>" + String(totalDistance) + "</p>");
    

    // End of HTML document
    client.println("</body>");
    client.println("</html>");
  }
}


