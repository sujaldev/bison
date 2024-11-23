#include <WiFi.h>

WiFiClient client;
WiFiServer server(80);

#include "Wire.h"
#include <math.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;

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

//HX711+LOAD CELL
#include "HX711.h"
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 14;
HX711 scale;

#define METABOLIC_EFFICIENCY 0.2  // Efficiency for cycling (20%)
#define TIME_STEP 0.1             // Sampling time step in seconds (100 ms)

// Global Variables
float totalEnergy = 0.0; // Accumulated energy in joules
float caloriesBurned = 0;
float Winkel_neu, Winkel_alt;     // Current and previous inclination angles
float gz_alt, gz_neu;             // Current and previous angular velocities
float time_previous, time_current;         // Current and previous time
float t_Start;          // Start time of a rotation
float t_End;           // End time of a rotation
float t_FUllROTATION;                // Time for a full rotation
float v;                          // Average pedal speed
float r=0.175;                          // Crank length
int n;                            // Number of force measurements during one rotation
float F;                          // Average force during a rotation
float k=292;                          // Calibration factor specifying which numerical value [0, 1023] corresponds to which force in Newtons
int offset;                       // Offset of the input voltage without any load, i.e., when F = 0
float P;                          // Average power P
 


void setup()
   {
    // Initialize serial communication
    Serial.begin(115200);

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
  //DHT
  dht.begin();
  //BMP
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    // =====================================================================================
    // Read raw accelerometer/gyro measurements from the device
    // Accelerometer output values: Resolution 2g: 16384/g, Resolution 4g: 8192/g
    // Gyroscope output values: Resolution 250°/s: 131/°/s; 500°/s: 65.5/°/s; 2000°/s: 16.375/°/s
    // =====================================================================================
    //HX11
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    if (!scale.is_ready()) {
    Serial.println("HX711 not ready. Check wiring.");
    delay(500);
    return;
    }
     //MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
	  mpu.setGyroRange(MPU6050_RANGE_500_DEG);// set gyro range to +- 500 deg/s
	  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);// set filter bandwidth to 21 Hz
    // Verify connection
    // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");  
    time_previous = 0.0;
    Winkel_alt = 0.0;
    gz_alt = 0.0;
   }



void loop(){
    //Temperature and Humidity
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
  

   //HX11 AND MPU6050
   float reading = scale.get_units(10); // Average 10 readings for better stability
    n = 0;  // Reset the counter variable to 0
    F = 0;  // Reset the average force to 0 
    t_Start = micros();     // Determine the start time of a rotation
    do {
         sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        gz_neu = -g.gyro.z;   // New angular velocity
    
        // Calculate the new angle using the old angle and the average of the old and new angular velocities:
        // ==============================================================================================================
    
        time_current = micros();    // Current runtime of the program in µs
    
        Winkel_neu = Winkel_alt + 0.5 * ((gz_alt + gz_neu) / 65.5) * ((time_current - time_previous) / 1000000.0);
        
        if (Winkel_neu < 0)   // Restart at negative angle
           {
            Winkel_neu = 0.0;
            t_Start = micros();
            n = 0;  // Reset counter variable
            F = 0;  // Reset the average force
           }
    
        F = F + k * (5.0 * (reading) / 1023.0);  // Integrate all forces acting during one rotation; Factor k = Newton/Volt!

        gz_alt = gz_neu;
        time_previous = time_current;
        Winkel_alt = Winkel_neu;
        n = n + 1;               // Increment the counter variable
        
       }
    while (Winkel_neu < 360);    // Exit the loop when the angle > 360°, i.e., after a full rotation
    // ============================
    // == Complete Rotation ==    
    // ============================
    t_End = micros();        // Determine the end time of a rotation
    Winkel_alt = Winkel_alt - 360.0;    // Reduce the angle to the interval [0, 360]
    F = F / n;                          // Calculate the average force during one rotation
    t_FUllROTATION = (t_End - t_Start) / 1000000.0;    // Calculate the rotation time in seconds
    v = 2.0 * r * 3.141592654 / t_FUllROTATION;    // Calculate the average pedal speed
    P = 2.0 * F * v;                            // Calculate the average power P
    int RPM = 60.0 / t_FUllROTATION;
    float energy = P * TIME_STEP; // Joules
    totalEnergy += energy;
    float caloriesBurned = METABOLIC_EFFICIENCY * (totalEnergy / 4184.0);
    // Print data for debugging
    Serial.print("RPM: ");
    Serial.print(RPM);
    Serial.print(" Power: ");
    Serial.print(P);
    Serial.print("W Calories Burned: ");
    Serial.println(caloriesBurned);

   
   
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
    client.println("<p><b>TEMPERATURE AND HUMIDITY Data:</b></p>");
    client.println("<p>" + String(h) + "</p>");
    client.println("<p>" + String(t) + "</p>");
    client.println("<p><b>PRESSURE AND ALTITUDE Data:</b></p>");
    client.println("<p>" + String(p) + "</p>"); 
    client.println("<p>" + String(alt) + "</p>");
    client.println("<p><b>IMU DATA:</b></p>");
    client.println("<p>" + String(RPM)+ "</p>");
    client.println("<p>" + String(P) + "</p>");
    client.println("<p>" + String(caloriesBurned) + "</p>");
    client.println("<p><b>FORCE APPLIED ON PEDAL:</b></p>");
    client.println("<p>Current Scale Reading: " + String(reading) + " grams</p>");
    

    // End of HTML document
    client.println("</body>");
    client.println("</html>");
    // Close the connection
    client.stop();
  }
}  



