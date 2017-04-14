// *************************************************START************************************************
// IBTIKAR GPS SELF DRIVING SYSTEM
// THIS SYSTEM IS BASICALLY WORKING ON SENSING FROM GPS AND
// GOING FOR A ROUTE (COMPSED OF ARRAY OF WAYPOINTS).
// PLUS, ADJUSTING READS BASED ON ARTIFICIAL INTELLIGENCE LOCALIZATION SYSTEM.
//==============================================================================
//============       CONNECTIONS ON ARDUINO MEGA 2560      =====================
//==============================================================================
//============ MODEULE  |   WIRE_COLOR    |    PIN NUMBER  =====================
//============ ------------------------------------------- =====================
//============  POWER   |     RED         |       3.3v     =====================
//============  POWER   |    BLACK        |       GND      =====================
//============  GPS     |    YELLOW       |       PIN8     =====================
//============  GPS     |    GREEN        |       PIN9     =====================
//============ COMPASS  |    WHITE        |      SCL21     =====================
//============ COMPASS  |    BLUE         |      SDA20     =====================
//============  SERVO   |    BLUE(1)      |       PIN5     =====================
//============  SERVO   |    move(2)      |       PIN6     =====================
//==============================================================================
// **************************************Libraries***********************************
#include <Wire.h> //I2C Arduino Library
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <math.h>
#include "waypoint.h"
// **************************************Global Constants***********************************
#define RX 9
#define TX 8
#define RxD 51
#define TxD 52
#define HEADING_TOLERANCE 6 // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading
#define DEBUG YES // enable/disable debugging using serial port (displays diagnostic information)
// Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/ 
// Cedar Park, TX: Magnetic declination: 4° 11' EAST (POSITIVE);  1 degreee = 0.0174532925 radians
/*
Jeddah Makkah al Mukarramah
Latitude: 21° 31' 1" N
Longitude: 39° 13' 9" E
Magnetic declination: +3° 26' 
Declination is POSITIVE (EAST)
Inclination: 30° 56' 
Magnetic field strength: 40583.0 nT
*/
#define DEC_ANGLE 0.0524
#define D 5   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 5  // enter the numebr of way points here (will run from 0 to (n-1))
#define TURN_LEFT 1
#define TURN_RIGHT -1
#define TURN_STRAIGHT 0
#define WAYPOINT_DIST_TOLERANE 3

#define stop 0x0000
#define one 0x01FA
#define two 0x0279
#define three 0x02FA
#define four 0x037C
#define five 0x03FF

#define FAST_SPEED five
#define NORMAL_SPEED three
#define TURN_SPEED two
#define SLOW_SPEED one

#define RPWM_OUTPUT 5
#define LPWM_OUTPUT 6
// ***********************************Global STRUCTS & ENUMS********************************
enum directions {left = TURN_LEFT, right = TURN_RIGHT, straight = TURN_STRAIGHT};
// **************************************Global Variables***********************************
SoftwareSerial gpsSerial(RX, TX); // RX(9), TX(8)
SoftwareSerial blueToothSerial(RxD,TxD);
TinyGPS gps;
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);
sensors_event_t compass_event;
int targetHeading; // where we want to go to reach current waypoint
int currentHeading; // where we are actually facing now
int headingError; // signed (+/-) difference between targetHeading and currentHeading
float currentLat,
      currentLong,
      oldLat,
      oldLong,
      targetLat,
      targetLong;
unsigned long age;
bool newGpsData = false;
int distanceToTarget,            // current distance to target (current waypoint)
    originalDistanceToTarget;    // distance to original waypoing when we started navigating to it
int waypointNumber = -1;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
waypoint waypointList[NUMBER_WAYPOINTS] = { waypoint(30.508302, -97.832624),
                                            waypoint(30.508085, -97.832494),
                                            waypoint(30.507715, -97.832357),
                                            waypoint(30.508422, -97.832760),
                                            waypoint(30.508518, -97.832665) };
directions turnDirection = straight;
int speed = NORMAL_SPEED;
int direction_m = 0;
int last_m = 0;
bool blu_read_done = false;
// **************************************Global Functions***********************************
void setupSerial(void);
void setupCompass(void);
void loopForever(void);
int readCompass(void);
float deg2rad(float deg);
void setupGPS(void);
float getDistanceFromLatLonInmeters(float lat1, float lon1, float lat2, float lon2);
bool hasGpsUpdate();
void processGPS(void);
int distanceToWaypoint();
int courseToWaypoint();
void calcDesiredTurn(void);
void moveBoat(void);
void setupPin11PWM(void);
void initMotorConfigs(void);
void setMotorDirection(directions d);
void setupBluetooth(void);
// **************************************Arduino Functions**********************************
void setup() {
  setupSerial();
  setupPin11PWM();
  setupCompass();
  setupGPS();
  delay(1000);
  Serial.println("Waiting gps");
  while(!hasGpsUpdate());
  setupBluetooth();
  // set motor to normal speed and straight direction
  initMotorConfigs();
}

void loop() {
  if(blu_read_done) {
    // 1- read gps points
    if(hasGpsUpdate()){
      oldLat = currentLat;
      oldLong = currentLong;
      gps.f_get_position(&currentLat, &currentLong, &age); 
      if(getDistanceFromLatLonInmeters(oldLat, oldLong, currentLat, currentLong) > WAYPOINT_DIST_TOLERANE){
         // process gps
         processGPS();
      }
    }
    // navigate
    currentHeading = readCompass();    // get our current heading
    calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles
    // move
    moveBoat(); // move ya man.
  } else {
    if(blueToothSerial.available()){
      delay(1000);
      int numOfReads = NUMBER_WAYPOINTS;
      float pLong, pLat;
      while(numOfReads--){
        pLong = blueToothSerial.parseFloat();
        blueToothSerial.read();
        pLat = blueToothSerial.parseFloat();
        blueToothSerial.read();
        waypointList[NUMBER_WAYPOINTS-numOfReads-1] = waypoint(pLong, pLat);
      }
      blu_read_done = true;
      digitalWrite(13, LOW);
      // get initial waypoint; also sets the distanceToTarget and courseToTarget varilables
      nextWaypoint();
    } else {
      delay(1000);
    }
  }
}

// *****************************************Functions Implementation***********************************
void setupSerial(void)
{
  #ifdef DEBUG
    Serial.begin(115200);
  #endif
}

void setupCompass(void)
{
  if(!compass.begin())
    {
      #ifdef DEBUG
        Serial.println(F("COMPASS ERROR"));
      #endif
      // set some led to turn on if error.
      // digitalWrite(compassLED, HIGH);
      loopForever();         // loop forever, can't operate without compass
    }
}

int readCompass(void)
{
  compass.getEvent(&compass_event);    
  float heading = atan2(compass_event.magnetic.y, compass_event.magnetic.x);
  
  heading += DEC_ANGLE;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  return ((int)headingDegrees); 
}  // readCompass()

void setupGPS(void)
{
   gpsSerial.begin(9600);
}

bool hasGpsUpdate()
{
  newGpsData = false;
  while (gpsSerial.available()) {
   char c = gpsSerial.read();
   // Serial.print(c); // uncomment to see raw GPS data
   if (gps.encode(c)) {
     newGpsData = true;
     break; // uncomment to print new data immediately!
   }
  }
  return newGpsData;
}

float getDistanceFromLatLonInmeters(float lat1, float lon1, float lat2, float lon2) {
  int R = 6371; // Radius of the earth in km
  float dLat = deg2rad(lat2-lat1);  // deg2rad below
  float dLon = deg2rad(lon2-lon1); 
  float a = 
    sin(dLat/2) * sin(dLat/2) +
    cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * 
    sin(dLon/2) * sin(dLon/2)
    ; 
  float c = 2 * atan2(sqrt(a), sqrt(1-a)); 
  float d = R * c; // Distance in km
  return d*1000.0; // return in meters
}

float deg2rad(float deg) {
  return deg * (M_PI/180);
}

void processGPS(void)
{
  // update the course and distance to waypoint based on our new position
  distanceToWaypoint();
  courseToWaypoint();  
}
// returns distance in meters between two positions, both specified 
// as signed decimal-degrees latitude and longitude. Uses great-circle 
// distance computation for hypothetical sphere of radius 6372795 meters.
// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
// copied from TinyGPS library
int distanceToWaypoint() 
{
  float delta = radians(currentLong - targetLong);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  float lat1 = radians(currentLat);
  float lat2 = radians(targetLat);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  distanceToTarget =  delta * 6372795; 
   
  // check to see if we have reached the current waypoint
  if (distanceToTarget <= WAYPOINT_DIST_TOLERANE)
    nextWaypoint();
    
  return distanceToTarget;
}  // distanceToWaypoint()

// returns course in degrees (North=0, West=270) from position 1 to position 2,
// both specified as signed decimal-degrees latitude and longitude.
// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
// copied from TinyGPS library
int courseToWaypoint()
{
  float dlon = radians(targetLong-currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  targetHeading = degrees(a2);
  return targetHeading;
}   // courseToWaypoint()

void nextWaypoint(void)
{
  waypointNumber++;
  targetLat = waypointList[waypointNumber].getLat();
  targetLong = waypointList[waypointNumber].getLong();
  
  if ((targetLat == 0 && targetLong == 0) || waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached? 
    {
      //stop the motor
      OCR1A = stop;
      loopForever();
    }
    
   processGPS();
   distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
   courseToWaypoint();
   
}  // nextWaypoint()

void calcDesiredTurn(void)
{
    // calculate where we need to turn to head to destination
    headingError = targetHeading - currentHeading;
    
    // adjust for compass wrap
    if (headingError < -180)
      headingError += 360;
    if (headingError > 180)
      headingError -= 360;
  
    // calculate which way to turn to intercept the targetHeading
    if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
      turnDirection = straight;
    else if (headingError < 0)
      turnDirection = left;
    else if (headingError > 0)
      turnDirection = right;
    else
      turnDirection = straight;
 
}

void moveBoat(void)
{
  // SET SPEED BASED ON DIRECTION
  if (turnDirection == straight) {
    speed = FAST_SPEED;
    OCR1A = FAST_SPEED;
  } else {
    speed = TURN_SPEED;
    OCR1A = TURN_SPEED;
  }
  setMotorDirection(turnDirection);
 return;
}

void setupPin11PWM(void)
{
  DDRB |= (1 << DDB5) | (1 << DDB6); // thisi is 11, 12 arduino mega
  TCCR1A =
      (1 << COM1A1) | (1 << COM1B1) |
      // Fast PWM mode.
      (1 << WGM11);
  TCCR1B =
      // Fast PWM mode.
      (1 << WGM12) | (1 << WGM13) |
      // No clock prescaling (fastest possible
      // freq).
      (1 << CS10);
  OCR1A = 0x0000;
  // Set the counter value that corresponds to
  // full duty cycle. For 15-bit PWM use
  // 0x7fff, etc. A lower value for ICR1 will
  // allow a faster PWM frequency.
  ICR1 = 0x03ff; // 1023 - 10 bit value
  OCR1A = 0x0000;
  delay(1000);
}

void initMotorConfigs(void)
{
  // set speed to normal speed
  OCR1A = NORMAL_SPEED;
  // Localize motor to straight direction
  pinMode(RPWM_OUTPUT, OUTPUT);
  pinMode(LPWM_OUTPUT, OUTPUT);
  delay(60);
  analogWrite(LPWM_OUTPUT, 0);
  analogWrite(RPWM_OUTPUT, 255);
  delay(1250);
  analogWrite(LPWM_OUTPUT, 255);
  analogWrite(RPWM_OUTPUT, 0);
  delay(1250);
  analogWrite(LPWM_OUTPUT, 255);
  analogWrite(RPWM_OUTPUT, 0);
  delay(625);
  analogWrite(LPWM_OUTPUT, 0);
  analogWrite(RPWM_OUTPUT, 255);
  delay(625);
  analogWrite(LPWM_OUTPUT, 0);
  analogWrite(RPWM_OUTPUT, 0);
}

void setMotorDirection(directions d)
{
  direction_m = d;
  if(direction_m == straight)
    {
      if(last_m == left){
        analogWrite(LPWM_OUTPUT, 0);
        analogWrite(RPWM_OUTPUT, 255);
        delay(625);
        analogWrite(LPWM_OUTPUT, 0);
        analogWrite(RPWM_OUTPUT, 0);
      } else if(last_m == right) {
        analogWrite(LPWM_OUTPUT, 255);
        analogWrite(RPWM_OUTPUT, 0);
        delay(625);
        analogWrite(LPWM_OUTPUT, 0);
        analogWrite(RPWM_OUTPUT, 0);
      }
      last_m = direction_m;
    }
    else if(direction_m == right){
      if(last_m == left){
        analogWrite(LPWM_OUTPUT, 0);
        analogWrite(RPWM_OUTPUT, 255);
        delay(1250);
        analogWrite(LPWM_OUTPUT, 0);
        analogWrite(RPWM_OUTPUT, 0);
      } else if(last_m == straight) {
        analogWrite(LPWM_OUTPUT, 0);
        analogWrite(RPWM_OUTPUT, 255);
        delay(625);
        analogWrite(LPWM_OUTPUT, 0);
        analogWrite(RPWM_OUTPUT, 0);
      }
      last_m = direction_m;
    }
    else if(direction_m == left){
      if(last_m == right){
        analogWrite(LPWM_OUTPUT, 255);
        analogWrite(RPWM_OUTPUT, 0);
        delay(1250);
        analogWrite(LPWM_OUTPUT, 0);
        analogWrite(RPWM_OUTPUT, 0);
      } else if(last_m == straight) {
        analogWrite(LPWM_OUTPUT, 255);
        analogWrite(RPWM_OUTPUT, 0);
        delay(625);
        analogWrite(LPWM_OUTPUT, 0);
        analogWrite(RPWM_OUTPUT, 0);
      }
      last_m = direction_m;
    }
}

void setupBluetooth(void)
{
   pinMode(RxD, INPUT);
   pinMode(TxD, OUTPUT); 
   pinMode(13,OUTPUT);
   digitalWrite(13, LOW);
   blueToothSerial.begin(9600); //Set BluetoothBee BaudRate to default baud rate 38400
   blueToothSerial.print("\r\n+STWMOD=0\r\n"); //set the bluetooth work in slave mode
   blueToothSerial.print("\r\n+STNA=SeeedBTSlave\r\n"); //set the bluetooth name as "SeeedBTSlave"
   blueToothSerial.print("\r\n+STPIN=0000\r\n");//Set SLAVE pincode"0000"
   blueToothSerial.print("\r\n+STOAUT=1\r\n"); // Permit Paired device to connect me
   blueToothSerial.print("\r\n+STAUTO=0\r\n"); // Autoconnection should be forbidden here
   delay(2000); // This delay is required.
   blueToothSerial.print("\r\n+INQ=1\r\n"); //make the slave bluetooth inquirable
//   Serial.println("The slave bluetooth is inquirable!");
   delay(2000); // This delay is required.
   blueToothSerial.flush();
   #ifdef DEBUG
    Serial.println("Done");
  #endif
}

void loopForever(void)
{
  while(1);
}
// *************************************************END************************************************
