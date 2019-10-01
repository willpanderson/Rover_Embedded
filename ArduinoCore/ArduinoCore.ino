
/******************************************************************************************************************************************************/
//                                --[[ HARDWARE SETUP ]]--                                  //
/******************************************************************************************************************************************************/

//                               --[[ Helpful Info Sections ]]--

//                          --[[ Serial Communication Info (I2C Protocol) ]]--
/*        I2C PINS
Board     SDA   SCL
Arduino UNO   A4    A5
Arduino MEGA  20    21
Raspberry Pi  0   1

** To transfer data, we need the component address and at least 1 byte of data. **

ex. Master Write (Data Send) code
  {
  -[ Setup ]-
  Wire.begin ();

  -[ Main Code ]-
  Wire.beginTransmission ([Addr]);
  Wire.write (Buffer, #);       // Here, # is the number of bytes to write
  Wire.endTransmission ();
  }

ex. Slave Read (Receive Data) code
  {

  }

ex. Master Read (Data Request) code
  {
  -[ Setup ]-
  Wire.begin ();  // address is optional for the Master device

  -[ Main Code ]-
  Wire.requestFrom ([Address], #);  // Here, # is the number of bytes requested
  if (Wire.available ())
    {
    Var1 = Wire.read ();
    Var2 = Wire.read ();
    etc...
    }
  }

ex. Slave Write (Data Send) code
  {
  -[ Setup ]-
  Wire.begin ([Address]);   // Here, Slave address is mandatory so that the slave knows when it's being called.
  Wire.onRequest (SendData);  // SendData is your function. No, you do not put (). This is because onRequest () directs a pointer
  //    to the memory address of your function. CSE 2325 explains why in more detail (the system stack).

  -[ Main Code ]-
  byte Buffer [#];  // Again, the total number of bytes you intend to send.

  Buffer [0] = Var1;
  Buffer [1] = Var2;
  Buffer [n] = etc...;

  Wire.write (Buffer, #);   // Wire.write () was a little finicky with multivariable communication and so sending multiple vars
  //    through a single communication sequence seemed nigh impossible. The best way I found to solve
  //    that problem was to stuff the vars into a single array, which was both easy to implement as
  //    well as scalable to any number of variables.
  }
*/
//                          --[[ Serial Communication Info (SPI Protocol) ]]--
/*
ICSP  PINS  (Arduino)
MISO  1 2  +VCC
SCLK  3 4  MOSI
RST   5 6  GND
SS: GPIO 53 (MEGA 2560) || GPIO 10 (UNO) **CAUTION** [5V] (Raspi is ~3.3V logic)
  ** PinMode (SS, OUTPUT) LOW to communicate with RASPI**

-[ Useful SPI Details ]-
  MaxSPISpeed?  - The chip uses the max speed it can, so programming a higher value allows the code to adapt to newer chips later.
  MSB or LSB?   - Is the Most Significant Bit or Least Significant Bit shifted first?
  Clock Polarity  - Is the clock idle when HIGH or when LOW
  Clock Phase   - Does the data land on the rising edge or falling edge of the clock pulse?

-[ SPI Mode Table ]-
  Clock   Clock   Output      Data
  Mode    Polarity  Phase  Edge   Capture

  SPI_MODE0   0     0   Falling   Rising
  SPI_MODE1   0     1   Rising    Falling
  SPI_MODE2   1     0   Rising    Falling
  SPI_MODE3   1     1   Falling   Rising
*/

#include "SoftwareSerial.h"
    //                                --[[ NETWORKS SETUP ]]--

    //                                 --[[ Server Comms ]]--
#include "global_defs.h"
#include "Packet.h"

#define ControllerUpperLim 254
#define ControllerLowerLim 0
#define ReportFrequency 100		// in milliseconds

Global_Definitions Command;
size_t ReportTimer;
size_t CommCheckTimer;
size_t Ctr;
static byte ReadBuffer [64];	// The Arduino only has a 64 byte buffer
static byte ReportBuffer [64];

//                                     --[[ I2C ]]--
#include <Wire.h>

#define Reset "000000000" // The reset command is 9 "0"s in a row.

//  SDA/SCL Pin Setting
#if defined (__AVR_ATmega2560__)  // If the board is an Arduino MEGA
  #define SDA 20
  #define SCL 21
#elif defined (__AVR_ATmega328P__)  // If the board is an Arduino UNO (I test at home on an UNO.)
  #define SDA A4
  #define SCL A5
#endif

#define SRAddr 0x08   // Sample Return Address
#define ArmAddr 0x09  // Arm & Manipulation Address


#define SRVars 3			// This is the number of bytes to be sent via I2C later. Typically every var we send will be 1 byte.
  byte  SRData [SRVars];	// Elements, in order, represent:
							//    [0] Function, [1] Command, [2] Sub-Function

#define ArmTgtBytes 12			// This is the number of bytes to be sent via I2C later. Every floating point we send will be 4 bytes.
#define ArmVars 24				// 6 motors * 4 bytes per float = 24 bytes for precise sensor data.
  byte  ArmTgt [ArmTgtBytes];	// Elements, in order, represent: X: {[0],[1],[2],[3]}, Y: {[4],[5],[6],[7]}, Z: {[8],[9],[10],[11]}
  byte	ArmPos [ArmVars];


//  bool  SRAttached  = false;  // Is Sample Return's module attached?
//  bool  ArmAttached = false;  // Is the Arm attached?

//                              --[[ Drive Motor setup ]]--
#define TANK_MODE 0
#define MIXED_MODE 1
#define MOTOR_DRIVER_MODE TANK_MODE

#define LeftMotorPin 11		//	The motors must be on pins 11 and 12 because those are on internal timer 1; Timer 1 has its PWM frequency
#define RightMotorPin 12	//		adjusted to prevent the motors from having creep and whining issues.
#define MotorTogglePin 22
#define FullForward 255
#define FullStop 127
#define FullReverse 0
#define MotorDelay 20		// Time in milliseconds
#define LeftMotorCalibration 7
#define RightMotorCalibration 8
#define Left 0
#define Right 1

size_t MotorDelayTimer = 0;
int TurnValue = FullStop;
int DriveValue = FullStop;
int MotorState [] = {FullStop, FullStop};
int CurrMotorTgt [] = {FullStop, FullStop};

//                               --[[ GPS & IMU setup ]]--

#include "Adafruit_GPS.h"

#define IMU_Addr 0x20
#define FloatSize 4

#define RollCalibration -2.31	// These numbers were determined experimentally from the MyAHRSplus_Calibration program on Google Drive
#define PitchCalibration -2.45	//    and they need to be floating point numbers.
#define YawCalibration 25.10	// Yaw will be calibrated to GPS heading at North = 0 degrees, value will be taken via IMU Magnetometer

#ifndef FifteenPercent
	#define FifteenPercent 38
#endif

enum ImuAddrMap {
	I2C_SLAVE_REG_WHO_AM_I = 0x01,
	I2C_SLAVE_REG_REV_ID_MAJOR,
	I2C_SLAVE_REG_REV_ID_MINOR,
	I2C_SLAVE_REG_STATUS,

// RAW DATA
	I2C_SLAVE_REG_I_ACC_X_LOW = 0x10,
	I2C_SLAVE_REG_I_ACC_X_HIGH,
	I2C_SLAVE_REG_I_ACC_Y_LOW,
	I2C_SLAVE_REG_I_ACC_Y_HIGH,
	I2C_SLAVE_REG_I_ACC_Z_LOW,
	I2C_SLAVE_REG_I_ACC_Z_HIGH,
	I2C_SLAVE_REG_I_GYRO_X_LOW,
	I2C_SLAVE_REG_I_GYRO_X_HIGH,
	I2C_SLAVE_REG_I_GYRO_Y_LOW,
	I2C_SLAVE_REG_I_GYRO_Y_HIGH,
	I2C_SLAVE_REG_I_GYRO_Z_LOW,
	I2C_SLAVE_REG_I_GYRO_Z_HIGH,
	I2C_SLAVE_REG_I_MAGNET_X_LOW,
	I2C_SLAVE_REG_I_MAGNET_X_HIGH,
	I2C_SLAVE_REG_I_MAGNET_Y_LOW,
	I2C_SLAVE_REG_I_MAGNET_Y_HIGH,
	I2C_SLAVE_REG_I_MAGNET_Z_LOW,
	I2C_SLAVE_REG_I_MAGNET_Z_HIGH,

// COMPENSATED DATA
	I2C_SLAVE_REG_C_ACC_X_LOW,
	I2C_SLAVE_REG_C_ACC_X_HIGH,
	I2C_SLAVE_REG_C_ACC_Y_LOW,
	I2C_SLAVE_REG_C_ACC_Y_HIGH,
	I2C_SLAVE_REG_C_ACC_Z_LOW,
	I2C_SLAVE_REG_C_ACC_Z_HIGH,
	I2C_SLAVE_REG_C_GYRO_X_LOW,
	I2C_SLAVE_REG_C_GYRO_X_HIGH,
	I2C_SLAVE_REG_C_GYRO_Y_LOW,
	I2C_SLAVE_REG_C_GYRO_Y_HIGH,
	I2C_SLAVE_REG_C_GYRO_Z_LOW,
	I2C_SLAVE_REG_C_GYRO_Z_HIGH,
	I2C_SLAVE_REG_C_MAGNET_X_LOW,
	I2C_SLAVE_REG_C_MAGNET_X_HIGH,
	I2C_SLAVE_REG_C_MAGNET_Y_LOW,
	I2C_SLAVE_REG_C_MAGNET_Y_HIGH,
	I2C_SLAVE_REG_C_MAGNET_Z_LOW,
	I2C_SLAVE_REG_C_MAGNET_Z_HIGH,
	I2C_SLAVE_REG_C_TEMPERATURE_LOW,
	I2C_SLAVE_REG_C_TEMPERATURE_HIGH,

// Attitude - Euler angle
	I2C_SLAVE_REG_ROLL_LOW,
	I2C_SLAVE_REG_ROLL_HIGH,
	I2C_SLAVE_REG_PITCH_LOW,
	I2C_SLAVE_REG_PITCH_HIGH,
	I2C_SLAVE_REG_YAW_LOW,
	I2C_SLAVE_REG_YAW_HIGH,

// Attitude - Quaternion
	I2C_SLAVE_REG_QUATERNIAN_X_LOW,
	I2C_SLAVE_REG_QUATERNIAN_X_HIGH,
	I2C_SLAVE_REG_QUATERNIAN_Y_LOW,
	I2C_SLAVE_REG_QUATERNIAN_Y_HIGH,
	I2C_SLAVE_REG_QUATERNIAN_Z_LOW,
	I2C_SLAVE_REG_QUATERNIAN_Z_HIGH,
	I2C_SLAVE_REG_QUATERNIAN_W_LOW,
	I2C_SLAVE_REG_QUATERNIAN_W_HIGH,
	};

#if defined (__AVR_ATmega2560__)			// If the board is an Arduino MEGA
	Adafruit_GPS  Gps (& Serial1);			// GPS TX -> Mega Pin 19, GPS RX -> Mega Pin 18
#elif defined (__AVR_ATmega328P__)			// If the board is an Arduino UNO (I test at home on an UNO.)
	SoftwareSerial  GPSReadChannel (3, 2);	// First param is the Arduino TX pin, Second param is the Arduino RX pin.
	Adafruit_GPS  Gps (& GPSReadChannel);
#endif

#define GPSECHO false // True to write to Serial monitor and debug, false in normal operation
	bool		usingInterrupt = false;
	void		useInterrupt (boolean);		// This function prototype is here to keep the Arduino happy

	float		RollAngle;
	float		PitchAngle;
	float		YawAngle;
	size_t		GpsTimer;
	char		c;


//                               --[[ Fan Speed setup ]]--

//                              --[[ Voltage Bus setup ]]--

/******************************************************************************************************************************************************/
//                               --[[ FUNCTION DEFINITIONS ]]--                               //
//                 Functions must be declared up before setup() and loop() because of C language issues                 //
/******************************************************************************************************************************************************/

/*
void GetSRData ()
  {
  Wire.requestFrom (SRAddr, SRVars);
  if (Wire.available ())
    for (int i = 0; i < SRVars; i++)
      SRData [i] = Wire.read ();
  }
*/

void GetArmData ()
	{
	Wire.requestFrom (ArmAddr, ArmVars);
	if (Wire.available ())
		for (int i = 0; i < ArmVars; i++)
			ArmPos [i] = Wire.read ();
	}

void PrepReport ()
	{
    int j;
    byte * FloatBuf;

    Ctr = 0;
    ReportBuffer [Ctr++] = SYS_INFO;
/*
    FloatBuf = (byte *) & CurrentLocation.LatitudeDegrees;
    for (j = 0; j < FloatSize; j++)
      ReportBuffer [Ctr++] = FloatBuf [j];
    FloatBuf = (byte *) & CurrentLocation.LongitudeDegrees;
    for (j = 0; j < FloatSize; j++)
      ReportBuffer [Ctr++] = FloatBuf [j];
    FloatBuf = (byte *) & RollAngle;
    for (j = 0; j < FloatSize; j++)
      ReportBuffer [Ctr++] = FloatBuf [j];
    FloatBuf = (byte *) & PitchAngle;
    for (j = 0; j < FloatSize; j++)
      ReportBuffer [Ctr++] = FloatBuf [j];
    FloatBuf = (byte *) & YawAngle;
    for (j = 0; j < FloatSize; j++)
      ReportBuffer [Ctr++] = FloatBuf [j];
//  ReportBuffer [Ctr++] = TemperatureVal;
*/
	for (j = 0; j < ArmVars; j++)
		ReportBuffer [Ctr++] = ArmPos [j];
//	for (j = 0; j < SRVars; j++)
//		ReportBuffer [Ctr++] = SRData [j];
	}


/*float ByteToFloat (byte * Input, int & Pos)		// This function is mostly used to convert gps waypoints to useful float data.
	{
	float Temp = 0;

	for (int i = 0; i < FloatSize; i++)
		Temp [i] = Input [Pos++];

	return Temp;
	}


bool ParseComms ()
	{
    int PayloadSize = 0;
    if (LookForPacket (ReadBuffer) == 0) // If this channel has no data coming in...
        return false; //  ...don't waste our time processing.

    Command = (Global_Definitions) ReadBuffer [0];
    switch (Command)
		{
		case DRIVE:
			digitalWrite (MotorTogglePin, HIGH);	// Enable the motors
		    DriveValue = ReadBuffer [1];
		    TurnValue = ReadBuffer [2];
		    SetMotorValueTargets ();
		    break;
		case SAMPLE_RETURN:
//			if (SRAttached)
//				{
//				for (int i = 0; i < SRVars; i++)
//				  SRData [i] = ReadBuffer [i + 1];
//				Wire.beginTransmission (SRAddr);
//				Wire.write (SRData, SRVars);
//				Wire.endTransmission ();
//				}
		    break;
		case ARM:
			{
			for (int i = 0; i < ArmTgtBytes; i++)
				ArmTgt [i] = ReadBuffer [i + 1]; // i + 1 because the first element of the command buffer is the ARM command
			Wire.beginTransmission (ArmAddr);
			Wire.write (ArmTgt, ArmTgtBytes);
			Wire.endTransmission ();
			}
		    break;
		case RESET:
		    Wire.write (Reset);
		    break;
		case COMM_PING:
		    byte Temp [1];
		    Temp [0] = COMM_PING;
		    SendPacket (1, Temp);
		    break;
		defualt:
			return false;
		}
	CommCheckTimer = millis ();
    return true;
	}

//	Temperature Control Functions (Not currently implemented in hardware)
/*
//void TemperatureRead (size_t RawADC)
//  {
//  TemperatureVal = map (RawADC, 0, 1024, -55.0, 150.0);
//  Serial.println (TemperatureVal);
//  TemperatureVal = (TemperatureVal - 500) / 10; // Temperature in *C
//  TemperatureVal = 1.8 * TemperatureVal + 32;   // *C -> *F
//  }

//void ControlTemperature ()
//  {
//  TemperatureRead (analogRead (TemperaturePin));
//  Serial.print ("*** TEMPERATURE:\t"); Serial.println (TemperatureVal);
//
//  if (TemperatureVal <= LowerTemperatureLimit)    // Decide our fan's duty cycle
//      DutyCycle = 0;
//    else if (TemperatureVal >= UpperTemperatureLimit)
//        DutyCycle = OneHundredPercent;
//      else DutyCycle = map (TemperatureVal, LowerTemperatureLimit, UpperTemperatureLimit, FifteenPercent, OneHundredPercent);
//
//  analogWrite (FanPin, DutyCycle);
//  }
*/

//  GPS & IMU Functions

SIGNAL (TIMER0_COMPA_vect)
	{
	char c = Gps.read ();	// if you want to debug, this is a good time to do it!
#ifdef UDR0
	if (GPSECHO)			// writing direct to UDR0 is much much faster than Serial.print
		if (c) UDR0 = c;	// but only one character can be written at a time.
#endif
	}

void useInterrupt (bool v)
	{
	if (v)
			{
			OCR0A = 0xAF;			// Timer0 is already used for millis() - we'll just interrupt somewhere
			TIMSK0 |= _BV (OCIE0A);	// in the middle and call the "Compare A" function above
			usingInterrupt = true;
			}
		else
				{
				TIMSK0 &= ~_BV (OCIE0A);	// do not call the interrupt function COMPA anymore
				usingInterrupt = false;
				}
	}

/*bool IMURead (uint8_t reg_add, uint8_t * buff, uint8_t len)
	{
	Wire.beginTransmission ((uint8_t) IMU_Addr);
	Wire.write (reg_add);
	Wire.endTransmission (false);
	Wire.requestFrom ((uint8_t) IMU_Addr, len);

	uint8_t ctr = 0;
	while (Wire.available ())
		buff [ctr++] = Wire.read ();

	return (ctr == len);
	}

bool IMUWrite (uint8_t reg_add, uint8_t * buff , uint8_t len)
	{
	Wire.beginTransmission ((uint8_t) IMU_Addr);
	Wire.write (reg_add);

	for (uint8_t cnt=0; cnt < len; cnt++)
		Wire.write (buff [cnt]);

	return Wire.endTransmission (true) == 0;
	}

//  Read Accelerometer, Gyroscope, Magnetometer (Compensated)
void IMUReadCompensated ()
	{
//	Accelerometer units:	g-forces (The force of Earth's gravity)
//	Gyroscope units:		Degrees/Second
//	Magnetometer units:		uTesla
	uint8_t buf_comp_data [18];

	IMURead (I2C_SLAVE_REG_C_ACC_X_LOW, buf_comp_data, 18);

	int16_t acc_c_x = (buf_comp_data [1] << 8) | buf_comp_data [0];
	int16_t acc_c_y = (buf_comp_data [3] << 8) | buf_comp_data [2];
	int16_t acc_c_z = (buf_comp_data [5] << 8) | buf_comp_data [4];
	int16_t gyro_c_x = (buf_comp_data [7] << 8) | buf_comp_data [6];
	int16_t gyro_c_y = (buf_comp_data [9] << 8) | buf_comp_data [8];
	int16_t gyro_c_z = (buf_comp_data [11] << 8) | buf_comp_data [10];
	int16_t mag_c_x = (buf_comp_data [13] << 8) | buf_comp_data [12];
	int16_t mag_c_y = (buf_comp_data [15] << 8) | buf_comp_data [14];
	int16_t mag_c_z = (buf_comp_data [17] << 8) | buf_comp_data [16];

	float comp_acc_x = (float) acc_c_x * 16.0 / 32767;
	float comp_acc_y = (float) acc_c_y * 16.0 / 32767;
	float comp_acc_z = (float) acc_c_z * 16.0 / 32767;
	float comp_gyro_x = (float) gyro_c_x * 2000 / 32767;
	float comp_gyro_y = (float) gyro_c_y * 2000 / 32767;
	float comp_gyro_z = (float) gyro_c_z * 2000 / 32767;
	float comp_mag_x = (float) mag_c_x * 0.3;
	float comp_mag_y = (float) mag_c_y * 0.3;
	float comp_mag_z = (float) mag_c_z * 0.3;
	}

//	EULER ANGLE (Roll, Pitch, Yaw)
void IMUReadEulerAngle ()
	{
	uint8_t Buffer [6];

	IMURead (I2C_SLAVE_REG_ROLL_LOW, Buffer, 6);
	int16_t euler_x = (Buffer [1] << 8) | Buffer [0];	// The MyAHRS+ IMU is little endian, so we bit-shift the MSB byte up.
	int16_t euler_y = (Buffer [3] << 8) | Buffer [2];
	int16_t euler_z = (Buffer [5] << 8) | Buffer [4];

	RollAngle = ((float) euler_x * 180 / 32767) + RollCalibration;		// Left is negative, Right is positive
	PitchAngle = ((float) euler_y * 180 / 32767) + PitchCalibration;	// Nose DOWN is negative, Nose UP is positive (inverted for intuitiveness)
	YawAngle = ((float) euler_z * 180 / 32767) + YawCalibration;		// Left is negative, Right is positive
	}

//	Read Quaternion
//	The Quaternion data gives (X)i + (Y)j + (Z)k + (W)
//		In other words, a 3D vector function
int IMUReadQuaternion ()
	{
	uint8_t buf_quat [8];

	IMURead (I2C_SLAVE_REG_QUATERNIAN_X_LOW, buf_quat, 8);

	int16_t quat_x = (buf_quat [1] << 8) | buf_quat [0];
	int16_t quat_y = (buf_quat [3] << 8) | buf_quat [2];
	int16_t quat_z = (buf_quat [5] << 8) | buf_quat [4];
	int16_t quat_w = (buf_quat [7] << 8) | buf_quat [6];

	quaternion_x = (float) quat_x / 32767;
	quaternion_y = (float) quat_y / 32767;
	quaternion_z = (float) quat_z / 32767;
	quaternion_w = (float) quat_w / 32767;

	Serial.print ("#QUATERNION = [");
	Serial.print (quaternion_x);  Serial.print (", ");
	Serial.print (quaternion_y);  Serial.print (", ");
	Serial.print (quaternion_z);  Serial.print (", ");
	Serial.print (quaternion_w);  Serial.println ("]");
	Serial.println ();
	}

bool IMU_Init ()
	{
	uint8_t buf_whomi [1];
	uint8_t buf_stat [1];

	if (IMURead (I2C_SLAVE_REG_WHO_AM_I, buf_whomi, 1) != 0xB1)
		return false;
	if (IMURead (I2C_SLAVE_REG_STATUS, buf_stat, 1) != 0x80)
		return false;
	return true;
	}
*/

//  Drive Functions
void SetMotorValueTargets ()
	{
//#ifdef MOTOR_DRIVER_MODE == TANK_MODE
	if ((DriveValue + TurnValue - FullStop + LeftMotorCalibration) >= FullForward)
			CurrMotorTgt [Left] = FullForward;
		else if ((TurnValue + LeftMotorCalibration) <= FullReverse)
				CurrMotorTgt [Left] = FullReverse;
			else CurrMotorTgt [Left] = DriveValue + TurnValue - FullStop + LeftMotorCalibration;
	if ((DriveValue - TurnValue + FullStop + RightMotorCalibration) <= FullReverse)
			CurrMotorTgt [Right] = FullReverse;
		else if ((DriveValue + (FullStop - TurnValue) + RightMotorCalibration) >= FullForward)
				CurrMotorTgt [Right] = FullForward;
			else CurrMotorTgt [Right] = DriveValue - TurnValue + FullStop + RightMotorCalibration;
//#else
    CurrMotorTgt [Left] = DriveValue;
    CurrMotorTgt [Right] = TurnValue;
//#endif
	}

/******************************************************************************************************************************************************/
//                                --[[ MAIN CODE ]]--                                   //
/******************************************************************************************************************************************************/

//                                  --[[ Setup ]]--
void setup ()
	{
	pinMode (MotorTogglePin, OUTPUT);
	digitalWrite (MotorTogglePin, LOW);
    Serial.begin (115200); // This is 115200 so that the GPS doesn't miss any data
    Serial.flush ();
	TCCR1B &= ~7;	//	These two lines manipulate the PWM registers for our motor pins
	TCCR1B |= 2;	//		It's now set to operate at 3900 Hz for the Sabertooth 2x25

    Wire.begin ();
    Wire.write (Reset);
	/*
    Gps.begin (9600); // This is 9600 so the gps can read its data, and the 115200 baud is for when the gps is writing data.
    Gps.sendCommand (PMTK_SET_NMEA_OUTPUT_RMCGGA);	// We want to account for altitude, so we use parameter: PMTK_SET_NMEA_OUTPUT_RMCGGA
    Gps.sendCommand (PMTK_SET_NMEA_UPDATE_1HZ);		// GPS will check for new location every second
    useInterrupt  (true);
    */

    //  while (!Gps.fix)  //  Wait until the GPS has a fix
    //    if (Gps.newNMEAreceived ())
    //      Gps.parse (Gps.lastNMEA ());

    analogWrite (LeftMotorPin, MotorState [Left]);
    analogWrite (RightMotorPin, MotorState [Right]);
    MotorDelayTimer /*= GpsTimer*/ = CommCheckTimer = ReportTimer = millis ();
}

//                                   --[[ Loop ]]--
void loop()
	{
//	Serial.print ("MotorState [Left] = "); Serial.println (MotorState [0]);
//	Serial.print ("CurrMotorTgt [Left] = "); Serial.println (CurrMotorTgt [0]);
//    if (ParseComms ());
//			Good, continue mission.
//		else if ((millis () - CommCheckTimer) >= 2000)	//	We can't talk with the server anymore.
				CurrMotorTgt [Left] = CurrMotorTgt [Right] = FullStop;

	GetArmData ();
//  GetSRData ();
//  ControlTemperature ();
    if ((millis () - ReportTimer) >= ReportFrequency)
		{
        PrepReport ();
		SendPacket (Ctr, ReportBuffer);
        ReportTimer = millis ();
		}

    if ((millis () - MotorDelayTimer) >= MotorDelay)
		{
        if (CurrMotorTgt [Left] > MotorState [Left])
			    analogWrite (LeftMotorPin, ++MotorState [Left]);
			else if (CurrMotorTgt [Left] < MotorState [Left])
				    analogWrite (LeftMotorPin, --MotorState [Left]);
        if (CurrMotorTgt [Right] > MotorState [Right])
			    analogWrite (RightMotorPin, ++MotorState [Right]);
			else if (CurrMotorTgt [Right] < MotorState [Right])
				    analogWrite (RightMotorPin, --MotorState [Right]);
		if ((MotorState [Left] == FullStop) && (MotorState [Right] == FullStop) && (CurrMotorTgt [Left] == FullStop) && (CurrMotorTgt [Right] == FullStop))
			digitalWrite (MotorTogglePin, LOW);	// If my motors are stopped, and I want them stopped, disable the motors.
		MotorDelayTimer = millis ();
		}

//	The rover may run for up to an hour. These checks are to make sure we're prepared for millis() to reach max value and flip over.
    if (CommCheckTimer > millis ())
        CommCheckTimer = millis ();
	if (ReportTimer > millis ())
		ReportTimer = millis ();
	if (MotorDelayTimer > millis ())
		MotorDelayTimer = millis ();
	}
