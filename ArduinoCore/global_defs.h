#ifndef GLOBAL_DEFS_H_
#define GLOBAL_DEFS_H_

enum Global_Definitions
	{
	/*
		send_buff[0] = DRV;
		send_buff[1] = 50;  //Backwards [0 <-> 254] Forwards - 127 = no motion
		send_buff[2] = 0; //Left [0 <-> 254] Right - 127 = no turning
	*/

	START			= 0xAA,

	/* Arduino Reporting Commands */
	SYS_INFO		= 0xAB,	// Sensor information.

	/* Heartbeat */
	COMM_PING		= 0xFA, // Just a comms check, can trigger a NOACK or REACK event
							//		Pi writes (COMM_PING), Arduino responds with (COMM_PING)

	/* Raspi Issuable Commands */
//	AUTOPILOT		= 0xBA,	// This command no longer used.
	DRIVE			= 0xBB,
	GPS_LOCATION	= 0xBC,	// This value is going to be passed as a pair of floating point values. (4 bytes)
	SAMPLE_RETURN	= 0xBD,
	ARM				= 0xBE,
	GPS_LOST		= 0xBF,
	GPS_CLEAR		= 0xC0,

	RESET			= 0x00	// Reset the I2C Network, All Arduinos and IMU communication will reset
	};

enum SRCmds
	{
	CARRIAGE		= 0xAA,
	DRILL_POWER		= 0xBA,
	DRILL_HEIGHT	= 0xBB,
	PROBE_MOTOR		= 0xCA,
	PROBE_ACTUATOR	= 0xCB,
	MODULE_HEIGHT	= 0xDA,
	ROTATE_TRAY		= 0xEA,
	VALVE			= 0xFA,
	};
#endif
