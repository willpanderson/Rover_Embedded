#ifndef PACKET_H
#define PACKET_H

#ifdef __linux__
	#include "utils/Arduino.h"
	#include "defs/global_defs.h"
	typedef unsigned char byte;
#endif

#define OverheadBytes 3
#define MaxBytes 255
#define MinBytes OverheadBytes + 1

bool ValidatePacket (size_t PacketSize, byte * Packet)
	{
	byte Checksum = 0x00;

	/* TESTING FOR INVALID PACKET */
	if ((PacketSize < MinBytes) || (PacketSize > MaxBytes)) // PacketSize out of range
			return false;
		else if (Packet [0] != START) // Invalid handshake
				return false;
			else if (Packet [1] != PacketSize) // Wrong size
					return false;

	/* TESTING FOR CORRUPT DATA */
	for (int i = 0; i < PacketSize - 1; i++)
		Checksum = Checksum ^ Packet [i];
	if (Checksum != Packet [PacketSize - 1])
		return false;

	/* TESTING COMPLETE */
	return true;
	}

int LookForPacket (byte * ReturnBuffer)
	{
	size_t Count = 0;
	size_t PacketSize = MinBytes;
	byte Buffer [MaxBytes];
	byte Payload [MaxBytes];
	
	while (Serial.available ())	// Changed this from "while (true)" so that the rover won't get caught in an infinite loop.
		{
		byte RX;
#ifdef __linux__
		unsigned char * read_buff = new unsigned char [1];
	
		int res = core.readPacket (read_buff);
		RX = read_buff [0];
//		std::cout << RX << std::endl;
		if (res == 0)
				{
				/* Debug */
				//std::cout << "Begin\n";
				//std::cout << (int)read_buff[0] << std::endl;
				//std::cout << RX << std::endl;
				//std::cout << "END \n";
				/* End Debug */
				usleep (1000);
				continue;
				}
			else ;//std::cout << RX << std::endl;
//		std::cout << Count << std::endl

#else
		if (Serial.available ())
				RX = Serial.read ();
			else
					{
					Serial.flush ();
					delay (5);
					continue;
					}
#endif	/* __linux__ */
		if ((Count == 0) && ((Global_Definitions) RX == START)) // New packet
				{
				Buffer [Count++] = RX;
				continue;
				}
			else if (Count == 0) // First byte invalid, ignore and move on.
					continue;
				else if (Count == 1)
						{
						Buffer [Count++] = RX;		// This is the packet length
						if ((Buffer [1] < MinBytes) || (Buffer [1] > MaxBytes))	// If the packet is NOT in the correct size range...
								Count = 0;			// ...reset the count, throw out the packet, and move on.
							else PacketSize = RX;	// ...else store the packet size
						continue;
						}
					else if (Count < PacketSize)
							Buffer [Count++] = RX;
		if (Count >= PacketSize)					// If we receive the entire packet...
				if (ValidatePacket (PacketSize, Buffer))	// Test if it's valid.
						{
						for (int i = 2, j = 0; j < (PacketSize - OverheadBytes); i++, j++)	// Ignore the overhead bytes, extract the payload.
							ReturnBuffer [j] = Buffer [i];
						return (PacketSize - OverheadBytes);
						}
					else return 0;
		}
	return 0;
	}

bool SendPacket (size_t PayloadSize, byte * Payload)
	{
	if ((PayloadSize + OverheadBytes) > MaxBytes)
		return false;

	size_t PacketSize = PayloadSize + OverheadBytes;
	static byte Packet [MaxBytes];	// Create the buffer

	/* POPULATE THE BUFFER */
	Packet [0] = START;
	Packet [1] = PacketSize;
	byte Checksum = Packet [0] ^ Packet [1];

	for (int i = 0; i < PayloadSize; i++)
		{
		Packet [i + 2] = Payload [i];
		Checksum = Checksum ^ Packet [i + 2];
		}
	Packet [PacketSize - 1] = Checksum;

	/* PACKET COMPLETE */
#ifdef __linux__
	core.writePacket (Packet, PacketSize);
#else
	Serial.write (Packet, PacketSize);
	Serial.flush ();
#endif /* __linux__ */
	return true;
	}

#endif /* header guard */