#ifndef aaWiiCamera_h
#define aaWiiCamera_h

typedef unsigned char byte;

#include <Arduino.h>
#include <Wire.h>
#include <AP_HAL.h>


// Structure to hold blob data
struct Blobs
{
   	int X;
   	int Y;
   	int Size;
   	byte number;
};


class WiiCamera
{
  private:
	int IRsensorAddress;
	int IRslaveAddress;
    byte WiiCamera_State;
	byte idx;
	void Write2bytes(byte, byte);
	AP_HAL::Semaphore*  _i2c_sem;
	
  public:
	WiiCamera();					// Constructor
	void init();					// initialise the camera
	int read();						// read the sensor - return the number of blobs found
	int blobcount;					// returns the number of blobs found
	Blobs Blob[4];					// array of four structures to store blob data	
};
#endif
 
