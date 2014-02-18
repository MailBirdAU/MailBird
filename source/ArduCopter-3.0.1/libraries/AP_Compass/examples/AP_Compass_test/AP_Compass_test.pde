/*
 *       Example of APM_Compass library (HMC5843 sensor).
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>

#include <AP_Math.h>    // ArduPilot Mega Vector/Matrix math Library
#include <AP_Declination.h>
#include <AP_Compass.h> // Compass Library

#include <aaWiiCamera.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
AP_Compass_PX4 compass;
#else
AP_Compass_HMC5843 compass;
#endif
uint32_t timer;

byte WiiCamera_State;
byte idx;
int blobcount;					// returns the number of blobs found
Blobs Blob[4];					// array of four structures to store blob data

//char     idx;
int ledPin = 13;
int i;
int Ix1,Iy1,Ix2,Iy2;
int Ix3,Iy3,Ix4,Iy4;
int s;

uint8_t  read[16];
uint8_t  *readptr = &read[0];

uint8_t  control[2]    = {0x30,0x01};
uint8_t  max [2]       = {0x06,0x90};
uint8_t  gain [2]      = {0x08,0xC0};
uint8_t  gainlimit [2] = {0x1A,0x40};
uint8_t  mode [2]      = {0x33,0x33};
uint8_t  last [2]      = {0x30,0x08};

uint8_t *controlptr = &control[0];
uint8_t *maxptr = &max[0];
uint8_t *gainptr = &gain[0];
uint8_t *gainlimitptr = &gainlimit[0];
uint8_t *modeptr = &mode[0];
uint8_t *lastptr = &last[0];

uint8_t  B0 [5] = {0xB0,0xB0,0xB0,0xB0,0xB0};
uint8_t *B0ptr  = &last[0];

uint8_t  data    = 0x36;
uint8_t *dataptr = &data;
AP_HAL::Semaphore*  _i2c_sem;

#define WII_CAMERA_PORT 0x58

void setup() {
#if 0
    //hal.i2c->begin();
    hal.scheduler->delay(100);
    
    hal.i2c->IRCam_init();
    
    hal.scheduler->delay(100);

    _i2c_sem = hal.i2c->get_semaphore();
    if (!_i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get HMC5843 semaphore"));
    }
    
    hal.i2c->setHighSpeed(false);
    
    //hal.i2c->write((uint8_t)WII_CAMERA_PORT,0, B0ptr);
    //hal.i2c->write((uint8_t)WII_CAMERA_PORT,0, B0ptr);
    //hal.i2c->write((uint8_t)WII_CAMERA_PORT,0, B0ptr);
    //hal.i2c->write((uint8_t)WII_CAMERA_PORT,0, B0ptr);
    //hal.i2c->write((uint8_t)WII_CAMERA_PORT,0, B0ptr);
    //hal.i2c->write((uint8_t)WII_CAMERA_PORT,0, B0ptr);
    //hal.i2c->write((uint8_t)WII_CAMERA_PORT,5, B0ptr);
    
    ///////////////////////
    //INITALIZE CAMERA HERE
    ///////////////////////
    //Write2bytes(0x30,0x01);				//Control byte, allows modification of settings
    //Write2bytes(0x30,0x08);				// was 2nd - suspect it really needs to be here
    //Write2bytes(0x06,0x90);				// MAXSIZE - Maximum blob size. Wii uses values from 0x62 to 0xc8.
    //Write2bytes(0x08,0xC0);				// GAIN - Sensor Gain. Smaller values = higher gain. Numerical gain is proportional to 1/2^(n/16) for n<0x40
    //Write2bytes(0x1A,0x40);				// GAINLIMIT - Sensor Gain Limit. Must be less than GAIN for camera to function. No other effect?
    //Write2bytes(0x33,0x33);				// MODE - Camera mode
    
    /*hal.scheduler->delay(20000);
    hal.i2c->writeRegister((uint8_t)WII_CAMERA_PORT,0x30,0x01);
    hal.scheduler->delay(50);
    hal.i2c->writeRegister((uint8_t)WII_CAMERA_PORT,0x30,0x08);
    hal.scheduler->delay(50);
    hal.i2c->writeRegister((uint8_t)WII_CAMERA_PORT,0x06,0x90);
    hal.scheduler->delay(50);
    hal.i2c->writeRegister((uint8_t)WII_CAMERA_PORT,0x08,0xC0);
    hal.scheduler->delay(50);
    hal.i2c->writeRegister((uint8_t)WII_CAMERA_PORT,0x1A,0x40);
    hal.scheduler->delay(50);
    hal.i2c->writeRegister((uint8_t)WII_CAMERA_PORT,0x33,0x33);
    hal.scheduler->delay(100);*/
    
    hal.i2c->write((uint8_t)WII_CAMERA_PORT,2, controlptr);
    hal.scheduler->delay(50);
    hal.i2c->write((uint8_t)WII_CAMERA_PORT,2, lastptr);
    hal.scheduler->delay(50);
    hal.i2c->write((uint8_t)WII_CAMERA_PORT,2, maxptr);
    hal.scheduler->delay(50);
    hal.i2c->write((uint8_t)WII_CAMERA_PORT,2, gainptr);
    hal.scheduler->delay(50);
    hal.i2c->write((uint8_t)WII_CAMERA_PORT,2, gainlimitptr);
    hal.scheduler->delay(50);
    hal.i2c->write((uint8_t)WII_CAMERA_PORT,2, modeptr);
    hal.scheduler->delay(50);
    
    _i2c_sem->give();

    hal.scheduler->delay(1000);
#endif
  
  hal.i2c->IRCam_init();  
  hal.scheduler->delay(100);

  _i2c_sem = hal.i2c->get_semaphore();
  if (!_i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
      hal.scheduler->panic(PSTR("Failed to get HMC5843 semaphore"));
  }
    
  hal.i2c->setHighSpeed(false);  
	
  hal.i2c->write((uint8_t)WII_CAMERA_PORT,2, controlptr);
  hal.scheduler->delay(50);
  hal.i2c->write((uint8_t)WII_CAMERA_PORT,2, lastptr);
  hal.scheduler->delay(50);
  hal.i2c->write((uint8_t)WII_CAMERA_PORT,2, maxptr);
  hal.scheduler->delay(50);
  hal.i2c->write((uint8_t)WII_CAMERA_PORT,2, gainptr);
  hal.scheduler->delay(50);
  hal.i2c->write((uint8_t)WII_CAMERA_PORT,2, gainlimitptr);
  hal.scheduler->delay(50);
  hal.i2c->write((uint8_t)WII_CAMERA_PORT,2, modeptr);
  hal.scheduler->delay(50);
    
  _i2c_sem->give();

  hal.scheduler->delay(1000);
    
  WiiCamera_State = 1;				    // Ready to Read state
}

void loop()
{   
#if 0
    //hal.i2c->setHighSpeed(false);
    
    //for(idx=0;idx<16;idx++) read[idx]=5;
    
    //hal.i2c->readRegisters((uint8_t)WII_CAMERA_PORT,0x36,16,readptr);
    
    hal.i2c->write((uint8_t)WII_CAMERA_PORT,1, dataptr);
    hal.i2c->read((uint8_t)WII_CAMERA_PORT,16, readptr);
    
    //_i2c_sem->give();
    
    // Wii Remote IR sensor  test sample code  by kako

    Ix1 = read[1];
    Iy1 = read[2];
    s   = read[3];
    Ix1 += (s & 0x30) <<4;
    Iy1 += (s & 0xC0) <<2;

    Ix2 = read[4];
    Iy2 = read[5];
    s   = read[6];
    Ix2 += (s & 0x30) <<4;
    Iy2 += (s & 0xC0) <<2;

    Ix3 = read[7];
    Iy3 = read[8];
    s   = read[9];
    Ix3 += (s & 0x30) <<4;
    Iy3 += (s & 0xC0) <<2;

    Ix4 = read[10];
    Iy4 = read[11];
    s   = read[12];
    Ix4 += (s & 0x30) <<4;
    Iy4 += (s & 0xC0) <<2;

    hal.console->printf_P(PSTR("Ix1: %d"),(int)Ix1);
    hal.console->printf_P(PSTR("Iy1: %d\r\n"),(int)Iy1);
    hal.console->printf_P(PSTR("Ix2: %d"),(int)Ix2);
    hal.console->printf_P(PSTR("Iy2: %d\r\n"),(int)Iy2);
    
    hal.scheduler->delay(1000);
#endif
	
  uint8_t data = 0x36;
  uint8_t*dataptr = &data;
  uint8_t data_buf[16];
  uint8_t*data_bufptr = &data_buf[0];
  int idx=0;
  int s;

if(WiiCamera_State==1)	// New Read request
  {
	// clear space for new data
	for (idx=0;idx<16;idx++) data_buf[idx]=0;
	// index for data pointer
	idx = 0;

	// Reset the blob counter
	blobcount = 0;

	WiiCamera_State++;	// new state expects data
  }

  if (WiiCamera_State>=2)	// Waiting for first Data byte from a previous read request
  {
    WiiCamera_State++;		// new state, now reading data
    
    //hal.i2c->readRegisters((uint8_t)WII_CAMERA_PORT,0x36,16,readptr);

    hal.i2c->write((uint8_t)WII_CAMERA_PORT,1, dataptr);
    hal.i2c->read((uint8_t)WII_CAMERA_PORT,16, data_bufptr);

	// if we have 16 bytes of data
	if (idx >= 16)
	{
	    Blob[0].X= data_buf[1];
        Blob[0].Y = data_buf[2];
        s   = data_buf[3];
        Blob[0].X+= (s & 0x30) <<4;
        Blob[0].Y += (s & 0xC0) <<2;
        Blob[0].Size = (s & 0x0F);
		if (Blob[0].Size<15) blobcount++;

        Blob[1].X = data_buf[4];
        Blob[1].Y = data_buf[5];
        s   = data_buf[6];
        Blob[1].X += (s & 0x30) <<4;
        Blob[1].Y += (s & 0xC0) <<2;
        Blob[1].Size = (s & 0x0F);
		if (Blob[1].Size<15) blobcount++;

        Blob[2].X = data_buf[7];
        Blob[2].Y = data_buf[8];
        s   = data_buf[9];
        Blob[2].X += (s & 0x30) <<4;
        Blob[2].Y += (s & 0xC0) <<2;
        Blob[2].Size = (s & 0x0F);
		if (Blob[2].Size<15) blobcount++;

        Blob[3].X = data_buf[10];
        Blob[3].Y = data_buf[11];
        s   = data_buf[12];
        Blob[3].X += (s & 0x30) <<4;
        Blob[3].Y += (s & 0xC0) <<2;
        Blob[3].Size = (s & 0x0F);
		if (Blob[3].Size<15) blobcount++;

		WiiCamera_State = 1;				// back to ready to Read state
	}
	if (WiiCamera_State>=10) WiiCamera_State = 1;	// Waited too long for data - back to ready to Read state
}

hal.scheduler->delay(1000);

}

AP_HAL_MAIN();
