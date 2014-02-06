

/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables

#ifdef USERHOOK_VARIABLES

#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#define WII_CAMERA_PORT 0x21             	 //i2C address of Wii camera ( = 0xB0 >> 1 to convert twi to i2c?)
#define TARGET_MAX_WIDTH 200  	 //width of IR blobs on IR target in mm
#define TARGET_MIN_WIDTH 10    	//width of IR blobs on IR target in mm
#define CAMERA_YAW 0	          		//rotation of the camera sensor from aircraft body in degrees

// Real world parameters
#define DEG2RAD  PI/180.0

// Camera parameters
#define WII_CAMERA_ADDRESS 0x21					// i2C address of Wii camera (0xB0 >> 1 to convert twi to i2c?)
#define X_PIX 1024								// number of pixels in X dimension of camera
#define Y_PIX 768								// number of pixels in Y dimension of camera
#define X_CENTRE X_PIX/2						// X coordinate of Centre Pixel in Camera
#define Y_CENTRE Y_PIX/2						// Y coordinate of Centre Pixel in Camera
#define X_FOV 47								// Field of view of Camera across X dimension -> from http://forum.wiibrew.org/read.php?7,11114
#define Y_FOV (X_FOV * Y_PIX / X_PIX) 			// Field of view of Camera across Y dimension Calculated from X FOV
#define PIX2DEG (X_FOV / X_PIX)					// Number of pixels per degree of view

#endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


