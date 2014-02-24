#ifndef aaWiiCamera_h
#define aaWiiCamera_h

typedef unsigned char byte;

#include <Arduino.h>
#include <AP_HAL.h>
#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AC_PID.h>             // PID library
#include <APM_PI.h>             // PID library
#include <AP_InertialNav.h>     // Inertial Navigation library


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
	
  protected:
	/// get_loiter_position_to_velocity - loiter position controller
	///     converts desired position held in _target vector to desired velocity
    void get_loiter_position_to_velocity(float dt, float max_speed_cms);

	/// get_loiter_velocity_to_acceleration - loiter velocity controller
	///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
	void get_loiter_velocity_to_acceleration(float vel_lat_cms, float vel_lon_cms, float dt);

	/// get_loiter_acceleration_to_lean_angles - loiter acceleration controller
	///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
	void get_loiter_acceleration_to_lean_angles(float accel_lat_cmss, float accel_lon_cmss);

#if 0	
	// references to inertial nav and ahrs libraries
	const AP_InertialNav* const _inav;
	const AP_AHRS*        const _ahrs;
	
    // pointers to pid controllers
	APM_PI*		const _pid_pos_lat;
	APM_PI*		const _pid_pos_lon;
	AC_PID*		const _pid_rate_lat;
	AC_PID*		const _pid_rate_lon;
#endif
	
  public:
	//Constructor
	//WiiCamVar(const AP_InertialNav* inav, const AP_AHRS* ahrs, APM_PI* pid_pos_lat, APM_PI* pid_pos_lon, AC_PID* pid_rate_lat, AC_PID* pid_rate_lon);
	WiiCamera();
	void init();					// initialise the camera
	int read();						// read the sensor - return the number of blobs found
	int blobcount;					// returns the number of blobs found
	Blobs Blob[4];					// array of four structures to store blob data	
	
};
#endif
 
