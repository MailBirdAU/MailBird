/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

	ircam.init();
}

#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
	ircam.read();

	  float theta = 0.0;
	  float distance = 0.0;
	  float HeightAGL = 0.0;
	  float YawAngle = 0.0;
	  float MidPointX = 0.0;
	  float MidPointY = 0.0;
	  float Pix2mm =0;

	  if (ircam.blobcount==2)
	  {

	    // CALCULATIONS
	    distance = sqrt(square(ircam.Blob[0].X-ircam.Blob[1].X) + square(ircam.Blob[0].Y-ircam.Blob[1].Y));    // Calculate the number of pixels between the two IR targets
	    Pix2mm = distance/TARGET_MAX_WIDTH;                              									   // the number of pixels per cm at the target distance - will change with distance changes
	    theta = distance * PIX2DEG / 2;                             										   // half the angle between the two outside targets in degrees
	    HeightAGL = TARGET_MAX_WIDTH * 0.5 / tan(theta*DEG2RAD);         									   // Calculate the height of camera above targets
	    YawAngle = atan2(ircam.Blob[0].Y-ircam.Blob[1].Y, ircam.Blob[0].X-ircam.Blob[1].X) * 180.0 / PI;       // Angle of rotation of the two targets in the image
	    YawAngle += CAMERA_YAW;                                  						 // add any sensor alignment angle to yaw angle

	    MidPointX = (ircam.Blob[0].X+ircam.Blob[1].X)/2.0;                            	     // find the mid point of the line in pixels
	    MidPointY = (ircam.Blob[0].Y+ircam.Blob[1].Y)/2.0;
	    WiiDisplacementX = int(X_CENTRE-MidPointX);                      					 // calculate the displacement of the midpoint from the centre of image in pixels
	    WiiDisplacementY = int(Y_CENTRE-MidPointY);
	    WiiDisplacementX /= Pix2mm;									  						 // Convert the displacement of the midpoint from the centre of image to mm
	    WiiDisplacementY /= Pix2mm;

	    //
	    WiiRange = HeightAGL/10.0;
	    WiiRotation = YawAngle;
	  }
	  else
	  {
	    WiiRange = 0;
	    WiiRotation = 0;
	  }
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
