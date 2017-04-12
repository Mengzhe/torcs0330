#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

class Driver
{
	public:
		Driver(int index);

	/* callback functions called from TORCS */
        void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
        void newRace(tCarElt* car, tSituation *s);
        // void drive(tCarElt* car, tSituation *s);
        void drive(tSituation *s);
        int pitCommand(tSituation *s);
        void endRace(tSituation *s);
	
         
        float getAllowedSpeed(tTrackSeg *segment);
        float getAccel();
        float getDistToSegEnd();

        private:
	/* utility functions */
        bool isStuck();
        void update(tSituation *s);

        int getGear(); 
        float getBrake();

        void initCa();

        void initCw();
        float CW;      /* aerodynamic drag coefficient */

        float filterABS(float brake);
        static const float ABS_SLIP;
        static const float ABS_MINSPEED;
        

        float filterTCL(float accel);
        float filterTCL_RWD();
        float filterTCL_FWD();
        float filterTCL_4WD();
        void initTCLfilter();
        float (Driver::*GET_DRIVEN_WHEEL_SPEED)();
        static const float TCL_SLIP;
        static const float TCL_MINSPEED;

	/* per robot global data */
        int stuck;
        float trackangle;
        float angle;

        
        
	/* data that should stay constant after first initialization */
        int MAX_UNSTUCK_COUNT;
        int INDEX;

	/* class constants */
        static const float MAX_UNSTUCK_ANGLE;
        static const float UNSTUCK_TIME_LIMIT;

        static const float MAX_UNSTUCK_SPEED;
        static const float MIN_UNSTUCK_DIST;

        static const float G;
        static const float FULL_ACCEL_MARGIN;

        static const float SHIFT;
        static const float SHIFT_MARGIN;

        float mass;        /* mass of car + fuel */
        tCarElt *car;      /* pointer to tCarElt struct */
        float CARMASS;     /* mass of the car only */
        float CA;          /* aerodynamic downforce coefficient */  
        
        /* track variables */
        tTrack* track;
}; 

#endif // _DRIVER_H_
