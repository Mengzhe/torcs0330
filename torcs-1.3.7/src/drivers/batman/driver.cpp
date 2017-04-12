#include "driver.h"

// const float Driver::MAX_UNSTUCK_ANGLE = 30.0/180.0*PI;  /* [radians] */
const float Driver::MAX_UNSTUCK_ANGLE = 20.0/180.0*PI;  /* [radians] */
const float Driver::UNSTUCK_TIME_LIMIT = 2.0;           /* [s] */

const float Driver::MAX_UNSTUCK_SPEED = 5.0;   /* [m/s] */
const float Driver::MIN_UNSTUCK_DIST = 3.0;    /* [m] */

const float Driver::G = 9.81;                  /* [m/(s*s)] */
const float Driver::FULL_ACCEL_MARGIN = 1.0;   /* [m/s] */

const float Driver::SHIFT = 0.9;         /* [-] (% of rpmredline) */
const float Driver::SHIFT_MARGIN = 4.0;  /* [m/s] */ 

const float Driver::ABS_SLIP = 0.9;        /* [-] range [0.95..0.3] */
const float Driver::ABS_MINSPEED = 3.0;    /* [m/s] */

const float Driver::TCL_SLIP = 0.9;        /* [-] range [0.95..0.3] */
const float Driver::TCL_MINSPEED = 3.0;    /* [m/s] */

Driver::Driver(int index)
{
    INDEX = index;
}


/* Called for every track change or new race. */
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
    track = t;
    *carParmHandle = NULL;
}

/* Start a new race. */
void Driver::newRace(tCarElt* car, tSituation *s)
{
    MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
    stuck = 0;

    this->car = car;
    CARMASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);
    initCa();

    initTCLfilter();
}

/* Drive during race. */
//void Driver::drive(tCarElt* car, tSituation *s)
void Driver::drive(tSituation *s)
{
    update(s);

    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    if (isStuck()) {
        car->ctrl.steer = -angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        // car->ctrl.accelCmd = 0.5; // 50% accelerator pedal
        // car->ctrl.brakeCmd = 0.0; // no brakes
        // car->ctrl.brakeCmd = getBrake();
        car->ctrl.brakeCmd = filterABS(getBrake());

        if (car->ctrl.brakeCmd == 0.0) {
            // car->ctrl.accelCmd = getAccel();
            car->ctrl.accelCmd = filterTCL(getAccel());
        } else {
            car->ctrl.accelCmd = 0.0;
        }
    } else {
        float steerangle = angle - car->_trkPos.toMiddle/car->_trkPos.seg->width;

        car->ctrl.steer = steerangle / car->_steerLock;
        // car->ctrl.gear = 3; // first gear
        car->ctrl.gear = getGear(); 

        
        //car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
        //car->ctrl.brakeCmd = 0.0; // no brakes
        // car->ctrl.brakeCmd = getBrake();
        car->ctrl.brakeCmd = filterABS(getBrake());

        if (car->ctrl.brakeCmd == 0.0) {
            // car->ctrl.accelCmd = getAccel();
            car->ctrl.accelCmd = filterTCL(getAccel());
        } else {
            car->ctrl.accelCmd = 0.0;
        }
    }
}

/* Set pitstop commands. */
int Driver::pitCommand(tSituation *s)
{
    return ROB_PIT_IM; /* return immediately */
}

/* End of the current race */
void Driver::endRace(tSituation *s)
{
}


/* */
/* Update my private data every timestep */
void Driver::update(tSituation *s)
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);

    mass = CARMASS + car->_fuel;
}

/* Check if I'm stuck */
bool Driver::isStuck()
{
    if (fabs(angle) > MAX_UNSTUCK_ANGLE &&
        car->_speed_x < MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
        if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*angle < 0.0) {
            return true;
        } else {
            stuck++;
            return false;
        }
    } else {
        stuck = 0;
        return false;
    }
}

/* Compute the allowed speed on a segment */
float Driver::getAllowedSpeed(tTrackSeg *segment)
{
    if (segment->type == TR_STR) {
        return FLT_MAX;
    } else {
        float mu = segment->surface->kFriction;
        return sqrt(mu*G*segment->radius);
    }
}   

/* Compute the length to the end of the segment */
float Driver::getDistToSegEnd()
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    } else {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
    }
}   

/* Compute fitting acceleration */
float Driver::getAccel()
{
    float allowedspeed = getAllowedSpeed(car->_trkPos.seg);
    float gr = car->_gearRatio[car->_gear + car->_gearOffset];
    float rm = car->_enginerpmRedLine;
    if (allowedspeed > car->_speed_x + FULL_ACCEL_MARGIN) {
        return 1.0;
    } else {
        return allowedspeed/car->_wheelRadius(REAR_RGT)*gr /rm;
    }
}  


/* Compute if need to brake */
float Driver::getBrake()
{
    tTrackSeg *segptr = car->_trkPos.seg;
    float currentspeedsqr = car->_speed_x*car->_speed_x;
    float mu = segptr->surface->kFriction;
    /* maxlookaheddist is the distance we have to check (formula with special case v2 = 0) */
    float maxlookaheaddist = currentspeedsqr/(2.0*mu*G);
    /* lookaheaddist holds the distance we have already checked */
    float lookaheaddist = getDistToSegEnd();
    /* Compute the allowed speed on the current segment.  */
    float allowedspeed = getAllowedSpeed(segptr);
    if (allowedspeed < car->_speed_x) return 1.0;
    segptr = segptr->next;
    while (lookaheaddist < maxlookaheaddist) {
    allowedspeed = getAllowedSpeed(segptr);
        if (allowedspeed < car->_speed_x) {
            /* Compute the braking distance according to the formula */
            float allowedspeedsqr = allowedspeed*allowedspeed;
            //float brakedist = (currentspeedsqr - allowedspeedsqr) / (2.0*mu*G);
            float brakedist = mass*(currentspeedsqr - allowedspeedsqr) /
                      (2.0*(mu*G*mass + allowedspeedsqr*(CA*mu + CW)));
            if (brakedist > lookaheaddist) {
            return 1.0;
            }
        }
        lookaheaddist += segptr->length;
        segptr = segptr->next;
    }
    return 0.0;
} 

/* Compute gear */
int Driver::getGear()
{
    /* shift to the first gear, if in neutral or reverse */
    if (car->_gear <= 0) return 1;  
    float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
    float omega = car->_enginerpmRedLine/gr_up;
    float wr = car->_wheelRadius(2);
    /* shift up if allowed speed for the current gear (omega*wr*SHIFT) is exceeded */
    if (omega*wr*SHIFT < car->_speed_x) 
    {
        return car->_gear + 1;
    } 
    else 
    {
        float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
        omega = car->_enginerpmRedLine/gr_down;
        /* If current gear is greater than one, check if current speed is lower than 
        allowed speed with the next lower gear. If so, shift down. */
        if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) 
        {
            return car->_gear - 1;
        }
    }
    /* If all of the above didn't apply, return the current gear. */
    return car->_gear;
}        

/* Compute aerodynamic downforce coefficient CA */
void Driver::initCa()
{
    const char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
    float rearwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING,
                                       PRM_WINGAREA, (char*) NULL, 0.0);
    float rearwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING,
                                        PRM_WINGANGLE, (char*) NULL, 0.0);
    float wingca = 1.23*rearwingarea*sin(rearwingangle);
    float cl = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,
                             PRM_FCL, (char*) NULL, 0.0) +
                GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,
                             PRM_RCL, (char*) NULL, 0.0);
    float h = 0.0;
    int i;
    for (i = 0; i < 4; i++)
        h += GfParmGetNum(car->_carHandle, WheelSect[i],
                          PRM_RIDEHEIGHT, (char*) NULL, 0.20);
    h*= 1.5; h = h*h; h = h*h; h = 2.0 * exp(-3.0*h);
    CA = h*cl + 4.0*wingca;
}

/* Compute aerodynamic drag coefficient CW */
void Driver::initCw()
{
    float cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,
                            PRM_CX, (char*) NULL, 0.0);
    float frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,
                                   PRM_FRNTAREA, (char*) NULL, 0.0);
    CW = 0.645*cx*frontarea;
}

/* Antilocking filter for brakes */
float Driver::filterABS(float brake)
{
    /* Don't apply ABS when speed is slow */
    if (car->_speed_x < ABS_MINSPEED) return brake;
    int i;
    float slip = 0.0;
    for (i = 0; i < 4; i++) {
        slip += car->_wheelSpinVel(i) * car->_wheelRadius(i) / car->_speed_x;
    }
    /* Compute average slip of the four wheels */
    slip = slip/4.0;
    /* If the slip variable is below a certain threshold, 
       reduce the brake value by multiplying it with the slip */
    if (slip < ABS_SLIP) brake = brake*slip;
    return brake;
}

/* TCL filter for accelerator pedal */
float Driver::filterTCL(float accel)
{
    if (car->_speed_x < TCL_MINSPEED) return accel;
    float slip = car->_speed_x/(this->*GET_DRIVEN_WHEEL_SPEED)();
    if (slip < TCL_SLIP) {
        accel = 0.0;
    }
    return accel;
}

/* Traction Control (TCL) setup */
void Driver::initTCLfilter()
{
    char const *traintype = GfParmGetStr(car->_carHandle, 
        SECT_DRIVETRAIN, PRM_TYPE, VAL_TRANS_RWD); // here is different from official tutorial
    if (strcmp(traintype, VAL_TRANS_RWD) == 0) {
        GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_RWD;
    } else if (strcmp(traintype, VAL_TRANS_FWD) == 0) {
        GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_FWD;
    } else if (strcmp(traintype, VAL_TRANS_4WD) == 0) {
        GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_4WD;
    }
}

/* TCL filter plugin for rear wheel driven cars */
float Driver::filterTCL_RWD()
{
    return (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
            car->_wheelRadius(REAR_LFT) / 2.0;
}


/* TCL filter plugin for front wheel driven cars */
float Driver::filterTCL_FWD()
{
    return (car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
            car->_wheelRadius(FRNT_LFT) / 2.0;
}


/* TCL filter plugin for all wheel driven cars */
float Driver::filterTCL_4WD()
{
    return (car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
            car->_wheelRadius(FRNT_LFT) / 4.0 +
           (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
            car->_wheelRadius(REAR_LFT) / 4.0;
}