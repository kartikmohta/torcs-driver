#include "driver.h"

const float Driver::MAX_UNSTUCK_ANGLE   = 30.0/180.0*PI;    /* [radians] */
const float Driver::UNSTUCK_TIME_LIMIT  = 1.5;              /* [s] */
const float Driver::MAX_UNSTUCK_SPEED   = 5.0;              /* [m/s] */
const float Driver::MIN_UNSTUCK_DIST    = 2.0;              /* [m] */
const float Driver::G                   = 9.81;             /* [m/(s*s)] */
const float Driver::FULL_ACCEL_MARGIN   = 3.0;              /* [m/s] */
const float Driver::SHIFT               = 0.95;             /* [-] (% of rpmredline) */
const float Driver::SHIFT_MARGIN        = 4.0;              /* [m/s] */
const float Driver::ABS_SLIP            = 3.0f;             /* [m/s] range [0..10] */
const float Driver::ABS_RANGE           = 5.0f;             /* [m/s] range [0..10] */
const float Driver::ABS_MINSPEED        = 3.0;              /* [m/s] */
const float Driver::TCL_SLIP            = 2.0f;             /* [m/s] range [0..10] */
const float Driver::TCL_RANGE           = 5.0f;             /* [m/s] range [0..10] */
const float Driver::TCL_MINSPEED        = 3.0;              /* [m/s] */
const float Driver::LOOKAHEAD_CONST     = 10.0;             /* [m] */
const float Driver::LOOKAHEAD_FACTOR    = 0.42;             /* [1/s] */
const float Driver::WIDTHDIV            = 3.0;              /* [-] */
const float Driver::MU_FACTOR           = 0.60f;            /* [-] */
const float Driver::STEER_DIRECTION_GAIN= 1.0f;             /* [-] Gain to apply for basic steering */
const float Driver::STEER_DRIFT_GAIN    = 0.01f;            /* [-] Gain for drift compensation */
const float Driver::STEER_PREDICT_GAIN  = 0.1f;             /* [-] Predictive gain, multiplied with yaw rate. */
const float Driver::MAX_FUEL_PER_METER  = 0.00092f;         /* [liter/m] fuel consumtion */
const float Driver::CLUTCH_SPEED        = 5.0f;             /* [m/s] */
const float Driver::CLUTCH_FULL_MAX_TIME = 1.0f;            /* [s] Time to apply full clutch. */

Driver::Driver(int index)
{
    INDEX = index;
    stuck = 0;
    trackangle = 0;
    angle = 0;
    oldlookahead = 0;
}

Driver::~Driver()
{
    delete opponents;
}

/* Called for every track change or new race. */
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
    track = t;
    const int BUFSIZE = 256;
    char buffer[BUFSIZE];

    // Load a custom setup if one is available.
    // Get a pointer to the first char of the track filename.
    /*
    char* trackname = strrchr(track->filename, '/') + 1;
    switch (s->_raceType) {
        case RM_TYPE_PRACTICE:
            snprintf(buffer, BUFSIZE, "drivers/kartik/%d/practice/%s", INDEX, trackname);
            break;
        case RM_TYPE_QUALIF:
            snprintf(buffer, BUFSIZE, "drivers/kartik/%d/qualifying/%s", INDEX, trackname);
            break;
        case RM_TYPE_RACE:
            snprintf(buffer, BUFSIZE, "drivers/kartik/%d/race/%s", INDEX, trackname);
            break;
        default:
            break;
    }

    *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
    */
    *carParmHandle = NULL;
    if (*carParmHandle == NULL)
    {
        snprintf(buffer, BUFSIZE, "drivers/kartik/%d/default.xml", INDEX);
        *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
    }

    float fuel = track->length*MAX_FUEL_PER_METER;
    fuel *= (s->_totLaps + 1.0f);
    GfParmSetNum(*carParmHandle, SECT_CAR, PRM_FUEL, (char*) NULL, fuel);
}

/* Start a new race. */
void Driver::newRace(tCarElt* car, tSituation *s)
{
    MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
    stuck = 0;
    this->car = car;
    CARMASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);
    initCa();
    initCw();
    initTCLfilter();
    initTireMu();

    /* initialize the list of opponents */
    opponents = new Opponents(s, this);
    opponent = opponents->getOpponentPtr();
}

/* Drive during race. */
void Driver::drive(tSituation *s)
{
    update(s);

    memset(&car->ctrl, 0, sizeof(tCarCtrl));
    if (isStuck())
    {
        car->ctrl.steer = -angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = filterTCL(1);
        car->ctrl.brakeCmd = 0.0; // no brakes
    }
    else
    {
        car->ctrl.steer = getSteer();
        car->ctrl.gear = getGear();
        car->ctrl.brakeCmd = filterABS(filterBColl(getBrake()));
        if (car->ctrl.brakeCmd == 0.0)
        {
            car->ctrl.accelCmd = filterTCL(filterTrk(getAccel()));
        }
        else
        {
            car->ctrl.accelCmd = 0.0;
        }
        car->_clutchCmd = getClutch();
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

/* Update my private data every timestep */
void Driver::update(tSituation *s)
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);

    speedangle = trackangle - atan2(car->_speed_Y, car->_speed_X);
    NORM_PI_PI(speedangle);

    mass = CARMASS + car->_fuel;

    speed = Opponent::getSpeed(car);
    opponents->update(s, this);
}

/* Check if I'm stuck */
bool Driver::isStuck()
{
    if (fabs(angle) > MAX_UNSTUCK_ANGLE &&
        car->_speed_x < MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST)
    {
        if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*angle < 0.0)
        {
            return true;
        }
        else
        {
            stuck++;
            return false;
        }
    }
    else
    {
        stuck = 0;
        return false;
    }
}

/* Compute the allowed speed on a segment */
float Driver::getAllowedSpeed(tTrackSeg *segment)
{
    if (segment->type == TR_STR)
    {
        return FLT_MAX;
    }
    else
    {
        float arc = 0.0;
        float bankangle = 0.0;
        //int i = 0;
        tTrackSeg *s = segment;

        bankangle = (s->angle[TR_XS] + s->angle[TR_XE])/2.0;

        if(s->type == TR_LFT)
            bankangle = -bankangle;

        float min_radius = FLT_MAX;
        while (s->type == segment->type)
        {
            arc += s->arc;
            min_radius = (min_radius < s->radius) ? min_radius : s->radius;
            s = s->next;
            /*if(arc >= M_PI*30/min_radius)
                break;*/
        }

        if(s->type != TR_STR && s->type != segment->type && arc < M_PI/4)
        {
            tTrackSeg* s2 = s;
            float arc2 = 0;
            while (s->type == s2->type && arc2 < M_PI/4)
            {
                arc2 += s->arc;
                s = s->next;
            }
            arc -=arc2;
            if(arc <= 0)
                arc = 0;
        }

        if(arc < 0)
            arc = -arc;

        if(arc > M_PI/2)
            arc = M_PI/2;

        float mu = segment->surface->kFriction*TIREMU*MU_FACTOR;
        //float car_width = car->_dimension_x* (float)sin(angle) + car->_dimension_y* (float)cos(angle);
        float seg_width = (segment->width)/((1+tan(arc/4))*WIDTHDIV);
        float r = (2*segment->radius*segment->radius*(1 - cos(arc/2)) - 2*seg_width*segment->radius*(1 - cos(arc/2)) + seg_width*seg_width)/(2*(segment->radius*(1-cos(arc/2)) - seg_width));
        //printf("Arc: %f, seg->radius: %f, seg->width: %f, new_width: %f, R: %f\n", arc*180/M_PI, segment->radius, segment->width, seg_width, r);
        if(r <= 0)
            return FLT_MAX;
        float banknum = sin(bankangle) + mu*cos(bankangle);
        float bankden = cos(bankangle) - mu*sin(bankangle);
        float allowedSpeed = sqrt((G*r*banknum)/(1.0 - MIN(1.0, r*CA*mu/mass/bankden))/bankden);
        return allowedSpeed;
    }
}

/* Compute the length to the end of the segment */
float Driver::getDistToSegEnd()
{
    if (car->_trkPos.seg->type == TR_STR)
    {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    }
    else
    {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
    }
}

/* Compute fitting acceleration */
float Driver::getAccel()
{
    float allowedspeed = getAllowedSpeed(car->_trkPos.seg);
    float gr = car->_gearRatio[car->_gear + car->_gearOffset];
    float rm = car->_enginerpmRedLine;
    float front_side_slip = (car->_wheelSlipSide(FRNT_LFT) + car->_wheelSlipSide(FRNT_RGT))/2.0;
    //float rear_side_slip = (car->_wheelSlipSide(REAR_LFT) + car->_wheelSlipSide(REAR_RGT))/2.0;
    //printf("front side slip: %f\n", front_side_slip);
    //printf("rear side slip: %f\n", rear_side_slip);
    float accel_scale = 1.0;

    if(fabsf(front_side_slip) > 7.0 /*|| fabs(rear_side_slip) > 8.0*/)
        accel_scale = 7.0/fabsf(front_side_slip);

    if (allowedspeed > car->_speed_x + FULL_ACCEL_MARGIN)
    {
        return accel_scale*1.0;
    }
    else
    {
        return accel_scale*allowedspeed/car->_wheelRadius(REAR_RGT)*gr /rm;
    }
}

float Driver::getBrake()
{
    // Car drives backward?
    if (car->_speed_x < -MAX_UNSTUCK_SPEED)
    {
        // Yes, brake.
        return 1;
    }
    else
    {
        tTrackSeg *segptr = car->_trkPos.seg;
        float currentspeedsqr = car->_speed_x*car->_speed_x;
        float mu = segptr->surface->kFriction;
        float maxlookaheaddist = currentspeedsqr/(2.0*mu*G);
        float lookaheaddist = getDistToSegEnd();
        float allowedspeed = getAllowedSpeed(segptr);

        if (allowedspeed < car->_speed_x)
            return MIN(1.0f, (car->_speed_x-allowedspeed)/(FULL_ACCEL_MARGIN));

        segptr = segptr->next;
        while (lookaheaddist < maxlookaheaddist)
        {
            allowedspeed = getAllowedSpeed(segptr);
            if (allowedspeed < car->_speed_x)
            {
                float allowedspeedsqr = allowedspeed*allowedspeed;
                float c = mu*G;
                float d = (CA*mu + CW)/mass;
                float v1sqr = currentspeedsqr;
                float v2sqr = allowedspeedsqr;
                float brakedist = -log((c + v2sqr*d)/(c + v1sqr*d))/(2.0*d);
                if (brakedist > lookaheaddist)
                {
                    return 1.0;
                }
            }
            lookaheaddist += segptr->length;
            segptr = segptr->next;
        }
    }
    return 0.0;
}

/* Compute gear */
int Driver::getGear()
{
    if (car->_gear <= 0)
        return 1;

    float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
    float omega = car->_enginerpmRedLine/gr_up;
    float wr = car->_wheelRadius(2);

    if (omega*wr*SHIFT < car->_speed_x)
    {
        return car->_gear + 1;
    }
    else
    {
        float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
        omega = car->_enginerpmRedLine/gr_down;
        if ( (car->_gear > 1) && (omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) )
            return car->_gear - 1;
    }
    return car->_gear;
}

/* Compute aerodynamic downforce coefficient CA */
void Driver::initCa()
{
    const char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
    float rearwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*) NULL, 0.0f);
    float rearwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*) NULL, 0.0f);
    float frontwingarea = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_WINGAREA, (char*) NULL, 0.0f);
    float frontwingangle = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_WINGANGLE, (char*) NULL, 0.0f);
    float wingca = 1.23f*(rearwingarea*sin(rearwingangle) + frontwingarea*sin(frontwingangle));

    float cl = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*) NULL, 0.0) +
            GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*) NULL, 0.0);
    float h = 0.0;
    for (int i = 0; i < 4; i++)
        h += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*) NULL, 0.20f);
    h*= 1.5; h = h*h; h = h*h; h = 2.0 * exp(-3.0*h);
    CA = h*cl + 4.0f*wingca;
    //printf("CA: %f\n", CA);
}

/* Compute aerodynamic drag coefficient CW */
void Driver::initCw()
{
    float cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*) NULL, 0.0);
    float frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*) NULL, 0.0);
    CW = 0.645*cx*frontarea;
    //printf("CW: %f\n", CW);
}

// Init the friction coefficient of the the tires.
void Driver::initTireMu()
{
    const char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
    float tm = 0;
    int i;

    for (i = 0; i < 4; i++)
    {
        tm += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_MU, (char*) NULL, 1.7f);
    }
    TIREMU = tm/4.0;
}

/* Antilocking filter for brakes */
float Driver::filterABS(float brake)
{
    //If we are too slow don't consider ABS (division by zero).
    if (car->_speed_x < ABS_MINSPEED)
        return brake;

#if 1
    //Compute the average slip of the four wheels.
    int i;
    float slip = 0.0;
    for (i = 0; i < 4; i++)
    {
        slip += car->_wheelSpinVel(i) * car->_wheelRadius(i);
    }
    slip = slip/4.0;
#else
    //Compute the average slip of the front wheels.
    int i;
    float slip = 0.0;
    for (i = 0; i < 2; i++)
    {
        slip += car->_wheelSpinVel(i) * car->_wheelRadius(i);
    }
    slip = slip/2.0;
#endif
    //printf("skid: F: %f, %f, R: %f, %f\n", car->_skid[0],car->_skid[1],car->_skid[2],car->_skid[3]);
    slip = car->_speed_x - slip;
    if (slip > ABS_SLIP)
    {
        brake = brake - MIN(brake, (slip - ABS_SLIP)/ABS_RANGE);
    }
    return brake;
}

/* TCL filter for accelerator pedal */
float Driver::filterTCL(float accel)
{
    float slip = (this->*GET_DRIVEN_WHEEL_SPEED)() - car->_speed_x;
    if(fabs(car->_trkPos.toMiddle) > car->_trkPos.seg->width/2.0)
        slip += TCL_SLIP; // Force trigger TC

    if (slip > TCL_SLIP)
    {
        accel = accel - MIN(accel, (slip - TCL_SLIP)/TCL_RANGE);
    }
    return accel;
}

/* Traction Control (TCL) setup */
void Driver::initTCLfilter()
{
    const char *traintype = GfParmGetStr(car->_carHandle, SECT_DRIVETRAIN, PRM_TYPE, (char*)VAL_TRANS_RWD);
    if (strcmp(traintype, VAL_TRANS_RWD) == 0)
        GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_RWD;
    else if (strcmp(traintype, VAL_TRANS_FWD) == 0)
        GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_FWD;
    else if (strcmp(traintype, VAL_TRANS_4WD) == 0)
        GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_4WD;
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
    return ((car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
        car->_wheelRadius(FRNT_LFT) / 4.0) +
        ((car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
        car->_wheelRadius(REAR_LFT) / 4.0);
}

v2d Driver::getTargetPoint()
{
    tTrackSeg *seg = car->_trkPos.seg;

    float lookahead = LOOKAHEAD_CONST + car->_speed_x*LOOKAHEAD_FACTOR;
    float length = getDistToSegEnd();

    lookahead = LOOKAHEAD_CONST + car->_speed_x*LOOKAHEAD_FACTOR;
    // Prevent "snap back" of lookahead on harsh braking.
    float cmplookahead = oldlookahead - car->_speed_x*RCM_MAX_DT_ROBOTS;
    if (lookahead < cmplookahead)
    {
        lookahead = cmplookahead;
    }

    oldlookahead = lookahead;

    // Search for the segment containing the target point.
    while (length < lookahead)
    {
        seg = seg->next;
        length += seg->length;
    }

    length = lookahead - length + seg->length;
    v2d s;
    s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0;
    s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0;
    if ( seg->type == TR_STR)
    {
        v2d d;

        d.x = ((seg->vertex[TR_EL].x + seg->vertex[TR_ER].x) - (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x))/(2*seg->length);
        d.y = ((seg->vertex[TR_EL].y + seg->vertex[TR_ER].y) - (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y))/(2*seg->length);
        return s + d*length;
    }
    else
    {
        float arc_done = 1, arc_left = 1, total_arc = 1;
        tTrackSeg* checkseg = seg;
        while (checkseg->type == seg->type)
        {
            checkseg = checkseg->prev;
            arc_done += checkseg->arc;
        }
        checkseg = seg;
        while (checkseg->type == seg->type)
        {
            checkseg = checkseg->next;
            arc_left += checkseg->arc;
        }
        total_arc = arc_left + arc_done;
        v2d c, n_start, n_end, n;
        n_start.x = (seg->vertex[TR_SR].x - seg->vertex[TR_SL].x)/seg->startWidth;
        n_start.y = (seg->vertex[TR_SR].y - seg->vertex[TR_SL].y)/seg->startWidth;
        n_end.x = (seg->vertex[TR_ER].x - seg->vertex[TR_EL].x)/seg->endWidth;
        n_end.y = (seg->vertex[TR_ER].y - seg->vertex[TR_EL].y)/seg->endWidth;

        float alpha = length/seg->length;
        n.x = n_start.x*(1-alpha) + n_end.x*alpha;
        n.y = n_start.y*(1-alpha) + n_end.y*alpha;

        c.x = seg->center.x;
        c.y = seg->center.y;

        float arc = length/seg->radius;
        float arcsign = (seg->type == TR_RGT) ? -1 : 1;
        arc = arc*arcsign;
        n = arcsign*n;
        alpha = 2*((arc_left < arc_done) ? arc_left : arc_done)/total_arc;
        float allowed_movein_dist = alpha*0.8*(seg->width/2 - car->_dimension_y/2)/*sqrt(car->_dimension_y*car->_dimension_y + car->_dimension_x*car->_dimension_x)/2*/;
        if(seg->radius < 500)
            return (s-n*allowed_movein_dist).rotate((c), arc);
        else
            return s.rotate(c, arc);
    }
}

float Driver::getSteer()
{
    float targetAngle;
    v2d target = getTargetPoint();

    targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);

    float expected_yaw_rate = 0;


    /*
    if(seg->type != TR_STR)
    {
        tTrackSeg* seg = car->_trkPos.seg;
        float radius = seg->radius;
        expected_yaw_rate = Opponent::getSpeed(car)/radius;
        expected_yaw_rate *= (car->_trkPos.seg->type == TR_RGT) ? -1 : 1;
        //printf("speed_x: %f, r: %f, expected yaw rate: %f, yaw_rate: %f\n", car->_speed_x, car->_trkPos.seg->radius, expected_yaw_rate, car->_yaw_rate);
    }
    */

    // from Olethros
    float steer_direction = STEER_DIRECTION_GAIN * (targetAngle - car->_yaw - STEER_PREDICT_GAIN * (car->_yaw_rate - expected_yaw_rate));
    float correct_drift = - STEER_DRIFT_GAIN * atan2(car->_speed_Y, car->_speed_X);

    NORM_PI_PI(steer_direction);
    return correct_drift + steer_direction/car->_steerLock;
}

/* Copied from Olethros */
float Driver::getClutch()
{
    if (car->_gear > 1) {
        clutchtime = 0.0;
        return 0.0;
    } else {
        float drpm = car->_enginerpm - car->_enginerpmRedLine/2.0;
        clutchtime = MIN(CLUTCH_FULL_MAX_TIME, clutchtime);
        float clutcht = (CLUTCH_FULL_MAX_TIME - clutchtime)/CLUTCH_FULL_MAX_TIME;
        if (car->_gear == 1 && car->_accelCmd > 0.0) {
            clutchtime += (float) RCM_MAX_DT_ROBOTS;
        }

        if (drpm > 0) {
            float speedr;
            if (car->_gearCmd == 1) {
                // Compute corresponding speed to engine rpm.
                float omega = car->_enginerpmRedLine/car->_gearRatio[car->_gear + car->_gearOffset];
                float wr = car->_wheelRadius(2);
                speedr = (CLUTCH_SPEED + MAX(0.0, car->_speed_x))/fabs(wr*omega);
                float clutchr = MAX(0.0, (1.0 - speedr*2.0*drpm/car->_enginerpmRedLine));
                return MIN(clutcht, clutchr);
            } else {
                // For the reverse gear.
                clutchtime = 0.0;
                return 0.0;
            }
        } else {
            return clutcht;
        }
    }
}

/* Hold car on the track */
float Driver::filterTrk(float accel)
{
    tTrackSeg* seg = car->_trkPos.seg;
    //printf("Speedangle: %f\n", speedangle*180/M_PI);

    if (car->_speed_x < MAX_UNSTUCK_SPEED ||
        car->_trkPos.toMiddle*speedangle > 0.0f)    // Speedvector points to the inside of the turn.
        return accel;

    if (seg->type == TR_STR)
    {
        float tm = fabs(car->_trkPos.toMiddle);
        float w = seg->width/WIDTHDIV;
        if (tm > w)
            //return MIN(0.3, accel);
            return accel/3;
        else
            return accel;
    }
    else
    {
        float sign = (seg->type == TR_RGT) ? -1 : 1;
        if (car->_trkPos.toMiddle*sign > 0.0)
            return accel;
        else
        {
            float tm = fabs(car->_trkPos.toMiddle);
            float w = seg->width/WIDTHDIV;
            if (tm > w)
                //return MIN(accel, 0.2);
                return accel;
            else
                return accel;
        }
    }
}

/* Brake filter for collision avoidance */
float Driver::filterBColl(float brake)
{
    float currentspeedsqr = car->_speed_x*car->_speed_x;
    float mu = car->_trkPos.seg->surface->kFriction;
    int i;

    for (i = 0; i < opponents->getNOpponents(); i++)
    {
        if (opponent[i].getState() & OPP_COLL)
        {
            float allowedspeedsqr = opponent[i].getSpeed();
            allowedspeedsqr *= allowedspeedsqr;
            float c = mu*G;
            float d = (CA*mu + CW)/mass;
            float v1sqr = currentspeedsqr;
            float v2sqr = allowedspeedsqr;
            float brakedist = -log((c + v2sqr*d)/(c + v1sqr*d))/(2.0*d);
            if (brakedist > opponent[i].getDistance())
            {
                return 1.0;
            }
        }
    }
    return brake;
}

