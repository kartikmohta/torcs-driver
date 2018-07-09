#include "driver.h"
#include "opponent.h"
#include "learning.h"
#include <fstream>
#include <pwd.h>

namespace kartik_ns
{
const float Driver::MAX_UNSTUCK_ANGLE   = 30.0/180.0*PI;    /* [radians] */
const float Driver::UNSTUCK_TIME_LIMIT  = 1.5f;             /* [s] */
const float Driver::MAX_UNSTUCK_SPEED   = 5.0f;             /* [m/s] */
const float Driver::MIN_UNSTUCK_DIST    = 2.0f;             /* [m] */
const float Driver::G                   = 9.81f;            /* [m/(s*s)] */
const float Driver::FULL_ACCEL_MARGIN   = 3.0f;             /* [m/s] */
const float Driver::SHIFT               = 0.95;             /* [-] (% of rpmredline) */
const float Driver::SHIFT_MARGIN        = 4.0f;             /* [m/s] */
const float Driver::ABS_SLIP            = 2.0f;             /* [m/s] range [0..10] */
const float Driver::ABS_RANGE           = 5.0f;             /* [m/s] range [0..10] */
const float Driver::ABS_MINSPEED        = 3.0f;             /* [m/s] */
const float Driver::TCL_SLIP            = 2.0f;             /* [m/s] range [0..10] */
const float Driver::TCL_RANGE           = 10.0f;            /* [m/s] range [0..10] */
const float Driver::LOOKAHEAD_CONST     = 10;//15.0f;            /* [m] */
const float Driver::LOOKAHEAD_FACTOR    = 0.25;//0.33f;            /* [1/s] */
const float Driver::WIDTHDIV            = 2.5f;             /* [-] */
const float Driver::MU_FACTOR           = 0.65f;            /* [-] */
const float Driver::STEER_DIRECTION_GAIN = 1.0f;            // [-] Gain to apply for basic steerin
const float Driver::STEER_DRIFT_GAIN     = 0.01f;           // [-] Gain for drift compensation
const float Driver::STEER_PREDICT_GAIN   = 0.1f;            // [-] Predictive gain, multiplied with yaw rate.
const float Driver::STEER_AVOIDANCE_GAIN = 0.15f;           // [lograd/m] Gain for border avoidance steering
const float Driver::MAX_FUEL_PER_METER  = 0.00097f;         /* [liter/m] fuel consumtion */
const float Driver::CLUTCH_SPEED        = 5.0f;             /* [m/s] */
const float Driver::CLUTCH_FULL_MAX_TIME = 1.0f;            /* [s] Time to apply full clutch. */

Driver::Driver(int index)
{
  INDEX = index;
  stuck_ = 0;
  trackangle_ = 0;
  angle_ = 0;
  oldlookahead_ = 0;
  prev_accel_cmd_ = 0;
  prev_brake_cmd_ = 0;
  prev_best_lap_time_ = FLT_MAX;

  opponents_ = NULL;
  prev_seg_ = NULL;
}

Driver::~Driver()
{
  if(save_learned_)
  {
    printf("Saving learned data\n");
    int const BUFSIZE = 1024;
    char buffer[BUFSIZE];
    snprintf(buffer, BUFSIZE, "%s%s%d", GetLocalDir(), "drivers/kartik/",
             INDEX);

#ifdef ROB_SECT_ARBITRARY
    if(GfDirCreate(buffer) == GF_DIR_CREATED)
#else
    if(GfCreateDir(buffer) == GF_DIR_CREATED)
#endif
    {
      snprintf(buffer, BUFSIZE, "%s%s%d/%s-learned.dat", GetLocalDir(),
               "drivers/kartik/", INDEX, track_->internalname);
      best_track_info_.save(buffer);

      snprintf(buffer, BUFSIZE, "%s%s%d/%s-fann.dat", GetLocalDir(),
               "drivers/kartik/", INDEX, track_->internalname);
      // fann_data_.save(buffer);
    }
  }
  delete opponents_;
}

void parseTrack(const tTrack *t)
{
  tTrackSeg *seg = t->seg;

  // Switch seg to seg 0 for sure.
  while (seg->id != 0) {
    seg = seg->prev;
  }

  const char *homedir;
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }
  std::string filename = std::string(homedir) + "/" + std::string(t->internalname) + ".yaml";
  std::ofstream out_file(filename.c_str());
  for(int seg_idx = 0; seg_idx < t->nseg; ++seg_idx)
  {
    out_file << "- id: " << seg->id << std::endl;
#define OUTPUT_SEG_FIELD(x) out_file << "  "#x": " << seg->x << std::endl;
    OUTPUT_SEG_FIELD(type);
    OUTPUT_SEG_FIELD(length);
    OUTPUT_SEG_FIELD(width);
    OUTPUT_SEG_FIELD(startWidth);
    OUTPUT_SEG_FIELD(endWidth);
    OUTPUT_SEG_FIELD(radius);
    OUTPUT_SEG_FIELD(radiusr);
    OUTPUT_SEG_FIELD(radiusl);
    OUTPUT_SEG_FIELD(arc);
    for(int i = 0; i < 4; ++i)
    {
      out_file << "  vertex" << i << ": [" << seg->vertex[i].x << ", " <<
          seg->vertex[i].y << ", " << seg->vertex[i].z << "]" << std::endl;
    }
#undef OUTPUT_SEG_FIELD

    seg = seg->next;
  }
  out_file.close();
}

/* Called for every track change or new race. */
void Driver::initTrack(tTrack * t, void *carHandle, void **carParmHandle,
                       tSituation *s)
{
  track_ = t;
  // parseTrack(t);
  struct timespec ts1, ts2;
  clock_gettime(CLOCK_MONOTONIC, &ts1);
  tdble trk_width = t->width;
  tdble car_width =
      2.0 * GfParmGetNum(carHandle, SECT_CAR, PRM_WIDTH, (char *)NULL,
                       2.0f); // Factor 4 for safety since we cut corners
  if(car_width > 0.5 * trk_width)
    car_width = 0.5 * trk_width;
  printf("car width: %f\n", car_width);
  traj_.processTrack(t, car_width / (2 * trk_width),
                     1 - (car_width / (2 * trk_width)));
  clock_gettime(CLOCK_MONOTONIC, &ts2);
  printf("processTrack dt: %f sec\n",
         (ts2.tv_sec - ts1.tv_sec) + (ts2.tv_nsec - ts1.tv_nsec) / 1e9);

  track_info_.init(t);

  const int BUFSIZE = 1024;
  char fname[BUFSIZE];
  snprintf(fname, BUFSIZE, "%s%s%d/%s-learned.dat", GetLocalDir(),
           "drivers/kartik/", INDEX, t->internalname);

  // Load learned params
  switch(s->_raceType)
  {
    case RM_TYPE_PRACTICE:
      best_track_info_.init(t);
      save_learned_ = true;
      break;
    case RM_TYPE_QUALIF:
      best_track_info_ = Track::load(fname, t);
      track_info_ = best_track_info_;
      save_learned_ = true;
      break;
    case RM_TYPE_RACE:
      best_track_info_ = Track::load(fname, t);
      track_info_ = best_track_info_;
      save_learned_ = false;
      break;
    default:
      save_learned_ = false;
      break;
  }

  snprintf(fname, BUFSIZE, "drivers/kartik/%d/default.xml", INDEX);
  *carParmHandle = GfParmReadFile(fname, GFPARM_RMODE_STD);

  float fuel = track_->length * MAX_FUEL_PER_METER;
  fuel *= (s->_totLaps + 0.25f);
  GfParmSetNum(*carParmHandle, SECT_CAR, PRM_FUEL, (char *)NULL, fuel);
}

/* Start a new race. */
void Driver::newRace(tCarElt* car, tSituation *s)
{
  MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
  stuck_ = 0;
  this->car_ = car;
  CARMASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);
  initCa();
  initCw();
  initTCLfilter();
  initTireMu();

  /* initialize the list of opponents */
  opponents_ = new Opponents(s, this);
  opponent_ = opponents_->getOpponentPtr();
}

/* Drive during race. */
void Driver::drive(tSituation *s)
{
  update(s);

  memset(&car_->ctrl, 0, sizeof(tCarCtrl));
  if (isStuck())
  {
    car_->ctrl.steer = -angle_ / car_->_steerLock;
    car_->ctrl.gear = -1; // reverse gear
    car_->ctrl.accelCmd = filterTCL(1);
    car_->ctrl.brakeCmd = 0.0; // no brakes
  }
  else
  {
    car_->ctrl.steer = getSteer();
    car_->ctrl.gear = getGear();

    const tdble brake_alpha = 0.2;
    const tdble current_brake = filterABS(filterBColl(getBrake()));
    if(current_brake > prev_brake_cmd_)
      car_->ctrl.brakeCmd =
          brake_alpha * current_brake + (1 - brake_alpha) * prev_brake_cmd_;
    else
      car_->ctrl.brakeCmd = current_brake;

    if (car_->ctrl.brakeCmd == 0.0)
    {
      const tdble alpha = 0.2;
      const tdble current_accel = filterTCL(filterTrk(getAccel()));
      if(current_accel > prev_accel_cmd_ && car_->_speed_x > 15)
        car_->ctrl.accelCmd =
            alpha * current_accel + (1 - alpha) * prev_accel_cmd_;
      else
        car_->ctrl.accelCmd = current_accel;
    }
    else
    {
      car_->ctrl.accelCmd = 0.0;
    }
    if(car_->_bestLapTime > 0 && car_->_bestLapTime < prev_best_lap_time_)
    {
      best_track_info_ = track_info_;
      prev_best_lap_time_ = car_->_bestLapTime;
      printf("Best lap time: %f\n", prev_best_lap_time_);
    }
    if(save_learned_)
      updateAllowedSpeedFactor();
    prev_accel_cmd_ = car_->ctrl.accelCmd;
    prev_brake_cmd_ = car_->ctrl.brakeCmd;
    car_->_clutchCmd = getClutch();
  }
  prev_seg_ = car_->_trkPos.seg;
}

/* Set pitstop commands. */
int Driver::pitCommand(tSituation *s)
{
  return ROB_PIT_IM; /* return immediately */
}

/* End of the current race */
void Driver::endRace(tSituation *s)
{
  printf("endRace\n");
}

/* Update my private data every timestep */
void Driver::update(tSituation *s)
{
  trackangle_ = RtTrackSideTgAngleL(&(car_->_trkPos));
  angle_ = trackangle_ - car_->_yaw;
  NORM_PI_PI(angle_);

  speedangle_ = trackangle_ - atan2(car_->_speed_Y, car_->_speed_X);
  NORM_PI_PI(speedangle_);

  mass_ = CARMASS + car_->_fuel;

  speed_ = Opponent::getSpeed(car_);
  opponents_->update(s, this);
}

/* Check if I'm stuck */
bool Driver::isStuck()
{
  if (fabs(angle_) > MAX_UNSTUCK_ANGLE &&
      car_->_speed_x < MAX_UNSTUCK_SPEED &&
      fabs(car_->_trkPos.toMiddle) > MIN_UNSTUCK_DIST)
  {
    if (stuck_ > MAX_UNSTUCK_COUNT && car_->_trkPos.toMiddle*angle_ < 0.0)
    {
      return true;
    }
    else
    {
      stuck_++;
      return false;
    }
  }
  else
  {
    stuck_ = 0;
    return false;
  }
}

/* Compute the allowed speed on a segment */
float Driver::getAllowedSpeed(tTrackSeg *segment, bool add_to_data)
{
  if (segment->type == TR_STR)
  {
    return FLT_MAX;
  }
  else
  {
    const float mu = segment->surface->kFriction*TIREMU*MU_FACTOR;

    float arc = 0.0;
    float bankangle = 0.0;
    float length = 0.0;
    tTrackSeg *s = segment;

    bankangle = (s->angle[TR_XS] + s->angle[TR_XE])/2.0;

    if(s->type == TR_LFT)
      bankangle = -bankangle;

    float min_radius = 1000;
    while (s->type == segment->type)
    {
      arc += s->arc;
      length += s->length;
      min_radius = (min_radius < s->radius) ? min_radius : s->radius;
      if(arc > M_PI/2)
        break;
      s = s->next;
    }

    const float R = length/arc;
    // printf("R: %f, min_radius: %f\n", R, min_radius);
#if 0
    float arc_limit = 2*acos((R - s->width/WIDTHDIV)/(R + s->width/WIDTHDIV));
    if(segment->radius < 20)
    {
      //float arc_limit2 = 2*acos((min_radius - s->width/WIDTHDIV)/(min_radius + s->width/WIDTHDIV));
      //printf("length: %f, arc: %f, arc_limit: %f, arc_limit2: %f\n", length, arc*180/M_PI, arc_limit*180/M_PI, arc_limit2*180/M_PI);
    }
    if(R > 10*segment->width)
    {
      //if(segment->radius < 20)
      //printf("1\n");
      //return FLT_MAX;
    }
    if(arc < 0.5*arc_limit)
    {
      //printf("arc < 0.65*arc_limit\n");
      return FLT_MAX;
    }
#endif

    float r = R - (segment->width / 2 - car_->_dimension_y / 2) / 5;

#if 0
    //printf("arc: %f\n", arc*180/M_PI);

    //if(arc > M_PI/2)
    //  arc = M_PI/2;

    if(R*cos(arc/2) - r == 0)
    {
      //if(segment->radius < 20)
      //  printf("2\n");
      return FLT_MAX;
    }
    if((R*R - r*r)/(2*(r - R*cos(arc/2))) > R) // Check!
    {
      //if(segment->radius < 20)
      // printf("3\n");
      return FLT_MAX;
    }

    r = (R*R - 2*R*r*cos(arc/2) + r*r)/(2*(r - R*cos(arc/2)));
    //printf("r: %f\n", r);
    if(r <= 0)
    {
      //if(segment->radius < 20)
      // printf("4\n");
      return FLT_MAX;
    }
#endif
    r = (R < 200) ? min_radius: R;
    float banknum = sin(bankangle) + mu*cos(bankangle);
    float bankden = cos(bankangle) - mu*sin(bankangle);
    float allowedSpeed = sqrt((G*r*banknum)/(1.0 - MIN(1.0 - 1e-6, r*CA*mu/mass_/bankden))/bankden);
    // printf("allowedspeed: %f, R: %f, min_radius: %f, r: %f\n", allowedSpeed, R, min_radius, r);
    // fann_data_.addData(bankangle, length, arc,
    //                    segment->width - car_->_dimension_y, mass_,
    //                    allowedSpeed);
    tdble const allowed_speed_factor =
        track_info_.getAllowedSpeedFactor(segment->id);
    return allowed_speed_factor * allowedSpeed;
  }
}

/* Compute the length to the end of the segment */
float Driver::getDistToSegEnd()
{
  if (car_->_trkPos.seg->type == TR_STR)
  {
    return car_->_trkPos.seg->length - car_->_trkPos.toStart;
  }
  else
  {
    return (car_->_trkPos.seg->arc - car_->_trkPos.toStart)*car_->_trkPos.seg->radius;
  }
}

/* Compute fitting acceleration */
float Driver::getAccel()
{
  float allowedspeed = getAllowedSpeed(car_->_trkPos.seg, true);
  float gr = car_->_gearRatio[car_->_gear + car_->_gearOffset];
  float rm = car_->_enginerpmRedLine;
  float front_side_slip = (car_->_wheelSlipSide(FRNT_LFT) + car_->_wheelSlipSide(FRNT_RGT))/2.0;
  float rear_side_slip = (car_->_wheelSlipSide(REAR_LFT) + car_->_wheelSlipSide(REAR_RGT))/2.0;
  //printf("front side slip: %f\n", front_side_slip);
  //printf("rear side slip: %f\n", rear_side_slip);
  float accel_scale = 1.0;
#if 1
  tdble max_front_slip = 8, max_rear_slip = 11;
  if(std::abs(front_side_slip) > max_front_slip)
    accel_scale *= max_front_slip / std::abs(front_side_slip);
  if(std::abs(rear_side_slip) > max_rear_slip)
    accel_scale *= max_rear_slip / std::abs(rear_side_slip);
  accel_scale *= accel_scale;
#endif

  if (allowedspeed > car_->_speed_x + FULL_ACCEL_MARGIN)
  {
    return accel_scale*1.0;
  }
  else
  {
    return accel_scale*allowedspeed/car_->_wheelRadius(REAR_RGT)*gr /rm;
  }
}

float Driver::getBrake()
{
  // Car drives backward?
  if (car_->_speed_x < -MAX_UNSTUCK_SPEED)
  {
    // Yes, brake.
    return 1;
  }
  else
  {
    tTrackSeg *segptr = car_->_trkPos.seg;
    const float currentspeedsqr = car_->_speed_x*car_->_speed_x;
    //const float mu = segptr->surface->kFriction;
    const float mu = segptr->surface->kFriction*TIREMU*MU_FACTOR;
    float maxlookaheaddist = currentspeedsqr/(2.0*mu*G);
    float lookaheaddist = getDistToSegEnd();
    float allowedspeed = getAllowedSpeed(segptr);

    if (allowedspeed < car_->_speed_x)
      return MIN(1.0f, (car_->_speed_x-allowedspeed)/(FULL_ACCEL_MARGIN));

    segptr = segptr->next;
    while (lookaheaddist < maxlookaheaddist)
    {
      allowedspeed = getAllowedSpeed(segptr);
      //printf("car_ speed: %f, allowedspeed: %f, seg radius: %f\n", car_->_speed_x, allowedspeed, segptr->radius);
      if (allowedspeed < car_->_speed_x)
      {
        float allowedspeedsqr = allowedspeed*allowedspeed;
        float c = mu*G;
        float d = (CA*mu + CW)/mass_;
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
  if (car_->_gear <= 0)
    return 1;

  float gr_up = car_->_gearRatio[car_->_gear + car_->_gearOffset];
  float omega = car_->_enginerpmRedLine/gr_up;
  float wr = car_->_wheelRadius(2);

  if (omega*wr*SHIFT < car_->_speed_x)
  {
    return car_->_gear + 1;
  }
  else
  {
    float gr_down = car_->_gearRatio[car_->_gear + car_->_gearOffset - 1];
    omega = car_->_enginerpmRedLine/gr_down;
    if ( (car_->_gear > 1) && (omega*wr*SHIFT > car_->_speed_x + SHIFT_MARGIN) )
      return car_->_gear - 1;
  }
  return car_->_gear;
}

/* Compute aerodynamic downforce coefficient CA */
void Driver::initCa()
{
  const char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
  float rearwingarea = GfParmGetNum(car_->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*) NULL, 0.0f);
  float rearwingangle = GfParmGetNum(car_->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*) NULL, 0.0f);
  float frontwingarea = GfParmGetNum(car_->_carHandle, SECT_FRNTWING, PRM_WINGAREA, (char*) NULL, 0.0f);
  float frontwingangle = GfParmGetNum(car_->_carHandle, SECT_FRNTWING, PRM_WINGANGLE, (char*) NULL, 0.0f);
  float wingca = 1.23f*(rearwingarea*sin(rearwingangle) + frontwingarea*sin(frontwingangle));

  float cl = GfParmGetNum(car_->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*) NULL, 0.0) +
      GfParmGetNum(car_->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*) NULL, 0.0);
  float h = 0.0;
  for (int i = 0; i < 4; i++)
    h += GfParmGetNum(car_->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*) NULL, 0.20f);
  h*= 1.5; h = h*h; h = h*h; h = 2.0 * exp(-3.0*h);
  CA = h*cl + 4.0f*wingca;
  //printf("CA: %f\n", CA);
}

/* Compute aerodynamic drag coefficient CW */
void Driver::initCw()
{
  float cx = GfParmGetNum(car_->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*) NULL, 0.4f);
  float frontarea = GfParmGetNum(car_->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*) NULL, 2.5);
  float rearwingarea = GfParmGetNum(car_->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*) NULL, 0.0f);
  float rearwingangle = GfParmGetNum(car_->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*) NULL, 0.0f);
  float frontwingarea = GfParmGetNum(car_->_carHandle, SECT_FRNTWING, PRM_WINGAREA, (char*) NULL, 0.0f);
  float frontwingangle = GfParmGetNum(car_->_carHandle, SECT_FRNTWING, PRM_WINGANGLE, (char*) NULL, 0.0f);
  float wingca = 1.23f*(rearwingarea*sin(rearwingangle) + frontwingarea*sin(frontwingangle));
  CW = 0.645f * cx * frontarea + wingca;
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
    tm += GfParmGetNum(car_->_carHandle, WheelSect[i], PRM_MU, (char*) NULL, 1.7f);
  }
  TIREMU = tm/4.0;
}

/* Antilocking filter for brakes */
float Driver::filterABS(float brake)
{
  //If we are too slow don't consider ABS (division by zero).
  if (car_->_speed_x < ABS_MINSPEED)
    return brake;

#if 1
  //Compute the average slip of the four wheels.
  int i;
  float slip = 0.0;
  for (i = 0; i < 4; i++)
  {
    slip += car_->_wheelSpinVel(i) * car_->_wheelRadius(i);
  }
  slip = slip/4.0;
#else
  //Compute the average slip of the front wheels.
  int i;
  float slip = 0.0;
  for (i = 0; i < 2; i++)
  {
    slip += car_->_wheelSpinVel(i) * car_->_wheelRadius(i);
  }
  slip = slip/2.0;
#endif
  //printf("skid: F: %f, %f, R: %f, %f\n", car_->_skid[0],car_->_skid[1],car_->_skid[2],car_->_skid[3]);
  slip = car_->_speed_x - slip;
  if (slip > ABS_SLIP)
  {
    brake = brake - MIN(brake, (slip - ABS_SLIP)/ABS_RANGE);
  }
  return brake;
}

/* TCL filter for accelerator pedal */
float Driver::filterTCL(float accel)
{
#if 0
  float slip = (this->*GET_DRIVEN_WHEEL_SPEED)() - car_->_speed_x;
  if(fabs(car_->_trkPos.toMiddle) > car_->_trkPos.seg->width/2.0)
    slip += TCL_SLIP; // Force trigger TC

  if (slip > TCL_SLIP)
  {
    accel = accel - MIN(accel, (slip - TCL_SLIP)/TCL_RANGE);
  }
  return accel;
#else
  // From Olethros
  static float TCL_status = 0.0;
  float slip = (this->*GET_DRIVEN_WHEEL_SPEED)() - car_->_speed_x;
  TCL_status = 0.75 * TCL_status;
  if (TCL_status < 0.1) TCL_status = 0.0;
  if (slip > TCL_SLIP) {
    TCL_status +=  0.5 * (slip - TCL_SLIP)/TCL_RANGE;
  }
  accel = accel - MIN(accel, TCL_status);
  return accel;
#endif
}

/* Traction Control (TCL) setup */
void Driver::initTCLfilter()
{
  const char *traintype = GfParmGetStr(car_->_carHandle, SECT_DRIVETRAIN, PRM_TYPE, (char*)VAL_TRANS_RWD);
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
  return (car_->_wheelSpinVel(REAR_RGT) + car_->_wheelSpinVel(REAR_LFT)) *
      car_->_wheelRadius(REAR_LFT) / 2.0;
}


/* TCL filter plugin for front wheel driven cars */
float Driver::filterTCL_FWD()
{
  return (car_->_wheelSpinVel(FRNT_RGT) + car_->_wheelSpinVel(FRNT_LFT)) *
      car_->_wheelRadius(FRNT_LFT) / 2.0;
}


/* TCL filter plugin for all wheel driven cars */
float Driver::filterTCL_4WD()
{
  return ((car_->_wheelSpinVel(FRNT_RGT) + car_->_wheelSpinVel(FRNT_LFT)) *
          car_->_wheelRadius(FRNT_LFT) / 4.0) +
      ((car_->_wheelSpinVel(REAR_RGT) + car_->_wheelSpinVel(REAR_LFT)) *
       car_->_wheelRadius(REAR_LFT) / 4.0);
}

v2d Driver::getTargetPoint()
{
  tTrackSeg *seg = car_->_trkPos.seg;

  float lookahead = LOOKAHEAD_CONST + car_->_speed_x*LOOKAHEAD_FACTOR;
  float length = getDistToSegEnd();

  // Prevent "snap back" of lookahead on harsh braking.
  float cmplookahead = oldlookahead_ - car_->_speed_x*RCM_MAX_DT_ROBOTS;
  if (lookahead < cmplookahead)
  {
    lookahead = cmplookahead;
  }

  oldlookahead_ = lookahead;

  // Search for the segment containing the target point.
  while (length < lookahead)
  {
    seg = seg->next;
    length += seg->length;
  }

  length = lookahead - length + seg->length;
  return traj_.getPoint(seg->id, length);

#if 0
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
    v2d c, n_start, n_end, n;
    n_start.x = (seg->vertex[TR_SR].x - seg->vertex[TR_SL].x)/seg->startWidth;
    n_start.y = (seg->vertex[TR_SR].y - seg->vertex[TR_SL].y)/seg->startWidth;
    n_end.x = (seg->vertex[TR_ER].x - seg->vertex[TR_EL].x)/seg->endWidth;
    n_end.y = (seg->vertex[TR_ER].y - seg->vertex[TR_EL].y)/seg->endWidth;

    float arc_done = 0, arc_left = 0, total_arc = 0;
    tTrackSeg* checkseg = seg->prev;
    while (checkseg->type == seg->type)
    {
      arc_done += checkseg->arc;
      checkseg = checkseg->prev;
    }
    checkseg = seg->next;
    while (checkseg->type == seg->type)
    {
      arc_left += checkseg->arc;
      checkseg = checkseg->next;
    }

    float alpha = length/seg->length;
    arc_done += alpha*seg->arc;
    arc_left += (1 - alpha)*seg->arc;
    total_arc = arc_done + seg->arc + arc_left;

    n.x = n_start.x*(1-alpha) + n_end.x*alpha;
    n.y = n_start.y*(1-alpha) + n_end.y*alpha;

    c.x = seg->center.x;
    c.y = seg->center.y;

    float arc = length/seg->radius;
    float arcsign = (seg->type == TR_RGT) ? -1 : 1;
    arc = arc*arcsign;
    n = arcsign*n;
    alpha = 2*((arc_left < arc_done) ? arc_left : arc_done)/total_arc;
    //printf("arc_left: %f, arc_done: %f, total_arc: %f, alpha: %f\n", arc_left, arc_done, total_arc, alpha);
    float allowed_movein_dist = alpha*0.9*(seg->width/2 - car_->_dimension_y/2)/*sqrt(car_->_dimension_y*car_->_dimension_y + car_->_dimension_x*car_->_dimension_x)/2*/;
    if(seg->radius < 500)
      return (s-n*allowed_movein_dist).rotate(c, arc);
    else
      return s.rotate(c, arc);
  }
#endif
}

float Driver::getSteer()
{
  float targetAngle;
  v2d target = getTargetPoint();
  float avoidance = 0;
  if(car_->_trkPos.toRight < car_->_dimension_y)
  {
    avoidance = tanh(STEER_AVOIDANCE_GAIN *
                     (car_->_dimension_y - car_->_trkPos.toRight));
  }
  else if(car_->_trkPos.toLeft < car_->_dimension_y)
  {
    avoidance = tanh(STEER_AVOIDANCE_GAIN *
                     (car_->_trkPos.toLeft - car_->_dimension_y));
  }

  targetAngle = atan2(target.y - car_->_pos_Y, target.x - car_->_pos_X);

  // from Olethros
  float angle_error = targetAngle - car_->_yaw;
  NORM_PI_PI(angle_error);
  float steer_direction = STEER_DIRECTION_GAIN *
                          (angle_error - STEER_PREDICT_GAIN * car_->_yaw_rate);
  float correct_drift =
      -STEER_DRIFT_GAIN * atan2(car_->_speed_y, car_->_speed_x);

  NORM_PI_PI(steer_direction);
  return avoidance + correct_drift + steer_direction/car_->_steerLock;
}

/* Copied from Olethros */
float Driver::getClutch()
{
  if (car_->_gear > 1) {
    clutchtime_ = 0.0;
    return 0.0;
  } else {
    float drpm = car_->_enginerpm - car_->_enginerpmRedLine/2.0;
    clutchtime_ = MIN(CLUTCH_FULL_MAX_TIME, clutchtime_);
    float clutcht = (CLUTCH_FULL_MAX_TIME - clutchtime_)/CLUTCH_FULL_MAX_TIME;
    if (car_->_gear == 1 && car_->_accelCmd > 0.0) {
      clutchtime_ += (float) RCM_MAX_DT_ROBOTS;
    }

    if (drpm > 0) {
      float speedr;
      if (car_->_gearCmd == 1) {
        // Compute corresponding speed to engine rpm.
        float omega = car_->_enginerpmRedLine/car_->_gearRatio[car_->_gear + car_->_gearOffset];
        float wr = car_->_wheelRadius(2);
        speedr = (CLUTCH_SPEED + MAX(0.0, car_->_speed_x))/fabs(wr*omega);
        float clutchr = MAX(0.0, (1.0 - speedr*2.0*drpm/car_->_enginerpmRedLine));
        return MIN(clutcht, clutchr);
      } else {
        // For the reverse gear.
        clutchtime_ = 0.0;
        return 0.0;
      }
    } else {
      return clutcht;
    }
  }
}

/* Hold car_ on the track */
float Driver::filterTrk(float accel)
{
  tTrackSeg* seg = car_->_trkPos.seg;
  //printf("Speedangle: %f\n", speedangle_*180/M_PI);

  if (car_->_speed_x < MAX_UNSTUCK_SPEED ||
      car_->_trkPos.toMiddle*speedangle_ > 0.0f)    // Speedvector points to the inside of the turn.
    return accel;

  if (seg->type == TR_STR)
  {
    float tm = fabs(car_->_trkPos.toMiddle);
    float const allowed_width = (seg->width - car_->_dimension_y) / 2;
    if(tm > 0.99 * allowed_width)
      //return MIN(0.3, accel);
      return accel/2;
    else
      return accel;
  }
  else
  {
    float sign = (seg->type == TR_RGT) ? -1 : 1;
    if (car_->_trkPos.toMiddle*sign > 0.0)
      return accel;
    else
    {
      float tm = fabs(car_->_trkPos.toMiddle);
      float w = seg->width/WIDTHDIV;
      if (tm > w)
        //return MIN(accel, 0.2);
        return accel/3;
      else
        return accel;
    }
  }
}

/* Brake filter for collision avoidance */
float Driver::filterBColl(float brake)
{
  float currentspeedsqr = car_->_speed_x*car_->_speed_x;
  //float mu = car_->_trkPos.seg->surface->kFriction;
  const float mu = car_->_trkPos.seg->surface->kFriction*TIREMU*MU_FACTOR;
  int i;

  for (i = 0; i < opponents_->getNOpponents(); i++)
  {
    if (opponent_[i].getState() & OPP_COLL)
    {
      float allowedspeedsqr = opponent_[i].getSpeed();
      allowedspeedsqr *= allowedspeedsqr;
      float c = mu*G;
      float d = (CA*mu + CW)/mass_;
      float v1sqr = currentspeedsqr;
      float v2sqr = allowedspeedsqr;
      float brakedist = -log((c + v2sqr*d)/(c + v1sqr*d))/(2.0*d);
      if (brakedist > opponent_[i].getDistance())
      {
        return 1.0;
      }
    }
  }
  return brake;
}

void Driver::updateAllowedSpeedFactor()
{
  tTrackSeg *const seg = car_->_trkPos.seg;
  if(seg == prev_seg_)
    return;

  //printf("updateAllowedSpeedFactor, seg->id: %d, seg->type: %d\n", seg->id,
  //       seg->type);

  tdble const front_side_slip = std::abs(car_->_wheelSlipSide(FRNT_LFT) +
                                         car_->_wheelSlipSide(FRNT_RGT)) /
                                2.0;
  tdble const rear_side_slip = std::abs(car_->_wheelSlipSide(REAR_LFT) +
                                        car_->_wheelSlipSide(REAR_RGT)) /
                               2.0;
  tdble const slip = std::max(front_side_slip, rear_side_slip);
  //printf("slip: %f\n", slip);

  //const tdble allowed_speed_factor =
  //    track_info_.getAllowedSpeedFactor(seg->id);
  //if(allowed_speed_factor < 0.5 || allowed_speed_factor > 1.5)
  //  return;

  float const tm = fabs(car_->_trkPos.toMiddle);
  float const allowed_width = (seg->width - car_->_dimension_y) / 2;
  //printf("tm/w: %f\n", tm / allowed_width);
  int increase_decrease = 0;

  if (seg->type == TR_STR)
  {
    if(tm > allowed_width)
      increase_decrease = -1;
    else if(tm <= allowed_width)
      increase_decrease = 1;
    else
      increase_decrease = 0;
  }
  else
  {
    float sign = (seg->type == TR_RGT) ? -1 : 1;
    if (car_->_trkPos.toMiddle*sign > 0.0)
      increase_decrease = 1;
    else
    {
      if(tm > 0.9*allowed_width)
        increase_decrease = -1;
      else if(tm <= 0.9 * allowed_width)
        increase_decrease = 1;
      else
        increase_decrease = 0;
    }
    if(tm > 1.2 * allowed_width)
      increase_decrease = -1;
  }

  tdble factor = 1.0;

  tdble const min_slip = 10, max_slip = 15;
  if(slip > max_slip ||
     //(increase_decrease < 0 && car_->ctrl.brakeCmd < 0.2 && slip > min_slip))
     //(car_->ctrl.brakeCmd < 0.2 && slip > min_slip))
     (car_->ctrl.accelCmd > 0.2 && slip > min_slip))
  {
    increase_decrease = -2;
    factor =
        1 - 0.1 * std::min(1.0f, (slip - min_slip) / (max_slip - min_slip));
  }
  tTrackSeg *s = seg;
  tdble lookback_dist = LOOKAHEAD_CONST + car_->_speed_x*LOOKAHEAD_FACTOR;
  tdble accum_length = 0;
  int count = 0;
  do
  {
    s = s->prev;
    accum_length += s->length;
    count++;
  }
  while(accum_length < lookback_dist);
  //printf("lookback, num_seg: %d\n", count);
  //for(int i = 0; i < 2; ++i)
  //  s = s->prev;
  if(increase_decrease < 0)
  {
    // printf("seg_id: %d, speed_factor: %f, Decrease, seg_type: %d, "
    //        "increase_decrease: %d\n",
    //        seg->id, track_info_.getAllowedSpeedFactor(seg->id), seg->type,
    //        increase_decrease);

    if(increase_decrease == -1) // Use default factor
      factor = 0.95;
    tdble max_dist = 50;
    tdble accum_dist = 0;
    do
    {
      float cur_factor = (factor + (1 - factor) * accum_dist / max_dist);
      if(track_info_.getAllowedSpeedFactor(s->id) > 0.1 / cur_factor)
        track_info_.changeAllowedSpeedFactor(s->id, cur_factor);
      s = s->prev;
      accum_dist += s->length;
    } while(accum_dist < max_dist);
  }
  else if(increase_decrease > 0)
  {
    // printf("seg_id: %d, speed_factor: %f, Increase, seg_type: %d, "
    //       "increase_decrease: %d\n",
    //       seg->id, track_info_.getAllowedSpeedFactor(seg->id), seg->type,
    //       increase_decrease);
    float const factor = 1.02;
    tdble max_dist = 25;
    tdble accum_dist = 0;
    do
    {
      float cur_factor = (factor + (1 - factor) * accum_dist / max_dist);
      if(track_info_.getAllowedSpeedFactor(s->id) < 10.0 / cur_factor)
        track_info_.changeAllowedSpeedFactor(s->id, cur_factor);
      s = s->prev;
      accum_dist += s->length;
    } while(accum_dist < max_dist);
  }
}
}
