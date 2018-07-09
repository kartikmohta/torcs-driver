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

#include "linalg.h"
#include "learning.h"
#include "trajectory.h"


namespace kartik_ns
{
class Opponents;
class Opponent;

class Driver
    {
    public:
        Driver(int index);
        ~Driver();

        /* callback functions called from TORCS */
        void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
        void newRace(tCarElt* car, tSituation *s);
        void drive(tSituation *s);
        int pitCommand(tSituation *s);
        void endRace(tSituation *s);
        tCarElt *getCarPtr() { return car_; }
        tTrack *getTrackPtr() { return track_; }
        float getSpeed() { return speed_; }

    private:
        /* utility functions */
        bool isStuck();
        void update(tSituation *s);
        float getAllowedSpeed(tTrackSeg *segment, bool add_to_data = false);
        float getAccel();
        float getDistToSegEnd();
        float getBrake();
        int getGear();
        float getClutch();
        void initCa();
        void initCw();
        void initTireMu();
        void initTCLfilter();
        float filterABS(float brake);
        float filterTCL(float accel);
        float filterTCL_RWD();
        float filterTCL_FWD();
        float filterTCL_4WD();
        float (Driver::*GET_DRIVEN_WHEEL_SPEED)();
        float getSteer();
        v2d getTargetPoint();
        float filterTrk(float accel);
        float filterBColl(float brake);
        void updateAllowedSpeedFactor();

        /* per robot global data */
        int stuck_;
        float trackangle_;
        float angle_;
        float speedangle_;   /* the angle of the speed vector relative to trackangle, > 0.0 points to right */
        float mass_;         /* mass of car + fuel */
        float oldlookahead_;
        float clutchtime_;   ///< Clutch timer.
        float speed_;    /* speed in track direction */
        Opponents *opponents_;
        Opponent *opponent_;
        tdble prev_accel_cmd_;
        tdble prev_brake_cmd_;
        tTrackSeg *prev_seg_;

        /* track variables */
        tTrack* track_;
        Track track_info_, best_track_info_;
        TrainingData fann_data_;
        bool save_learned_;
        bool update_learned_;
        double prev_best_lap_time_;
        Trajectory traj_;

        /* data that should stay constant after first initialization */
        int MAX_UNSTUCK_COUNT;
        int INDEX;
        tCarElt *car_;   /* pointer to tCarElt struct */
        float CARMASS;  /* mass of the car only */
        float CA;       /* aerodynamic downforce coefficient */
        float CW;       /* aerodynamic drag coefficient */
        float TIREMU;

        /* class constants */
        static const float MAX_UNSTUCK_ANGLE;
        static const float UNSTUCK_TIME_LIMIT;
        static const float MAX_UNSTUCK_SPEED;
        static const float MIN_UNSTUCK_DIST;
        static const float G;
        static const float FULL_ACCEL_MARGIN;
        static const float SHIFT;
        static const float SHIFT_MARGIN;
        static const float ABS_SLIP;
        static const float ABS_RANGE;
        static const float ABS_MINSPEED;
        static const float TCL_RANGE;
        static const float TCL_SLIP;
        static const float TCL_MINSPEED;
        static const float LOOKAHEAD_CONST;
        static const float LOOKAHEAD_FACTOR;
        static const float WIDTHDIV;
        static const float MU_FACTOR;
        static const float MAX_FUEL_PER_METER;
        static const float CLUTCH_SPEED;
        static const float CLUTCH_FULL_MAX_TIME;
        static const float STEER_DIRECTION_GAIN;
        static const float STEER_DRIFT_GAIN;
        static const float STEER_PREDICT_GAIN;
        static const float STEER_AVOIDANCE_GAIN;
};
}

#endif // _DRIVER_H_
