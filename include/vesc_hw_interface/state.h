#ifndef STATE_H
#define STATE_H

#include <string>


struct VescState{
    double position;
    double velocity;

    double duty;
    double current;

    // double amp_hours;
    // double amp_hours_chg;

    // double watt_hours;
    // double watt_hours_chg;

    double temp_fet;
    // double temp_motor;
    double in_current;

    double target_velocity;

};


#endif 