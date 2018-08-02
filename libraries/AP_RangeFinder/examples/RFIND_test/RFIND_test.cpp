/*
 *  RangeFinder test code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_SerialManager serial_manager;
static RangeFinder sonar {serial_manager, ROTATION_PITCH_270};

void setup()
{
    // print welcome message
    hal.console->printf("Range Finder library test V3HP\n");

    // setup for analog pin 13
    AP_Param::set_object_value(&sonar, sonar.var_info, "_TYPE", RangeFinder::RangeFinder_TYPE_PLI2CV3HP);
    AP_Param::set_object_value(&sonar, sonar.var_info, "_PIN", -1.0f);
    AP_Param::set_object_value(&sonar, sonar.var_info, "_SCALING", 1.0f);

    // initialise sensor, delaying to make debug easier
    hal.scheduler->delay(2000);
    sonar.init();
    hal.console->printf("RangeFinder: %d devices detected\n", sonar.num_sensors());
}

void loop()
{
    // Delay between reads
    hal.scheduler->delay(100);
    sonar.update();

    hal.console->printf("distance_cm %d %d\r\n",
    sonar.distance_cm(0),sonar.distance_cm(1));

}
AP_HAL_MAIN();
