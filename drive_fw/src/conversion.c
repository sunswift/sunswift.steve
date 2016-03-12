#include <scandal/uart.h>
#include <scandal/stdio.h>

#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/types.h>
#include <arch/uart.h>

#include <project/conversion.h>

/* Cruise Helpers */
float mps2kph(float mps) {
        return (mps * 3.6);
}

float mps2rpm(float mps, float wheel_diameter) {
        return 60.0 * (mps / (PI * wheel_diameter));
}

float rpm2mps(float rpm, float wheel_diameter) {
        return (rpm * (PI * wheel_diameter)) / 60;
}

float kph2mps(float kph) {
        return (kph / 3.6);
}