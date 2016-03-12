//conversion.h
#define PI 3.141592654

/* Cruise Helpers */
float mps2kph(float mps);
float mps2rpm(float mps, float wheel_diameter);
float rpm2mps(float rpm, float wheel_diameter);
float kph2mps(float kph);