#ifndef SENSORSMAIN_H
#define SENSORSMAIN_H
#include <stdint.h>

extern bool rain_detected; //flag for if rain detected
extern volatile uint16_t whiper_speed_ms; //set whiper speed for a 180 degree movement in ms. e.g, 1000ms to do 180 degree

void sensors_run(void *pvParameters);

#endif // SENSORSMAIN_H