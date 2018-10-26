#include <math.h>
#include <stdint.h>

// Fox explanation of algorithm please see Example section at
// https://en.wikipedia.org/wiki/Mean_of_circular_quantities

#define ANGLE_ARRAY_SIZE 40
float angleArray[ANGLE_ARRAY_SIZE];

// Next free position
uint32_t angle_array_position = 0;
uint32_t angle_array_items = 0;

#define M_PI 3.14159265
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

void angle_average_add(float angle)
{
    angleArray[angle_array_position] = angle;

    angle_array_position++;
    angle_array_position = angle_array_position % ANGLE_ARRAY_SIZE;

    // Increment until buffer is full
    if(angle_array_items < ANGLE_ARRAY_SIZE)
        angle_array_items++;
}


float angle_average_get()
{
    uint32_t i;

    float s = 0; // Sum of sines
    float c = 0; // Sum of cosines

    for(i = 0; i < angle_array_items; i++)
    {
        s += sin(degreesToRadians(angleArray[i]));
        c += cos(degreesToRadians(angleArray[i]));
    }

    // Divide by number of items
    s /= angle_array_items;
    c /= angle_array_items;

    float average = radiansToDegrees(atan(s / c));

    if(s > 0 && c > 0)
    {

    }
    if(c < 0)
    {
        average += 180.0f;
    }
    else if(s < 0 && c > 0)
    {
        average += 360.0f;
    }

    return average;
}
