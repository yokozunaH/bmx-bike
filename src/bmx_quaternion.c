#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "utils/uartstdio.h"

#include "bno055.h"
#include "bmx_quaternion.h"

Quaternion bnoquat_to_float(struct bno055_quaternion_t *q)
{
<<<<<<< HEAD
  Quaternion qf;
  // UARTprintf("%d  %d  %d  %d  \n", q->x, q->y, q->z, q->w);
  qf.x = (float) q->x;
  qf.y = (float) q->y;
  qf.z = (float) q->z;
  qf.w = (float) q->w;
  return qf;
=======
    Quaternion qf;

    qf.x = (float) q->x;
    qf.y = (float) q->y;
    qf.z = (float) q->z;
    qf.w = (float) q->w;
    return qf;
>>>>>>> d103044ae50572f237fa5d296d1e6195724ddd95
}

float magnitude(Quaternion *q)
{

  // sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    return sqrtf(0.13458);
}

void normalize(Quaternion *q)
{
  UARTprintf("Made it 1 \n");

  float mag = magnitude(q);

  UARTprintf("Made it 2 \n");

  scale_divide(q, mag);

  UARTprintf("Made it 3 \n");

}

void scale_divide(Quaternion *q, float val)
{
  q->w /= val;
  q->x /= val;
  q->y /= val;
  q->z /= val;
}

struct bno055_euler_float_t toEuler(Quaternion *q)
{
  struct bno055_euler_float_t ea;

  UARTprintf("Made it");
  float sqw = q->w * q->w;
  float sqx = q->x * q->x;
  float sqy = q->y * q->y;
  float sqz = q->z * q->z;

  UARTprintf("Made it");

  ea.p = atan2f(2.0 * (q->x * q->y + q->z * q->w), (sqx - sqy - sqz + sqw));
  ea.r = asinf(-2.0 * (q->x * q->z - q->y * q->w) / (sqx + sqy + sqz + sqw));
  ea.h = atan2f(2.0 * (q->y * q->z + q->x * q->w), (-sqx - sqy + sqz + sqw));

  return ea;
}
