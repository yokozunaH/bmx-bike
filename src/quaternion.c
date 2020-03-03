/*
    This library is based on the quaternion.h file included in:

    Inertial Measurement Unit Maths Library
    Copyright (C) 2013-2014  Samuel Cowen

        Disclaimer for the Inertial Measurement Unit Maths Library:

        Bug fixes and cleanups by Gé Vissers (gvissers@gmail.com)

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see <http://www.gnu.org/licenses/>.


    This file contains the select functionality as the original, but is compatible
    without a C++ compiler and is meant to be used with the BNO055 sensor driver
    provided by Bosch.
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "bno055.h"
#include "quaternion.h"


float magnitude(struct bno055_quaternion_t *q)
{
    return sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
}

void normalize(struct bno055_quaternion_t *q)
{

  float mag = magnitude(q);
  scale_divide(q, mag);
}

void scale_divide(struct bno055_quaternion_t *q, float val)
{
  q->w /= val;
  q->x /= val;
  q->y /= val;
  q->z /= val;
}

struct bno055_euler_float_t toEuler(struct bno055_quaternion_t *q)
{
  struct bno055_euler_float_t ea;

  float sqw = q->w * q->w;
  float sqx = q->x * q->x;
  float sqy = q->y * q->y;
  float sqz = q->z * q->z;

  ea.p = atan2(2.0 * (q->x * q->y + q->z * q->w), (sqx - sqy - sqz + sqw));
  ea.r = asin(-2.0 * (q->x * q->z - q->y * q->w) / (sqx + sqy + sqz + sqw));
  ea.h = atan2(2.0 * (q->y * q->z + q->x * q->w), (-sqx - sqy + sqz + sqw));

  return ea;
}
