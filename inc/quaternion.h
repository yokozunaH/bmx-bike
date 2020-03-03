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



        This file contains the same functionality as the original, but is compatible
        without a C++ compiler and is meant to be used with the BNO055 sensor driver
        provided by Bosch.
*/

#ifndef IMUMATH_QUATERNION_H
#define IMUMATH_QUATERNION_H

#include "bno055.h"
/// \brief Calculate the magnitude of a quaternion
/// \param *q: a pointer to a quaternion
/// \return the magnitude
float magnitude(struct bno055_quaternion_t *q);

/// \brief Normalize a quaternion
/// \param *q: a pointer to a quaternion
void normalize(struct bno055_quaternion_t *q);

/// \brief Scale down a quaternion through division
/// \param *q: a pointer to a quaternion
/// \param val: the number to divde by
void scale_divide(struct bno055_quaternion_t *q, float val);

/// \brief Calculate the corresponding euler angles of a quaternion
/// \param *q: a pointer to a quaternion
/// \return ea: a struct containing euler angles
struct bno055_euler_float_t toEuler(struct bno055_quaternion_t *q);

#endif
