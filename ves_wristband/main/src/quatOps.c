/*
 * Copyright (c) 2020, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the B105 Electronic Systems Lab.
 * 4. Neither the name of the B105 Electronic Systems Lab nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY UNIVERSITY AND CONTRIBUTORS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * quatOps.c
 *
 *  Created on: Apr 2, 2020
 *      Author: Manuel Simal Perez <msimal@b105.upm.es>, Santiago Real Valdes <sreal@b105.upm.es>
 */

#include "quatOps.h"
#include "stdio.h"
#include <math.h>


quaternion ToQuaternion(vector dir){ // yaw (Z), pitch (Y), roll (X)
    // Abbreviations for the various angular functions
    float cy = cos(dir.x / 2);
    float sy = sin(dir.x / 2);
    float cp = cos(dir.y / 2);
    float sp = sin(dir.y / 2);
    float cr = cos(dir.z / 2);
    float sr = sin(dir.z / 2);

    quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


//vector ToEulerAngles(quaternion q) {
//	vector angles;
//
//    // roll (x-axis rotation)
//    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
//    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
//    angles.x = atan2(sinr_cosp, cosr_cosp);
//
//    // pitch (y-axis rotation)
//    double sinp = 2 * (q.w * q.y - q.z * q.x);
//    if (abs(sinp) >= 1)
//        angles.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//    else
//        angles.y = asin(sinp);
//
//    // yaw (z-axis rotation)
//    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
//    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
//    angles.z = atan2(siny_cosp, cosy_cosp);
//
//    return angles;
//}

quaternion conjugateQ(quaternion q1){
	return (quaternion) { .w = q1.w, .x = -q1.x, .y = -q1.y, .z = -q1.z };
}

quaternion normaliseQ(quaternion q1){
	float mod = sqrt(q1.x*q1.x + q1.y*q1.y + q1.z*q1.z + q1.w*q1.w);

	return (quaternion) { .w = q1.w/mod, .x = q1.x/mod, .y = q1.y/mod, .z = q1.z/mod };
}

quaternion multiplicationQ(quaternion q1, quaternion q2){
	quaternion q;

	q.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
	q.x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
	q.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
	q.z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;

	return q;
}

quaternion rotationQ(quaternion q, quaternion p0){
	quaternion qC;

	qC = conjugateQ(q);

	quaternion q1 = multiplicationQ(qC, p0);
	quaternion q2 = multiplicationQ(q1, q);

	return q2;
}

quaternion fromToRotationQ(vector a, vector b){
	quaternion retval;
	vector axis;
	float angle;
	float s;

	axis = vector_normalize(vector_crossProduct(a, b));
	angle = acos(vector_scalarProduct(a, b));
	s = sin(angle/2);

	retval.x = axis.x*s;
	retval.y = axis.y*s;
	retval.z = axis.z*s;
	retval.w = cos(angle/2);

	return retval;
}


vector transformDirection(vector dir, quaternion rotation){
	quaternion retvalQuat;
	quaternion dirQuat = { .w = 0, .x = dir.x, .y = dir.y, .z = dir.z };

	retvalQuat = multiplicationQ(rotation, dirQuat);
	retvalQuat = multiplicationQ(retvalQuat, conjugateQ(rotation));

	return (vector) { .x = retvalQuat.x, .y = retvalQuat.y, .z = retvalQuat.z };
}

