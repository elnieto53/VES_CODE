/*
 * vector.c
 *
 *  Created on: 22 oct. 2020
 *      Author: sreal
 */

#include "vector.h"
#include "math.h"


vector vector_normalize(vector a){
	vector retval;
	float mod = vector_module(a);
//	if(mod != 0){
		retval.x = a.x/mod;
		retval.y = a.y/mod;
		retval.z = a.z/mod;
		return retval;
//	}else{
//		return (vector) {.x = 0, .y = 0, .z = 0 };
//	}
}

vector vector_crossProduct(vector a, vector b){
	vector retval;
	retval.x = a.y*b.z - a.z*b.y;
	retval.y = a.z*b.x - a.x*b.z;
	retval.z = a.x*b.y - a.y*b.x;

	return retval;
}

float vector_scalarProduct(vector a, vector b){
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

float vector_module(vector a){
	return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}
