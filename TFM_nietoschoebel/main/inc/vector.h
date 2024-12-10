/*
 * vector.h
 *
 *  Created on: 22 oct. 2020
 *      Author: sreal
 */

#ifndef MAIN_VECTOR_H_
#define MAIN_VECTOR_H_

typedef struct vector {
	float x;
	float y;
	float z;
} vector;

#define VECTOR_UP		(vector) { .x = 0,  .y = 1,  .z = 0 }
#define VECTOR_DOWN		(vector) { .x = 0,  .y = -1, .z = 0 }
#define VECTOR_LEFT 	(vector) { .x = 1,  .y = 0,  .z = 0 }
#define VECTOR_RIGHT 	(vector) { .x = -1, .y = 0,  .z = 0 }
#define VECTOR_FORWARD 	(vector) { .x = 0,  .y = 0,  .z = 1 }
#define VECTOR_BACK 	(vector) { .x = 0,  .y = 0,  .z = -1}
#define VECTOR_ZERO		(vector) { .x = 0,  .y = 0,  .z = 0}

//vector vectorInit(float x, float y, float z);
vector vector_normalize(vector a);
vector vector_crossProduct(vector a, vector b);
float vector_scalarProduct(vector a, vector b);
float vector_module(vector a);


#endif /* MAIN_VECTOR_H_ */
