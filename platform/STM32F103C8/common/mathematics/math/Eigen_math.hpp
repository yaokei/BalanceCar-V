/*
 * eigen_math.h
 *
 */

#ifndef EIGEN_MATH_H_
#define EIGEN_MATH_H_

// struct for using arm_math functions, represents column vector
struct eigen_matrix_instance {
	int numRows;
	int numCols;
	float *pData;
    
    eigen_matrix_instance(int r, int c, float *p) {
        numRows = r;
        numCols = c;
        pData = p;
    }
};

#endif /* EIGEN_MATH_H_ */
