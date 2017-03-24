/**
 * @file Matrix.hpp
 *
 * Matrix class
 */

#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <stdio.h>
#include <math.h>

#include "Vector.hpp"
#include "SquareMatrix.hpp"

namespace math
{

#ifndef M_PI_2_F
    #define M_PI_2_F  1.57079632679489661923f
#endif

template<unsigned int M, unsigned int N>
class Matrix;

// MxN matrix with float elements
template <unsigned int M, unsigned int N>
class MatrixBase
{
public:
	/**
	 * matrix data[row][col]
	 */
	float data[M][N];

	/**
	 * struct for using arm_math functions
	 */
	eigen_matrix_instance arm_mat;

	/**
	 * trivial ctor
	 * Initializes the elements to zero.
	 */
	MatrixBase() : 
		data(),
		arm_mat(M, N, &data[0][0])
	{
	}

	//virtual ~MatrixBase() {};

	/**
	 * copyt ctor
	 */
	MatrixBase(const MatrixBase<M, N> &m) :
		arm_mat(M, N, &data[0][0])
	{
		memcpy(data, m.data, sizeof(data));
	}

	MatrixBase(const float *d) :
		arm_mat(M, N, &data[0][0])
	{
		memcpy(data, d, sizeof(data));
	}

	MatrixBase(const float d[M][N]) : 
		arm_mat(M, N, &data[0][0])
	{
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set data
	 */
	void set(const float *d) {
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set data
	 */
	void set(const float d[M][N]) {
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set row from vector
	 */
	void set_row(unsigned int row, const Vector<N> v) {
		for (unsigned i = 0; i < N; i++) {
			data[row][i] = v.data[i];
		}
	}

	/**
	 * set column from vector
	 */
	void set_col(unsigned int col, const Vector<M> v) {
		for (unsigned i = 0; i < M; i++) {
			data[i][col] = v.data[i];
		}
	}

	/**
	 * access by index
	 */
	float &operator()(const unsigned int row, const unsigned int col) {
		return data[row][col];
	}

	/**
	 * access by index
	 */
	float operator()(const unsigned int row, const unsigned int col) const {
		return data[row][col];
	}

	/**
	 * get rows number
	 */
	unsigned int get_rows() const {
		return M;
	}

	/**
	 * get columns number
	 */
	unsigned int get_cols() const {
		return N;
	}

	/**
	 * test for equality
	 */
	bool operator ==(const Matrix<M, N> &m) const {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				if (data[i][j] != m.data[i][j])
					return false;

		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const Matrix<M, N> &m) const {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				if (data[i][j] != m.data[i][j])
					return true;

		return false;
	}

	/**
	 * set to value
	 */
	const Matrix<M, N> &operator =(const Matrix<M, N> &m) {
		memcpy(data, m.data, sizeof(data));
		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * negation
	 */
	Matrix<M, N> operator -(void) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = -data[i][j];

		return res;
	}

	/**
	 * addition
	 */
	Matrix<M, N> operator +(const Matrix<M, N> &m) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] + m.data[i][j];

		return res;
	}

	Matrix<M, N> &operator +=(const Matrix<M, N> &m) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] += m.data[i][j];

		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * subtraction
	 */
	Matrix<M, N> operator -(const Matrix<M, N> &m) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] - m.data[i][j];

		return res;
	}

	Matrix<M, N> &operator -=(const Matrix<M, N> &m) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] -= m.data[i][j];

		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * uniform scaling
	 */
	Matrix<M, N> operator *(const float num) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] * num;

		return res;
	}

	Matrix<M, N> &operator *=(const float num) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] *= num;

		return *static_cast<Matrix<M, N>*>(this);
	}

	Matrix<M, N> operator /(const float num) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] / num;

		return res;
	}

	Matrix<M, N> &operator /=(const float num) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] /= num;

		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * multiplication by another matrix
	 */
	template <unsigned int P>
	Matrix<M, P> operator *(const Matrix<N, P> &m) const {
		matrix::Matrix<float, M, N> Me(this->arm_mat.pData);
		matrix::Matrix<float, N, P> Him(m.arm_mat.pData);
		matrix::Matrix<float, M, P> Product = Me * Him;
		Matrix<M, P> res(Product.data());
		return res;
	}

	/**
	 * transpose the matrix
	 */
	Matrix<N, M> transposed(void) const {
		matrix::Matrix<float, N, M> Me(this->arm_mat.pData);
		Matrix<N, M> res(Me.transpose().data());
		return res;
	}

	/**
	 * invert the matrix
	 */
	Matrix<M, N> inversed(void) const {
		matrix::SquareMatrix<float, M> Me = matrix::Matrix<float, M, N>(this->arm_mat.pData);
		Matrix<M, N> res(Me.I().data());
		return res;
	}

	/**
	 * set zero matrix
	 */
	void zero(void) {
		memset(data, 0, sizeof(data));
	}

	/**
	 * set identity matrix
	 */
	void identity(void) {
		memset(data, 0, sizeof(data));
		unsigned int n = (M < N) ? M : N;

		for (unsigned int i = 0; i < n; i++)
			data[i][i] = 1;
	}

	void print(void) {
		for (unsigned int i = 0; i < M; i++) {
			printf("[ ");

			for (unsigned int j = 0; j < N; j++)
				printf("%.3f\t", (double)data[i][j]);

			printf(" ]\n");
		}
	}
};

template <unsigned int M, unsigned int N>
class Matrix : public MatrixBase<M, N>
{
public:
	using MatrixBase<M, N>::operator *;

	Matrix() : MatrixBase<M, N>() {}

	Matrix(const Matrix<M, N> &m) : MatrixBase<M, N>(m) {}

	Matrix(const float *d) : MatrixBase<M, N>(d) {}

	Matrix(const float d[M][N]) : MatrixBase<M, N>(d) {}

	/**
	 * set to value
	 */
	const Matrix<M, N> &operator =(const Matrix<M, N> &m) {
		memcpy(this->data, m.data, sizeof(this->data));
		return *this;
	}

	/**
	 * multiplication by a vector
	 */
	Vector<M> operator *(const Vector<N> &v) const {
		matrix::Matrix<float, M, N> Me(this->arm_mat.pData);
		matrix::Matrix<float, N, 1> Vec(v.arm_col.pData);
		matrix::Matrix<float, M, 1> Product = Me * Vec;
		Vector<M> res(Product.data());
		return res;
	}
};

template <>
class Matrix<3, 3> : public MatrixBase<3, 3>
{
public:
	using MatrixBase<3, 3>::operator *;

	Matrix() : MatrixBase<3, 3>() {}

	Matrix(const Matrix<3, 3> &m) : MatrixBase<3, 3>(m) {}

	Matrix(const float *d) : MatrixBase<3, 3>(d) {}

	Matrix(const float d[3][3]) : MatrixBase<3, 3>(d) {}
	/**
	 * set data
	 */
	void set(const float d[9]) {
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set to value
	 */
	const Matrix<3, 3> &operator =(const Matrix<3, 3> &m) {
		memcpy(this->data, m.data, sizeof(this->data));
		return *this;
	}

	/**
	 * multiplication by a vector
	 */
	Vector<3> operator *(const Vector<3> &v) const {
		Vector<3> res(data[0][0] * v.data[0] + data[0][1] * v.data[1] + data[0][2] * v.data[2],
			      data[1][0] * v.data[0] + data[1][1] * v.data[1] + data[1][2] * v.data[2],
			      data[2][0] * v.data[0] + data[2][1] * v.data[1] + data[2][2] * v.data[2]);
		return res;
	}

	/**
	 * create a rotation matrix from given euler angles
	 * based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
	 */
	void from_euler(float roll, float pitch, float yaw) {
		float cp = cosf(pitch);
		float sp = sinf(pitch);
		float sr = sinf(roll);
		float cr = cosf(roll);
		float sy = sinf(yaw);
		float cy = cosf(yaw);

		data[0][0] = cp * cy;
		data[0][1] = (sr * sp * cy) - (cr * sy);
		data[0][2] = (cr * sp * cy) + (sr * sy);
		data[1][0] = cp * sy;
		data[1][1] = (sr * sp * sy) + (cr * cy);
		data[1][2] = (cr * sp * sy) - (sr * cy);
		data[2][0] = -sp;
		data[2][1] = sr * cp;
		data[2][2] = cr * cp;
	}

	/**
	 * get euler angles from rotation matrix
	 */
	Vector<3> to_euler(void) const {
		Vector<3> euler;
		euler.data[1] = asinf(-data[2][0]);

		if (fabsf(euler.data[1] - M_PI_2_F) < 1.0e-3f) {
			euler.data[0] = 0.0f;
			euler.data[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) + euler.data[0];

		} else if (fabsf(euler.data[1] + M_PI_2_F) < 1.0e-3f) {
			euler.data[0] = 0.0f;
			euler.data[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) - euler.data[0];

		} else {
			euler.data[0] = atan2f(data[2][1], data[2][2]);
			euler.data[2] = atan2f(data[1][0], data[0][0]);
		}

		return euler;
	}
};

}

#endif // MATRIX_HPP
