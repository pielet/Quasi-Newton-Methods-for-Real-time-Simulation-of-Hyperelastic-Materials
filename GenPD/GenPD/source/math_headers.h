// ---------------------------------------------------------------------------------//
// Copyright (c) 2013, Regents of the University of Pennsylvania                    //
// All rights reserved.                                                             //
//                                                                                  //
// Redistribution and use in source and binary forms, with or without               //
// modification, are permitted provided that the following conditions are met:      //
//     * Redistributions of source code must retain the above copyright             //
//       notice, this list of conditions and the following disclaimer.              //
//     * Redistributions in binary form must reproduce the above copyright          //
//       notice, this list of conditions and the following disclaimer in the        //
//       documentation and/or other materials provided with the distribution.       //
//     * Neither the name of the <organization> nor the                             //
//       names of its contributors may be used to endorse or promote products       //
//       derived from this software without specific prior written permission.      //
//                                                                                  //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND  //
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED    //
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           //
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY               //
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES       //
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     //
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND      //
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       //
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS    //
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                     //
//                                                                                  //
// Contact Tiantian Liu (ltt1598@gmail.com) if you have any questions.              //
//----------------------------------------------------------------------------------//

#ifndef _MATH_HEADERS_H_
#define _MATH_HEADERS_H_

// eigen
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Sparse"

// glm
#include "glm.hpp"
#include "gtc\matrix_transform.hpp"
#include <gtx\color_space.hpp>

// global header
#include "global_headers.h"

// std numeric header
#include <limits>

const ScalarType M_PI = 3.14159265358979323846;
const ScalarType inf = std::numeric_limits<ScalarType>::infinity();

template <typename T> 
T clamp(const T& x, const T& a, const T& b) {
	return std::min(std::max(x, a), b);
}

template <typename T>
T Atan2(T y, T x);

// eigen vectors and matrices
typedef int IndexType;
typedef Eigen::Matrix<ScalarType, 12, 12, 0, 12, 12> EigenMatrix12;
typedef Eigen::Matrix<ScalarType, 12, 1, 0, 12, 1> EigenVector12;
typedef Eigen::Matrix<ScalarType, 4, 4, 0, 4, 4> EigenMatrix4;
typedef Eigen::Matrix<ScalarType, 4, 1, 0, 4, 1> EigenVector4;
typedef Eigen::Matrix<ScalarType, 3, 3, 0, 3, 3> EigenMatrix3;
typedef Eigen::Matrix<ScalarType, 3, 1, 0, 3 ,1> EigenVector3;
typedef Eigen::Matrix<ScalarType, 2, 2, 0, 2 ,2> EigenMatrix2;
typedef Eigen::Matrix<ScalarType, 2, 1, 0, 2 ,1> EigenVector2;
typedef Eigen::Matrix<ScalarType, -1, 3, 0, -1 ,3> EigenMatrixx3;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> VectorX;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> Matrix;
typedef Eigen::SparseMatrix<ScalarType> SparseMatrix;
typedef Eigen::Triplet<ScalarType,IndexType> SparseMatrixTriplet;

typedef Eigen::Matrix<ScalarType, 3, 2> EigenMatrix3x2;
typedef Eigen::Matrix<ScalarType, 2, 3> EigenMatrix2x3;
typedef Eigen::Matrix<ScalarType, 9, 9> EigenMatrix9;
typedef Eigen::Matrix<ScalarType, 3, 9> EigenMatrix3x9;
typedef Eigen::Matrix<ScalarType, 9, 1> EigenVector9;

// eigen quaternions
typedef Eigen::AngleAxis<ScalarType> EigenAngleAxis;
typedef Eigen::Quaternion<ScalarType, Eigen::DontAlign> EigenQuaternion;

// eigen vector accessor
#define block_vector(a) block<3,1>(3*(a), 0)
#define block_mat(i, j) block<3, 3>(3 * i, 3 * j)

template <int m, int n, int p, int q>
Eigen::Matrix<ScalarType, m* p, n* q> kronecker(const Eigen::Matrix<ScalarType, m, n>& A, const Eigen::Matrix<ScalarType, p, q>& B) {
	Eigen::Matrix<ScalarType, m* p, n* q> C;
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			for (int k = 0; k < p; k++)
				for (int l = 0; l < q; l++)
					C(i * p + k, j * q + l) = A(i, j) * B(k, l);
	return C;
}

// eigen 2 glm, glm 2 eigen
glm::vec3 Eigen2GLM(const EigenVector3& eigen_vector);
EigenVector2 GLM2Eigen(const glm::vec2& glm_vec);
EigenVector3 GLM2Eigen(const glm::vec3& glm_vector);
void Eigen2GLM(const VectorX& eigen_vector, std::vector<glm::vec3>& glm_vector);
void GLM2Eigen(const std::vector<glm::vec3>& glm_vector, VectorX& eigen_vector);

// eigen make sparse identity
void EigenMakeSparseIdentityMatrix(unsigned int rows, unsigned int cols, SparseMatrix& I);

// eigen sparse matrix to triplets
void EigenSparseMatrixToTriplets(const SparseMatrix& A, std::vector<SparseMatrixTriplet>& A_triplets);

void EigenExtractDiagonalOffDiagonal(const SparseMatrix& A, VectorX& D, SparseMatrix& OD);
void EigenExtractTriangular(const SparseMatrix& A, SparseMatrix& DL, SparseMatrix& U);

// for subspace usage
void EigenExtractCols(Matrix& dst, const Matrix& src, const std::vector<unsigned int>& indices);
void EigenVectorExtractElements(VectorX& dst, const VectorX& src, const std::vector<unsigned int>& indices);
void EigenExtractCols(Matrix& dst, const Matrix& src, unsigned int d, bool from_front_end = true);
void EigenVectorExtractElements(VectorX& dst, const VectorX& src, unsigned int d, bool from_front_end = true);

// for matlab usage
#ifndef HIGH_PRECISION
void EigenSparseMatrixToTriplets(const SparseMatrix& A, std::vector<Eigen::Triplet<double, int>>& A_triplets);
void EigenVectorSingleToDouble(const Eigen::VectorXf& src, Eigen::VectorXd& dst);
#endif //HIGH_PRECISION

class QueueLBFGS
{
public:
	QueueLBFGS(unsigned int vector_size, unsigned int queue_size);
	~QueueLBFGS();

	inline int capacity() { return m_capacity; }
	inline bool isFull() { return m_is_full; }

	int size();
	void enqueue(const VectorX& sk, const VectorX& yk);
	void dequeue();

	void visitSandY(ScalarType** s, ScalarType** y, int i);

protected:
	inline void checkEmpty() { m_is_empty = (m_head_pointer == m_tail_pointer) ? true : false; }
	inline void checkFull() { m_is_full = (m_head_pointer == m_tail_pointer) ? true : false; }

protected:
	ScalarType* m_data_s;
	ScalarType* m_data_y;

	bool m_is_empty;
	bool m_is_full;

	int m_vector_size;
	int m_capacity;
	int m_head_pointer;
	int m_tail_pointer;
};

#endif