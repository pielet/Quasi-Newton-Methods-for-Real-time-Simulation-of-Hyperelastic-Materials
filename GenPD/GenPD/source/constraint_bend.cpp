#include "constraint.h"

const int test_size = 2;
int test_p0[test_size]{ 8, 5 };
int test_p1[test_size]{ 9, 6 };

ScalarType adjustAngle(ScalarType theta, ScalarType refer_theta)
{
	if (theta - refer_theta > M_PI)
		theta -= 2 * M_PI;
	else if (refer_theta - theta > M_PI)
		theta += 2 * M_PI;
	return theta;
}

ScalarType dihedralAngle(const EigenVector3& p1, const EigenVector3& p2, const EigenVector3& p3, const EigenVector3& p4, ScalarType refer_theta)
{
	EigenVector3 e = (p2 - p1).normalized();
	EigenVector3 n1 = e.cross(p3 - p1).normalized();
	EigenVector3 n2 = (p4 - p1).cross(e).normalized();

	ScalarType sin = n1.cross(n2).dot(e);
	ScalarType cos = n1.dot(n2);

	//if (refer_theta > EPSILON) {
	//	printf("e: (%f, %f, %f), n1: (%f, %f, %f), n2: (%f, %f, %f)\n", e(0), e(1), e(2), n1(0), n1(1), n1(2), n2(0), n2(1), n2(2));
	//	printf("p0: (%f, %f, %f), p1: (%f, %f, %f), p2: (%f, %f, %f), p3: (%f, %f, %f)\n", p1(0), p1(1), p1(2), p2(0), p2(1), p2(2), p3(0), p3(1), p3(2), p4(0), p4(1), p4(2), p4(3));
	//}

	//return adjustAngle(atan2(sin, cos), refer_theta);  // cause turning over in theory
	return atan2(sin, cos);
}

ScalarType distance(const EigenVector3& x, const EigenVector3& a, const EigenVector3& b)
{
	EigenVector3 e = b - a;
	EigenVector3 xp = e.dot(x - a) / e.squaredNorm() * e;
	return std::max((x - a - xp).norm(), EPSILON);
}

EigenVector2 barycentricWeights(const EigenVector3& x, const EigenVector3& a, const EigenVector3& b)
{
	EigenVector3 e = b - a;
	ScalarType t = e.dot(x - a) / e.squaredNorm();
	return EigenVector2(1 - t, t);
}

int BendConstraint::m_hessian_triple_count = 4 * 4 * 3 * 3;

BendConstraint::BendConstraint(unsigned int p1, unsigned int p2, unsigned int p3, unsigned int p4, const std::vector<EigenVector2>& uv, const VectorX& x):
	Constraint(CONSTRAINT_TYPE_BEND)
{
	m_p[0] = p1;
	m_p[1] = p2;
	m_p[2] = p3;
	m_p[3] = p4;

	EigenVector3 x1 = x.block_vector(p1);
	EigenVector3 x2 = x.block_vector(p2);
	EigenVector3 x3 = x.block_vector(p3);
	EigenVector3 x4 = x.block_vector(p4);

	m_rest_theta = dihedralAngle(x1, x2, x3, x4, 0);
	m_l = (x2 - x1).norm();
	m_duv = uv[p2] - uv[p1];
	EigenVector2 area;
	area[0] = (x3 - x1).cross(x2 - x1).norm();
	area[1] = (x4 - x1).cross(x2 - x1).norm();
	m_height_inv = m_l / (area[0] + area[1]);

	EigenVector2 h;
	h[0] = area[0] / m_l;
	h[1] = area[1] / m_l;
	EigenVector4 K;
	EigenVector2 w_f1 = barycentricWeights(x3, x1, x2), w_f2 = barycentricWeights(x4, x1, x2);
	K[0] = w_f1[0] / h[0] + w_f2[0] / h[1];
	K[1] = w_f1[1] / h[0] + w_f2[1] / h[1];
	K[2] = -1. / h[0];
	K[3] = -1. / h[1];
	K *= m_l;
	EigenMatrix4 KKT = K * K.transpose();
	m_Q = 6 / (area[0] + area[1]) * kronecker<4, 4, 3, 3>(KKT, EigenMatrix3::Identity());
}

void BendConstraint::EvaluateGradientAndHessian(const VectorX& x, bool definiteness_fix)
{
	EigenVector3 x1 = x.block_vector(m_p[0]);
	EigenVector3 x2 = x.block_vector(m_p[1]);
	EigenVector3 x3 = x.block_vector(m_p[2]);
	EigenVector3 x4 = x.block_vector(m_p[3]);

	ScalarType h1 = distance(x3, x1, x2), h2 = distance(x4, x1, x2);
	EigenVector2 w_f1 = barycentricWeights(x3, x1, x2), w_f2 = barycentricWeights(x4, x1, x2);
	EigenVector3 n1 = (x2 - x1).cross(x3 - x1).normalized(),
				 n2 = (x4 - x1).cross(x2 - x1).normalized();

	ScalarType theta = dihedralAngle(x1, x2, x3, x4, m_rest_theta);

	ScalarType curv = theta * m_height_inv;
	ScalarType k;
	if (m_material_type == MATERIAL_TYPE_DATA_DRIVEN)
		k = std::min(m_material->Kb(curv, m_duv, 0), m_material->Kb(curv, m_duv, 1));
	else k = m_bending_stiffness;

	EigenVector12 dtheta;
	dtheta.block_vector(0) = -(w_f1[0] * n1 / h1 + w_f2[0] * n2 / h2);
	dtheta.block_vector(1) = -(w_f1[1] * n1 / h1 + w_f2[1] * n2 / h2);
	dtheta.block_vector(2) = n1 / h1;
	dtheta.block_vector(3) = n2 / h2;
	dtheta = -dtheta; // theta increase direction

	//m_g = k * m_l * m_height_inv * (theta - m_rest_theta) * dtheta;
	//m_H = k * m_l * m_height_inv * dtheta * dtheta.transpose();

	EigenVector12 local_x;
	local_x.block_vector(0) = x1;
	local_x.block_vector(1) = x2;
	local_x.block_vector(2) = x3;
	local_x.block_vector(3) = x4;
	m_g = k * m_Q * local_x;
	m_H = k * m_Q;

	//for (int i = 0; i < test_size; ++i)
	//{
	//	if ((m_p[0] == test_p0[i] && m_p[1] == test_p1[i]) || (m_p[0] == test_p1[i] && m_p[1] == test_p0[i])) {
	//		printf("edge (%i -> %i, %i, %i) theta %f, rest_theta %f\n", m_p[0], m_p[1], m_p[2], m_p[3], dihedralAngle(x1, x2, x3, x4, 1) / M_PI * 180, m_rest_theta);
	//		for (int i = 0; i < 4; ++i)
	//			printf("{%f, %f, %f} ", dtheta(3 * i), dtheta(3 * i + 1), dtheta(3 * i + 2));
	//		std::cout << std::endl;
	//	}
	//}
}

void BendConstraint::GetGradientAndHessian(VectorX& gradient, std::vector<SparseMatrixTriplet>& hessian_triplets)
{
	for (unsigned int i = 0; i < 4; i++)
	{
		gradient.block_vector(m_p[i]) += m_g.block_vector(i);

		for (unsigned int j = 0; j < 4; j++)
		{
			for (unsigned int row = 0; row < 3; row++)
			{
				for (unsigned int col = 0; col < 3; col++)
				{
					hessian_triplets.push_back(SparseMatrixTriplet(m_p[i] * 3 + row, m_p[j] * 3 + col, m_H(i * 3 + row, j * 3 + col)));
				}
			}
		}
	}
}

void BendConstraint::EvaluateGradientAndHessian(const VectorX& x, VectorX& gradient, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix)
{
	EvaluateGradientAndHessian(x, definiteness_fix);
	GetGradientAndHessian(gradient, hessian_triplets);
}

ScalarType BendConstraint::EvaluateEnergy(const VectorX& x)
{
	EigenVector3 x1 = x.block_vector(m_p[0]);
	EigenVector3 x2 = x.block_vector(m_p[1]);
	EigenVector3 x3 = x.block_vector(m_p[2]);
	EigenVector3 x4 = x.block_vector(m_p[3]);

	ScalarType theta = dihedralAngle(x1, x2, x3, x4, m_rest_theta);
	ScalarType curv = theta * m_height_inv;
	ScalarType k;
	if (m_material_type == MATERIAL_TYPE_DATA_DRIVEN)
		k = std::min(m_material->Kb(curv, m_duv, 0), m_material->Kb(curv, m_duv, 1));
	else k = m_bending_stiffness;

	//m_energy = 0.5 * k * m_l * m_height_inv * (theta - m_rest_theta) * (theta - m_rest_theta);	// FIXME: not sure about the actual energy

	EigenVector12 local_x;
	local_x.block_vector(0) = x1;
	local_x.block_vector(1) = x2;
	local_x.block_vector(2) = x3;
	local_x.block_vector(3) = x4;
	m_energy = 0.5 * k * local_x.transpose() * m_Q * local_x;

	return m_energy;
}

ScalarType BendConstraint::EvaluateEnergyAndGradient(const VectorX& x)
{
	EigenVector3 x1 = x.block_vector(m_p[0]);
	EigenVector3 x2 = x.block_vector(m_p[1]);
	EigenVector3 x3 = x.block_vector(m_p[2]);
	EigenVector3 x4 = x.block_vector(m_p[3]);

	ScalarType h1 = distance(x3, x1, x2), h2 = distance(x4, x1, x2);
	EigenVector2 w_f1 = barycentricWeights(x3, x1, x2), w_f2 = barycentricWeights(x4, x1, x2);
	EigenVector3 n1 = (x2 - x1).cross(x3 - x1).normalized(),
		n2 = (x4 - x1).cross(x2 - x1).normalized();

	ScalarType theta = dihedralAngle(x1, x2, x3, x4, m_rest_theta);
	ScalarType curv = theta * m_height_inv;
	ScalarType k;
	if (m_material_type == MATERIAL_TYPE_DATA_DRIVEN)
		k = std::min(m_material->Kb(curv, m_duv, 0), m_material->Kb(curv, m_duv, 1));
	else k = m_bending_stiffness;

	EigenVector12 dtheta;
	dtheta.block_vector(0) = -(w_f1[0] * n1 / h1 + w_f2[0] * n2 / h2);
	dtheta.block_vector(1) = -(w_f1[1] * n1 / h1 + w_f2[1] * n2 / h2);
	dtheta.block_vector(2) = n1 / h1;
	dtheta.block_vector(3) = n2 / h2;
	dtheta = -dtheta; // theta increase direction

	//m_g = k * m_l * m_height_inv * (theta - m_rest_theta) * dtheta;
	//m_energy = 0.5 * k * m_l * m_height_inv * (theta - m_rest_theta) * (theta - m_rest_theta);

	EigenVector12 local_x;
	local_x.block_vector(0) = x1;
	local_x.block_vector(1) = x2;
	local_x.block_vector(2) = x3;
	local_x.block_vector(3) = x4;
	m_g = k * m_Q * local_x;
	m_energy = 0.5 * k * local_x.transpose() * m_Q * local_x;

	return m_energy;
}

ScalarType BendConstraint::GetEnergyAndGradient(VectorX& gradient)
{
	for (unsigned int i = 0; i < 4; i++)
	{
		gradient.block_vector(m_p[i]) += m_g.block_vector(i);
	}
	return m_energy;
}

ScalarType BendConstraint::EvaluateEnergyAndGradient(const VectorX& x, VectorX& gradient)
{
	EvaluateEnergyAndGradient(x);
	return GetEnergyAndGradient(gradient);
}

void BendConstraint::EvaluateHessian(const VectorX& x, bool definiteness_fix)
{
	EigenVector3 x1 = x.block_vector(m_p[0]);
	EigenVector3 x2 = x.block_vector(m_p[1]);
	EigenVector3 x3 = x.block_vector(m_p[2]);
	EigenVector3 x4 = x.block_vector(m_p[3]);

	ScalarType h1 = distance(x3, x1, x2), h2 = distance(x4, x1, x2);
	EigenVector2 w_f1 = barycentricWeights(x3, x1, x2), w_f2 = barycentricWeights(x4, x1, x2);
	EigenVector3 n1 = (x2 - x1).cross(x3 - x1).normalized(),
		n2 = (x4 - x1).cross(x2 - x1).normalized();

	ScalarType theta = dihedralAngle(x1, x2, x3, x4, m_rest_theta);
	ScalarType curv = theta * m_height_inv;
	ScalarType k;
	if (m_material_type == MATERIAL_TYPE_DATA_DRIVEN)
		k = std::min(m_material->Kb(curv, m_duv, 0), m_material->Kb(curv, m_duv, 1));
	else k = m_bending_stiffness;

	EigenVector12 dtheta;
	dtheta.block_vector(0) = -(w_f1[0] * n1 / h1 + w_f2[0] * n2 / h2);
	dtheta.block_vector(1) = -(w_f1[1] * n1 / h1 + w_f2[1] * n2 / h2);
	dtheta.block_vector(2) = n1 / h1;
	dtheta.block_vector(3) = n2 / h2;
	dtheta = -dtheta; // theta increase direction

	EigenVector12 local_x;
	local_x.block_vector(0) = x1;
	local_x.block_vector(1) = x2;
	local_x.block_vector(2) = x3;
	local_x.block_vector(3) = x4;
	m_H = k * m_Q;
	//m_H = k * m_l * m_height_inv * dtheta * dtheta.transpose();
}

void BendConstraint::EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix)
{
	EvaluateHessian(x, definiteness_fix);
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			for (unsigned int row = 0; row < 3; row++)
			{
				for (unsigned int col = 0; col < 3; col++)
				{
					hessian_triplets.push_back(SparseMatrixTriplet(m_p[i] * 3 + row, m_p[j] * 3 + col, m_H(i * 3 + row, j * 3 + col)));
				}
			}
		}
	}
}

void BendConstraint::ApplyHessian(const VectorX& x, VectorX& b)
{
	EigenVector12 x_blocks;
	for (unsigned int i = 0; i != 4; i++)
	{
		x_blocks.block_vector(i) = x.block_vector(m_p[i]);
	}
	EigenVector12 b_blocks = m_H * x_blocks;
	for (unsigned int i = 0; i != 4; i++)
	{
		b.block_vector(m_p[i]) += b_blocks.block_vector(i);
	}
}
