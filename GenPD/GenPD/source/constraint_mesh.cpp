// Author: Shiyang Jia
// Date: 11/14/2020
// 
// 3D embedding of 2D surface

#include "constraint.h"

const ScalarType smallest_lambda = 1e-6;

//int test_p0[6]{ 106, 106, 107, 108, 108, 108 };
//int test_p1[6]{ 117, 118, 118, 118, 120, 119 };
//int test_p2[6]{ 118, 107, 108, 119, 109, 120 };

int MeshConstraint::m_hessian_triple_count = 9 * 9;

MeshConstraint::MeshConstraint(unsigned int p1, unsigned int p2, unsigned int p3, const std::vector<EigenVector2>& uv, const VectorX& x):
	Constraint(CONSTRAINT_TYPE_MESH)
{
	m_p[0] = p1;
	m_p[1] = p2;
	m_p[2] = p3;

	m_a = GetArea(x);

	m_Dm.col(0) = uv[p1] - uv[p3];	// FIXME: use uvmap directly [0, 1]
	m_Dm.col(1) = uv[p2] - uv[p3];
	m_Dm_inv = m_Dm.inverse();
	m_Dm_invT = m_Dm_inv.transpose();
}

MeshConstraint::MeshConstraint(const MeshConstraint& other)
{
	m_p[0] = other.m_p[0];
	m_p[1] = other.m_p[1];
	m_p[2] = other.m_p[2];

	m_Dm = other.m_Dm;
	m_Dm_inv = other.m_Dm_inv;

	m_a = other.m_a;

	m_material_type = other.m_material_type;
	m_material = other.m_material;
	m_mu = other.m_mu;
	m_lambda = other.m_lambda;
	m_kappa = other.m_kappa;
}

void MeshConstraint::GetMaterialProperty(MaterialType& type, ScalarType& mu, ScalarType& lambda, ScalarType& kappa, Material* mp)
{
	type = m_material_type;
	mu = m_mu;
	lambda = m_lambda;
	kappa = m_kappa;
	mp = m_material;
}

void MeshConstraint::SetMaterialProperty(MaterialType type, ScalarType mu, ScalarType lambda, ScalarType kappa, Material* mp)
{
	m_material_type = type;
	m_mu = mu;
	m_lambda = lambda;
	m_kappa = kappa;
	m_material = mp;
}

ScalarType MeshConstraint::SetMassMatrix(std::vector<SparseMatrixTriplet>& m, std::vector<SparseMatrixTriplet>& m_1d)
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
			m.push_back(SparseMatrixTriplet(3 * m_p[i] + j, 3 * m_p[i] + j, m_a / 3));
		m_1d.push_back(SparseMatrixTriplet(m_p[i], m_p[i], m_a / 3));
	}

	return m_a;
}

ScalarType MeshConstraint::GetArea(const VectorX& x)
{
	EigenVector3 x1 = x.block_vector(m_p[0]) - x.block_vector(m_p[2]);
	EigenVector3 x2 = x.block_vector(m_p[1]) - x.block_vector(m_p[2]);

	return 0.5 * x1.cross(x2).norm();
}

ScalarType MeshConstraint::getEnergyDensity(const EigenMatrix3x2& F, const EigenMatrix2& E)
{
	switch (m_material_type)
	{
	case MATERIAL_TYPE_COROT:
	case MATERIAL_TYPE_StVK:
		return m_mu * E.squaredNorm() + 0.5 * m_lambda * E.trace() * E.trace();
	case MATERIAL_TYPE_NEOHOOKEAN_EXTEND_LOG:
	case MATERIAL_TYPE_DATA_DRIVEN:
		EigenVector4 k = m_material->Ks(E);
		return 0.5 * (k[0] * E(0, 0) * E(0, 0) + k[2] * E(1, 1) * E(1, 1)
			+ 2 * k[1] * E(0, 0) * E(1, 1) + k[3] * E(0, 1) * E(0, 1));
	}
}

void MeshConstraint::getStressTensor(EigenMatrix3x2& P, const EigenMatrix3x2& F, const EigenMatrix2& G)
{
	switch (m_material_type)
	{
	case MATERIAL_TYPE_COROT:
	{

	}
	break;
	case MATERIAL_TYPE_StVK:
	{
		EigenMatrix2 I = EigenMatrix2::Identity();
		P = F * (2 * m_mu * G + m_lambda * G.trace() * I);
		//P = F * (2 * m_mu * G + m_lambda * (G.trace() + 1) * I);
		//ScalarType J = (F.col(0)).cross(F.col(1)).norm();	// FIXME: maybe we should consider flipping
		//if (J < 1)
		//{
		//	P += -m_kappa / 24 * std::pow((1 - J) / 6, 2) * J * F.inverse().transpose();	// FIXME: what's this?
		//}
	}
	break;
	case MATERIAL_TYPE_NEOHOOKEAN_EXTEND_LOG:
	{

	}
	break;
	}
}

void MeshConstraint::calculateDPDF(Matrix3232& DPDF, const EigenMatrix3x2& F, const EigenMatrix2& E)
{
	switch (m_material_type)
	{
	case MATERIAL_TYPE_StVK:
	{
		EigenMatrix3x2 deltaF;
		EigenMatrix2 deltaE;

		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 2; ++j)
			{
				deltaF.setZero();
				deltaF(i, j) = 1.0;
				deltaE = 0.5 * (deltaF.transpose() * F + F.transpose() * deltaF);
				DPDF(i, j) = 2 * m_mu * (deltaF * E + F * deltaE) + m_lambda * (E.trace() * deltaF + deltaE.trace() * F);
				//DPDF(i, j) = 2 * m_mu * (deltaF * E + F * deltaE) + m_lambda * ((E.trace() + 1) * deltaF + deltaE.trace() * F);
			}
		}
	}
	break;
	}
}

void MeshConstraint::EvaluateGradientAndHessian(const VectorX& x, bool definiteness_fix)
{
	EigenVector3 x1 = x.block_vector(m_p[0]) - x.block_vector(m_p[2]);
	EigenVector3 x2 = x.block_vector(m_p[1]) - x.block_vector(m_p[2]);

	EigenMatrix3x2 F;
	F.col(0) = x1; F.col(1) = x2;
	F *= m_Dm_inv;

	EigenMatrix2 G = 0.5 * (F.transpose() * F - EigenMatrix2::Identity());

	if (m_material_type == MATERIAL_TYPE_DATA_DRIVEN)
	{
		EigenVector4 k = m_material->Ks(G);

		EigenMatrix3x2 D;
		D.col(0) = EigenVector3(1, 0, -1); D.col(1) = EigenVector3(0, 1, -1);
		D *= m_Dm_inv;

		EigenVector3 du = D.col(0), dv = D.col(1);
		EigenMatrix3x9 Du = kronecker<1, 3, 3, 3>(du.transpose(), EigenMatrix3::Identity()),
			Dv = kronecker<1, 3, 3, 3>(dv.transpose(), EigenMatrix3::Identity());
		EigenVector3 xu = F.col(0), xv = F.col(1);
		EigenVector9 fuu = Du.transpose() * xu, fvv = Dv.transpose() * xv, fuv = 0.5 * (Du.transpose() * xv + Dv.transpose() * xu);

		m_g = k[0] * G(0, 0) * fuu + k[2] * G(1, 1) * fvv
			+ k[1] * (G(0, 0) * fvv + G(1, 1) * fuu) + k[3] * G(0, 1) * fuv;
		m_g *= m_a;

		if (definiteness_fix) G.cwiseMax(0.);
		m_H = k[0] * (fuu * fuu.transpose() + G(0, 0) * Du.transpose() * Du)
			+ k[2] * (fvv * fvv.transpose() + G(1, 1) * Dv.transpose() * Dv)
			+ k[1] * (fuu * fvv.transpose() + G(0, 0) * Dv.transpose() * Dv + fvv * fuu.transpose() + G(1, 1) * Du.transpose() * Du)
			+ k[3] * (fuv * fuv.transpose());
		if (!definiteness_fix)
			m_H += k[3] * G(0, 1) * (Du.transpose() * Dv + Dv.transpose() * Du) / 2;
		m_H *= m_a;
	}
	else
	{
		EigenMatrix3x2 P;
		getStressTensor(P, F, G);

		EigenMatrix3x2 H = m_a * P * m_Dm_invT;
		m_g.block_vector(0) = H.col(0);
		m_g.block_vector(1) = H.col(1);
		m_g.block_vector(2) = -(H.col(0) + H.col(1));

		Matrix3232 DPDF;
		calculateDPDF(DPDF, F, G);
		//for (int i = 0; i < 3; ++i)
		//{
		//	for (int j = 0; j < 2; ++j)
		//	{
		//		printf("(%d, %d):\n", i, j);
		//		for (int k = 0; k < 3; ++k)
		//		{
		//			for (int l = 0; l < 2; ++l)
		//				std::cout << DPDF(i, j)(k, l) << ' ';
		//			std::cout << '\n';
		//		}
		//		std::cout << '\n';
		//	}
		//}
		Matrix3232 dH = DPDF * m_Dm_invT * m_a;
		dH.innerProduct(m_Dm_invT);

		EigenMatrix3 H_block;
		m_H.setZero();
		for (int k = 0; k < 2; ++k) for (int i = 0; i < 2; ++i)
		{
			for (int l = 0; l < 3; ++l)
				H_block.row(l) = dH(l, k).col(i).transpose();
			m_H.block_mat(k, i) = H_block;
			m_H.block_mat(k, 2) -= H_block;
		}
		for (int i = 0; i < 3; ++i)
		{
			m_H.block_mat(2, i) = -m_H.block_mat(0, i) - m_H.block_mat(1, i);
		}

		if (definiteness_fix)
		{
			Eigen::EigenSolver<EigenMatrix9> evd;
			evd.compute(m_H);
			EigenMatrix9 Q = evd.eigenvectors().real();
			VectorX lambda = evd.eigenvalues().real();
			for (unsigned int i = 0; i != lambda.size(); i++)
			{
				lambda(i) = std::max(lambda(i), smallest_lambda);
			}
			m_H = Q * lambda.asDiagonal() * Q.transpose();
		}
	}
	//std::cout << "m_g: ";
	//for (int i = 0; i < 9; ++i)
	//	std::cout << m_g(i) << ' ';
	//std::cout << "\nm_H: \n";
	//for (int i = 0; i < 9; ++i)
	//{
	//	for (int j = 0; j < 9; ++j)
	//		std::cout << m_H(i, j) << '\t';
	//	std::cout << '\n';
	//}
}

void MeshConstraint::EvaluateGradientAndHessian(const VectorX& x, VectorX& gradient, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix)
{
	EvaluateGradientAndHessian(x, definiteness_fix);
	GetGradientAndHessian(gradient, hessian_triplets);
}

void MeshConstraint::GetGradientAndHessian(VectorX& gradient, std::vector<SparseMatrixTriplet>& hessian_triplets)
{
	for (int i = 0; i < 3; ++i)
	{
		gradient.block_vector(m_p[i]) += m_g.block_vector(i);
		for (int j = 0; j < 3; ++j)
			for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col)
				hessian_triplets.push_back(SparseMatrixTriplet(3 * m_p[i] + row, 3 * m_p[j] + col, m_H(3 * i + row, 3 * j + col)));
	}
}

ScalarType MeshConstraint::EvaluateEnergy(const VectorX& x)
{
	EigenVector3 x1 = x.block_vector(m_p[0]) - x.block_vector(m_p[2]);
	EigenVector3 x2 = x.block_vector(m_p[1]) - x.block_vector(m_p[2]);

	EigenMatrix3x2 F;
	F.col(0) = x1; F.col(1) = x2;
	F *= m_Dm_inv;

	EigenMatrix2 E = 0.5 * (F.transpose() * F - EigenMatrix2::Identity());
	
	m_energy = m_a * getEnergyDensity(F, E);

	return m_energy;
}


ScalarType MeshConstraint::EvaluateEnergyAndGradient(const VectorX& x)
{
	EigenVector3 x1 = x.block_vector(m_p[0]) - x.block_vector(m_p[2]);
	EigenVector3 x2 = x.block_vector(m_p[1]) - x.block_vector(m_p[2]);

	EigenMatrix3x2 F;
	F.col(0) = x1; F.col(1) = x2;
	F *= m_Dm_inv;

	EigenMatrix2 G = 0.5 * (F.transpose() * F - EigenMatrix2::Identity());

	if (m_material_type == MATERIAL_TYPE_DATA_DRIVEN)
	{
		EigenVector4 k = m_material->Ks(G);

		EigenMatrix3x2 D;
		D.col(0) = EigenVector3(1, 0, -1); D.col(1) = EigenVector3(0, 1, -1);
		D *= m_Dm_inv;

		EigenVector3 du = D.col(0), dv = D.col(1);
		EigenMatrix3x9 Du = kronecker<1, 3, 3, 3>(du.transpose(), EigenMatrix3::Identity()),
			Dv = kronecker<1, 3, 3, 3>(dv.transpose(), EigenMatrix3::Identity());
		EigenVector3 xu = F.col(0), xv = F.col(1);
		EigenVector9 fuu = Du.transpose() * xu, fvv = Dv.transpose() * xv, fuv = 0.5 * (Du.transpose() * xv + Dv.transpose() * xu);

		m_g = k[0] * G(0, 0) * fuu + k[2] * G(1, 1) * fvv
			+ k[1] * (G(0, 0) * fvv + G(1, 1) * fuu) + k[3] * G(0, 1) * fuv;
		m_g *= m_a;
	}
	else
	{
		EigenMatrix3x2 P;
		getStressTensor(P, F, G);

		EigenMatrix3x2 H = m_a * P * m_Dm_invT;
		m_g.block_vector(0) = H.col(0);
		m_g.block_vector(1) = H.col(1);
		m_g.block_vector(2) = -(H.col(0) + H.col(1));
	}

	m_energy = m_a * getEnergyDensity(F, G);

	return m_energy;
}

ScalarType MeshConstraint::GetEnergyAndGradient(VectorX& gradient)
{
	for (int i = 0; i < 3; ++i)
	{
		gradient.block_vector(m_p[i]) += m_g.block_vector(i);
	}
	return m_energy;
}

ScalarType MeshConstraint::EvaluateEnergyAndGradient(const VectorX& x, VectorX& gradient)
{
	EvaluateEnergyAndGradient(x);
	return GetEnergyAndGradient(gradient);
}

void MeshConstraint::EvaluateHessian(const VectorX& x, bool definiteness_fix)
{
	EigenVector3 x1 = x.block_vector(m_p[0]) - x.block_vector(m_p[2]);
	EigenVector3 x2 = x.block_vector(m_p[1]) - x.block_vector(m_p[2]);

	EigenMatrix3x2 F;
	F.col(0) = x1; F.col(1) = x2;
	F *= m_Dm_inv;

	EigenMatrix2 G = 0.5 * (F.transpose() * F - EigenMatrix2::Identity());

	if (m_material_type)
	{
		EigenVector4 k = m_material->Ks(G);

		EigenMatrix3x2 D;
		D.col(0) = EigenVector3(1, 0, -1); D.col(1) = EigenVector3(0, 1, -1);
		D *= m_Dm_inv;

		EigenVector3 du = D.col(0), dv = D.col(1);
		EigenMatrix3x9 Du = kronecker<1, 3, 3, 3>(du.transpose(), EigenMatrix3::Identity()),
			Dv = kronecker<1, 3, 3, 3>(dv.transpose(), EigenMatrix3::Identity());
		EigenVector3 xu = F.col(0), xv = F.col(1);
		EigenVector9 fuu = Du.transpose() * xu, fvv = Dv.transpose() * xv, fuv = 0.5 * (Du.transpose() * xv + Dv.transpose() * xu);

		if (definiteness_fix) G.cwiseMax(0.);
		m_H = k[0] * (fuu * fuu.transpose() + G(0, 0) * Du.transpose() * Du)
			+ k[2] * (fvv * fvv.transpose() + G(1, 1) * Dv.transpose() * Dv)
			+ k[1] * (fuu * fvv.transpose() + G(0, 0) * Dv.transpose() * Dv + fvv * fuu.transpose() + G(1, 1) * Du.transpose() * Du)
			+ k[3] * (fuv * fuv.transpose());
		if (!definiteness_fix)
			m_H += k[3] * G(0, 1) * (Du.transpose() * Dv + Dv.transpose() * Du) / 2;
		m_H *= m_a;
	}
	else
	{
		Matrix3232 DPDF;
		calculateDPDF(DPDF, F, G);
		Matrix3232 dH = DPDF * m_Dm_invT * m_a;
		dH.innerProduct(m_Dm_invT);

		EigenMatrix3 H_block;
		m_H.setZero();
		for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j)
		{
			for (int k = 0; k < 3; ++k)
				H_block.col(k) = dH(k, j).col(i);
			m_H.block_mat(i, j) = H_block;
			m_H.block_mat(i, 2) -= H_block;
		}
		for (int j = 0; j < 3; ++j)
		{
			m_H.block_mat(2, j) = -m_H.block_mat(0, j) - m_H.block_mat(1, j);
		}

		if (definiteness_fix)
		{
			Eigen::EigenSolver<EigenMatrix9> evd;
			evd.compute(m_H);
			EigenMatrix9 Q = evd.eigenvectors().real();
			VectorX lambda = evd.eigenvalues().real();
			for (unsigned int i = 0; i != lambda.size(); i++)
			{
				lambda(i) = std::max(lambda(i), smallest_lambda);
			}
			m_H = Q * lambda.asDiagonal() * Q.transpose();
		}
	}
}

void MeshConstraint::EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets, bool definiteness_fix)
{
	EvaluateHessian(x, definiteness_fix);
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
			for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col)
				hessian_triplets.push_back(SparseMatrixTriplet(3 * m_p[i] + row, 3 * m_p[j] + col, m_H(3 * i + row, 3 * j + col)));
	}
}

void MeshConstraint::ApplyHessian(const VectorX& x, VectorX& b)
{
	//for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
	//{
	//	b.block_vector(m_p[i]) += m_H.block<3, 3>(3 * i, 3 * j) * x.block_vector(m_p[j]);
	//}
	EigenVector9 x_blocks;
	for (int i = 0; i < 3; ++i)
	{
		x_blocks.block_vector(i) = x.block_vector(m_p[i]);
	}
	EigenVector9 b_blocks = m_H * x_blocks;
	for (int i = 0; i < 3; i++)
	{
		b.block_vector(m_p[i]) += b_blocks.block_vector(i);
	}
}

