// Author: Shiyang Jia
// Date: 11/13/2020
//
// Load material data from json file

#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include <iostream>
#include <fstream>
#include "global_headers.h"
#include "math_headers.h"
#include "json/json.h"

class BendConstraint;

struct StretchingData
{
	EigenVector4 d[2][5];
};

class Material
{
public:
	Material() {}
	~Material() {}

	void Reset();

	EigenVector4 Ks(const EigenMatrix2& G);
	ScalarType Kb(ScalarType curv, const EigenVector2& du, int side);

	char m_material_file_path[256];

	static int n_samples;

	ScalarType m_density; // area density
	ScalarType m_damping; // stiffness-proportional damping coefficient
	ScalarType m_strain_min, m_strain_max; // strain limits
	ScalarType m_yield_curv, m_weakening; // plasticity parameters

protected:
	void evaluate_stretching_samples(const StretchingData& data);
	EigenVector4 evaluate_stretching_sample(EigenMatrix2 G, const StretchingData& data);

	EigenVector4 m_stretching_samples[40][40][40];
	ScalarType m_bending_data[3][5];
};

#endif