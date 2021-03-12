#include "material.h"
#include "json_loader.h"

int Material::n_samples = 30;

void Material::Reset()
{
	std::string fname(DEFAULT_MATERIAL_PATH);
	fname.append("material.json");

	std::ifstream file(fname, std::ifstream::binary);
	if (!file)
	{
		std::cerr << "Can NOT open " << fname << ". EXIT." << std::endl;
		exit(-1);
	}

	Json::Value root;
	file >> root;
	file.close();

	root = root["materials"];
	fname = root["data"].asString();
	//FIXME: can be dropped
	// parse material parameters 
	ScalarType density_mult, stretching_mult, bending_mult, thicken;  // FIXME: unused multipliar
	parse(density_mult, root["density_mult"], ScalarType(1.));
	parse(stretching_mult, root["stretching_mult"], ScalarType(1.));
	parse(bending_mult, root["bending_mult"], ScalarType(1.));
	parse(thicken, root["thicken"], ScalarType(1.));
	stretching_mult *= thicken;
	bending_mult *= thicken;

	parse(m_damping, root["damping"], ScalarType(0.));
	parse(m_yield_curv, root["yield_curv"], inf);
	parse(m_weakening, root["weakening"], ScalarType(0.));
	parse(m_strain_min, root["strain_limits"][0], -inf);
	parse(m_strain_max, root["strain_limits"][1], inf);

	// parse sample data
	//file.open(str);
	fname = DEFAULT_MATERIAL_PATH;
	fname.append(m_material_file_path);
	fname.append(".json");
	file.open(fname, std::ifstream::binary);
	if (!file)
	{
		std::cerr << "Can NOT open " << fname << ". EXIT." << std::endl;
		exit(-1);
	}
	root.clear();
	file >> root;
	file.close();

	parse(m_density, root["density"]);

	// evaluate strething samples
	StretchingData data;
	Json::Value json = root["stretching"];
	parse(data.d[0][0], json[0]);
	for (int i = 1; i < 5; ++i) data.d[0][i] = data.d[0][0];
	for (int i = 0; i < 5; ++i) parse(data.d[1][i], json[i + 1]);
	evaluate_stretching_samples(data);

	json = root["bending"];
	for (int i = 0; i < 3; ++i) for (int j = 0; j < 5; ++j)
		parse(m_bending_data[i][j], json[i][j]);
}

// From ArcSim 0.2 dde.cpp
void Material::evaluate_stretching_samples(const StretchingData& data)
{
	for (int i = 0; i < n_samples; i++)
		for (int j = 0; j < n_samples; j++)
			for (int k = 0; k < n_samples; k++)
			{
				EigenMatrix2 G;
				G(0, 0) = -0.25 + i / (n_samples * 1.0);
				G(1, 1) = -0.25 + j / (n_samples * 1.0);
				G(0, 1) = G(1, 0) = k / (n_samples * 1.0);
				m_stretching_samples[i][j][k] = evaluate_stretching_sample(G, data);
			}
}

// From ArcSim 0.2 dde.cpp
EigenVector4 Material::evaluate_stretching_sample(EigenMatrix2 G, const StretchingData& data) {
	G = G * 2. + EigenMatrix2::Identity();
	Eigen::SelfAdjointEigenSolver<EigenMatrix2> eig(G);
	EigenVector2 w = eig.eigenvalues();
	EigenVector2 V = eig.eigenvectors().col(0);
	ScalarType angle_weight = fabs(atan2(V[1], V[0]) / M_PI) * 8;
	if (angle_weight < 0) angle_weight = 0;
	if (angle_weight > 4 - EPSILON) angle_weight = 4 - EPSILON;
	int angle_id = (int)angle_weight;
	angle_weight = angle_weight - angle_id;
	ScalarType strain_value = (w[0] - 1) * 6;
	if (strain_value < 0) strain_value = 0;
	if (strain_value > 1 - EPSILON) strain_value = 1 - EPSILON;
	int strain_id = (int)strain_value;
	ScalarType strain_weight = strain_value - strain_id;
	EigenVector4 real_elastic =
		data.d[strain_id][angle_id] * (1 - strain_weight) * (1 - angle_weight) +
		data.d[strain_id + 1][angle_id] * (strain_weight) * (1 - angle_weight) +
		data.d[strain_id][angle_id + 1] * (1 - strain_weight) * (angle_weight)+
		data.d[strain_id + 1][angle_id + 1] * (strain_weight) * (angle_weight);
	if (real_elastic[0] < 0)   real_elastic[0] = 0;
	if (real_elastic[1] < 0)   real_elastic[1] = 0;
	if (real_elastic[2] < 0)   real_elastic[2] = 0;
	if (real_elastic[3] < 0)   real_elastic[3] = 0;
	real_elastic.cwiseMax(0);
	real_elastic *= 2;
	return real_elastic;
}

EigenVector4 Material::Ks(const EigenMatrix2& G)
{
	ScalarType a = (G(0, 0) + 0.25) * n_samples;
	ScalarType b = (G(1, 1) + 0.25) * n_samples;
	ScalarType c = fabs(G(0, 1)) * n_samples;
	a = clamp(a, ScalarType(0.), ScalarType(n_samples - 1 - EPSILON));
	b = clamp(b, ScalarType(0.), ScalarType(n_samples - 1 - EPSILON));
	c = clamp(c, ScalarType(0.), ScalarType(n_samples - 1 - EPSILON));
	int ai = clamp((int)floor(a), 0, n_samples - 2);
	int bi = clamp((int)floor(b), 0, n_samples - 2);
	int ci = clamp((int)floor(c), 0, n_samples - 2);
	a = a - ai;
	b = b - bi;
	c = c - ci;
	ScalarType weight[2][2][2];
	weight[0][0][0] = (1 - a) * (1 - b) * (1 - c);
	weight[0][0][1] = (1 - a) * (1 - b) * (c);
	weight[0][1][0] = (1 - a) * (b) * (1 - c);
	weight[0][1][1] = (1 - a) * (b) * (c);
	weight[1][0][0] = (a) * (1 - b) * (1 - c);
	weight[1][0][1] = (a) * (1 - b) * (c);
	weight[1][1][0] = (a) * (b) * (1 - c);
	weight[1][1][1] = (a) * (b) * (c);
	EigenVector4 stiffness;
	stiffness.setZero();
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
				for (int l = 0; l < 4; l++)
				{
					stiffness[l] += m_stretching_samples[ai + i][bi + j][ci + k][l] * weight[i][j][k];
				}
	return stiffness;
}

ScalarType Material::Kb(ScalarType curv, const EigenVector2& du, int side)
{
	ScalarType value = curv * 0.2; // because samples are per 0.05 cm^-1 = 5 m^-1
	if (value > 4) value = 4;
	int value_i = clamp((int)value, 0, 3);
	value -= value_i;
	ScalarType bias_angle = atan2(du[1], du[0]) * 4 / M_PI;
	if (bias_angle < 0) bias_angle = -bias_angle;
	if (bias_angle > 4) bias_angle = 8 - bias_angle;
	if (bias_angle > 2) bias_angle = 4 - bias_angle;
	int bias_id = clamp((int)bias_angle, 0, 1);
	bias_angle -= bias_id;
	ScalarType actual_ke = m_bending_data[bias_id][value_i] * (1 - bias_angle) * (1 - value)
		+ m_bending_data[bias_id + 1][value_i] * (bias_angle) * (1 - value)
		+ m_bending_data[bias_id][value_i + 1] * (1 - bias_angle) * (value)
		+ m_bending_data[bias_id + 1][value_i + 1] * (bias_angle) * (value);
	if (actual_ke < 0) actual_ke = 0;
	return actual_ke;
}