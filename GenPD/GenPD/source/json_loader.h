#ifndef _JSON_LOADER_H_
#define _JSON_LOADER_H_

#include "json/json.h"
#include <Eigen/Core>

template <typename T>
void parse(T& value, const Json::Value& json);

template<> 
void parse(float& value, const Json::Value& json)
{
	value = json.asFloat();
}

template<> 
void parse(double& value, const Json::Value& json)
{
	value = json.asDouble();
}

template <typename T>
void parse(T& value, const Json::Value& json, const T& default_value)
{
	if (json.isNull())
		value = default_value;
	else
		parse<T>(value, json);
}

template <typename T, int n>
void parse(Eigen::Matrix<T, n, 1>& vec, const Json::Value& json)
{
	assert(json.size() == n);
	for (int i = 0; i < n; ++i)
		parse<T>(vec(i), json[i]);
}

#endif // !_JSON_LOADER_H_
