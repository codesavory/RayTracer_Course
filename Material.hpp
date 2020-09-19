#pragma once
#include "Vector.hpp"


class Material {
public:
	float ka = 0.0f, ks = 0.0f, ktran = 0.0f;
	float specularExponent = 0.0f;
	Vec3f diffuseColor = Vec3f(0.0f, 0.0f, 0.0f);
	float ior = 1.5f;

	inline Material scaleByBarycentric(float b) {
		Material m;
		m.ka = b * ka;
		m.ks = b * ks;
		m.ktran = b * ktran;
		m.specularExponent = b * specularExponent;
		m.diffuseColor = b * diffuseColor;
		return m;
	}

	Material operator + (Material const& m2) {
		Material m3;

		m3.ka = ka + m2.ka;
		m3.ks = ks + m2.ks;
		m3.ktran = ktran + m2.ktran;
		m3.specularExponent = specularExponent + m2.specularExponent;
		m3.diffuseColor = diffuseColor + m2.diffuseColor;
		return m3;
	}

	static Material interpolateByBaryCentric(
		Material m0, Material m1, Material m2,
		float u, float v, float w)
	{
		m0 = m0.scaleByBarycentric(u);
		m1 = m1.scaleByBarycentric(v);
		m2 = m2.scaleByBarycentric(w);
		return m0 + m1 + m2;
	}
};



