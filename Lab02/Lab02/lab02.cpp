#include "lab02.h"
#include <cassert>
#include <iostream>

using std::vector;

void random_points(vector<Vect3d> &a, int n) {
	a.resize(n);
	for (int i = 0; i < n; ++i) {
		random_point(a[i]);
	}
}

void random_control_points(vector<Vect3d> &beizer_points, int total_constraint_point) {
	beizer_points.resize(total_constraint_point);

	// initialize beizer points
	Vect3d delta_offset(0.5f, 0.0f, 0.0f), center(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < total_constraint_point; ++i) {
		random_point(beizer_points[i], Vect3d(0.0f, 1.0f, 1.0f));
		beizer_points[i] = beizer_points[i] + (float)i * delta_offset;
		center += beizer_points[i] / (float)total_constraint_point;
	}

	// recenter
	for (int pi = 0; pi < beizer_points.size(); ++pi) {
		std::cerr << beizer_points[pi].Length() << std::endl;
		beizer_points[pi] = (beizer_points[pi] - center);
		std::cerr << beizer_points[pi].Length() << std::endl;
	}
}

void sample_beizer(const vector<Vect3d> &cpoints, const int sampling_num, vector<Vect3d> &point) {
	assert(cpoints.size() == 4);

	auto get_beizer_weight = [](float t) {
		return Vect4d((1 - t) * (1 - t) * (1 - t), 3.0f * t * (1 - t) * (1 - t), 3.0f * t * t * (1 - t), t * t * t);
	};

	int pieces = ((int)cpoints.size() - 2) / 2;
	point.resize(sampling_num);

	Vect3d b0 = cpoints[0], b1 = cpoints[1], b2 = cpoints[2], b3 = cpoints[3];
	
	// given four constraints, sample points from beizer curves
	Matrix4d point_matrix(b0, b1, b2, b3); 
	for (int ti = 0; ti < sampling_num; ++ti) {
		float fract = (float)ti / (sampling_num - 1);
		Vect4d weight = get_beizer_weight(fract);
		point[ti] = point_matrix * weight;
	}
}

void compute_curve(const std::vector<Vect3d> &points, int n, std::vector<Vect3d> &out_curves, std::vector<Vect3d> &out_visualize_points) {
	Vect3d p0 = points[0], p1 = points[3], p0_ = Vect3d(0.0,1.0,0.0), p1_ = Vect3d(0.0,1.0,0.0);
	hermite_curve curve(p0, p1, p0_, p1_);
	
	const int sampling_n = 30;
	out_curves.resize(sampling_n);
	for(int i = 0; i < sampling_n; ++i) {
		float t = (float)i / (sampling_n - 1);
		curve.p(t, out_curves[i]);
	}

	// solve t for each point
	for(auto &p:points) {

	}
}

bool newton(std::function<float(float)> f, std::function<float(float)> f_prime, float &out_root) {
	int max_iteration = 1e5;
	float eps = 1e-3;
	for(int i = 0; i < max_iteration; ++i) {
		out_root = out_root - f(out_root) / f_prime(out_root);

		if(std::abs(f(out_root)) < eps) {
			return true;
		}
	}

	return false;
}
