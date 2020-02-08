#pragma once
#include <random>
#include <vector>
#include <functional>

#include "vect3d.h"
#include "vect4d.h"
#include "matrix4d.h"

//returns random number from <-1,1>
inline float random11() {
	return 2.f*rand() / (float)RAND_MAX - 1.f;
}

inline void random_point(Vect3d &out_vec, Vect3d scale = Vect3d(1.0f, 1.0f, 1.0f)) {
	out_vec = Vect3d(random11() * scale.GetX(), random11() * scale.GetY(), random11() * scale.GetZ());
}

void random_points(std::vector<Vect3d> &a, int n);

void random_control_points(std::vector<Vect3d> &control_points, int n=4);

// [input]  cpoints: 4 constraints points
// [input]  sampling #
// [output] sampled points
void sample_beizer(const std::vector<Vect3d> &cpoints, const int sampling_num, std::vector<Vect3d> &point);

struct hermite_curve {
	Matrix4d coefficients; // coefficient
	
	// Q(t) = P0 * F0 + P1 * F1 + P0' * F3 + P1' * F4
	hermite_curve(Vect3d p0, Vect3d p1, Vect3d p0_prime, Vect3d p1_prime) {
		coefficients = Matrix4d(p0, p1, p0_prime, p1_prime);
	}

	void p(float t, Vect3d &out_p) {
		float t_cubic = t * t * t, t_square = t * t;
		Vect4d ft(2.0f * t_cubic - 3.0f * t_square + 1,
				  -2.0f * t_cubic + 3.0f * t_square,
				  t_cubic - 2.0f * t_square + t,
				  t_cubic - t_square);

		out_p = Vect3d(coefficients * ft) ;
	}
};

// inputs are points and piece number
// outputs are cubic curve points, closest points
void compute_curve(const std::vector<Vect3d> &points, int n, 
				   std::vector<Vect3d> &out_curves,
				   std::vector<Vect3d> &out_visualize_points
				   );

// newton's method to compute the root of some function f using f and f'
// out_root should be given an initial guess
// t = t - f/f'
bool newton(std::function<float(float)> f, std::function<float(float)> f_prime, float &root);