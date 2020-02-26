#pragma once
#include <random>
#include <vector>
#include <functional>
#include <chrono>
#include <iostream>

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

// newton's method to compute the root of some function f using f and f'
// out_root should be given an initial guess
// t = t - f/f'
bool newton(std::function<float(float)> f, std::function<float(float)> f_prime, float &root);

struct hermite_curve {
	Matrix4d coefficients; // coefficient

	hermite_curve()=default;

	// Q(t) = P0 * F0 + P1 * F1 + P0' * F3 + P1' * F4
	hermite_curve(Vect3d p0, Vect3d p1, Vect3d p0_prime, Vect3d p1_prime) {
		coefficients = Matrix4d(p0, p1, p0_prime, p1_prime);
	}

	void p(float t, Vect3d &out_p) {
		float t_cubic = t * t * t, t_square = t * t;
		Vect4d ft; get_phi(t, ft);
		out_p = Vect3d(coefficients * ft);
	}

	void dp(float t, Vect3d &out_p) {
		float t_square = t * t;
		Vect4d dft(6.0f * (t_square - t), 
				   -6.0f * (t_square-t), 
				   3.0f * t_square - 4.0f * t + 1.0f,
				   3.0f * t_square - 2.0f * t);
		out_p = Vect3d(coefficients * dft);
	}

	void ddp(float t, Vect3d &out_p) {
		Vect4d ddft(12.0f * t - 6.0f,
				   -12.0f * t + 6.0f,
				   6.0f * t - 4.0f,
				   6.0f * t - 2.0f);
		out_p = Vect3d(coefficients * ddft);
	}

	// Given a p(x,y,z), find the closest point on the curve
	// i.e. minimizing the distance between p and the curve, given t in [0.0,1.0]
	void find_closet_point(const Vect3d &p, float &t, Vect3d &out_p) {
		std::function<float(float)> f = [&](float t){
			Vect3d cur_p; this->p(t, cur_p);
			Vect3d cur_dp; dp(t, cur_dp);
			return (cur_p - p).Dot(cur_dp);
		};

		std::function<float(float)> df = [&](float t) {
			Vect3d cur_p; this->p(t, cur_p);
			Vect3d cur_dp; dp(t, cur_dp);
			Vect3d cur_ddp; ddp(t, cur_ddp);

			return cur_dp.Dot(cur_dp) + (cur_p - p).Dot(cur_ddp);
		};

		auto loss = [&](float t) {
			Vect3d curve_p; this->p(t, curve_p);
			return (curve_p - p).Length();
		};

		t = 0.5f; newton(f, df, t);
		float best_loss = loss(t);

		int max_iter = 10;
		for(int i = 0; i < max_iter; ++i) {
			float init_fract = (float)i / max_iter;
			float cur_t = init_fract + 1e-2f; newton(f, df, cur_t);
			float cur_loss = loss(cur_t);
			if(cur_loss < best_loss) {
				best_loss = cur_loss;
				t = cur_t;
			}
		}

		if (t < 0.0f) t = 0.0f;
		if (t > 1.0f) t = 1.0f;

		this->p(t, out_p);
	}

	static void get_phi(float t, Vect4d &ft) {
		float t_cubic = t * t * t, t_square = t * t;
		ft = Vect4d(2.0f * t_cubic - 3.0f * t_square + 1,
				  -2.0f * t_cubic + 3.0f * t_square,
				  t_cubic - 2.0f * t_square + t,
				  t_cubic - t_square);
	}

	std::string to_string() {
		return coefficients.to_string();
	}
};

void solve(const std::vector<Vect3d> &points, 
		   int n, 
		   std::vector<hermite_curve> &curves,
		   bool verbose=true);

// inputs are points and piece number
// outputs are cubic curve points, closest points
void compute_curve(const std::vector<Vect3d> &points, int n,
				   std::vector<Vect3d> &out_curves,
				   std::vector<Vect3d> &out_visualize_points
);

typedef std::chrono::high_resolution_clock Clock;
class timer {
public:
	timer() = default;
	~timer() {};

	void tic() {
		_is_ticed = true;
		_tic = Clock::now();
	}
	void toc() {
		_toc = Clock::now();
	}

	long long get_elapse() {
		return std::chrono::duration_cast<std::chrono::nanoseconds>(_toc - _tic).count();
	}

	void print_elapsed() {
		if (!_is_ticed) {
			std::cerr << "timer has not been ticed \n";
			return;
		}

		auto elapsed = get_elapse();
		std::cerr << "Time: " << elapsed * 1e-9 << " seconds \n";
	}

	std::string to_string() {
		if (!_is_ticed) {
			std::cerr << "timer has not been ticed. \n";
		}
		std::stringstream oss;
		oss << get_elapse() * 1e-9;
		return oss.str();
	}

private:
	bool _is_ticed = false;
	std::chrono::time_point<std::chrono::steady_clock> _tic;
	std::chrono::time_point<std::chrono::steady_clock> _toc;
};