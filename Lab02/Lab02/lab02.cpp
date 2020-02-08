#include "lab02.h"
#include <cassert>
#include <iostream>

#include <Eigen/Dense>
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
	for (size_t pi = 0; pi < beizer_points.size(); ++pi) {
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
	// regress points using piecewise hermite curve
	std::vector<hermite_curve> pw_curves;
	solve(points, n, pw_curves);

	hermite_curve curve = pw_curves[0];
	// output result hermite curves
	const int sampling_n = 30;
	out_curves.resize(sampling_n);
	for(int i = 0; i < sampling_n; ++i) {
		float t = (float)i / (sampling_n - 1);
		curve.p(t, out_curves[i]);
	}

	// solve t for each point
	out_visualize_points.resize(points.size());
	for (size_t pi = 0; pi < points.size(); ++pi) {
		float t;
		curve.find_closet_point(points[pi], t, out_visualize_points[pi]);
	}
}

bool newton(std::function<float(float)> f, std::function<float(float)> f_prime, float &out_root) {
	int max_iteration = (int)1e5;
	float eps = 1e-3f;
	for(int i = 0; i < max_iteration; ++i) {
		float fx = f(out_root);
		float dfx = f_prime(out_root);

		out_root = out_root - fx / dfx;

		if(std::abs(f(out_root)) < eps) {
			return true;
		}
	}

	return false;
}

void solve(const std::vector<Vect3d> &points, int n, std::vector<hermite_curve> &curves, bool verbose) {
	using Eigen::MatrixXd;
	using Eigen::VectorXd;

	// L2 distance loss
	auto loss = [](const std::vector<Vect3d> &points, hermite_curve curve) {
		float total_loss = 0.0f;
		for(auto &p: points) {
			float t; Vect3d pp;
			curve.find_closet_point(p, t, pp);
			total_loss += (pp - p).Length();
		}
		return total_loss;
	};
	
	const int max_iteration = (int)1e2;
	const size_t sample_num = points.size();
	// todo, extend to n
	curves.resize(1);

	MatrixXd phi_km(4, sample_num), m(sample_num,4);
	MatrixXd y(sample_num, 3);
	
	// initialize
	// todo, find knots
	float min_loss = std::numeric_limits<float>::max();
	hermite_curve best_curve = hermite_curve(points.front(), points.back(), points.front(), points.back());
	for (int i = 0; i < max_iteration; ++i) {
		for (size_t si = 0; si < sample_num; ++si) {
			float t; Vect3d tmp;
			best_curve.find_closet_point(points[si], t, tmp);
			Vect4d phi; hermite_curve::get_phi(t, phi);
			phi_km(0, si) = phi[0]; phi_km(1, si) = phi[1]; phi_km(2, si) = phi[2]; phi_km(3, si) = phi[3];
			y(si, 0) = points[si][0]; y(si, 1) = points[si][1]; y(si, 2) = points[si][2];
		}
		m = phi_km.transpose();
		MatrixXd ma = (phi_km * m).inverse() * phi_km * y; // 4x3
		Vect3d p0(ma(0, 0), ma(0, 1), ma(0, 2)), p1(ma(1, 0), ma(1, 1), ma(1, 2)), p2(ma(2, 0), ma(2, 1), ma(2, 2)), p3(ma(3, 0), ma(3, 1), ma(3, 2));
		hermite_curve new_curve(p0, p1, p2, p3);

		float cur_loss = loss(points, curves[0]);
		if(verbose)
			std::cerr << "loss: " << cur_loss << std::endl;

		// converge
		if(std::abs(min_loss-cur_loss) < 1e-6) {
			std::cerr << "Converge \n";
			break;
		}

		// update
		if(min_loss > cur_loss) {
			min_loss = cur_loss; best_curve = new_curve;
		}
	}

	curves[0] = best_curve;
}