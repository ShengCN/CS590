#include "lab02.h"
#include <cassert>
#include <iostream>
#include <set>
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

	// output result hermite curves
	const int sampling_n = 30;
	out_curves.resize(sampling_n);
	int piece_sample_n = sampling_n / (int)pw_curves.size();
	for(int i = 0; i < sampling_n; ++i) {
		int piece_ind = i / piece_sample_n;
		float t = (float)(i- piece_ind * piece_sample_n) / (piece_sample_n - 1);
		pw_curves[piece_ind].p(t, out_curves[i]);
	}

	// solve t for each point
	out_visualize_points.resize(points.size());
	piece_sample_n = points.size()/ (int)pw_curves.size();
	for (size_t pi = 0; pi < points.size(); ++pi) {
		int piece_ind = pi / piece_sample_n;
		float t; pw_curves[piece_ind].find_closet_point(points[pi], t, out_visualize_points[pi]);
	}

	for(auto &c:pw_curves) {
		std::cout << c.to_string() << std::endl;
	}
}

bool newton(std::function<float(float)> f, std::function<float(float)> f_prime, float &out_root) {
	int max_iteration = (int)1e2;
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
	assert(points.size() >= 4);

	// L2 distance loss
	auto loss = [](const std::vector<Vect3d> &points,std::vector<hermite_curve> curves) {
		float total_loss = 0.0f;

		const int piece_size = points.size() / curves.size();
		for(size_t pi =0; pi < points.size(); ++pi) {
			size_t piece_ind = pi / piece_size;
			float t; Vect3d pp; curves[piece_ind].find_closet_point(points[pi], t, pp);
			total_loss += (pp - points[pi]).Length();
		}

		return total_loss;
	};
	
	timer clc;
	const int max_iteration = (int)1e2;
	const size_t sample_num = points.size();
	// todo, extend to n
	curves.resize(n);

	const int free_variable_num = 2 * n + 2;
	MatrixXd phi_km(free_variable_num, sample_num), m(sample_num, free_variable_num);
	MatrixXd y(sample_num, 3);
	
	// initialize
	// todo, find knots
	// just evenly divide the points
	int piece_size = sample_num / n; // if (piece_size < 0) piece_size = 1;
	for (int i = 0; i < n; ++i) {
		int cur_begin_ind = piece_size * i, cur_end_ind = piece_size * (i + 1);
		if(i==0) {
			curves[i] = hermite_curve(points[cur_begin_ind],
									  points[cur_end_ind - 1],
									  points[cur_begin_ind + 1] - points[cur_begin_ind],
									  points[cur_end_ind - 1] - points[cur_end_ind - 2]);
		} else {
			int last_end_ind = piece_size * i;
			curves[i] = hermite_curve(points[last_end_ind-1],
									  points[cur_end_ind - 1],
									  points[last_end_ind - 1] - points[last_end_ind-2],
									  points[cur_end_ind - 1] - points[cur_end_ind - 2]);
		}
	}
	float min_loss = std::numeric_limits<float>::max();
	
	// compute the rows that should set non-zero values
	auto get_rows=[](size_t curve_ind) {
		std::set<size_t> ret;
		if(curve_ind ==0) {
			ret.insert(0); ret.insert(1); ret.insert(2); ret.insert(3);
		} else {
			ret.insert(2 * curve_ind - 1);
			ret.insert(2 * curve_ind + 1);
			ret.insert(2 * curve_ind + 2);
			ret.insert(2 * curve_ind + 3);
		}
		return ret;
	};

	auto tmp_curve = curves;
	for (int i = 0; i < max_iteration; ++i) {
		for (size_t si = 0; si < sample_num; ++si) {
			float t; Vect3d tmp;

			const size_t curve_ind = si / piece_size;
			clc.tic();
			tmp_curve[curve_ind].find_closet_point(points[si], t, tmp);
			clc.toc(); // std::cerr << "finding t spent: " << clc.to_string() << " " << t << std::endl;

			Vect4d phi; hermite_curve::get_phi(t, phi);
			// update corresponding curve's rows
			std::set<size_t> non_zero_rows = get_rows(curve_ind);
			int counter = 0;
			for(size_t ri = 0; ri < (size_t)free_variable_num; ++ri) {
				if(non_zero_rows.find(ri) != non_zero_rows.end()) {
					if (curve_ind != 0) {
						if (counter == 0) {
							phi_km(ri, si) = phi[0];
						}

						if (counter == 1) {
							phi_km(ri, si) = phi[2];
						}

						if (counter == 2) {
							phi_km(ri, si) = phi[1];
						}

						if (counter == 3) {
							phi_km(ri, si) = phi[3];
						}
					} else {
						if (counter == 0) {
							phi_km(ri, si) = phi[0];
						}

						if (counter == 1) {
							phi_km(ri, si) = phi[1];
						}

						if (counter == 2) {
							phi_km(ri, si) = phi[2];
						}

						if (counter == 3) {
							phi_km(ri, si) = phi[3];
						}
					}

					counter++;
				}
				else {
					phi_km(ri, si) = 0.0f;
				}
			}

			y(si,0) = points[si][0]; y(si, 1) = points[si][1]; y(si,2) = points[si][2];
		}	

		m = phi_km.transpose();
		MatrixXd ma = (phi_km * m).inverse() * phi_km * y; // 4x3
		Vect3d p0(ma(0, 0), ma(0, 1), ma(0, 2)), 
			   p1(ma(1, 0), ma(1, 1), ma(1, 2)), 
			   p2(ma(2, 0), ma(2, 1), ma(2, 2)), 
			   p3(ma(3, 0), ma(3, 1), ma(3, 2));
		std::vector<hermite_curve> new_curves = tmp_curve;
		new_curves[0] = hermite_curve(p0, p1, p2, p3);
		for(int i = 1; i < new_curves.size(); ++i) {
			Vect3d p0(ma(2 * i - 1, 0), ma(2 * i - 1, 1), ma(2 * i - 1, 2)), 
				   p1(ma(2 * i + 1, 0), ma(2 * i + 1, 1), ma(2 * i + 1, 2)),
				   p2(ma(2 * i + 2, 0), ma(2 * i + 2, 1), ma(2 * i + 2, 2)),
				   p3(ma(2 * i + 3, 0), ma(2 * i + 3, 1), ma(2 * i + 3, 2));

			new_curves[i] = hermite_curve(p0, p2, p1, p3);
		}

		float cur_loss = loss(points, new_curves);
		if(verbose)
			std::cerr << "loss: " << cur_loss << std::endl;

		// converge
		if (cur_loss < 1e-3 || std::abs(min_loss - cur_loss) < 1e-3 || isnan(cur_loss)) {
			std::cerr << "Converge \n";
			break;
		}

		// update
		if (min_loss > cur_loss) {
			min_loss = cur_loss; curves = new_curves;
		}
		tmp_curve = new_curves;
	}

}