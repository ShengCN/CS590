#include "lab02.h"
#include <cassert>


using std::vector;

void create_beizer(const vector<Vect3d> &cpoints, const int sampling_num, vector<Vect3d> &point) {
	assert(cpoints.size() >= 4);

	auto get_beizer_weight = [](float t) {
		return Vect4d((1 - t) * (1 - t) * (1 - t), 3.0f * t * (1 - t) * (1 - t), 3.0f * t * t * (1 - t), t * t * t);
	};

	int pieces = ((int)cpoints.size() - 2) / 2;
	point.resize(sampling_num);

	Vect3d b0, b1, b2, b3;
	for (int i = 0; i < pieces; ++i) {
		if (i == 0) {
			b0 = cpoints[2 * i + 0];  b1 = cpoints[2 * i + 1]; b2 = cpoints[2 * i + 2]; b3 = cpoints[2 * i + 3];
		}
		else {
			b0 = b3;  b1 = b3 + (b3 - b2); b2 = cpoints[2 * i + 2]; b3 = cpoints[2 * i + 3];
		}

		// given four constraints, sample points from beizer curves
		Matrix4d point_matrix(b0, b1, b2, b3);
		for (int ti = 0; ti < sampling_num / pieces; ++ti) {
			float fract = (float)ti / (sampling_num - 1);
			Vect4d weight = get_beizer_weight(fract);
			point[i * sampling_num / pieces + ti] = point_matrix * weight;
		}
	}
}

void random_points(vector<Vect3d> &a, int n) {
	a.resize(n);
	for (int i = 0; i < n; ++i) {
		random_point(a[i]);
	}
}

void random_points_beizer(vector<Vect3d> &control_points, int n, vector<Vect3d> &sampled_points) {
	// beizer constraint points
	create_beizer(control_points, n, sampled_points);
}