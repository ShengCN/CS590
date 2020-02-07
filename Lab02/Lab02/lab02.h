#pragma once
#include <random>
#include <vector>

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

// sampling points from constraint points
// cpoints: constraint points(2n + 2), where n is # of beizer curves
// sampling_num: total sample #, assume sampling_number is factor of pieces
// [output] points: sampled points from beizer curves 
void create_beizer(const std::vector<Vect3d> &cpoints, const int sampling_num, std::vector<Vect3d> &point);

void random_points(std::vector<Vect3d> &a, int n);

void random_points_beizer(std::vector<Vect3d> &control_points, int n, std::vector<Vect3d> &sampled_points);