#pragma once
#include <random>
#include <vector>
#include <functional>
#include <chrono>
#include <iostream>

#include "vect3d.h"
#include "vect4d.h"
#include "matrix4d.h"

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

/* Data structure to define a tree node */
struct tree_node {
	std::vector<std::shared_ptr<tree_node>> children_list; // in this experiment, size will only be in range [0,3]
	std::shared_ptr<tree_node> parent_node;
	float width, height, thickness;
	float local_rotation;

	// functions
	tree_node(std::shared_ptr<tree_node> parent, float w, float h, float t, float r):
		parent_node(parent) , width(w), height(h), thickness(t), local_rotation(r) {}

	void get_box(Vect3d &a, Vect3d &b, Vect3d &c, Vect3d &d,
				 Vect3d &e, Vect3d &f, Vect3d &g, Vect3d &h);
};

struct aabb {
	Vect3d p0, p1;

	aabb(Vect3d p) {
		p0 = p1 = p;
	}

	void add(Vect3d p) {
		p0[0] = std::min(p0[0], p[0]);
		p0[1] = std::min(p0[1], p[1]);
		p0[2] = std::min(p0[2], p[2]);

		p1[0] = std::max(p1[0], p[0]);
		p1[1] = std::max(p1[1], p[1]);
		p1[2] = std::max(p1[2], p[2]);
	}

	Vect3d diagonal() {
		return p1 - p0;
	}
};

/* polygonal mesh */
struct polygon_mesh { 
	std::vector<Vect3d> verts;

	void normalize(float scale_fact=1.0f) {
		if (verts.empty())
			return;

		aabb poly_aabb(verts[0]);
		for(auto &v:verts) {
			poly_aabb.add(v);
		}

		float diag_length = poly_aabb.diagonal().Length();
		float scale = scale_fact * 1.0f / diag_length;
		for(auto &v:verts) {
			v = v * scale;
		}
	}
};

void tree2mesh(std::shared_ptr<tree_node> head, int subdivision_num, polygon_mesh &out_mesh);
