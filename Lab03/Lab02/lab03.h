#pragma once
#include <random>
#include <vector>
#include <functional>
#include <chrono>

#include <sstream>
#include <iostream>
#include <glm/glm.hpp>
using glm::vec3;

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
	vec3 global_rotation_axis;
	float global_rotation_deg;
	std::vector<vec3> node_four_points;

	static int id;
	int cur_id;

	// functions
	tree_node(std::shared_ptr<tree_node> parent, float w, float h, float t, vec3 axis,float r):
		parent_node(parent) , width(w), height(h), thickness(t), 
		global_rotation_axis(axis), global_rotation_deg(r) {
		id++; cur_id = id;
	}

	void add_child(std::shared_ptr<tree_node> &child_ptr) {
		children_list.push_back(child_ptr);
	}

	void compute_box(vec3 &a, vec3 &b, vec3 &c, vec3 &d,
				 vec3 &e, vec3 &f, vec3 &g, vec3 &h);
};

struct aabb {
	vec3 p0, p1;

	aabb(vec3 p) {
		p0 = p1 = p;
	}

	void add(vec3 p) {
		p0[0] = std::min(p0[0], p[0]);
		p0[1] = std::min(p0[1], p[1]);
		p0[2] = std::min(p0[2], p[2]);

		p1[0] = std::max(p1[0], p[0]);
		p1[1] = std::max(p1[1], p[1]);
		p1[2] = std::max(p1[2], p[2]);
	}

	vec3 diagonal() {
		return p1 - p0;
	}
};

/* polygonal mesh */
struct polygon_mesh { 
	std::vector<vec3> verts;

	void normalize(float scale_fact=1.0f) {
		if (verts.empty())
			return;

		aabb poly_aabb(verts[0]);
		for(auto &v:verts) {
			poly_aabb.add(v);
		}

		float diag_length = glm::length(poly_aabb.diagonal());
		float scale = scale_fact * 1.0f / diag_length;
		for(auto &v:verts) {
			v = v * scale;
		}
	}
};

void tree2mesh(std::shared_ptr<tree_node> head, int subdivision_num, polygon_mesh &out_mesh);
