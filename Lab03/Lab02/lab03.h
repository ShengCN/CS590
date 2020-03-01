#pragma once
#include <random>
#include <vector>
#include <functional>

#include <sstream>
#include <iostream>
#include <glm/glm.hpp>
using glm::vec3;

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

struct edge;
struct face;

struct point {
	vec3 pos;
	std::vector<std::shared_ptr<edge>> edges;

	point(vec3 p) :pos(p) {}
};

struct edge {
	std::shared_ptr<point> p1, p2;
	std::vector<std::shared_ptr<face>> faces;

	edge(std::shared_ptr<point> p1,
		 std::shared_ptr<point> p2) :p1(p1), p2(p2) {
	}
};

// assume quadrilateral 
struct face {
	std::shared_ptr<edge> e1, e2, e3, e4;
	face(std::shared_ptr<edge> e1,
		 std::shared_ptr<edge> e2,
		 std::shared_ptr<edge> e3,
		 std::shared_ptr<edge> e4) :
		e1(e1), e2(e2), e3(e3), e4(e4) {
	}
};

void add_edge(std::shared_ptr<point> p1,
			  std::shared_ptr<point> p2,
			  std::shared_ptr<edge> &e);

void add_face(std::shared_ptr<edge> e1,
			  std::shared_ptr<edge> e2,
			  std::shared_ptr<edge> e3,
			  std::shared_ptr<edge> e4,
			  std::shared_ptr<face> &f);


/* Data structure to define a tree node */
struct tree_node {
	std::vector<std::shared_ptr<tree_node>> children_list; // in this experiment, size will only be in range [0,3]
	std::shared_ptr<tree_node> parent_node;
	float width, height, thickness;
	vec3 global_rotation_axis;
	float global_rotation_deg;
	std::vector<std::shared_ptr<point>> node_four_points;

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

	void compute_box_point(std::shared_ptr<point> &a, 
					 std::shared_ptr<point> &b, 
					 std::shared_ptr<point> &c, 
					 std::shared_ptr<point> &d,
					 std::shared_ptr<point> &e, 
					 std::shared_ptr<point> &f, 
					 std::shared_ptr<point> &g, 
					 std::shared_ptr<point> &h);
};

/* polygonal mesh */
struct polygon_mesh { 
	std::vector<std::shared_ptr<face>> faces;

	void normalize(float scale_fact, glm::vec3 &scale, glm::vec3 &center);
};

void tree2mesh(std::shared_ptr<tree_node> head, int subdivision_num, polygon_mesh &out_mesh);

void catmull_clark_subdivision(std::vector<std::shared_ptr<face>> in_face,
							   std::vector<std::shared_ptr<face>> &out_face);