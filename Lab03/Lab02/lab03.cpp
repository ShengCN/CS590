#include "lab03.h"
#include <cassert>
#include <iostream>
#include <set>

#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtx/transform.hpp>

using glm::vec4;
using glm::mat4;
int tree_node::id = 0;

void traverse_tree(std::shared_ptr<tree_node> &cur_node, polygon_mesh &out_mesh) {
	assert(cur_node != nullptr);

	// a --- b
	// | \   | \
	// |  e--|--f
	// c --- d  |
	//  \ |   \ |
	//    g----- h
	auto add_box = [&](std::vector<vec3> &verts,
					   vec3 &a, vec3 &b, vec3 &c, vec3 &d,
					   vec3 &e, vec3 &f, vec3 &g, vec3 &h) {

						   verts.push_back(a); verts.push_back(b);
						   verts.push_back(b); verts.push_back(d);
						   verts.push_back(d); verts.push_back(c);
						   verts.push_back(c); verts.push_back(a);

						   verts.push_back(e); verts.push_back(f);
						   verts.push_back(f); verts.push_back(h);
						   verts.push_back(h); verts.push_back(g);
						   verts.push_back(g); verts.push_back(e);

						   verts.push_back(a); verts.push_back(e);
						   verts.push_back(b); verts.push_back(f);
						   verts.push_back(c); verts.push_back(g);
						   verts.push_back(d); verts.push_back(h);
	};

	vec3 a(0.0f),b(0.0f), c(0.0f), d(0.0f), e(0.0f), f(0.0f), g(0.0f), h(0.0f);
	cur_node->compute_box(a, b, c, d, e, f, g, h);
	add_box(out_mesh.verts, a, b, c, d, e, f, g, h);

	for (auto &c : cur_node->children_list) {
		traverse_tree(c, out_mesh);
	}
}

void tree2mesh(std::shared_ptr<tree_node> head, int subdivision_num, polygon_mesh &out_mesh) {
	assert(head != nullptr);

	out_mesh.verts.clear();
	traverse_tree(head, out_mesh);
}

// a --- b
// | \   | \
// |  e--|--f
// c --- d  |
//  \ |   \ |
//    g----- h
void tree_node::compute_box(vec3 &a, vec3 &b, vec3 &c, vec3 &d, vec3 &e, vec3 &f, vec3 &g, vec3 &h) {
	// head node
	vec3 x(1.0f, 0.0f, 0.0f), y(0.0f, 1.0f, 0.0f), z(0.0f, 0.0f, 1.0f);
	if(parent_node == nullptr) {
		// we assume root is from (0,0,0)
		a = vec3(0.0f, 0.0f, 0.0f);
		b = a + x * width;
		c = a + z * thickness;
		d = b + c - a;

		e = a + y * height;
		f = b + y * height;
		h = d + y * height;
		g = c + y * height;
	}
	else {
		auto deg2rad = [](float deg) {
			return deg / 180.0f * 3.1415926f;
		};

		// normal node
		// a,b,c,d are coming from parent node
		// e,f,g,h are coming from current node
		auto &abcd = parent_node->node_four_points;
		a = abcd[0]; b = abcd[1]; c = abcd[2]; d = abcd[3];
		mat4 rot_mat = glm::rotate(deg2rad(global_rotation_deg), global_rotation_axis);
		x = rot_mat * vec4(x, 0.0f);
		y = rot_mat * vec4(y, 0.0f);
		z = rot_mat * vec4(z, 0.0f);

		vec3 center_abcd = (abcd[0] + abcd[1] + abcd[2] + abcd[3]) * 0.25f + y * height;
		e = center_abcd - 0.5f * width * x - 0.5f * thickness * z;
		f = center_abcd + 0.5f * width * x - 0.5f * thickness * z;
		g = center_abcd - 0.5f * width * x + 0.5f * thickness * z;
		h = center_abcd + 0.5f * width * x + 0.5f * thickness * z;
	}

	node_four_points.resize(4);
	node_four_points[0] = e;
	node_four_points[1] = f;
	node_four_points[2] = g;
	node_four_points[3] = h;
}
