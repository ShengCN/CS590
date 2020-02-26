#include "lab03.h"
#include <cassert>
#include <iostream>
#include <set>

void traverse_tree(std::shared_ptr<tree_node> &cur_node, polygon_mesh &out_mesh) {
	assert(cur_node != nullptr);

	// a --- b
	// | \   | \
	// |  e--|--f
	// c --- d  |
	//  \ |   \ |
	//    g----- h
	auto add_box = [&](std::vector<Vect3d> &verts,
					   Vect3d &a, Vect3d &b, Vect3d &c, Vect3d &d,
					   Vect3d &e, Vect3d &f, Vect3d &g, Vect3d &h) {

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

	Vect3d a, b, c, d, e, f, g, h;
	cur_node->get_box(a, b, c, d, e, f, g, h);
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
void tree_node::get_box(Vect3d &a, Vect3d &b, Vect3d &c, Vect3d &d, Vect3d &e, Vect3d &f, Vect3d &g, Vect3d &h) {
	// head node
	if(parent_node == nullptr) {
		// we assume root is from (0,0,0)
		a = Vect3d(0.0f, 0.0f, 0.0f);
		Vect3d x(1.0f, 0.0f, 0.0f), y(0.0f, 1.0f, 0.0f), z(0.0f, 0.0f, 1.0f);
		b = a + x * width;
		c = a + z * thickness;
		d = b + c - a;

		e = a + y * height;
		f = b + y * height;
		h = d + y * height;
		g = c + y * height;

		return;
	}

	// normal node
}
