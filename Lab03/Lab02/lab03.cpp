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

std::set<std::shared_ptr<edge>> edge_set;

void traverse_tree(std::shared_ptr<tree_node> &cur_node, polygon_mesh &out_mesh) {
	assert(cur_node != nullptr);

	// a --- b
	// | \   | \
	// |  e--|--f
	// c --- d  |
	//  \ |   \ |
	//    g----- h
	auto compute_face = [&](std::shared_ptr<point> &a, 
							std::shared_ptr<point> &b, 
							std::shared_ptr<point> &c, 
							std::shared_ptr<point> &d,
							std::shared_ptr<point> &e, 
							std::shared_ptr<point> &f, 
							std::shared_ptr<point> &g, 
							std::shared_ptr<point> &h,
							std::vector<std::shared_ptr<face>> &faces) {
						// point to edge
						std::shared_ptr<edge> ab, bd, dc, ca;
						std::shared_ptr<edge> ef, fh, hg, ge;
						std::shared_ptr<edge> ae, bf, dh, cg;

						add_edge(a, b, ab);
						add_edge(b, d, bd);
						add_edge(d, c, dc);
						add_edge(c, a, ca);

						add_edge(e, f, ef);
						add_edge(f, h, fh);
						add_edge(h, g, hg);
						add_edge(g, e, ge);

						add_edge(a, e, ae);
						add_edge(b, f, bf);
						add_edge(d, h, dh);
						add_edge(c, g, cg);

						faces.resize(4);
						// edge to face
						// add_face(ab, bd, dc, ca, faces[0]);
						add_face(ab, bf, ef, ae, faces[0]);
						add_face(bd, dh, fh, bf, faces[1]);

						add_face(dc, cg, hg, dh, faces[2]);
						add_face(ca, ae, ge, cg, faces[3]);
						//add_face(ef, fh, hg, ge, faces[5]);
	};

	std::shared_ptr<point> a, b, c, d, e, f, g, h;
	cur_node->compute_box_point(a, b, c, d, e, f, g, h);
	
	std::vector<std::shared_ptr<face>> faces;
	compute_face(a, b, c, d, e, f, g, h, faces);
	out_mesh.faces.insert(out_mesh.faces.end(), faces.begin(), faces.end());

	for (auto &c : cur_node->children_list) {
		traverse_tree(c, out_mesh);
	}
}

void add_edge(std::shared_ptr<point> p1, std::shared_ptr<point> p2, std::shared_ptr<edge> &e) {
	assert(p1 != nullptr && p2 != nullptr);
	for(auto &ed:edge_set) {
		if(ed->p1 == p1 && ed->p2 == p2) {
			e = ed;
			return;
		}

		if (ed->p1 == p2 && ed->p2 == p1) {
			e = ed;
			return;
		}
	}

	e = std::make_shared<edge>(p1, p2);

	p1->edges.push_back(e);
	p2->edges.push_back(e);

	edge_set.insert(e);
}

void add_face(std::shared_ptr<edge> e1, 
			  std::shared_ptr<edge> e2, 
			  std::shared_ptr<edge> e3, 
			  std::shared_ptr<edge> e4,
			  std::shared_ptr<face> &f) {
	assert(e1 != nullptr && e2 != nullptr && e3 != nullptr && e4 != nullptr);
	f = std::make_shared<face>(e1, e2, e3, e4);
	e1->faces.push_back(f);
	e2->faces.push_back(f);
	e3->faces.push_back(f);
	e4->faces.push_back(f);
}

void tree2mesh(std::shared_ptr<tree_node> head, int subdivision_num, polygon_mesh &out_mesh) {
	assert(head != nullptr);

	// initialize the tree vertices
	edge_set.clear();
	out_mesh.faces.clear();
	traverse_tree(head, out_mesh);

	// subdivision
	for(int i= 1; i <= subdivision_num; ++i) {
		std::vector<std::shared_ptr<face>> next_iter_faces;
		catmull_clark_subdivision(out_mesh.faces, next_iter_faces);
		out_mesh.faces = next_iter_faces;
	}
}

std::vector<vec3> face_points;
std::vector<vec3> edge_points;
std::vector<vec3> old_vert_points;

void catmull_clark_subdivision(std::vector<std::shared_ptr<face>> in_face, 
							   std::vector<std::shared_ptr<face>> &out_face) {
	face_points.clear(); edge_points.clear(); old_vert_points.clear();
	edge_set.clear();

	std::set<std::shared_ptr<point>> vertices;
	std::set<std::shared_ptr<edge>> edges;
	
	std::unordered_map <std::shared_ptr<face>, std::shared_ptr<point>> face_point_map;
	std::unordered_map <std::shared_ptr<edge>, std::shared_ptr<point>> edge_point_map;

	std::set<std::shared_ptr<point>> new_face_points;

	auto get_face_point = [&](vec3 face_point_pos) {
		for(auto &p:new_face_points) {

			if(glm::length(p->pos-face_point_pos) < 1e-2) {
				return p;
			}
		}

		std::shared_ptr<point> new_p = std::make_shared<point>(face_point_pos);
		new_face_points.insert(new_p);
		return new_p;
	};

	// compute new face point
	for(auto &f:in_face) {
		vec3 face_point_pos(0.0f);
		face_point_pos = (f->e1->p1->pos + f->e1->p2->pos + 
						  f->e2->p1->pos + f->e2->p2->pos + 
						  f->e3->p1->pos + f->e3->p2->pos + 
						  f->e4->p1->pos + f->e4->p2->pos) / 8.0f;
		face_point_map[f] = get_face_point(face_point_pos);

		vertices.insert(f->e1->p1);
		vertices.insert(f->e1->p2);

		vertices.insert(f->e2->p1);
		vertices.insert(f->e2->p2);

		vertices.insert(f->e3->p1);
		vertices.insert(f->e3->p2);

		vertices.insert(f->e4->p1);
		vertices.insert(f->e4->p2);

		edges.insert(f->e1);
		edges.insert(f->e2);
		edges.insert(f->e3);
		edges.insert(f->e4);
	}

	// compute new edge points
	for(auto &e:edges) {
		vec3 edge_point_pos(0.0f);
		int avg_num = 2;
		edge_point_pos += e->p1->pos; edge_point_pos += e->p2->pos;
		for(auto &f:e->faces) {
			avg_num++;
			edge_point_pos += face_point_map.at(f)->pos;
		}

		edge_point_pos = edge_point_pos / (float)avg_num;
		std::shared_ptr<point> new_edge_point = std::make_shared<point>(edge_point_pos);
		edge_point_map[e] = new_edge_point;
	}

	// recompute the old vertices positions
	// F + 2R + (n-3)p /n
	for(auto &p:vertices) {
		std::set<std::shared_ptr<point>> adj_face_point;
		vec3 f(0.0f),r(0.0f);

		for(auto &e:p->edges) {
			for(auto &f:e->faces) {
				adj_face_point.insert(face_point_map.at(f));
			}
			r += 0.5f * (e->p1->pos + e->p2->pos);
		}
		r = r / (float)p->edges.size();

		for(auto &adj_fp:adj_face_point) {
			f += adj_fp->pos;
		}
		f = f / (float)adj_face_point.size();

		float n = p->edges.size();
		p->pos = (f + 2.0f * r + (n - 3) * p->pos) / n;
		old_vert_points.push_back((f + 2.0f * r + (n - 3) * p->pos) / n);
	}

	auto find_same_face = [](std::shared_ptr<edge> a, std::shared_ptr<edge> b) {
		for(auto &af:a->faces) {
			for(auto &bf:b->faces) {
				if (af == bf)
					return af;
			}
		}

		std::shared_ptr<face> not_found = nullptr;
		return not_found;
	};

	auto find_edge = [](std::shared_ptr<point> p1, 
						std::shared_ptr<point> p2, 
						std::set<std::shared_ptr<edge>> &edge_set) {
		for(auto &e:edge_set) {
			if (e->p1 == p1 && e->p2 == p2) return e;

			if (e->p1 == p2 && e->p2 == p1) return e;
		}

		std::shared_ptr<edge> not_found = nullptr;
		return not_found;
	};

	// reconstruct the faces given those points
	std::set<std::shared_ptr<edge>> new_edges;
	for(auto &old_vert:vertices) {
		std::shared_ptr<point> new_p = std::make_shared<point>(old_vert->pos);
		auto &edges = old_vert->edges;

		// iterate over all the edge pairs 
		for(size_t ei = 0; ei < edges.size()-1; ++ei) {
			for(size_t eii = ei + 1; eii < edges.size(); ++eii) {
				auto ei_edge = edges[ei];
				auto eii_edge = edges[eii];
				auto ei_edge_point = edge_point_map.at(ei_edge);
				auto eii_edge_point = edge_point_map.at(eii_edge);

				auto the_face = find_same_face(ei_edge, eii_edge);
				if(the_face == nullptr) {
					continue;
				}

				auto face_point = face_point_map.at(the_face);
				
				std::shared_ptr<edge> e1, e2, e3, e4;
				e1 = find_edge(new_p, ei_edge_point, new_edges);
				if(e1 == nullptr) {
					add_edge(new_p, ei_edge_point, e1);
					new_edges.insert(e1);
				}
				
				e2 = find_edge(ei_edge_point, face_point, new_edges);
				if (e2 == nullptr) {
					add_edge(ei_edge_point, face_point, e2);
					new_edges.insert(e2);
				}

				e3 = find_edge(face_point, eii_edge_point, new_edges);
				if (e3 == nullptr) {
					add_edge(face_point, eii_edge_point, e3);
					new_edges.insert(e3);
				}

				e4 = find_edge(eii_edge_point, new_p, new_edges);
				if (e4 == nullptr) {
					add_edge(eii_edge_point, new_p, e4);
					new_edges.insert(e4);
				}

				std::shared_ptr<face> new_face;
				add_face(e1, e2, e3, e4, new_face);
				out_face.push_back(new_face);
			}
		}
	}

	for(auto &m:face_point_map)  face_points.push_back(m.second->pos);
	for (auto &m : edge_point_map)  edge_points.push_back(m.second->pos);
}

// a --- b
// | \   | \
// |  e--|--f
// c --- d  |
//  \ |   \ |
//    g----- h
void tree_node::compute_box_point(std::shared_ptr<point> &a, std::shared_ptr<point> &b, std::shared_ptr<point> &c, std::shared_ptr<point> &d, std::shared_ptr<point> &e, std::shared_ptr<point> &f, std::shared_ptr<point> &g, std::shared_ptr<point> &h) {
	// head node
	vec3 x(1.0f, 0.0f, 0.0f), y(0.0f, 1.0f, 0.0f), z(0.0f, 0.0f, 1.0f);
	if (parent_node == nullptr) {
		// we assume root is from (0,0,0)
		a = std::make_shared<point>(vec3(0.0f, 0.0f, 0.0f));
		b = std::make_shared<point>(a->pos + x * width);
		c = std::make_shared<point>(a->pos + z * thickness);
		d = std::make_shared<point>(b->pos + c->pos - a->pos);

		e = std::make_shared<point>(a->pos + y * height);
		f = std::make_shared<point>(b->pos + y * height);
		g = std::make_shared<point>(c->pos + y * height);
		h = std::make_shared<point>(d->pos + y * height);
	}
	else {
		auto deg2rad = [](float deg) {
			return deg / 180.0f * 3.1415926f;
		};

		// normal node
		// a,b,c,d are coming from parent node
		// e,f,g,h are coming from current node
		auto &abcd = parent_node->node_four_points;
		mat4 rot_mat = glm::rotate(deg2rad(global_rotation_deg), global_rotation_axis);
		x = rot_mat * vec4(x, 0.0f);
		y = rot_mat * vec4(y, 0.0f);
		z = rot_mat * vec4(z, 0.0f);

		a = abcd[0]; b = abcd[1]; c = abcd[2]; d = abcd[3];
		vec3 center_abcd = (abcd[0]->pos + abcd[1]->pos + abcd[2]->pos + abcd[3]->pos) * 0.25f + y * height;
		e = std::make_shared<point>(center_abcd - 0.5f * width * x - 0.5f * thickness * z);
		f = std::make_shared<point>(center_abcd + 0.5f * width * x - 0.5f * thickness * z);
		g = std::make_shared<point>(center_abcd - 0.5f * width * x + 0.5f * thickness * z);
		h = std::make_shared<point>(center_abcd + 0.5f * width * x + 0.5f * thickness * z);
	}

	node_four_points.resize(4);
	node_four_points[0] = e;
	node_four_points[1] = f;
	node_four_points[2] = g;
	node_four_points[3] = h;
}

void polygon_mesh::normalize(float scale_fact, glm::vec3 &scale, glm::vec3 &center) {
	if (faces.empty())
		return;

	aabb poly_aabb(faces[0]->e1->p1->pos);
	for (auto &f : faces) {
		poly_aabb.add(f->e1->p1->pos);
		poly_aabb.add(f->e1->p2->pos);

		poly_aabb.add(f->e2->p1->pos);
		poly_aabb.add(f->e2->p2->pos);

		poly_aabb.add(f->e3->p1->pos);
		poly_aabb.add(f->e3->p2->pos);

		poly_aabb.add(f->e4->p1->pos);
		poly_aabb.add(f->e4->p2->pos);
	}

	float diag_length = glm::length(poly_aabb.diagonal());
	scale = vec3(scale_fact * 1.0f / diag_length);

	center = (poly_aabb.p0 + poly_aabb.p1) * 0.5f;
}

