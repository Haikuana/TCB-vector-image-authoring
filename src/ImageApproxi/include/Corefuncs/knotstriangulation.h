#pragma once
#ifndef _KNOTSTRIANGULATION_H_
#define _KNOTSTRIANGULATION_H_

#include "Assistant/data_types.h"

class CKnotsTriangulation
{
public:
	CKnotsTriangulation(Mesh *mesh, map<unsigned, KnotData> *data, int deg);
	CKnotsTriangulation(map<unsigned, KnotData>* data, int deg);
	CKnotsTriangulation();
	void link_triangulation_procedure(CT &tri, vector<vector<TConfig2>> &alltconfigs);
	void set_mesh(Mesh *m);
	void set_kdtree(ANNkd_tree *tree);
	void set_knots(map<unsigned, KnotData> *pknots);
	void set_deg(int d);
	void set_triangulation_type(Triangulation_Type type);
	void set_correspondence(map<unsigned, unsigned> *cor);
	void set_optimization_algorithm(TriangulationOptimizationAlg opt);
	void set_corner_indices(vector<unsigned> *cornerIds);
	void set_collinear_knots_config(const vector<vector<PointTriple>>& collinear_k,
		vector<vector<PointTriple>> feature_bound_corner,
		vector<RuptureKnotConfig>	rupture_edgebasis_config,int ncollinear);

private:
	//algorithm for optimize edge connection
	void lop(CT &tri, vector<pair<unsigned, unsigned>> &convexLink);
	void lop(CT &tri, int niterations=1);
	void look_ahead(CT &tri);
	void look_ahead(CT &tri, vector<pair<unsigned, unsigned>> &convexLink);
	void simulated_annealing(CT &tri);
	bool flip_neighbour_four_edges(CT &tri, CT::Face_handle &f, int index, FaceInfo &orgFInfo, 
		FaceInfo &orgGInfo, std::vector<NeighbourFaceInfo> &neighbourInfos, std::map<EdgeInfo, Edge> &suspicionEdges);
	bool flip_one_edge(CT &tri, CT::Face_handle &f, int index, double orgCost, FaceInfo &restInfo, std::map<EdgeInfo, Edge> &suspicionEdges);
	bool flip_neighbour_four_edges(CT &tri, CT::Face_handle &f, int index, vector<pair<unsigned, unsigned>> &convexLink, 
		FaceInfo &orgFInfo, FaceInfo &orgGInfo, std::vector<NeighbourFaceInfo> &neighbourInfos, std::map<EdgeInfo, Edge> &suspicionEdges);
	bool flip_one_edge(CT &tri, CT::Face_handle &f, int index, vector<pair<unsigned, unsigned>> &convexLink, 
		double orgCost, FaceInfo &restInfo, std::map<EdgeInfo, Edge> &suspicionEdges);
	void face_plane_deviation(FaceInfo &info);
	void get_neighbour_four_face_info(CT &tri, CT::Face_handle &f, int index, vector<NeighbourFaceInfo> &neighbourInfos);
	bool edge_cost_function(CT::Face_handle &f, int index, double &cost);
	void flipped_edge_cost_function(CT &tri, CT::Face_handle &f, int index, double &cost, FaceInfo &fInfo, FaceInfo &gInfo);

	//triangulation
	void initial_triangulation(CT &tri);//construct a initial tri if no-input
	void extract_convex_hull();//get corner
	void multipleknots_perturbation_oncorner();//corner multiple knots
	void multipleknots_perturbation_onfeature(CT &tri);
	void high_degree_triangulation(vector<vector<TConfig2>> &alltconfigs);
	void triangulate_configs(Link &link, vector<TConfig2> &configs,int);
	void collect_configs(CT &tri, vector<vector<TConfig2>> &alltconfigs);
	void get_all_links(vector<TConfig2> &configs, vector<Link> &allLinks);
	void get_configs(CT &tri, Link &link, vector<TConfig2> &configs, int deg);
	void add_original_suspicions(CT &tri, std::map<EdgeInfo, Edge> &suspicionEdges, bool flag=false);
	void add_suspicions(CT &tri, Edge &e, std::map<EdgeInfo, Edge> &suspicionEdges);
	void add_suspicions(CT &tri, CT::Face_handle &f, int index, std::map<EdgeInfo, Edge> & suspicionEdges);

	//predicate
	bool is_flippable(CT &tri, Edge &e);
	bool is_corners(Edge &e);
	bool is_convex(CT &tri, Edge &e);
	bool is_finite(CT &tri, Edge &e);
	bool is_constrained(vector<pair<unsigned, unsigned>> &convexLink, Edge &e);
	void get_convex_link(Link &link, vector<pair<unsigned, unsigned>> &convexLink);
	void insert_link(CDT &tri, vector<pair<unsigned, unsigned>> &convexLink);
	bool is_constrained(CT &tri, CT::Finite_edges_iterator &e);
	bool is_constrained(CT &tri, Edge &e);

private:
	Mesh							*mesh;
	ANNkd_tree						*kdTree;
	map<unsigned, KnotData>			*pDataMaps;
	vector<unsigned>				*chIdx;
	int								degree;

	map<unsigned, unsigned>			*correspondence;
	
	vector<vector<PointTriple>>		collinear_knots_lines;
	int								ncollinear_rate;

	vector<vector<PointTriple>>		feature_boundto_corner;
	vector<RuptureKnotConfig>		rupture_basis_config;

	Triangulation_Type				triType;
	TriangulationOptimizationAlg	optAlg;
};

#endif