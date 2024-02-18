#pragma once

#ifndef _AUXFUNC_H_
#define _AUXFUNC_H_

#include "Assistant/data_types.h"
#include "Assistant/ColorRamp.h"
#include "Assistant/parser.h"
#include <QPainter>
#include <QColor>
#include <fstream>

inline Vpair make_epair(int v1, int v2) {
	if (v1 < v2)
		return Vpair(v1, v2);
	else
		return Vpair(v2, v1);
}

inline std::pair<Vpair,Vpair> make_dpair(int v1, int v1t,int v2,int v2t) {
	if (v1 < v2)
	{
		Vpair e1(v1, v2);
		Vpair e2(v1t,v2t);
		return std::pair<Vpair, Vpair>(e1, e2);
	}
	else
	{
		Vpair e1(v2, v1);
		Vpair e2(v2t, v1t);
		return std::pair<Vpair, Vpair>(e1, e2);
	}
}

//sort
bool SortFunction0(BaryPoint p1, BaryPoint p2);
bool SortFunction1(BaryPoint p1, BaryPoint p2);
bool SortFunction2(BaryPoint p1, BaryPoint p2);
bool sortintPair_SecondGreater(pair<int, int> p1, pair<int, int> p2);
bool sortintPair_SecondSmaller(pair<int, int> p1, pair<int, int> p2);
bool sortFaceGreater(FaceSort f1, FaceSort f2);
bool sortConfigGreater(TConfig2 c1, TConfig2 c2);
bool sortdoublePair_SecondGreater(pair<int, double> p1, pair<int, double> p2);
bool sortdoublePair_SecondSmaller(pair<int, double> p1, pair<int, double> p2);
bool sortspoterr_Smaller(PixelAttribute p1, PixelAttribute p2);

//test
void draw_test_basis(Mesh *mesh_);
void remove_same_elements(vector<int> &temp_set);
void remove_same_elements(vector<Point_2> &temp_set);
void remove_same_elements(vector<Point2withIndex> &temp_set);

//data transform
void from_vfdata_to_mesh(Mesh **temp_mesh, vector<Point_3>&vs,vector<vector<int>>&faces);
void from_triangulationdata_to_mesh(CDT &temp_cdt, Mesh **temp_mesh, vector<double>	&vertice_cdt_value);
void from_triangulationdata_to_mesh(CDT_Refine &temp_cdt, Mesh **temp_mesh, vector<double>	&vertice_cdt_value);
void from_cdt2mesh(CDT &temp_cdt, Mesh **temp_mesh);

//one ring
void from_interior_to_two_ring_facets(int inte, set<Mesh::Facet_iterator> &teo_ring_face, Mesh**tempmesh);
void from_interior_to_adjacant_facets(int inte, int sz_ring, set<Mesh::Facet_iterator> &ring_face, Mesh**tempmesh);
void from_interior_to_ccw_one_ring_cycle(const vector<int> &interior_points, vector<int> &one_ring_cycle, Mesh **originalmesh);

//"findMaximumDistance" used as part of implementation for RDP algorithm.
std::pair<int, double> findMaximumDistance(std::vector<PointTriple>& Points, double &error);
void simplifyWithRDP(std::vector<PointTriple>& Points, double epsilon, bool do_consider_curvature=1);

const std::pair<int, double> findMaximumDistance(const std::vector<Point2withIndex>& Points);
std::vector<Point2withIndex> simplifyWithRDP(std::vector<Point2withIndex>& Points, double epsilon);

double dis_between_p_line(Point_2 line_p1, Point_2 line_p2, Point_2 tempp);

void search_neighboring_pixels(pair<int,int> center, vector<pair<int,int>> &region,int nfield, 
	vector<pair<int, int>> select_ps);

void screen_controlps_outside_feature_polygon(CMInfo* cm_, vector<int> current_index_, 
	vector<vector<CMVertexInfo>>control_points_seqs, vector<vector<PointTriple>> collinear_knots,
	Mesh *knot_mesh, Mesh *fitted_mesh, OneStepInfo *fittingInf, QString output_file);
void screen_controlps_outside_feature_polygon(CMInfo* cm_, vector<int> current_index_, vector<int> &selected_index_,
	vector<vector<CMVertexInfo>>control_points_seqs, vector<vector<PointTriple>> collinear_knots,
	Mesh *knot_mesh, Mesh *fitted_mesh, OneStepInfo *fittingInf, QString output_file);
void restore_controlps_outside_feature_polygon(CMInfo* cm_);

bool find_onering_pixels(cv::Point center,vector<cv::Point> ring,vector<cv::Point> & out_);

//predicate
bool is_singular(Link &link);
vector<Link>::iterator find_links(vector<Link> &allLinks, vector<unsigned> &interior);
void points_in_bbox(Mesh *mesh, ANNkd_tree *kdTree, vector<Point_2> &pts, vector<Vertex_iterator> &vIndices);
void points_on_polygon(Mesh *mesh, ANNkd_tree *kdTree, vector<Point_2> &pts, vector<Vertex_iterator> &indices);
void points_on_polygon(Mesh *mesh, ANNkd_tree *kdTree, vector<Point_2> &pts, FaceInfo &info);
void points_on_polygon(Mesh *mesh, vector<Point_2> &pts, FaceInfo &info);
void points_on_polygon(Mesh *mesh, ANNkd_tree *kdTree, vector<Point_2> &pts, vector<SupportInfo> &supportInfo);
void points_on_polygon(Mesh *mesh, ANNkd_tree *kdTree, vector<Point_2> &pts, vector<SupportInfo> &supportInfo, bool flag);
bool is_on_polygon_convex_bound(Point_2 temp, const vector<Point_2> poly_convex);
bool is_point_on_segment(Point_2 p, Segment_2 seg_, double precision);
bool is_on_domain_bound(Point_2 a);
int domain_bound_type(Point_2 a);
bool is_on_domain_corner(Point_2 a);
void points_on_polygon(Mesh *mesh, vector<Point_2> &pts, map<unsigned, DomainValue> &supports);
void points_on_polygon(vector<Point_2> &pts, FaceInfo &dstInfo, FaceInfo &srcInfo);
void points_on_polygon(vector<Point_2> &fPts, FaceInfo &fInfo, vector<Point_2> &gPts, FaceInfo &gInfo, FaceInfo &srcInfo);
void points_on_polygon(vector<Point_2> &sample_point, vector<Point_2> &pts, map<unsigned, DomainValue> &supports);
void points_on_polygon(vector<SurfaceTangentForG1> &sample_point, vector<Point_2> &pts, map<unsigned, DomainValue> &supports);

//configurations and basis
void update_tconfig3s(std::vector<TConfig3> &tconfigs);
void combine_near_all(std::vector<unsigned> a, std::vector<std::vector<unsigned>> &combs);
void combine(std::vector<unsigned> a, int n, int m, std::vector<unsigned> &b, const int M, std::vector<std::vector<unsigned>> &combs);
int number_multiple_knot(vector<unsigned> &config, unsigned index);
void compute_centroid_triangulation(const vector<vector<TConfig2>> &alltconfigs1, const vector<vector<TConfig2>> &alltconfigs2, 
	map<unsigned, KnotData> &dataMaps, CMInfo &cmInfo);
int generate_control_mesh(BSplineBasis &bSplineBasis, CMInfo &cmInfo,
	vector<pair<int, int>> split_edgebasis_pair, double translation_length);

void from_corner_triple_to_constrains(int deg, int id_cn, 
	vector<unsigned int> corner_seq, vector<RuptureKnotConfig> &rupture_edge_basis_config);

//restore
void restore_index(map<unsigned, unsigned> &correspondence, vector<TConfig1> &configsk);
void restore_index(map<unsigned, unsigned> &correspondence, vector<TConfig2> &configsk);
void restore_index(map<unsigned, unsigned> &correspondence, vector<unsigned> &configsk);
void release_basis(SplineBasisConfigs &bConfig);
void release_basis(BSplineBasis &bSplineBasis);

//output
void centroid_triangulation_output(const CMInfo &cmInfo, QString fileName);
void control_mesh_output(const CMInfo &cmInfo, QString fileName);
void extract_real_control_mesh(const CMInfo &srcInfo, CMInfo &dstInfo);
void extract_selected_control_mesh(const CMInfo &srcInfo, CMInfo &dstInfo);
void mesh_output(Mesh *mesh, QString fileName);
void mesh_output(Mesh *mesh, QString fileName, bool bColor, bool bheight);
void coplanar_info_output(const BSplineBasis &bSplineBasis, map<unsigned, KnotData> &dataMaps, QString fileName);
bool generate_output_directory(QString allDir);
bool generate_output_directory(QString allDir, QString prefix_, int removelast = 0);

//polygon operation
void union_two_ccw_region(vector<Point_2> &poly1, vector<Point_2> &poly2, vector<Point_2> &out);
void intersect_two_ccw_region(vector<Point_2> &poly1, vector<Point_2> &poly2, vector<Point_2> &out);

//error
void compute_mesh_error(Mesh *dstMesh, Mesh *refMesh, OneStepInfo &timeInfo);

#define GET_VECTOR(X, u, v)\
	(u) = CV_MAT_ELEM(*(X), double, 0, 0);\
	(v) = CV_MAT_ELEM(*(X), double, 1, 0);


#endif