#pragma once

#ifndef _SAMPLEDRAWFUNC_H_
#define _SAMPLEDRAWFUNC_H_

#include "Assistant/data_types.h"
#include "Assistant/ColorRamp.h"
#include <QPainter>
#include <QColor>

//draw functions
void draw_image(QImage *image_);
void draw_image_anchor(vector<cv::Point2i> anchorp, QImage *image);
void draw_image_boundary(double dw, double dh);
void draw_quad(double x, double y, double pisel_l);
void draw_image_grid(QImage *image_);
void draw_image_knots_point(Mesh *knot_config_);
void draw_image_knots_tri(Mesh *knot_config_);

void draw_parameterization(Mesh *surface,bool);
void draw_parameter_image(Mesh * surface_);

void draw_featureregion_sample_points(vector<Point_2> sample_domain1, vector<RGBPoint> original_values1,
	vector<Point_2> sample_domain2, vector<RGBPoint> original_values2, vector<Point_2> sample_domain3, vector<RGBPoint> original_values3, Mesh *);
void draw_featureregion_original_points(vector<RGBPoint> sample_domain1, vector<RGBPoint> original_values2, vector<RGBPoint> original_values3);
//domain knots
void draw_domain_knots_pointcdt(CDT *knot_config_, Point_2 p = Point_2(-1, -1));
void draw_domain_knots_tricdt(CDT *knot_config_,Point_2 p = Point_2(-1,-1));
void draw_domain_knots_pointmesh(Mesh *knot_config_);
void draw_domain_knots_trimesh(Mesh *knot_config_);
void draw_domain_boundary();

//test knots optimize
void draw_knots_mesh_pf(Mesh *);
//test knots face error for iterate
void draw_knot_face_error(Mesh *);

void draw_image_error_regions(vector<pair<int, int>>pixels, vector<vector<pair<int, int>>> regions, QImage *image_);

//resample image and knots mesh
void draw_resampled_image_and_knottri(QImage image_, Mesh *mesh_ = NULL);

//feature ploy and collinear line
void draw_image_feature_lines(vector<vector<PointTriple>> polys_, int image_wid, int image_hei);
void draw_feature_points(vector<vector<PointTriple>> polys_);
void draw_feature_poly(vector<vector<PointTriple>>);
void draw_collinear_knots_lines(vector<vector<PointTriple>> polys_);

void draw_selected_feature_sequence(CMInfo* cm_, vector<int> current_index_, vector<vector<CMVertexInfo>>control_points_seqs);

//input mesh
void draw_original_surface_points(Mesh *mesh);
void draw_original_surface_edges(Mesh *mesh);
void draw_original_surface_faces(Mesh *mesh);

//fitted mesh
void draw_fitted_surface_points(Mesh *mesh);
void draw_fitted_surface_edges(Mesh *mesh);
void draw_fitted_surface_faces(Mesh *mesh);

//control mesh
void draw_control_mesh_points(CMInfo &cm_, int);
void draw_control_mesh_edges(CMInfo &cm_, int,Point_2 psize = Point_2(0,0));
void draw_control_points_featurepair(CMInfo &cmInfo);

//sample
void draw_image_error_bymesh(Mesh *temp_mesh,QImage *image);
void draw_image_error_based_onthres(Mesh *temp_mesh, QImage *image_, double thres);
void draw_fitted_image_bymesh(Mesh *mesh_temp);
void draw_feature_line_sample(vector<vector<RGBPoint>> flsp);
void draw_sample_image_byfacets(vector<vector<pair<int, bool>>> facets, vector<RGBPoint> values_);
void draw_sample_image_byfacets_lineloop(vector<vector<pair<int, bool>>> facets, vector<RGBPoint> values_);

//sample
void SamplingPoint2dInTriangleUniformly(const vector<Point_2> triagle, double insert_rate, vector<Point_2> &triangle_interior_points);
void SamplingPoint2dInTriangleCGAL(const vector<Point_2> triangle, double cgal_insert_rate, vector<Point_2> &triangle_interior_points);
void SamplingPoint2dInLineUniformly(const Point_2 p1, const Point_2 p2, double insert_rate, vector<Point_2> &sample_points);
void SamplingPoint2dInPolygonUniform(vector<Point_2> polygon, double sample_rate, vector<Point_2> &sample);
Point_2 ComputePolygonCentroid(vector<Point_2> polygon);
Point_2 ComputePolygonCentroid(Polygon_2 polygon);
void SamplingPoint2dInPolygonCGAL(vector<Point_2> polygon, double sample_rate, CDT_Refine &cdt_temp);

vector<double> return_box2d_from_vector_point(vector<Point_2> points_2d);

#endif