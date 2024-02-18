#pragma once
#ifndef IMAGEAPPROXIMATION_H
#define IMAGEAPPROXIMATION_H

#include "Assistant/data_types.h"
#include "Corefuncs/simplex_spline.h"
#include "Assistant/geometric_predicates.h"
#include "Corefuncs/fitting.h"
#include "Corefuncs/knotstriangulation.h"
#include "Assistant/parser.h"
#include "Corefuncs/auxfunc.h"
#include "Corefuncs/sample_drawfunc.h"
#include "Corefuncs/basis_computation.h"
#include "Corefuncs/constrained_parameterization.h"
#include "OptimizeKnotTri/vmg_scene.h"
#include "ImageFeatureEdge/ED.h"
#include "ImageFeatureEdge/EDColor.h"
#include <fstream>
#include <algorithm>
#include <QDate>
#include <QTime>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#include <OptimizeKnotTri/stb_image.h>

#define COLORGRAY 1
#define REMOVABLE_ONE_ONLY 1

#define OPTIMIZE_GEO 0

#define TOOLBARWIDTH 150
#define BUTTONWIDTH 35
#define BUTTONHEIGHT 25
//RGB2GRAY: 0.2989R+ 0.5870G + 0.1140B

using namespace std;

class Image_Approximation
{
public:
	Image_Approximation(void);
	~Image_Approximation(void);

	//main function
	void					mesh_fitting();

	//fitting option set
	void					set_fitted_surface_degree(int d);
	void					set_triangulation_type(int type);
	void					set_triangulation_optimization_alg(int opt);
	void					set_initial_parameter(double gradient_thres, double anchor_thres,
		double anchor_width, double min_length, int initial_knots, int num_ite);

	//file IO
	bool					read_surface(const QString &name);
	bool					write_surface(const QString &name);
	bool					read_image(const QString &name);
	bool					write_image(const QString &name);
	bool					read_feature_lines_set(const QString &name);

	bool					read_editdata(const QString name, EditFM &fm_,
		CPs &cp_seqs, EditCM &cm_, QPoint &);
	bool					write_editdata(const QString name);

	//data output
	vector<vector<CMVertexInfo>> get_control_points_seqs();
	vector<vector<PointTriple>>	&get_feature_lines(bool is_original);
	vector<vector<PointTriple>>	&get_knots_lines();
	QImage 					*get_input_image();
	QString					get_output_filename();
	Point_2					get_image_size();
	QImage 					*get_approxi_image();
	void					get_supplement_data(vector<Point_2>&, vector<RGBPoint>&,
							vector<Point_2>&, vector<RGBPoint>&, vector<Point_2>&, vector<RGBPoint>&);
	CDT						*get_knots_config_ct();
	Mesh					*get_knots_config_mesh();
	CDT						*get_inserted_knots_config();
	Mesh					*get_originalsurface_mesh();
	Mesh					*get_fittedsurface_mesh();
	CMInfo					*get_control_mesh();
	bool					is_parameterization();
	BSplineBasis			*get_basis_configs();
	OneStepInfo				*get_fitting_infor();
	bool					&check_insert_feature_neighbor_knots();

	void					processForSimplification();

	//image sample display
	void					sample_local_image(vector<pair<MatrixXd,MatrixXd>> &mtin, vector<MatrixXd> &mtout);
	void					sample_local_image(Point_2 p1, Point_2 p2, vector<vector<pair<int, bool>>> &cdt_, vector<RGBPoint> &);
	void					sample_feature_region(vector<Point_2> &sample_domain, vector<RGBPoint> &image_values);
	void					sample_feature_curve_domainps();
	void					sample_local_image(Point_2 psta, Point_2 pend, CDT &cdt_, vector<RGBPoint> &);
	void					resample_image(vector<vector<pair<int, bool>>> &cdt_, vector<RGBPoint> &values_);
	void					sample_feature_line(vector<vector<RGBPoint>>&);
	void					rasterize_image(int magnified_level);//wrong

	//get and set model view
	void					GetCameraPosAndOrientation(vector<vector<double>> &pos_orient);
	void					SetCameraPosAndOrientation(vector<vector<double>> pos_orientt);
	bool					&is_fixed_model_view();
	void					set_model_view(bool);

	///////////////////////////////////////////////////////////////////////////////////////////////////////test begin
	//test simplex config and basis normalization
	vector<Point_2>			wrong_points;
	vector<Point_2>			get_basis_normalize_wrongp();

	//basis test
	void					MakeTestKnotsDomain();
	void					compute_test_basis();
	Mesh					*mesh_basis;
	CDT_Refine				cdt_domain;
	SplineBasisConfigs		basis_config;

	//test image feature
	void					extract_image_feature();
	vector<cv::Point2i>		return_image_anchor();
	vector<cv::Point2i>		image_anchor;//test

	//test insert collinear knots
	void					build_collinear_knots_test();

	void					optimize_knots_triangulation_delaunay();

	//draw: re-sampled image and knots tri
	void					get_resampled_image_and_knotstri(QImage &image_,Mesh &mesh_);

	//test polyline error
	void					matching_feature_with_polyline_test(double);

	//test optimize knots
	void					Init_knots_triangulation_test();
	void					optimize_knots_triangulation_test();

	//test parameterization
	void					compute_constrained_parameterization_test();

	//error test
	double					err_thres;
	double					&get_error_threshold();
	//test
	int						&get_numberof_inserted_knots();
	void					insert_new_knots_through_error();
	void					insert_new_knots_through_threshold(Point_2 psta = Point_2(0, 0),
		Point_2 pend = Point_2(0, 0), bool do_select_region = false);
	void					get_inserted_regions(vector<pair<int, int>>	&, vector<vector<pair<int, int>>> &);
	vector<vector<pair<int, int>>> insert_regions;
	vector<pair<int, int>>	insert_pixels;

	void					gene_control_mesh(QString filename, CMInfo& realCm);
	///////////////////////////////////////////////////////////////////////////////////////////////////////test end

public:
	//file IO
	bool					read_mesh(Mesh **pmesh, const QString &name);
	bool					write_mesh(Mesh *, const QString &, bool, bool, bool);
	
	//data transform
	void					image2cdt(QImage *image_, CDT &cdt_image, vector<double> &);
	void					image2mesh(QImage *image_,Mesh **mesh_);
	void					resample_domain_image_on_mesh();
	void					mesh2image();
	void					compute_originalcolor_at_domainpoint(Point_2 pt, vector<int> &triangles_int, RGBPoint &pout);
	void					compute_parametepoint_at_imagedpoint(Point_2 pimage, Point_2 &pparameter);

	///////////////////////////////////////////////////////////////////////////////////////////////////////fitting process begin
	//first stage: extract image feature
	void					compute_image_feature();
	void					smooth_and_sample_feature_curve_points();
	void					project_featureps_onto_smoothps();
	void					remedy_color_of_one_neighbor();
	//second stage: parameterization
	void					compute_constrained_parameterization();//parameterization
	void					create_kd_tree();//image-mesh to kd-tree
	//third stage: making knots
	bool					build_collinear_knots();//making collinear knots
	void					matching_feature_with_polyline(double error_);//polyline feature
	void					optimize_knots_triangulation();
	//link triangulation
	void					knots_transform();//knots data from cdt to map
	void					generate_configs_ofsplitbasis();
	void					link_triangulation_procedure();
	//fourth stage: compute basis
	void					remove_degraded_simplex(vector<TConfig2> &configs);//maybe useless
	void					merge_linearly_dependent_simplex(vector<TConfig2> &configs);//merge simplex splines which are linearly dependent
	void					splite_rupture_basis();//splitting basis behind feature
	void					verify_basis_normalizng();//verify basis value and basis derivate
	void					compute_supplement_5Ddata();//supplement data for null support basis
	//fifth stage: fitting control point
	void					initial_control_points();//maybe useless, which depend on optimization method
	void					generate_G1_pairs();
	void					generate_control_mesh();
	void					postprocess_feature_corner_controlps();
	//last: result output
	void					archive_results();
	QString					generate_output_dir();
	void					control_mesh_output(QString fileName);
	void					reconstruction_info_output(QString fileName);
	void					compute_mesh_error();
	void					insert_collinear_knots_through_error();
	///////////////////////////////////////////////////////////////////////////////////////////////////////fitting process end

public:
	//input data
	QImage							*input_image;
	Mesh							*input_imageMesh;
	double							imagemesh_dx;
	double							imagemesh_dy;

	//fitting data
	Fitting							*fitting;
	Mesh							*fittedMesh;
	QImage							*fitted_image;
	Mesh							*originalMesh;

	//fitting options
	int								Nite_now;
	double							image_bound_weight;
	double							feature_pos_weight; 
	double							pos_fair_firder_wei; 
	double							pos_fair_sedder_wei;
	double							feature_color_1band_weight;
	unsigned						nDeg;
	Triangulation_Type				triType;
	TriangulationOptimizationAlg	optAlg;

	//image feature
	int								gradient_threshold;
	int								anchor_threshold;
	int								anchor_width;
	int								min_legth_feature;
	vector<vector<PointTriple>>		image_feature_lines_set;
	vector<vector<PointTriple>>		feature_lines_modify;
	bool							do_consider_cirlcewane;
	bool							has_compute_feature;
	int								degree_ofsmoothing_feaP;

	//polyline feature
	double							simply_error;
	bool							bsimplyfeature;
	
	//parameterization
	bool							bparameterization;
	ANNkd_tree						*kdTree_newdomain;
	ANNkd_tree						*kdTree_olddomain;
	bool							is_build_domain_kd_tree;

	//knots collinear
	bool							do_insert_feature_neighbor_knots;
	int								nknotinsert_collinear;
	vector<vector<PointTriple>>		collinear_knots_lines;
	vector<vector<PointTriple>>		feature_bound_corner;//0-bound side;1-center;2-line side;3-further line side
	vector<PointTriple>				Norepeat_knot;
	bool							do_compute_collinear_knots;
	//knots data
	OptiKnotTri						*opti_knottri;
	int								nknotinsert_opti;
	int								niterate_opti;
	double							remove_criteria_closeto_featureedge;
	CDT								*knots_config;//normalize
	Mesh							*knot_mesh;
	std::map<unsigned, KnotData>	dataMaps;
	int								nknot;
	//knot LTP
	CKnotsTriangulation				*ltp;
	vector<vector<TConfig2>>		alltconfigs;

	//sample original points
	vector<Point_2>					featureregion_sampleps;
	vector<RGBPoint>				featureregion_originalps;

	vector<Point_2>					supplementdomaindata;
	vector<RGBPoint>				supplement5Ddata;

	vector<Point_2>					featurecurve_sampleps;
	vector<RGBPoint>				featurecurve_originalps;
	//vector<vector<PointTriple>>		feature_lines_backups;
	int								sample_feature_curve_rate;

	//basis
	map<unsigned, unsigned>			correspondence;
	vector<unsigned>				cornerIds;
	vector<TConfig3>				alltconfig3s;
	BSplineBasis					bSplineBasis;

	//split feature basis
	vector<RuptureKnotConfig>		rupture_edge_basis_config;

	//control points pair
	vector<vector<CMVertexInfo>>	control_points_seqs;
	vector<pair<int, int>>			basis_point_pair;//basis split by feature edge
	map<int, int>					basis_index_transform;
	CMInfo							cmInfo; //control mesh

	//G1 constrains: 
	vector<G1Corner>				G1_constrains;

	//fitting information
	QString							fileName;
	QString							last_file_name;
	QString							out_file_name;
	StatisticsInfos					statisticsInfos;

	//knot mesh iterate
	CDT								*new_knot_config;
	int								nknotinsert_ite;
	int								nfixedknots;
	vector<pair<int, double>>		faceerror_sort;
	int								Nite_add_collinear_knots;
	double							threshold_error;
	QString							filename_foraddcollinear_knots;

	//refinement
	double							local_refine_rate_fea;
	double							local_refine_rate;
	double							whole_refine_rate_fea;
	double							whole_refine_rate;

	//model view
	vector<vector<double>>			model_position_orientation;
	bool							is_fixed_modelview;
};

#endif
