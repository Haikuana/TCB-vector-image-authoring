#pragma once
#ifndef _SURFACE_FITTING_
#define _SURFACE_FITTING_

#include "Assistant/enriched_polyhedron.h"
#include "Assistant/data_types.h"
#include "Assistant/fairing.h"
#include <sparse_quad_prog.h>

class Fitting
{
public:
	Fitting(void);
	Fitting(Mesh *origin_mesh, Mesh *fitted_mesh,QString);
	Fitting(Mesh *origin_mesh, Mesh *fitted_mesh, QString filename, double image_bound_weight,
		double feature_pos_weight, double pos_fair_firder_wei, double pos_fair_sedder_wei,
		double feature_color_weight);
	~Fitting(void);
	void set_fairing_weight(double,double);
	void set_original_mesh(Mesh *mesh);
	void set_fitted_mesh(Mesh *mesh);
	void set_basis(BSplineBasis *basis);
	void set_allsimplex(vector<TConfig3> *);
	void set_knots_data(std::map<unsigned, KnotData>);
	void set_constrains(vector<pair<int, int>> cpc, vector<G1Corner> G1_constrains);
	void set_supplement_data(vector<RGBPoint> &basisdata, vector<RGBPoint> &featuredata, vector<RGBPoint> &featurecdata);
	void fitting(OneStepInfo &timeInfo);

private:
	void assemble_known_matrix();
	void set_initial_value(mwArray &X0);
	void compute_range_box();
	void assemble_constraint_matrix(mwArray &Aeq, mwArray &beq);
	void assemble_range_matrix(mwArray &lb, mwArray &ub);
	void assemble_coefficient_matrix(mwArray &A, mwArray &Af, mwArray &Aboundary, mwArray &AfeaColor);
	void assemble_fairing_matrix(mwArray &);
	void extract_control_points(mxDouble *cps, int nNum);
	void update_fitted_mesh(mwArray &Ax);

private:
	QString							ImageFileName;

	//position weights
	double							pos_feature_weight_origin;
	double							pos_adjacent_weight_origin;
	double							pos_boundary_weight;
	double							pos_supple_featuredata_weight;
	double							pos_featurecurve_weight;
	//color weights
	double							color_feature_weight;
	double							color_adjacent_weight;
	double							color_supple_featuredata_weight;
	double							color_featurecurve_weight;

	Mesh							*originalMesh;
	Mesh							*fittedMesh;
	BSplineBasis					*bSplineBasis;
	std::map<unsigned, KnotData>	dataMaps;
	vector<TConfig3>				*alltconfig3s;

	vector<RGBPoint>				supplement_basisdata;
	vector<RGBPoint>				supplement_featuredata;
	vector<RGBPoint>				supplement_featurecurvedata;

	CFairing						*cfair;
	double							firorder_weight_;
	double							secorder_weight_;
	double							max_A;

	vector<G1Corner>				G1_constrains;
	vector<pair<int, int>>			controlpointpair;
	vector<mxDouble>				bData;

	Iso_cuboid						box;
	int								basisNum;//number of basis plus 1
};
#endif

