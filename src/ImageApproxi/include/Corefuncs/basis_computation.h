#pragma once
#ifndef _BASIS_COMPUTATION_H_
#define _BASIS_COMPUTATION_H_

#include "Assistant/data_types.h"

//basis configuration
void generate_basis_configuraitons(vector<TConfig2> &tconfigs, BSplineBasis &bSplineBasis);
void remove_singularities(BSplineBasis &bSplineBasis, vector<unsigned> &cornerIds, map<int, int> &controlpointpair);

//basis value
void compute_all_basis(Mesh *mesh, BSplineBasis &bSplineBasis, map<unsigned, KnotData> &dataMaps, 
	vector<TConfig3> &alltconfig3s, vector<Point_2> &suppledata, vector<Point_2> &supplefdata,vector<Point_2> &supplefcdata);
void update_basic_tconfigurations(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, Mesh *mesh, map<unsigned, KnotData> &dataMaps);
void compute_tconfig_basis_support(TConfig3 &tconfig, Mesh *mesh, map<unsigned, KnotData> &dataMaps);
void compute_tconfig_basis_data(TConfig3 &tconfig, map<unsigned, KnotData> &dataMaps);
void update_basis_support(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs);
bool update_basis_data(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, map<unsigned, KnotData> &dataMaps);

//supplement data
bool replenish_basis_data(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, map<unsigned, KnotData> &dataMaps, 
	int current_datasize, vector<Point_2> &suppledata);
void replenish_feature_data(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, map<unsigned, KnotData> &dataMaps, 
	int current_datasize, vector<Point_2> &suppledata);

//simplex
void compute_simplex_spline(vector<Point_2> simplexps, vector<Point_2> sampleps, vector<DomainValue> &values);

//for sample points
void compute_sample_basis(vector<Point_2> &sample_points, BSplineBasis &bSplineBasis, map<unsigned, KnotData> &dataMaps, vector<TConfig3> &alltconfig3s);
void update_sample_basic_tconfigurations(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, vector<Point_2> &sample_point, map<unsigned, KnotData> &dataMaps);
void compute_tconfig_sample_basis_support(TConfig3 &tconfig, vector<Point_2> &sample_point, map<unsigned, KnotData> &dataMaps);

//for basis derivate at a random direction
void compute_basis_derivate_forG1(vector<SurfaceTangentForG1> &inputdata, BSplineBasis &bSplineBasis, map<unsigned, KnotData> &dataMaps,vector<TConfig3> &alltconfig3s);
void update_basis_tconfigurations_forG1(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, vector<SurfaceTangentForG1> &inputdata, map<unsigned, KnotData> &dataMaps);
void compute_tconfig_basis_data_forG1(TConfig3 &tconfig, map<unsigned, KnotData> &dataMaps);
void update_basis_supportanddata_forG1(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, map<unsigned, KnotData> &dataMaps);

//useless yet
bool generate_coplanar_condition(Mesh *mesh, BSplineBasis &bSplineBasis, map<unsigned, KnotData> &dataMaps);
#endif
