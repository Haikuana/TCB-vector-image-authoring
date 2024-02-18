#pragma once
#ifndef CONSTRAINED_PARAMETERIZATION
#define CONSTRAINED_PARAMETERIZATION

#include "Assistant/data_types.h"
//#include "auxfunc.h"
#include <fstream>
#include "Assistant/parser.h"

#define PI 3.1415926

class ConstrainedPara
{
public:
	ConstrainedPara(Mesh *surface,int object_type);
	~ConstrainedPara();

public:
	void				compute_para_domain();

private:
	void				AssembleMatrixCoeff();
	void				SloveEquation();

	void 				CalculateAlpha(const Mesh::Vertex_iterator v_it);
	double				direction(pair<double, double>, pair<double, double>, pair<double, double>);
	bool				OnSegment(pair<double, double>, pair<double, double>, pair<double, double>);
	bool				SegmentIntersect(pair<double, double>, pair<double, double>, pair<double, double>, pair<double, double>);

private:
	Mesh				*surface;

	int					type_object;//0-domain, 1-point
	int					num_vertex_;
	int					num_inner_vertex;
	int					num_bound_vertex;

	SpMat				matrix_assemble;
	MatrixXd			alpha_;
	
//deformation
public:
	void				mesh_deformation();


};







#endif
