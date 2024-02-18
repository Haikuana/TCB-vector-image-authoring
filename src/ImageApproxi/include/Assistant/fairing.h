#pragma once
#ifndef CFAIRING_H
#define CFAIRING_H

#include "Assistant/data_types.h"
#include "Corefuncs/basis_computation.h"
#include "Corefuncs/auxfunc.h"
#include "Assistant/geometric_predicates.h"
#include "Assistant/triangle_dunavant_rule.hpp"

#define DO_COMPUTE_INSTEAD_READFILE 1

class CFairing
{

public:
	CFairing(const std::map<unsigned, KnotData>, BSplineBasis *, vector<TConfig3>*, QString);
	CFairing();
	~CFairing();

public:
	double	*testFunction(int deg);
	SpMat	&get_Bx();
	SpMat	&get_By();
	SpMat	&get_Bxx();
	SpMat	&get_Bxy();
	SpMat	&get_Byy();
	bool	&has_compute();
	void	compute_fairing_matrix();
	void	test_quadrature_precision(vector<int> simplex1,vector<int> simplex2);
	void	test_polygon_trigulation(vector<vector<Point_2>>& convex1,
			vector<vector<Point_2>>& convex2, vector<vector<vector<Point_2>>>& trifacets);

private:
	void	compute_simplex_derivate_innerproduct();
	void	compute_basis_derivate_innerproduct();

	void	triangulate_two_simplex_support(
			vector<Point2withIndex> simplex1, vector<Point2withIndex> simplex2,
			vector<vector<Point_2>> &facets, vector<Point_2> support_join);
	void	dunavant_quadrature_rule(int degree, Point_2 v1, Point_2 v2, Point_2 v3,
			vector<Point_2> &knots, vector<double> &weights, int &num_knots);
	void	compute_quadrature_knots_and_weights(int qua_degree, vector<vector<Point_2>> &facets,
			vector<Point_2> &quadrature_knots, vector<double> &quadrature_weights);

	void	generate_output_dir();

private:
	std::map<unsigned, KnotData>	dataMaps;
	BSplineBasis					*bSplineBasis;
	vector<TConfig3>				allconfigs;

	QString							OutFileName;
	vector<QString>					quad_file;

	bool			bhas_compute;

	int				nquadrature_sample;
	double			u_direction[2];
	double			v_direction[2];

	MatrixXd		matrix_u_simplex;
	MatrixXd		matrix_v_simplex;
	MatrixXd		matrix_uu_simplex;
	MatrixXd		matrix_uv_simplex;
	MatrixXd		matrix_vv_simplex;

	SpMat			matrix_u_basis;
	SpMat			matrix_v_basis;
	SpMat			matrix_uu_basis;
	SpMat			matrix_uv_basis;
	SpMat			matrix_vv_basis;

};
#endif