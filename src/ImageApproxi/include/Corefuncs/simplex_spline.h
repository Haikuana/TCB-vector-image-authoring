#pragma once

#ifndef SIMPLEX_SIMPLINE_N_H
#define SIMPLEX_SIMPLINE_N_H

#include "Assistant/data_types.h"
#include "Assistant/geometric_predicates.h"

typedef GeometricPredicates_2d<1> Pred;

enum PointsSetType
{
	ONONEVERTEX,
	ONLINE,
	NORMAL
}; 

class Simplex_Spline_n
{
public:
	Simplex_Spline_n(double **X, int degree, double xmax, double ymax);
	Simplex_Spline_n(double **X, int degree);

	double computeSimplexSpline(double *pt);
	double firstdirectionalderivate(double* pt, double *direction);
	double seconddirectionalderivate(double* pt, double *firstdirection, double *seconddirection);
	double computePolorCoefficient(double *pt);

private:
	PointsSetType findTriangle(int& a, int& b, int&c, double& area);
	double** pts;
	int deg;
	bool is_bound;
	Pred gp;

	//double small_perturb;
};

#endif