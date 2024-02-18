#pragma once
#ifndef GEOMETRIC_PREDICATES
#define GEOMETRIC_PREDICATES
#include <iostream>

/*
*  geometric_predicates.h
*  Description: a collection of simple geometric routines,
mostly just a wrapper for Johathan Schewchuk's predicates
*  Created by Yuanxin Liu on 1/30/08.
*  Dis
*/
#include <assert.h>
//#include "x_auxiliary_function.h"

extern double orient2d(double*, double*, double*);


enum LocateOfSamplePoint
{
	RIGHTBOTOMCORNER, LEFTTOPCORNER, RIGHTTOPSIDEBUTCORNER, NORMAL_OPEN
}; 

//if the perturbation policy is 1,
//  this means that the "test point" is moved infinitesmally in the direction (eps*eps,eps)
//This policy good enough if we are not concerned with resolving only a single
//  point with respect to a set of of points. 

template<int perturbation_policy>
class GeometricPredicates_2d
{
public:
	typedef int Sign;
	double xmax;
	double ymax;
	//signed area of a triangle
	double signedArea(double* a, double* b, double* c)
	{
		//double d = orient2d(a,b,c)*0.5;
		return orient2d(a, b, c)*0.5;
	}

	//returns pos, zero, neg if x is on the left, on, right side of the line through ab
	Sign sideOfLine(double* x, double* a, double* b)
	{
		//on point
		assert(!(abs(b[0] - a[0])<1e-8 && abs(b[1] - a[1])<1e-8));
		if (abs(x[0]-a[0])<1e-8 && abs(x[1]-a[1])<1e-8 || abs(x[0] - b[0])<1e-8 && abs(x[1] - b[1])<1e-8)
		{
			return 0;
		}
		//on line---need predicate segment
		double d = orient2d(x, a, b);

		if (abs(d) > 1e-16) //modified from if (d!=0)
		{
			return (d>0 ? 1 : -1);
		}
		else
		{
			return 0;
		}
	}

	bool is_on_domain_edge(double *a)
	{
		bool on_edge = abs(a[1] - 1) < 1e-8 ||
			abs(a[0] - 1) < 1e-8 ||
			abs(a[1] - 0) < 1e-8 ||
			abs(a[0] - 0) < 1e-8;
		return on_edge ? true : false;
	}

	bool is_on_corner(double *a)
	{
		bool on_corner = (abs(a[0] - 0) < 1e-8 && abs(a[1] - 0) < 1e-8) ||
			(abs(a[0] - 0) < 1e-8 && abs(a[1] - 1) < 1e-8) ||
			(abs(a[0] - 1) < 1e-8 && abs(a[1] - 0) < 1e-8) ||
			(abs(a[0] - 1) < 1e-8 && abs(a[1] - 1) < 1e-8);
		return on_corner ? true : false;
	}

	bool is_on_knot(double *a, double *b, double *c, double *x)
	{
		bool on_knot = (abs(x[0] - a[0]) < 1e-8 && abs(x[1] - a[1]) < 1e-8) ||
			(abs(x[0] - b[0]) < 1e-8 && abs(x[1] - b[1]) < 1e-8) ||
			(abs(x[0] - c[0]) < 1e-8 && abs(x[1] - c[1]) < 1e-8);
		return on_knot;
	}

	bool is_on_square_right(double *x)
	{
		return abs(x[0] - 1) < 1e-8 ? true : false;
	}

	bool is_on_square_top(double *x)
	{
		return abs(x[1] - 1) < 1e-8 ? true : false;
	}

	bool is_on_top_left_corner(double *x)
	{
		return abs(x[0] - 0) < 1e-8 && abs(x[1] - 1) < 1e-8 ? true : false;
	}

	bool is_on_bottom_right_corner(double *x)
	{
		return abs(x[0] - 1) < 1e-8 && abs(x[1] - 0) < 1e-8 ? true : false;
	}

	Sign sideOfOpenTriangle(double* x, double* a, double* b, double* c)
	{
		if ((sideOfLine(x, a, b) == 1) && (sideOfLine(x, b, c) == 1) && (sideOfLine(x, c, a) == 1))
			return 1;
		else if ((sideOfLine(x, a, b) == 0) && (sideOfLine(x, b, c) == 1) && (sideOfLine(x, c, a) == 1) ||
			(sideOfLine(x, a, b) == 1) && (sideOfLine(x, b, c) == 0) && (sideOfLine(x, c, a) == 1) ||
			(sideOfLine(x, a, b) == 1) && (sideOfLine(x, b, c) == 1) && (sideOfLine(x, c, a) == 0))
		{
			return 0;
		}
		else
			return -1;
	}

	bool sideOfHalfopenTriangle(double* x, double* a, double* b, double* c)
	{
		double temp1[2], temp2[2], temp3[2];//sample point is on temp1-temp2 edge
		if (sideOfLine(x, a, b) == 1 && sideOfLine(x, b, c) == 1 && sideOfLine(x, c, a) == 1)
			return 1;
		else if (sideOfLine(x, a, b) == 0 && !(sideOfLine(x, b, c) == -1) && !(sideOfLine(x, c, a) == -1))
		{
			temp1[0] = a[0]; temp1[1] = a[1]; temp2[0] = b[0]; temp2[1] = b[1]; temp3[0] = c[0]; temp3[1] = c[1];
		}
		else if (sideOfLine(x, b, c) == 0 && !(sideOfLine(x, a, b) == -1) && !(sideOfLine(x, c, a) == -1))
		{
			temp1[0] = b[0]; temp1[1] = b[1]; temp2[0] = c[0]; temp2[1] = c[1]; temp3[0] = a[0]; temp3[1] = a[1];
		}
		else if (sideOfLine(x, c, a) == 0 && !(sideOfLine(x, a, b) == -1) && !(sideOfLine(x, b, c) == -1))
		{
			temp1[0] = c[0]; temp1[1] = c[1]; temp2[0] = a[0]; temp2[1] = a[1]; temp3[0] = b[0]; temp3[1] = b[1];
		}
		else
		{
			return 0;
		}
		//is on segment
		if (x[0] < std::min(temp1[0], temp2[0]) - 1e-8 || x[0] > std::max(temp1[0], temp2[0]) + 1e-8 ||
			x[1] < std::min(temp1[1], temp2[1]) - 1e-8 || x[1] > std::max(temp1[1], temp2[1]) + 1e-8)
		{
			return 0;
		}

		LocateOfSamplePoint is_onedge = NORMAL_OPEN;
		if (is_on_domain_edge(x))
		{
			if (!is_on_knot(temp1, temp2, temp3, x))
				return 1;
			else if (is_on_square_right(x) || is_on_square_top(x))
			{
				if (is_on_top_left_corner(x))
					is_onedge = LEFTTOPCORNER;
				else if (is_on_bottom_right_corner(x))
					is_onedge = RIGHTBOTOMCORNER;
				else
					is_onedge = RIGHTTOPSIDEBUTCORNER;
			}
		}

		if (is_onedge == RIGHTBOTOMCORNER)
		{
			double test_x[2];
			test_x[0] = x[0] - 1e-4;
			test_x[1] = x[1] + 1e-6;
			bool inner = sideOfLine(test_x, a, b) == 1 && sideOfLine(test_x, b, c) == 1 && sideOfLine(test_x, c, a) == 1;
			return inner ? 1 : 0;
		}
		else if (is_onedge == LEFTTOPCORNER)
		{
			double test_x[2];
			test_x[0] = x[0] + 1e-4;
			test_x[1] = x[1] - 1e-6;
			bool inner = sideOfLine(test_x, a, b) == 1 && sideOfLine(test_x, b, c) == 1 && sideOfLine(test_x, c, a) == 1;
			return inner ? 1 : 0;
		}
		else if (is_onedge == RIGHTTOPSIDEBUTCORNER)
		{
			if (abs(temp1[0] - x[0]) < 1e-8 && abs(temp1[1] - x[1]) < 1e-8 ||
				abs(temp2[0] - x[0]) < 1e-8 && abs(temp2[1] - x[1]) < 1e-8)
			{
				//transformation method
				double test_x[2];
				test_x[0] = x[0] - 1e-4 * sqrt(pow(temp1[0] - temp2[0], 2) + pow(temp1[1] - temp2[1], 2));
				test_x[1] = x[1] - 1e-8 * sqrt(pow(temp1[0] - temp2[0], 2) + pow(temp1[1] - temp2[1], 2));
				bool inner = sideOfLine(test_x, a, b) == 1 && sideOfLine(test_x, b, c) == 1 && sideOfLine(test_x, c, a) == 1;
				return inner ? 1 : 0;
			}
			else
			{
				if (abs(temp1[1] - temp2[1]) > 1e-8)
				{
					return temp2[1] > temp1[1] ? 1 : 0;
				}
				else
				{
					return temp2[0] < temp1[0] ? 1 : 0;
				}
			}
		}
		else
		{
			if (abs(temp1[0] - x[0]) < 1e-8 && abs(temp1[1] - x[1]) < 1e-8 || abs(temp2[0] - x[0]) < 1e-8 && abs(temp2[1] - x[1]) < 1e-8)
			{
				//transformation method
				/*double test_x[2];
				test_x[0] = x[0] + 1e-4 * sqrt(pow(temp1[0] - temp2[0], 2) + pow(temp1[1] - temp2[1], 2));
				test_x[1] = x[1] + 1e-8 * sqrt(pow(temp1[0] - temp2[0], 2) + pow(temp1[1] - temp2[1], 2));
				if (sideOfLine(test_x, a, b) == 1 && sideOfLine(test_x, b, c) == 1 && sideOfLine(test_x, c, a) == 1)
				return 1;
				else
				return 0;*/
				//a better method
				double ccw_triangle1[2], ccw_triangle2[2], ccw_triangle3[2];
				if (abs(temp1[0] - x[0]) < 1e-8 && abs(temp1[1] - x[1]) < 1e-8)
				{
					ccw_triangle1[0] = temp1[0]; ccw_triangle1[1] = temp1[1];
					ccw_triangle2[0] = temp2[0]; ccw_triangle2[1] = temp2[1];
					ccw_triangle3[0] = temp3[0]; ccw_triangle3[1] = temp3[1];
				}
				else
				{
					ccw_triangle1[0] = temp2[0]; ccw_triangle1[1] = temp2[1];
					ccw_triangle2[0] = temp3[0]; ccw_triangle2[1] = temp3[1];
					ccw_triangle3[0] = temp1[0]; ccw_triangle3[1] = temp1[1];
				}
				//predicate a point belonging to a triangle
				bool is_edge_12_belongto = false;
				if (abs(ccw_triangle1[1] - ccw_triangle2[1]) > 1e-8)
				{
					is_edge_12_belongto = ccw_triangle1[1] > ccw_triangle2[1] ? true : false;
				}
				else
				{
					is_edge_12_belongto = ccw_triangle1[0] < ccw_triangle2[0] ? true : false;
				}
				bool is_edge_13_belongto = false;
				if (abs(ccw_triangle3[1] - ccw_triangle1[1]) > 1e-8)
				{
					is_edge_13_belongto = ccw_triangle3[1] > ccw_triangle1[1] ? true : false;
				}
				else
				{
					is_edge_13_belongto = ccw_triangle3[0] < ccw_triangle1[0] ? true : false;
				}
				return (is_edge_12_belongto && is_edge_13_belongto) ? 1 : 0;
			}
			else
			{
				if (abs(temp1[1] - temp2[1]) > 1e-8)
				{
					return temp1[1] > temp2[1] ? 1 : 0;
				}
				else
				{
					return temp1[0] < temp2[0] ? 1 : 0;
				}
			}
		}
	}

};


#endif
