#include "Corefuncs/simplex_spline.h"
#include <algorithm> 


Simplex_Spline_n::Simplex_Spline_n(double **X, int degree, double xmax, double ymax)
{
	pts = X;
	deg = degree;
	gp.xmax = xmax;
	gp.ymax = ymax;
}

Simplex_Spline_n::Simplex_Spline_n(double **X, int degree)
{
	pts = X;
	deg = degree;
	gp.xmax = 1;
	gp.ymax = 1;
}

double	Simplex_Spline_n::computeSimplexSpline(double *pt)
{
	double val = 0;
	if (deg>0)
	{	
		double signed_area;
		int A[3];
		PointsSetType res = findTriangle(A[0], A[1], A[2], signed_area);//affine-independent subset of X
		
		if (res == ONONEVERTEX)
		{
				return 0;
		}
		else if (res == ONLINE)
		{
			return 0;
		}
		if (res == NORMAL)
		{
			double d1 = gp.signedArea(pt, pts[A[1]], pts[A[2]]);
			double d2 = gp.signedArea(pts[A[0]], pt, pts[A[2]]);
			double d3 = gp.signedArea(pts[A[0]], pts[A[1]], pt);

			double a[3] =
			{
				d1 / signed_area,
				d2 / signed_area,
				0
			};
			a[2] = 1 - a[0] - a[1];
			for (int i = 0; i < 3; i++)
			{
				std::swap(pts[A[i]], pts[deg + 3 - 1]);
				Simplex_Spline_n f(pts, deg - 1, gp.xmax, gp.ymax);
				val += a[i] * f.computeSimplexSpline(pt);
				std::swap(pts[deg + 3 - 1], pts[A[i]]);
			}
		}
		else
		{
			std::cout << __FUNCTION__ << ": " << "perturbation of points>3 is wrong" << std::endl;
		}
	}
	else
	{
		double doublearea = orient2d(pts[0], pts[1], pts[2]);
		if (std::abs(doublearea)<1e-16)
		{
			//only consider the case three interior points are collinear
			return 0;
			
		}

		//test 3-types
		/*if (abs(pt[0] - 0.426393)<1e-5 && abs(pt[1] - 0.535966) < 1e-5)
		{
			if (abs(pts[0][0] - 0.369211) < 1e-5 && abs(pts[1][0] - 0.407045) < 1e-5 &&abs(pts[2][0] - 0.457577))
			{
				int triangle1 = 1;
			}
			if (abs(pts[0][0] - 0.457577) < 1e-5 && abs(pts[1][0] - 0.550087) < 1e-5 &&abs(pts[2][0] - 0.369211))
			{
				int triangle2 = 1;
			}
			if (abs(pts[0][0] - 0.407045) < 1e-5 && abs(pts[1][0] - 0.426614) < 1e-5 &&abs(pts[2][0] - 0.457577))
			{
				int triangle3 = 1;
			}
			if (abs(pts[0][0] - 0.457577) < 1e-5 && abs(pts[1][0] - 0.443218) < 1e-5 &&abs(pts[2][0] - 0.407045))
			{
				int triangle4 = 1;
			}
		}*/

		Pred::Sign sign;
		if (doublearea > 0)
			sign = gp.sideOfHalfopenTriangle(pt, pts[0], pts[1], pts[2]);
		else
			sign = gp.sideOfHalfopenTriangle(pt, pts[1], pts[0], pts[2]);
		val = (sign == 1 ? 2.0 / fabs(doublearea) : 0);

		//test
		/*if (abs(pt[0] - 1.0) < 1e-5 && abs(pt[1] - 0.473581) < 1e-5 && sign == 1)
		{
			std::cout << __FUNCTION__ << ": " << "result: " << sign << " -(" << pts[0][0] << "," << pts[0][1] << ");("
				<< pts[1][0] << "," << pts[1][1] << ");(" << pts[2][0] << "," << pts[2][1] << ");\n";
		}*/
	}
	return val;
}

PointsSetType	Simplex_Spline_n::findTriangle(int& a, int& b, int&c, double& area)
{
	//the first point
	a = 0;

	//the second point
	int i = 1;
	bool is_find = false;
	for (; i<deg + 3; i++)
	{
		if (abs(*pts[i] - *pts[a]) > 1e-8 || abs(*(pts[i] + 1) - *(pts[a] + 1)) > 1e-8)
		{
			is_find = true;
			b = i;
			break;
		}
	} 
	if (!is_find)
		return ONONEVERTEX;

	//the third point
	is_find = false;
	for (i++; i<deg + 3; i++)
	{
		area = gp.signedArea(pts[a], pts[b], pts[i]);
		//if(std::abs(area)>std::numeric_limits<double>::epsilon()) 
		if (abs(area) > 1e-16)
		{
			is_find = true;
			c = i;
			break;
		}
	}
	if (!is_find) return ONLINE;
	return NORMAL;
}

double	Simplex_Spline_n::firstdirectionalderivate(double* pt, double *direction)
{
	double val = 0;
	if (deg>0)
	{
		double signed_area012;
		int A[3];
		PointsSetType res = findTriangle(A[0], A[1], A[2], signed_area012);//affinely independent subset of X

		if (res == ONONEVERTEX)
		{
			//std::cout << __FUNCTION__ << ": " << deg + 3 << "-points are on vertex during compute firstdirectionalderivate()" << std::endl;
			return 0;
		}
		else if (res == ONLINE)
		{
			return 0;
		}
		if (res == NORMAL)
		{
			double barycoord[3] =
			{
				gp.signedArea(pt, pts[A[1]], pts[A[2]]) / signed_area012,
				gp.signedArea(pts[A[0]], pt, pts[A[2]]) / signed_area012,
				gp.signedArea(pts[A[0]], pts[A[1]], pt) / signed_area012
			};
			double derivate[3] =
			{
				0.5 * ((pts[A[1]][1] - pts[A[2]][1])*direction[0] + (pts[A[2]][0] - pts[A[1]][0])*direction[1]) / signed_area012,
				0.5 * ((pts[A[2]][1] - pts[A[0]][1])*direction[0] + (pts[A[0]][0] - pts[A[2]][0])*direction[1]) / signed_area012,
				0.5 * ((pts[A[0]][1] - pts[A[1]][1])*direction[0] + (pts[A[1]][0] - pts[A[0]][0])*direction[1]) / signed_area012
			};
			for (int i = 0; i < 3; i++)
			{
				std::swap(pts[A[i]], pts[deg + 3 - 1]);
				Simplex_Spline_n f(pts, deg - 1, gp.xmax, gp.ymax);
				
				val += derivate[i] * f.computeSimplexSpline(pt);
				val += barycoord[i] * f.firstdirectionalderivate(pt, direction);
				std::swap(pts[deg + 3 - 1], pts[A[i]]);
			}
		}
		else
		{
			std::cout << __FUNCTION__ << ": " << "perturbation of points>3 is wrong in first_derivate" << std::endl;
		}
	}
	else
	{
		val = 0;
	}
	return val;
}

double	Simplex_Spline_n::seconddirectionalderivate(double* pt, double *firstdirection, double *seconddirection)
{
	double val = 0;
	if (deg > 0)
	{
		double signed_area012;
		int A[3];
		PointsSetType res = findTriangle(A[0], A[1], A[2], signed_area012);//affine-independent subset of X

		if (res == ONONEVERTEX)
		{
			//std::cout << __FUNCTION__ << ": " << deg + 3 << "-points are on vertex during compute seconddirectionalderivate() of simplex spline" << std::endl;
			return 0;
		}
		else if (res == ONLINE)
		{
			return 0;
		}
		if (res == NORMAL)
		{
			double barycentriccoor[3] =
			{
				gp.signedArea(pt, pts[A[1]], pts[A[2]]) / signed_area012,
				gp.signedArea(pts[A[0]], pt, pts[A[2]]) / signed_area012,
				gp.signedArea(pts[A[0]], pts[A[1]], pt) / signed_area012
			};
			double derivare_fd[3] =
			{
				0.5 * ((pts[A[1]][1] - pts[A[2]][1])*firstdirection[0] + (pts[A[2]][0] - pts[A[1]][0])*firstdirection[1]) / signed_area012,
				0.5 * ((pts[A[2]][1] - pts[A[0]][1])*firstdirection[0] + (pts[A[0]][0] - pts[A[2]][0])*firstdirection[1]) / signed_area012,
				0.5 * ((pts[A[0]][1] - pts[A[1]][1])*firstdirection[0] + (pts[A[1]][0] - pts[A[0]][0])*firstdirection[1]) / signed_area012
			};
			double derivare_sd[3] =
			{
				0.5 * ((pts[A[1]][1] - pts[A[2]][1])*seconddirection[0] + (pts[A[2]][0] - pts[A[1]][0])*seconddirection[1]) / signed_area012,
				0.5 * ((pts[A[2]][1] - pts[A[0]][1])*seconddirection[0] + (pts[A[0]][0] - pts[A[2]][0])*seconddirection[1]) / signed_area012,
				0.5 * ((pts[A[0]][1] - pts[A[1]][1])*seconddirection[0] + (pts[A[1]][0] - pts[A[0]][0])*seconddirection[1]) / signed_area012
			};
			for (int i = 0; i < 3; i++)
			{
				std::swap(pts[A[i]], pts[deg + 3 - 1]);
				Simplex_Spline_n f(pts, deg - 1, gp.xmax, gp.ymax);
				
				val += derivare_fd[i] * f.firstdirectionalderivate(pt, seconddirection);
				val += derivare_sd[i] * f.firstdirectionalderivate(pt, firstdirection);
				val += barycentriccoor[i] * f.seconddirectionalderivate(pt, firstdirection, seconddirection);
				std::swap(pts[deg + 3 - 1], pts[A[i]]);
			}
		}
		else
		{
			std::cout << __FUNCTION__ << ": " << "perturbation of points>3 is wrong in second derivate" << std::endl;
		}
	}
	else
	{
		val = 0;
	}
	return val;
}

double Simplex_Spline_n::computePolorCoefficient(double *pt)
{
	double val = 0;
	if (deg > 0)
	{
		double signed_area;
		int A[3];
		bool res = findTriangle(A[0], A[1], A[2], signed_area);//affinely independent subset of X
		if (!res)
			return 0;

		double d1 = gp.signedArea(pt, pts[A[1]], pts[A[2]]);
		double d2 = gp.signedArea(pts[A[0]], pt, pts[A[2]]);
		double d3 = gp.signedArea(pts[A[0]], pts[A[1]], pt);

		double a[3] =
		{
			d1 / (signed_area + std::numeric_limits<double>::epsilon()),
			d2 / (signed_area + std::numeric_limits<double>::epsilon()),
			0
		};
		a[2] = 1 - a[0] - a[1];
		for (int i = 0; i < 3; i++)
		{
			std::swap(pts[A[i]], pts[deg + 3 - 1]);
			Simplex_Spline_n f(pts, deg - 1, gp.xmax, gp.ymax);
			double *p;
			if (deg > 1)
				p = pt + 2;
			else
				p = pt;
			val += a[i] * f.computePolorCoefficient(p);
			std::swap(pts[deg + 3 - 1], pts[A[i]]);
		}
	}
	else
	{
		double doublearea = orient2d(pts[0], pts[1], pts[2]);
		if (std::abs(doublearea) < 1e-16)
		{
			return 0;
		}
		else
		{
			Pred::Sign sign = doublearea > 0 ?
				gp.sideOfHalfopenTriangle(pt, pts[0], pts[1], pts[2]) :
				gp.sideOfHalfopenTriangle(pt, pts[1], pts[0], pts[2]);
			val = (sign == 1 ? 2.0 / fabs(doublearea) : 0);
		}
	}
	return val;

}

