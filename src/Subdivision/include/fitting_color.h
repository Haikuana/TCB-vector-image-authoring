#pragma once
#ifndef _FITTING_COLOR_
#define _FITTING_COLOR_

#include "Assistant/data_types.h"
#include "sparse_quad_prog_color.h"
#include "mesh.h"

class FittingColor
{
public:
	FittingColor()
	{
		mclInitializeApplication(NULL, 0);
		sparse_quad_prog_colorInitialize();
	}

	FittingColor(SUB_SIMPLIFY::MeshSimplify *target_mesh,
		SUB_SIMPLIFY::MeshSimplify *control_mesh)
	{
		mesh_sub = target_mesh;
		mesh_simplified = control_mesh;
		mclInitializeApplication(NULL, 0);
		sparse_quad_prog_colorInitialize();
	}

	~FittingColor()
	{
		sparse_quad_prog_colorTerminate();
		mclTerminateApplication();
	}

	inline void set_At(SpMat matA) { mat_sub = matA; }

	void solve_equation();

private:
	SUB_SIMPLIFY::MeshSimplify *mesh_sub;
	SUB_SIMPLIFY::MeshSimplify *mesh_simplified;
	SpMat						mat_sub;
};

#endif