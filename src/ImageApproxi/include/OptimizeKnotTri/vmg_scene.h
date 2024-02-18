#pragma once
#ifndef VMG_SCENE_H
#define VMG_SCENE_H

#include "Assistant/data_types.h"
#include "OptimizeKnotTri/image_domain.h"
#include "OptimizeKnotTri/openmesh_trimesh.h"

#include "OptimizeKnotTri/image_rasterizer.h"
#include "OptimizeKnotTri/timer.h"
#include "OptimizeKnotTri/image_polynomial.h"

#define MAX_ANGLE (M_PI * 17.0 / 18.0)
#define global_max_search_count 20
#define GEOMETRY_CONTROL true

static double global_base = 1.0;
static double global_alpha = 0.8;
static double global_beta = 0.5;

static bool inside_triangle(const double *triangle, const double *p)
{
	double vec1[2], vec2[2], vec3[2];
	vec1[0] = triangle[0] - p[0];
	vec1[1] = triangle[1] - p[1];
	vec2[0] = triangle[2] - p[0];
	vec2[1] = triangle[3] - p[1];
	double cross1 = vec1[0] * vec2[1] - vec2[0] * vec1[1];

	vec3[0] = triangle[4] - p[0];
	vec3[1] = triangle[5] - p[1];
	double cross2 = vec2[0] * vec3[1] - vec3[0] * vec2[1];
	if (cross1 * cross2 < 0.0)
		return false;

	double cross3 = vec3[0] * vec1[1] - vec1[0] * vec3[1];
	if (cross2 * cross3 < 0.0)
		return false;

	if (cross3 * cross1 < 0.0)
		return false;

	return true;
}

class OptiKnotTri
{
public:
	enum { Accelerator_CUDA = 0, Accelerator_OpenMP = 1, Accelerator_None = 2};

protected:
	typedef OpenMesh::VPropHandleT<TriMesh::Normal>  VPropGradient;
	typedef OpenMesh::VPropHandleT<int>              VPropFlag;
	typedef OpenMesh::FPropHandleT<PixelSet>         FPropPixelSet;
	typedef OpenMesh::FPropHandleT<Polynomial>       FPropPolynomial;
	typedef OpenMesh::FPropHandleT<double>           FPropEnergy;

	VPropGradient    _vertexGradient;
	VPropFlag        _vertexFlag;
	FPropPixelSet    _facePixelSet;
	FPropPolynomial  _facePolynomial;
	FPropEnergy      _faceEnergy;

private:
	ImageDomain *_image;
	TriMesh     *_mesh;
	ImageDomain *_approxImage;

	int           _degree;

	std::vector<std::vector<int>> _verticesBatch;
	std::vector<std::vector<int>> _edgesBatch;

	int _accelerator;

	int _fixedVerticesNb;
	std::multimap<int, int> _fixedEdges;

public:
	OptiKnotTri()
		: _image(0), _mesh(NULL), _approxImage(NULL), 
		_degree(1), _fixedVerticesNb(4)
	{
	}

	~OptiKnotTri()
	{
		clear();
	}

	void new_mesh()
	{
		if (_mesh != NULL)
			delete_mesh();
		_mesh = new TriMesh();
		_mesh->add_property(_vertexGradient);
		_mesh->add_property(_vertexFlag);
		_mesh->add_property(_facePixelSet);
		_mesh->add_property(_facePolynomial);
		_mesh->add_property(_faceEnergy);
		_fixedVerticesNb = 4;
	}

	void delete_mesh()
	{
		if (_mesh != NULL)
		{
			_mesh->remove_property(_vertexGradient);
			_mesh->remove_property(_vertexFlag);
			_mesh->remove_property(_facePixelSet);
			_mesh->remove_property(_facePolynomial);
			_mesh->remove_property(_faceEnergy);

			delete _mesh;
			_mesh = NULL;
		}

		_fixedEdges.clear();
	}

	void clear();
	void set_degree(const int& d) 
	{ 
		_degree = d;
		if (_mesh)
		{
			assign_pixels();
			compute_polynomials();
			compute_energy();
		}
		printf("%s: degree = %d\n", __FUNCTION__, _degree); 
	}
	void set_accelerator(const int& acc) { _accelerator = acc; printf("%s: accelerator = %d\n", __FUNCTION__, _accelerator); }

	// io
	void set_image(const unsigned char* data, const int& w, const int& h, const int& c);

	void load_mesh(const char* fileName,int);
	void load_fixed_edge(const char* fileName);

	bool is_fixed_edge(const TriMesh::EdgeHandle& eh);

	// assign
	void assign_pixels();
	void assign_pixel(const int& f);

	// polynomials
	void compute_polynomials();
	void compute_polynomial(const int& f);

	// energy
	double compute_energy();
	double compute_energy(const int& f);

	// init
	bool init_greedy_mesh(const int& vnb, const char*file = NULL);
	void insert_vertices(const int& vnb);

	// algorithm
	bool optimize_mesh(const int& iterations,const char*, bool);

protected:
	void optimize_vertices();
	void process_boundary();
	void optimize_connectivity();

	void compute_vertices_gradient();
	void compute_vertices_gradient(const double *p1, const double *p2, 
		const Polynomial *poly1, const Polynomial *poly2, double *result1, double *result2) const;

	void compute_vertices_batch();
	void update_positions();
	void update_position(const int& v);
	void compute_vertex_gradient(const int& v);
	void process_vertex_gradient(const int& v);
	double compute_maximum_step(const int &v, const double *direction) const;
	double compute_safe_step(const double *p1, const double *p2, const double *p3, const double *direction) const;

	bool process_boundary_face(TriMesh::HalfedgeHandle& borderHe);

	void compute_edges_batch();
	void update_connectivity();
	bool update_connection(const int& e);

	double compute_psnr();
};

#endif
