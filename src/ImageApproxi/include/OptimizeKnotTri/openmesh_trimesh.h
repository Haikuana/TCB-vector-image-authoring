
#ifndef OPENMESH_TRIMESH_H
#define OPENMESH_TRIMESH_H

//OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/IO/Options.hh>
#include <OpenMesh/Core/Mesh/TriConnectivity.hh>

// define traits
struct MyTraits : public OpenMesh::DefaultTraits
{
	// use double valued coordinates
	typedef OpenMesh::Vec3d Point;
	typedef OpenMesh::Vec3d Normal;

	VertexAttributes(OpenMesh::Attributes::Status | OpenMesh::Attributes::Normal);
	FaceAttributes(OpenMesh::Attributes::Status | OpenMesh::Attributes::Normal);
	EdgeAttributes(OpenMesh::Attributes::Status | OpenMesh::Attributes::Normal);
};
// Select mesh type (TriMesh) and kernel (ArrayKernel)
// and define my personal mesh type (MyMesh)
typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  TriMesh;

double* mesh_face_triangle(const TriMesh *mesh, const int f);
void mesh_face_normal(const TriMesh *mesh, const TriMesh::FaceHandle &fh, TriMesh::Normal &fnorm);
bool is_delaunay_edge(const TriMesh *mesh, const TriMesh::EdgeHandle &edge);
bool delaunay_mesh(TriMesh *mesh);
std::set<int> delaunay_mesh(TriMesh *mesh, TriMesh::VertexHandle &vh);
bool locate_mesh_face(const TriMesh::Point &p, const TriMesh *mesh, TriMesh::FaceHandle &fh);
void build_mesh(const double *points, const int vnb, const int* facets, const int fnb, TriMesh *mesh);
void init_delaunay_mesh(const double *points, const int vnb, TriMesh *mesh);
void add_points_to_mesh(const double *points, const int vnb, bool doDelaunay, TriMesh *mesh);

float* mesh_points(const TriMesh *mesh);
float* mesh_boundary_points(const TriMesh *mesh, int &boundarynb);
int*   mesh_points_boundary_flag(const TriMesh *mesh);
int*   mesh_facets(const TriMesh *mesh);
float* mesh_connectivity(const TriMesh *mesh);
float* mesh_barycentric_coordinate(const TriMesh *mesh);

bool inside_mesh_face(const TriMesh* mesh, const TriMesh::FaceHandle& fh, const TriMesh::Point &p, TriMesh::FaceHandle &nfh);
bool is_degenerated(const TriMesh* mesh, const TriMesh::VertexHandle& vh);
bool is_flippable(const TriMesh *mesh, const int e);
bool is_collapse_ok(TriMesh *mesh, const TriMesh::HalfedgeHandle &he);

#endif
