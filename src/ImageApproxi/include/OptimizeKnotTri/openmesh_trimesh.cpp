
#include <list>
#include <map>
#include "OptimizeKnotTri/openmesh_trimesh.h"

double* mesh_face_triangle(const TriMesh *mesh, const int f)
{
	if (mesh == NULL)
		return NULL;

	TriMesh::HalfedgeHandle fhe = mesh->halfedge_handle(mesh->face_handle(f));
	TriMesh::Point p1 = mesh->point(mesh->from_vertex_handle(fhe));
	TriMesh::Point p2 = mesh->point(mesh->to_vertex_handle(fhe));
	TriMesh::Point p3 = mesh->point(mesh->to_vertex_handle(mesh->next_halfedge_handle(fhe)));

	double *t = new double[6];
	t[0] = p1[0];
	t[1] = p1[1];
	t[2] = p2[0];
	t[3] = p2[1];
	t[4] = p3[0];
	t[5] = p3[1];

	return t;
}

void mesh_face_normal(const TriMesh *mesh, const TriMesh::FaceHandle &fh, TriMesh::Normal &fnorm)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return;
	}

	TriMesh::HalfedgeHandle he = mesh->halfedge_handle(fh);
	TriMesh::VertexHandle v0 = mesh->from_vertex_handle(he);
	TriMesh::VertexHandle v1 = mesh->to_vertex_handle(he);
	TriMesh::VertexHandle v2 = mesh->to_vertex_handle(mesh->next_halfedge_handle(he));

	TriMesh::Point p0 = mesh->point(v0);
	TriMesh::Point p1 = mesh->point(v1);
	TriMesh::Point p2 = mesh->point(v2);

	TriMesh::Normal u = p1 - p0;
	TriMesh::Normal v = p2 - p0;

	fnorm = u % v;
}

bool is_delaunay_edge(const TriMesh *mesh, const TriMesh::EdgeHandle &edge)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return true;
	}

	if (mesh->is_boundary(edge) || !mesh->is_flip_ok(edge))
		return true;

	TriMesh::HalfedgeHandle he = mesh->halfedge_handle(edge, 0);
	double angle1 = mesh->calc_sector_angle(mesh->next_halfedge_handle(he));
	double angle2 = mesh->calc_sector_angle(mesh->next_halfedge_handle(mesh->opposite_halfedge_handle(he)));

	if (angle1 + angle2 > M_PI + 1e-20)
		return false;

	return true;
}

bool delaunay_mesh(TriMesh *mesh)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return false;
	}

	std::vector<bool> faceFlags;
	while (1)
	{
		faceFlags.clear();
		faceFlags.resize(mesh->n_faces(), false);

		// mark edges
		std::vector<TriMesh::EdgeHandle> edgesList;
		for (TriMesh::EdgeIter eit = mesh->edges_begin(); eit != mesh->edges_end(); ++eit)
		{
			TriMesh::EdgeHandle he = *eit;
			if (!mesh->is_flip_ok(he))
				continue;

			TriMesh::HalfedgeHandle heh1 = mesh->halfedge_handle(he, 0);
			TriMesh::HalfedgeHandle heh2 = mesh->halfedge_handle(he, 1);
			TriMesh::FaceHandle hf1 = mesh->face_handle(heh1);
			TriMesh::FaceHandle hf2 = mesh->face_handle(heh2);

			if (faceFlags[hf1.idx()] || faceFlags[hf2.idx()])
				continue;

			if (!is_delaunay_edge(mesh, he))
			{
				edgesList.push_back(he);

				faceFlags[hf1.idx()] = true;
				faceFlags[hf2.idx()] = true;
			}
		}

		if (edgesList.size() < 1)
		{
			break;
		}

		// flip edges
		for (int i = 0; i < edgesList.size(); ++i)
			mesh->flip(edgesList[i]);
	}

	return true;
}

std::set<int> delaunay_mesh(TriMesh *mesh, TriMesh::VertexHandle &vh)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return std::set<int>();
	}

	std::list<TriMesh::EdgeHandle> edgeList;
	for (TriMesh::VertexOHalfedgeIter voheit = mesh->voh_begin(vh); voheit != mesh->voh_end(vh); ++voheit)
	{
		TriMesh::EdgeHandle he = mesh->edge_handle(*voheit);
		if (!is_delaunay_edge(mesh, he))
			edgeList.push_back(he);

		he = mesh->edge_handle(mesh->next_halfedge_handle(*voheit));
		if (!is_delaunay_edge(mesh, he))
			edgeList.push_back(he);
	}

	std::set<int> faces;
	while (!edgeList.empty())
	{
		TriMesh::EdgeHandle he = edgeList.front();
		edgeList.pop_front();

		mesh->flip(he);

		TriMesh::HalfedgeHandle heh = mesh->halfedge_handle(he, 0);
		TriMesh::EdgeHandle temp = mesh->edge_handle(mesh->next_halfedge_handle(heh));
		if (!is_delaunay_edge(mesh, temp))
			edgeList.push_front(temp);

		temp = mesh->edge_handle(mesh->next_halfedge_handle(mesh->next_halfedge_handle(heh)));
		if (!is_delaunay_edge(mesh, temp))
			edgeList.push_front(temp);

		temp = mesh->edge_handle(mesh->next_halfedge_handle(mesh->opposite_halfedge_handle(heh)));
		if (!is_delaunay_edge(mesh, temp))
			edgeList.push_front(temp);

		temp = mesh->edge_handle(mesh->next_halfedge_handle(mesh->next_halfedge_handle(mesh->opposite_halfedge_handle(heh))));
		if (!is_delaunay_edge(mesh, temp))
			edgeList.push_front(temp);

		faces.insert(mesh->face_handle(heh).idx());
		faces.insert(mesh->opposite_face_handle(heh).idx());
	}

	return faces;
}

bool locate_mesh_face(const TriMesh::Point &p, const TriMesh *mesh, TriMesh::FaceHandle &fh)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return false;
	}

	std::map<int, bool> faceVisit;

	fh = mesh->face_handle(0);
	while (1)
	{
		faceVisit[fh.idx()] = true;

		if(inside_mesh_face(mesh, fh, p, fh))
			return true;

		if (fh.idx() == -1)
			return false;

		if (faceVisit.find(fh.idx()) != faceVisit.end())
			return false;
	}

	return false;
}

void build_mesh(const double *points, const int vnb, const int* facets, const int fnb, TriMesh *mesh)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return;
	}

	mesh->clear();

	for (int i = 0; i < vnb; i++)
	{
		int d_index = 3 * i;
		TriMesh::Point p(points[d_index], points[d_index + 1], points[d_index + 2]);

		mesh->add_vertex(p);
	}

	for (int i = 0; i < fnb; i++)
	{
		TriMesh::VertexHandle vh0 = mesh->vertex_handle(facets[3 * i]);
		TriMesh::VertexHandle vh1 = mesh->vertex_handle(facets[3 * i + 1]);
		TriMesh::VertexHandle vh2 = mesh->vertex_handle(facets[3 * i + 2]);
		mesh->add_face(vh0, vh1, vh2);
	}
}

void init_delaunay_mesh(const double *points, const int vnb, TriMesh *mesh)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return;
	}

	mesh->clear();

	const double largenb = 1e10;
	TriMesh::VertexHandle v0 = mesh->add_vertex(TriMesh::Point(-largenb, -largenb, 0.0));
	TriMesh::VertexHandle v1 = mesh->add_vertex(TriMesh::Point(largenb, -largenb, 0.0));
	TriMesh::VertexHandle v2 = mesh->add_vertex(TriMesh::Point(largenb, largenb, 0.0));
	TriMesh::VertexHandle v3 = mesh->add_vertex(TriMesh::Point(-largenb, largenb, 0.0));

	mesh->add_face(v0, v1, v2);
	mesh->add_face(v0, v2, v3);

	for (int i = 0; i < vnb; i++)
	{
		int d_index = 3 * i;
		TriMesh::Point p(points[d_index], points[d_index + 1], points[d_index + 2]);

		TriMesh::FaceHandle fh;
		if (locate_mesh_face(p, mesh, fh))
		{
			TriMesh::VertexHandle v = mesh->split(fh, p);
			delaunay_mesh(mesh, v);
		}
	}
	
	for (int i = 3; i >= 0; i--)
	{
		TriMesh::VertexHandle vh = mesh->vertex_handle(i);
		mesh->delete_vertex(vh);
	}
	mesh->garbage_collection();
}

void add_points_to_mesh(const double *points, const int vnb, bool doDelaunay, TriMesh *mesh)
{
	for (int i = 0; i < vnb; i++)
	{
		int d_index = 3 * i;
		TriMesh::Point p(points[d_index], points[d_index + 1], points[d_index + 2]);

		TriMesh::FaceHandle fh;
		if (locate_mesh_face(p, mesh, fh))
		{
			TriMesh::VertexHandle v = mesh->split(fh, p);
			if (doDelaunay)
				delaunay_mesh(mesh, v);
		}
	}
}

float* mesh_points(const TriMesh *mesh)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return NULL;
	}

	float *points = new float[3 * mesh->n_vertices()];
	for (int i = 0; i < (int)mesh->n_vertices(); i++)
	{
		TriMesh::Point p = mesh->point(mesh->vertex_handle(i));
		points[3 * i] = float(p[0]);
		points[3 * i + 1] = float(p[1]);
		points[3 * i + 2] = float(p[2]);
	}

	return points;
}

float* mesh_boundary_points(const TriMesh *mesh, int &boundarynb)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return NULL;
	}

	boundarynb = 0;
	for (int i = 0; i < (int)mesh->n_vertices(); i++)
	{
		TriMesh::VertexHandle vh = mesh->vertex_handle(i);
		if (mesh->is_boundary(vh))
		{
			boundarynb++;
		}
	}

	float *boundaryPoints = new float[3 * boundarynb];
	int j = 0;
	for (int i = 0; i < (int)mesh->n_vertices(); i++)
	{
		TriMesh::VertexHandle vh = mesh->vertex_handle(i);
		if (mesh->is_boundary(vh))
		{
			TriMesh::Point p = mesh->point(vh);
			boundaryPoints[3 * j] = float(p[0]);
			boundaryPoints[3 * j + 1] = float(p[1]);
			boundaryPoints[3 * j + 2] = float(p[2]);
			j++;
		}
	}

	return boundaryPoints;
}

int* mesh_points_boundary_flag(const TriMesh *mesh)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return NULL;
	}

	int *flags = new int[mesh->n_vertices()];
	for (int i = 0; i < (int)mesh->n_vertices(); i++)
	{
		flags[i] = mesh->is_boundary(mesh->vertex_handle(i)) ? 0 : 1;
	}

	return flags;
}

int* mesh_facets(const TriMesh *mesh)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return NULL;
	}

	int *facets = new int[3 * mesh->n_faces()];
	for (int i = 0; i < (int)mesh->n_faces(); i++)
	{
		TriMesh::HalfedgeHandle fhe = mesh->halfedge_handle(mesh->face_handle(i));
		TriMesh::VertexHandle v0 = mesh->from_vertex_handle(fhe);
		TriMesh::VertexHandle v1 = mesh->to_vertex_handle(fhe);
		TriMesh::VertexHandle v2 = mesh->to_vertex_handle(mesh->next_halfedge_handle(fhe));

		facets[3 * i] = v0.idx();
		facets[3 * i + 1] = v1.idx();
		facets[3 * i + 2] = v2.idx();
	}

	return facets;
}

float* mesh_connectivity(const TriMesh *mesh)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return NULL;
	}

	float *points = new float[mesh->n_faces() * 3 * 3];
	for (int i = 0; i < (int)mesh->n_faces(); i++)
	{
		TriMesh::HalfedgeHandle fhe = mesh->halfedge_handle(mesh->face_handle(i));
		TriMesh::VertexHandle fv[3];
		fv[0] = mesh->from_vertex_handle(fhe);
		fv[1] = mesh->to_vertex_handle(fhe);
		fv[2] = mesh->to_vertex_handle(mesh->next_halfedge_handle(fhe));

		int headadd = 9 * i;
		for (int j = 0; j < 3; j++)
		{
			int padd = headadd + 3 * j;
			TriMesh::Point p = mesh->point(fv[j]);
			for (int k = 0; k < 3; k++)
			{
				points[padd + k] = float(p[k]);
			}
		}
	}

	return points;
}

float* mesh_barycentric_coordinate(const TriMesh *mesh)
{
	if (!mesh)
	{
		printf("%s: mesh is null\n", __FUNCTION__);
		return NULL;
	}

	float *bary = new float[mesh->n_faces() * 3 * 3];
	for (int i = 0; i < (int)mesh->n_faces(); i++)
	{
		int headadd = 9 * i;
		bary[headadd + 0] = 1.0f;
		bary[headadd + 1] = 0.0f;
		bary[headadd + 2] = 0.0f;

		bary[headadd + 3] = 0.0f;
		bary[headadd + 4] = 1.0f;
		bary[headadd + 5] = 0.0f;

		bary[headadd + 6] = 0.0f;
		bary[headadd + 7] = 0.0f;
		bary[headadd + 8] = 1.0f;
	}

	return bary;
}

bool inside_mesh_face(const TriMesh* mesh, const TriMesh::FaceHandle& fh, 
	const TriMesh::Point &p, TriMesh::FaceHandle &nfh)
{
	TriMesh::Normal fnorm;
	mesh_face_normal(mesh, fh, fnorm);

	TriMesh::HalfedgeHandle fhe = mesh->halfedge_handle(fh);
	TriMesh::HalfedgeHandle temphe = fhe;
	do
	{
		TriMesh::VertexHandle v1 = mesh->from_vertex_handle(temphe);
		TriMesh::VertexHandle v2 = mesh->to_vertex_handle(temphe);

		TriMesh::Point p1 = mesh->point(v1);
		TriMesh::Point p2 = mesh->point(v2);

		TriMesh::Normal u = p1 - p;
		TriMesh::Normal v = p2 - p;

		TriMesh::Normal tempnorm = u % v;

		if ((tempnorm | fnorm) < 0.0)
		{
			nfh = mesh->face_handle(mesh->opposite_halfedge_handle(temphe));
			return false;
		}

		temphe = mesh->next_halfedge_handle(temphe);
	} while (temphe != fhe);

	return true;
}

bool is_degenerated(const TriMesh* mesh, const TriMesh::VertexHandle& vh)
{
	if (!mesh)
		return false;

	TriMesh::Point vp = mesh->point(vh);
	for (TriMesh::ConstVertexOHalfedgeIter vohit = mesh->cvoh_begin(vh); vohit != mesh->cvoh_end(vh); vohit++)
	{
		TriMesh::HalfedgeHandle temphe = mesh->next_halfedge_handle(*vohit);
		if (mesh->is_boundary(mesh->edge_handle(temphe)))
			continue;

		TriMesh::FaceHandle fh = mesh->opposite_face_handle(temphe);
		if (inside_mesh_face(mesh, fh, vp, fh))
			return true;
	}

	return false;
}

bool is_flippable(const TriMesh *mesh, const int e)
{
	TriMesh::EdgeHandle eh = mesh->edge_handle(e);
	if (!mesh->is_flip_ok(eh))
		return false;

	TriMesh::HalfedgeHandle he = mesh->halfedge_handle(eh, 0);
	TriMesh::Point v1 = mesh->point(mesh->to_vertex_handle(mesh->opposite_halfedge_handle(he)));
	TriMesh::Point v2 = mesh->point(mesh->to_vertex_handle(mesh->next_halfedge_handle(he)));
	TriMesh::Point v3 = mesh->point(mesh->to_vertex_handle(he));
	TriMesh::Point v4 = mesh->point(mesh->to_vertex_handle(mesh->next_halfedge_handle(mesh->opposite_halfedge_handle(he))));

	TriMesh::Normal v3v2 = v2 - v3;
	TriMesh::Normal v4v2 = v2 - v4;
	double cos423 = dot(v3v2, v4v2) / v3v2.norm() / v4v2.norm();
	TriMesh::Normal v3v4 = v4 - v3;
	TriMesh::Normal v2v4 = v4 - v2;
	double cos243 = dot(v3v4, v2v4) / v3v4.norm() / v2v4.norm();

	TriMesh::Normal v1v2 = v2 - v1;
	double cos421 = dot(v1v2, v4v2) / v1v2.norm() / v4v2.norm();
	TriMesh::Normal v1v4 = v4 - v1;
	double cos241 = dot(v1v4, v2v4) / v1v4.norm() / v2v4.norm();

	TriMesh::Normal cross1 = cross(v1v2, v1v4);
	TriMesh::Normal cross2 = cross(v3v4, v3v2);

	double dotval = dot(cross1, cross2);

	bool is_flipping = false;

	if (/*cos421 < cos(M_PI / 18.0) && cos241 < cos(M_PI / 18.0)
		&& cos423 < cos(M_PI / 18.0)&& cos243 < cos(M_PI / 18.0)
		&& */dotval > 0.0)
	{
		is_flipping = true;
	}

	return is_flipping;
}

bool is_collapse_ok(TriMesh *mesh, const TriMesh::HalfedgeHandle &he)
{
	if (!mesh->is_collapse_ok(he))
		return false;

	TriMesh::VertexHandle tovh = mesh->to_vertex_handle(he);
	TriMesh::Point tovp = mesh->point(tovh);

	TriMesh::HalfedgeHandle temp = mesh->opposite_halfedge_handle(mesh->prev_halfedge_handle(he));

	std::vector<TriMesh::Normal> crosses;
	while (temp != mesh->next_halfedge_handle(mesh->opposite_halfedge_handle(he)))
	{
		TriMesh::HalfedgeHandle next = mesh->opposite_halfedge_handle(mesh->prev_halfedge_handle(temp));
		TriMesh::Point v2 = mesh->point(mesh->to_vertex_handle(temp));
		TriMesh::Point v3 = mesh->point(mesh->to_vertex_handle(next));

		TriMesh::Normal vec1 = v2 - tovp;
		TriMesh::Normal vec2 = v3 - tovp;
		crosses.push_back(cross(vec1, vec2));

		temp = next;
	}

	int size = (int)crosses.size();

	if (size < 1)
		return false;

	if (size == 1)
		return true;

	for (int i = 0; i < size; i++)
	{
		int j = (i + 1) % size;
		if (dot(crosses[i], crosses[j]) < 0.0)
			return false;
	}

	return true;
}
