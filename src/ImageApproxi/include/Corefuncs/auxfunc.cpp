#include "Corefuncs/auxfunc.h"
#include <QColor>
#include <QFile>
#include <QTextStream>
#include <QDir>

bool SortFunction0(BaryPoint p1, BaryPoint p2) { return (p1.lamda0 < p2.lamda0); }
bool SortFunction1(BaryPoint p1, BaryPoint p2) { return (p1.lamda1 < p2.lamda1); }
bool SortFunction2(BaryPoint p1, BaryPoint p2) { return (p1.lamda2 < p2.lamda2); }
bool sortintPair_SecondGreater( pair<int, int> p1, pair<int, int> p2) { return p1.second < p2.second; }
bool sortintPair_SecondSmaller(pair<int, int> p1, pair<int, int> p2) { return p1.second > p2.second; }
bool sortFaceGreater(FaceSort f1, FaceSort f2)
{
	assert(f1.face.size() == 3 && f2.face.size() == 3);
	if (f1.face[0] < f2.face[0])
		return  true;
	else if (f1.face[0] == f2.face[0] && f1.face[1] < f2.face[1])
		return  true;
	else if (f1.face[0] == f2.face[0] && f1.face[1] == f2.face[1] && f1.face[2] < f2.face[2])
		return  true;
	return false;
}

bool sortConfigGreater(TConfig2 t1, TConfig2 t2)
{
	assert(t1.tconfig.size() == t2.tconfig.size());

	TConfig1 c1 = t1.tconfig;
	TConfig1 c2 = t2.tconfig;

	sort(c1.begin(), c1.end());
	sort(c2.begin(), c2.end());

	for (int i = 0;i<c1.size();i++)
	{
		if (i == c1.size() - 1)
		{
			if (c1[i] < c2[i])
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		if (c1[i] < c2[i])
		{
			return true;
		}
		else if(c1[i] == c2[i])
		{
			continue;
		}
		else
		{
			return false;
		}
	}
}

bool sortdoublePair_SecondGreater(pair<int,double> p1, pair<int, double> p2){return p1.second < p2.second;}
bool sortdoublePair_SecondSmaller(pair<int, double> p1, pair<int, double> p2) { return p1.second > p2.second; }
bool sortspoterr_Smaller(PixelAttribute p1, PixelAttribute p2) { return p1.err_spotsum > p2.err_spotsum; };

void from_vfdata_to_mesh(Mesh **temp_mesh, vector<Point_3>&vs, 
	vector<vector<int>>&faces)
{
	if (*temp_mesh == NULL)
		*temp_mesh = new Mesh();
	else
		(*temp_mesh)->clear();

	//cdt2 can not replace
	int v_num = vs.size();
	int f_num = faces.size();
	int fv_num = 3;
	Point_3 *vertex = new Point_3[v_num];
	int **face = new int*[f_num];
	for (int i = 0; i < f_num; i++)
	{
		face[i] = new int[fv_num];
	}

	for (int i = 0;i<vs.size();i++)
	{
		vertex[i] =vs[i];
	}

	for (int i = 0;i<faces.size();i++)
	{
		for (int j = 0;j<faces[i].size();j++)
		{
			face[i][j] = faces[i][j];
		}
	}

	Parser_obj<Kernel3, Enriched_items> parser;
	Builder_obj<Mesh::HalfedgeDS> builder(vertex, face, v_num, f_num, fv_num);
	(*temp_mesh)->delegate(builder);

	(*temp_mesh)->compute_bounding_box();
	(*temp_mesh)->compute_type();
	(*temp_mesh)->compute_normals();
	(*temp_mesh)->compute_index();
	(*temp_mesh)->initial_mesh_status();
	(*temp_mesh)->map_vertex_index_to_iterator();
	(*temp_mesh)->computeBorderVerticesType();

	delete[]vertex;
	for (int i = 0; i < f_num; i++)
	{
		delete[]face[i];
	}
	delete[]face;
}

void from_triangulationdata_to_mesh(CDT &temp_cdt, Mesh **temp_mesh, vector<double>	&vertice_cdt_value)
{
	if (*temp_mesh == NULL)
		*temp_mesh = new Mesh();
	else
		(*temp_mesh)->clear();

	//cdt2 can not replace
	int v_num = temp_cdt.number_of_vertices();
	int f_num = temp_cdt.number_of_faces();
	int fv_num = 3;
	Point_3 *vertex = new Point_3[v_num];
	int **face = new int*[f_num];
	for (int i = 0; i < f_num; i++)
	{
		face[i] = new int[fv_num];
	}

	CDT::Finite_vertices_iterator v_it;

	int i = 0;
	for (v_it = temp_cdt.vertices_begin(); v_it != temp_cdt.vertices_end(); v_it++)
	{
		v_it->set_associated_index(i);
		i++;
	}

	int vi = 0;
	for (v_it = temp_cdt.vertices_begin(); v_it != temp_cdt.vertices_end(); v_it++)
	{
		double x = v_it->point().x();
		double y = v_it->point().y();
		double z = vertice_cdt_value[vi];
		vertex[vi] = Point_3(x, y, z);
		vi++;
	}

	CDT::Finite_faces_iterator f_it;
	int fi = 0;
	for (f_it = temp_cdt.faces_begin(); f_it != temp_cdt.faces_end(); f_it++)
	{
		face[fi][0] =f_it->vertex(0)->get_associated_index();
		face[fi][1] =f_it->vertex(1)->get_associated_index();
		face[fi][2] =f_it->vertex(2)->get_associated_index();
		fi++;
	}

	Parser_obj<Kernel3, Enriched_items> parser;
	Builder_obj<Mesh::HalfedgeDS> builder(vertex, face, v_num, f_num, fv_num);
	(*temp_mesh)->delegate(builder);

	(*temp_mesh)->compute_bounding_box();
	(*temp_mesh)->compute_type();
	(*temp_mesh)->compute_normals();
	(*temp_mesh)->compute_index();
	(*temp_mesh)->initial_mesh_status();
	(*temp_mesh)->map_vertex_index_to_iterator();
	(*temp_mesh)->computeBorderVerticesType();

	delete[]vertex;
	for (int i = 0; i < f_num; i++)
	{
		delete[]face[i];
	}
	delete[]face;
}

void from_triangulationdata_to_mesh(CDT_Refine &temp_cdt, Mesh **temp_mesh, vector<double>	&vertice_cdt_value)
{
	//cdt2 can not replace
	int v_num = temp_cdt.number_of_vertices();
	int f_num = temp_cdt.number_of_faces();
	int fv_num = 3;
	Point_3 *vertex = new Point_3[v_num];
	int **face = new int*[f_num];
	for (int i = 0; i < f_num; i++)
	{
		face[i] = new int[fv_num];
	}

	CDT_Refine::Vertex_iterator v_it;
	for (v_it = temp_cdt.vertices_begin(); v_it != temp_cdt.vertices_end(); v_it++)
	{
		double x = v_it->point().x();
		double y = v_it->point().y();
		int index_ = v_it->get_associated_index();
		double z = vertice_cdt_value[v_it->get_associated_index()];
		vertex[v_it->get_associated_index()] = Point_3(x, y, z);
	}

	CDT_Refine::Face_iterator f_it;
	int fi = 0;
	for (f_it = temp_cdt.faces_begin(); f_it != temp_cdt.faces_end(); f_it++)
	{
		face[fi][0] = f_it->vertex(0)->get_associated_index();
		face[fi][1] = f_it->vertex(1)->get_associated_index();
		face[fi][2] = f_it->vertex(2)->get_associated_index();
		fi++;
	}

	Parser_obj<Kernel3, Enriched_items> parser;
	Builder_obj<Mesh::HalfedgeDS> builder(vertex, face, v_num, f_num, fv_num);
	(*temp_mesh)->delegate(builder);

	(*temp_mesh)->compute_bounding_box();
	(*temp_mesh)->compute_type();
	(*temp_mesh)->compute_normals();
	(*temp_mesh)->compute_index();
	(*temp_mesh)->initial_mesh_status();
	(*temp_mesh)->map_vertex_index_to_iterator();
	(*temp_mesh)->computeBorderVerticesType();

	delete[]vertex;
	for (int i = 0; i < f_num; i++)
	{
		delete[]face[i];
	}
	delete[]face;
}

void from_cdt2mesh(CDT &temp_cdt, Mesh **temp_mesh)
{
	int v_num = temp_cdt.number_of_vertices();
	int f_num = temp_cdt.number_of_faces();
	int fv_num = 3;
	Point_3 *vertex = new Point_3[v_num];
	int **face = new int*[f_num];
	for (int i = 0; i < f_num; i++)
	{
		face[i] = new int[fv_num];
	}

	CDT::Vertex_iterator v_it;
	for (v_it = temp_cdt.vertices_begin(); v_it != temp_cdt.vertices_end(); v_it++)
	{
		double x = v_it->point().x();
		double y = v_it->point().y();
		int index_ = v_it->get_associated_index();
		double z = 0.0;
		vertex[v_it->get_associated_index()] = Point_3(x, y, z);
	}

	CDT::Face_iterator f_it;
	int fi = 0;
	for (f_it = temp_cdt.faces_begin(); f_it != temp_cdt.faces_end(); f_it++)
	{
		face[fi][0] = f_it->vertex(0)->get_associated_index();
		face[fi][1] = f_it->vertex(1)->get_associated_index();
		face[fi][2] = f_it->vertex(2)->get_associated_index();
		fi++;
	}

	Parser_obj<Kernel3, Enriched_items> parser;
	Builder_obj<Mesh::HalfedgeDS> builder(vertex, face, v_num, f_num, fv_num);
	(*temp_mesh)->delegate(builder);

	(*temp_mesh)->compute_bounding_box();
	(*temp_mesh)->compute_type();
	(*temp_mesh)->compute_normals();
	(*temp_mesh)->compute_index();
	(*temp_mesh)->initial_mesh_status();
	(*temp_mesh)->map_vertex_index_to_iterator();
	(*temp_mesh)->computeBorderVerticesType();

	delete[]vertex;
	for (int i = 0; i < f_num; i++)
	{
		delete[]face[i];
	}
	delete[]face;
}

void from_interior_to_two_ring_facets(int inte, set<Mesh::Facet_iterator> &two_ring_face, Mesh**tempmesh)
{
	Mesh::Vertex_iterator v_it;
	v_it = (*tempmesh)->get_vertex_iterator(inte);
	Mesh::Halfedge_around_vertex_circulator h_it = v_it->vertex_begin();
	vector<int> one_ring_cycle;
	do
	{
		Mesh::Facet_iterator fit = h_it->facet();
		if (!h_it->is_border())
		{
			two_ring_face.insert(fit);
		}
		one_ring_cycle.push_back(h_it->opposite()->vertex()->vertex_index());
	} while (--h_it != v_it->vertex_begin());

	for (int i = 0;i<one_ring_cycle.size();i++)
	{
		Mesh::Vertex_iterator vit;
		vit = (*tempmesh)->get_vertex_iterator(one_ring_cycle[i]);
		Mesh::Halfedge_around_vertex_circulator hit = vit->vertex_begin();
		do
		{
			Mesh::Facet_iterator fit = hit->facet();
			if (!hit->is_border())
			{
				two_ring_face.insert(fit);
			}
		} while (--hit != vit->vertex_begin());
	}
}

void from_interior_to_adjacant_facets(int inte,int sz_ring, set<Mesh::Facet_iterator> &ring_face, Mesh**tempmesh)
{
	Mesh::Vertex_iterator v_it;
	v_it = (*tempmesh)->get_vertex_iterator(inte);
	Mesh::Halfedge_around_vertex_circulator h_it = v_it->vertex_begin();
	vector<int> one_ring_cycle;
	do
	{
		Mesh::Facet_iterator fit = h_it->facet();
		if (!h_it->is_border())
		{
			ring_face.insert(fit);
		}
		one_ring_cycle.push_back(h_it->opposite()->vertex()->vertex_index());
	} while (--h_it != v_it->vertex_begin());

	if (sz_ring > 1)
	{
		sz_ring--;

		for (int i = 0; i < one_ring_cycle.size(); i++)
		{
			from_interior_to_adjacant_facets(one_ring_cycle[i], sz_ring, ring_face, tempmesh);
		}		
	}
}

void from_corner_triple_to_constrains(int deg, int id_cn,vector<unsigned int> corner_seq, vector<RuptureKnotConfig> &rupture_edge_basis_config)
{
	if (deg == 1)
	{
		return;
	}

	if (corner_seq.size()%2 == 0 || id_cn != int(corner_seq.size()-1)/2)
	{
		//degree 2
		if (deg > 1)
		{
			RuptureKnotConfig rcd20;
			rcd20.inte = { corner_seq[1] ,corner_seq[1] }; 
			rcd20.constrain = { corner_seq[0],corner_seq[2] };
			rupture_edge_basis_config.push_back(rcd20);

			RuptureKnotConfig rcd21;
			rcd21.inte = { corner_seq[1] ,corner_seq[2] };
			rcd21.constrain = { corner_seq[3],corner_seq[1] };
			rupture_edge_basis_config.push_back(rcd21);
		}
		//degree 3
		if (deg > 2)
		{
			RuptureKnotConfig rcd30;
			rcd30.inte = { corner_seq[1] ,corner_seq[1], corner_seq[1] };
			rcd30.constrain = { corner_seq[0],corner_seq[2] };
			rcd30.split_spare_index = -1;
			rupture_edge_basis_config.push_back(rcd30);

			RuptureKnotConfig rcd31;
			rcd31.inte = { corner_seq[1] ,corner_seq[2],corner_seq[3] };
			rcd31.constrain = { corner_seq[4],corner_seq[1] };
			rcd31.split_spare_index = corner_seq[0];
			rupture_edge_basis_config.push_back(rcd31);

			RuptureKnotConfig rcd32;
			rcd32.inte = { corner_seq[1] ,corner_seq[1],corner_seq[2] };
			rcd32.constrain = { corner_seq[3],corner_seq[1] };
			rcd32.split_spare_index = corner_seq[0];
			rupture_edge_basis_config.push_back(rcd32);
		}
	}
	else
	{
		//degree 2
		if (deg > 1)
		{
			RuptureKnotConfig rcd20;
			rcd20.inte = { corner_seq[id_cn] ,corner_seq[id_cn] };
			rcd20.constrain = { corner_seq[id_cn-1],corner_seq[id_cn+1] };
			rupture_edge_basis_config.push_back(rcd20);

			RuptureKnotConfig rcd21;
			rcd21.inte = { corner_seq[id_cn] ,corner_seq[id_cn+1] };
			rcd21.constrain = { corner_seq[id_cn + 2],corner_seq[id_cn] };
			rcd21.bInverse_order = true;
			rupture_edge_basis_config.push_back(rcd21);

			RuptureKnotConfig rcd22;
			rcd22.inte = { corner_seq[id_cn] ,corner_seq[id_cn - 1] };
			rcd22.constrain = { corner_seq[id_cn - 2],corner_seq[id_cn] };
			rupture_edge_basis_config.push_back(rcd22);
		}
		//degree 3
		if (deg > 2)
		{
			RuptureKnotConfig rcd30;
			rcd30.inte = { corner_seq[id_cn] ,corner_seq[id_cn], corner_seq[id_cn] };
			rcd30.constrain = { corner_seq[id_cn-1],corner_seq[id_cn+1] };
			rcd30.split_spare_index = -1;
			rupture_edge_basis_config.push_back(rcd30);

			RuptureKnotConfig rcd3r1;
			rcd3r1.inte = { corner_seq[id_cn] ,corner_seq[id_cn],corner_seq[id_cn+1] };
			rcd3r1.constrain = { corner_seq[id_cn+2],corner_seq[id_cn] };
			rcd3r1.split_spare_index = corner_seq[id_cn-1];
			rupture_edge_basis_config.push_back(rcd3r1);

			RuptureKnotConfig rcd3r2;
			rcd3r2.inte = { corner_seq[id_cn] ,corner_seq[id_cn+1],corner_seq[id_cn+2] };
			rcd3r2.constrain = { corner_seq[id_cn+3],corner_seq[id_cn] };
			rcd3r2.split_spare_index = corner_seq[id_cn - 1];
			rupture_edge_basis_config.push_back(rcd3r2);

			RuptureKnotConfig rcd3l1;
			rcd3l1.inte = { corner_seq[id_cn] ,corner_seq[id_cn],corner_seq[id_cn - 1] };
			rcd3l1.constrain = { corner_seq[id_cn - 2],corner_seq[id_cn] };
			rcd3l1.split_spare_index = corner_seq[id_cn + 1];
			rupture_edge_basis_config.push_back(rcd3l1);

			RuptureKnotConfig rcd3l2;
			rcd3l2.inte = { corner_seq[id_cn] ,corner_seq[id_cn - 1],corner_seq[id_cn - 2] };
			rcd3l2.constrain = { corner_seq[id_cn - 3],corner_seq[id_cn] };
			rcd3l2.split_spare_index = corner_seq[id_cn + 1];
			rupture_edge_basis_config.push_back(rcd3l2);
		}
	}
}

void from_interior_to_ccw_one_ring_cycle(const vector<int> &interior_points, vector<int> &one_ring_cycle, Mesh **originalmesh)
{
	if (interior_points.size() == 0)
	{
		return;
	}
	else if (interior_points.size() == 1)
	{
		Mesh::Vertex_iterator v_it;
		v_it = (*originalmesh)->get_vertex_iterator(interior_points[0]);
		Mesh::Halfedge_around_vertex_circulator h_it = v_it->vertex_begin();
		if (v_it->is_corner())
		{
			Mesh::Halfedge_iterator h_push, push_start;
			do
			{
				if (h_it->opposite()->vertex()->is_border() && h_it->next_on_vertex()->opposite()->vertex()->is_border())
				{
					push_start = h_it;
					break;
				}
			} while (--h_it != v_it->vertex_begin());

			h_push = push_start;
			do
			{
				one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
				h_push = h_push->prev_on_vertex();
			} while (h_push != push_start);
		}
		else
		{
			do
			{
				one_ring_cycle.push_back(h_it->opposite()->vertex()->vertex_index());
			} while (--h_it != v_it->vertex_begin());
		}

	}
	else if (interior_points.size() == 2)
	{
		Mesh::Vertex_iterator vertex1, vertex2;
		vertex1 = (*originalmesh)->get_vertex_iterator(interior_points[0]);
		vertex2 = (*originalmesh)->get_vertex_iterator(interior_points[1]);
		Point_2 v1 = vertex1->get_domain(), v2 = vertex2->get_domain();
		bool is_adjacent_corner = false;
		if (abs(v1.x() - v2.x()) > 1e-8 && abs(v1.y() - v2.y()) > 1e-8)
		{
			is_adjacent_corner = true;
		}

		//test
		if ((interior_points[0] == 4 && interior_points[1] == 9) || (interior_points[0] == 9 && interior_points[1] == 4))
		{
			int t = 1;
		}

		if (vertex1->is_border() && vertex2->is_border() && is_adjacent_corner == false)
		{
			Mesh::Halfedge_iterator h1_adjacent, h2_adjacent;
			bool is_find = false;
			Mesh::Halfedge_around_vertex_circulator h1_it = vertex1->vertex_begin();
			do
			{
				Mesh::Halfedge_around_vertex_circulator h2_it = vertex2->vertex_begin();
				do
				{
					if (h2_it->opposite()->vertex()->vertex_index() == h1_it->opposite()->vertex()->vertex_index())
					{
						is_find = true;
						h1_adjacent = h1_it;
						h2_adjacent = h2_it;
					}
				} while (!is_find && ++h2_it != vertex2->vertex_begin());
			} while (!is_find && ++h1_it != vertex1->vertex_begin());

			Mesh::Halfedge_iterator boundh_to1, boundh_to2, h2to1;
			Mesh::Halfedge_around_vertex_circulator h_it = vertex1->vertex_begin();
			do
			{
				if (h_it->opposite()->vertex() == vertex2)
				{
					h2to1 = h_it;
					break;
				}
			} while (++h_it != vertex1->vertex_begin());

			/*h_it = vertex2->vertex_begin();
			do
			{
			if (h_it->opposite()->vertex()->is_border() && h_it->opposite()->vertex() != vertex1)
			{
			boundh_to2 = h_it;
			break;
			}
			} while (++h_it != vertex2->vertex_begin());*/
			Mesh::Halfedge_iterator push_start_edge, push_inter_tostart, push_inter_toend, push_end_edge;
			if (h1_adjacent->next_on_vertex() == h2to1 && h2_adjacent->prev_on_vertex() == h2to1->opposite())//only one condition maybe wrong
			{
				boundh_to1 = h2to1->next_on_vertex();
				boundh_to2 = h2to1->opposite()->prev_on_vertex();
				push_start_edge = boundh_to2;
				push_inter_tostart = h2_adjacent;
				push_inter_toend = h1_adjacent;
				push_end_edge = boundh_to1;
			}
			else
			{
				boundh_to1 = h2to1->prev_on_vertex();
				boundh_to2 = h2to1->opposite()->next_on_vertex();
				push_start_edge = boundh_to1;
				push_inter_tostart = h1_adjacent;
				push_inter_toend = h2_adjacent;
				push_end_edge = boundh_to2;
			}

			Mesh::Halfedge_iterator h_push = push_start_edge;
			do
			{
				if (h_push == push_inter_tostart)
				{
					break;
				}
				one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
				h_push = h_push->prev_on_vertex();
			} while (h_push != push_inter_tostart);

			h_push = push_inter_toend;
			do
			{
				if (h_push == push_end_edge)
				{
					break;
				}
				one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
				h_push = h_push->prev_on_vertex();
			} while (h_push != push_end_edge);
			one_ring_cycle.push_back(push_end_edge->opposite()->vertex()->vertex_index());

		}
		else
		{
			Mesh::Halfedge_iterator h1_adjacent1, h2_adjacent1, h1_adjacent2, h2_adjacent2;

			Mesh::Halfedge_around_vertex_circulator h_it = vertex1->vertex_begin();
			Mesh::Halfedge_iterator halfede_2to1;
			do
			{
				if (h_it->opposite()->vertex() == vertex2)
				{
					halfede_2to1 = h_it;
					break;
				}
			} while (++h_it != vertex1->vertex_begin());

			h1_adjacent1 = halfede_2to1->next_on_vertex();
			h2_adjacent1 = halfede_2to1->opposite()->prev_on_vertex();
			h1_adjacent2 = halfede_2to1->prev_on_vertex();
			h2_adjacent2 = halfede_2to1->opposite()->next_on_vertex();

			Mesh::Halfedge_iterator start_to1, end_to1, start_to2, end_to2;
			if (h1_adjacent1->next_on_vertex()->opposite()->vertex() == vertex2)
			{
				start_to1 = h1_adjacent1;
				end_to1 = h1_adjacent2;
			}
			else
			{
				start_to1 = h1_adjacent2;
				end_to1 = h1_adjacent1;
			}

			if (h2_adjacent1->next_on_vertex()->opposite()->vertex() == vertex1)
			{
				start_to2 = h2_adjacent1;
				end_to2 = h2_adjacent2;
			}
			else
			{
				start_to2 = h2_adjacent2;
				end_to2 = h2_adjacent1;
			}

			Mesh::Halfedge_iterator h_push = start_to1;
			do
			{
				if (h_push == end_to1)
				{
					break;
				}
				one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
				h_push = h_push->prev_on_vertex();
			} while (h_push != end_to1);
			h_push = start_to2;
			do
			{
				if (h_push == end_to2)
				{
					break;
				}
				one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
				h_push = h_push->prev_on_vertex();
			} while (h_push != end_to2);
		}
	}
	else if (interior_points.size() == 3)
	{
		Mesh::Vertex_iterator vertex1, vertex2, vertex3;
		vertex1 = (*originalmesh)->get_vertex_iterator(interior_points[0]);
		vertex2 = (*originalmesh)->get_vertex_iterator(interior_points[1]);
		vertex3 = (*originalmesh)->get_vertex_iterator(interior_points[2]);

		if (CGAL::area(vertex1->get_domain(), vertex2->get_domain(), vertex3->get_domain()) < 0)
		{
			vertex1 = (*originalmesh)->get_vertex_iterator(interior_points[2]);
			vertex3 = (*originalmesh)->get_vertex_iterator(interior_points[0]);
		}

		Mesh::Halfedge_iterator halfedge_1to2, halfedge_2to3, halfedge_3to1;

		Mesh::Halfedge_around_vertex_circulator h_it = vertex1->vertex_begin();
		do
		{
			if (h_it->opposite()->vertex() == vertex3)
			{
				halfedge_3to1 = h_it;
				break;
			}
		} while (++h_it != vertex1->vertex_begin());

		halfedge_1to2 = halfedge_3to1->next_on_vertex()->opposite();
		halfedge_2to3 = halfedge_1to2->next_on_vertex()->opposite();

		bool bonly_v1_nobound = !vertex1->is_border() && vertex2->is_border() && vertex3->is_border();
		bool bonly_v2_nobound = vertex1->is_border() && !vertex2->is_border() && vertex3->is_border();
		bool bonly_v3_nobound = vertex1->is_border() && vertex2->is_border() && !vertex3->is_border();

		bool is_case1, is_case2, is_case3;
		if (vertex1->is_border() && vertex2->is_border() && vertex3->is_border())
		{
			is_case1 = true; is_case2 = false; is_case3 = false;
		}
		else if (bonly_v1_nobound || bonly_v2_nobound || bonly_v3_nobound)
		{
			is_case1 = false; is_case2 = true; is_case3 = false;
		}
		else
		{
			is_case1 = false; is_case2 = false; is_case3 = true;
		}


		if (is_case1)
		{
			Mesh::Vertex_iterator v_corner, v_pre, v_next;
			Mesh::Halfedge_iterator halfedge_cor_to_next, halfedge_pre_to_cor, halfedge_next_to_pre;
			if (vertex1->is_corner())
			{
				v_corner = vertex1; v_pre = vertex3; v_next = vertex2;
				halfedge_cor_to_next = halfedge_1to2; halfedge_next_to_pre = halfedge_2to3; halfedge_pre_to_cor = halfedge_3to1;
			}
			else if (vertex2->is_corner())
			{
				v_corner = vertex2; v_pre = vertex1; v_next = vertex3;
				halfedge_cor_to_next = halfedge_2to3; halfedge_next_to_pre = halfedge_3to1; halfedge_pre_to_cor = halfedge_1to2;
			}
			else
			{
				v_corner = vertex3; v_pre = vertex2; v_next = vertex1;
				halfedge_cor_to_next = halfedge_3to1; halfedge_next_to_pre = halfedge_1to2; halfedge_pre_to_cor = halfedge_2to3;
			}

			Mesh::Halfedge_iterator h_intersect_to_next, h_intersect_to_pre;
			h_intersect_to_next = halfedge_next_to_pre->opposite()->next_on_vertex();
			h_intersect_to_pre = halfedge_next_to_pre->prev_on_vertex();

			Mesh::Halfedge_iterator start_tonext, end_tonext, start_topre, end_topre;
			start_tonext = halfedge_cor_to_next->prev_on_vertex();
			end_tonext = h_intersect_to_next;

			end_topre = halfedge_pre_to_cor->opposite()->next_on_vertex();
			start_topre = h_intersect_to_pre;

			Mesh::Halfedge_iterator h_push = start_tonext;
			do
			{
				if (h_push == end_tonext)
				{
					break;
				}
				one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
				h_push = h_push->prev_on_vertex();
			} while (h_push != end_tonext);
			h_push = start_topre;
			do
			{
				if (h_push == end_topre)
				{
					break;
				}
				one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
				h_push = h_push->prev_on_vertex();
			} while (h_push != end_topre);
			one_ring_cycle.push_back(end_topre->opposite()->vertex()->vertex_index());
		}
		if (is_case2)
		{
			Mesh::Vertex_iterator vbound_ccw, vbound_cw, v_inte;
			Mesh::Halfedge_iterator h_inte_to_cw, h_ccw_to_inte, h_cw_to_ccw;

			if (bonly_v1_nobound)
			{
				vbound_ccw = vertex3; vbound_cw = vertex2; v_inte = vertex1;
				h_inte_to_cw = halfedge_1to2; h_ccw_to_inte = halfedge_3to1; h_cw_to_ccw = halfedge_2to3;
			}
			else if (bonly_v2_nobound)
			{
				vbound_ccw = vertex1; vbound_cw = vertex3; v_inte = vertex2;
				h_inte_to_cw = halfedge_2to3; h_ccw_to_inte = halfedge_1to2; h_cw_to_ccw = halfedge_3to1;
			}
			else if (bonly_v3_nobound)
			{
				vbound_ccw = vertex2; vbound_cw = vertex1; v_inte = vertex3;
				h_inte_to_cw = halfedge_3to1; h_ccw_to_inte = halfedge_2to3; h_cw_to_ccw = halfedge_1to2;
			}

			Mesh::Halfedge_iterator start_toccw, end_toccw, start_tocw, end_tocw, start_tointe, end_tointe;
			start_toccw = h_cw_to_ccw->prev_on_vertex();
			end_tocw = h_cw_to_ccw->opposite()->next_on_vertex();
			end_toccw = h_ccw_to_inte->opposite()->next_on_vertex();
			start_tointe = h_ccw_to_inte->prev_on_vertex();
			end_tointe = h_inte_to_cw->opposite()->next_on_vertex();
			start_tocw = h_inte_to_cw->prev_on_vertex();

			bool is_continuing = true;
			if (start_toccw->opposite()->vertex() == end_tocw->opposite()->vertex())
			{
				is_continuing = false;
				is_case3 = true;
			}

			if (is_continuing)
			{
				Mesh::Halfedge_iterator h_push = start_toccw;
				do
				{
					if (start_toccw == end_toccw)
					{
						break;
					}
					one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
					h_push = h_push->prev_on_vertex();
				} while (h_push != end_toccw);
				h_push = start_tointe;
				do
				{
					if (start_tointe == end_tointe)
					{
						break;
					}
					one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
					h_push = h_push->prev_on_vertex();
				} while (h_push != end_tointe);
				h_push = start_tocw;
				do
				{
					if (start_tocw == end_tocw)
					{
						break;
					}
					one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
					h_push = h_push->prev_on_vertex();
				} while (h_push != end_tocw);
				one_ring_cycle.push_back(end_tocw->opposite()->vertex()->vertex_index());
			}
		}
		if (is_case3)
		{
			Mesh::Halfedge_iterator h_intersect12_to_1, h_intersect12_to_2, h_intersect23_to_2, h_intersect23_to_3, h_intersect31_to_1, h_intersect31_to_3;

			Mesh::Halfedge_iterator start_to1, end_to1, start_to2, end_to2, start_to3, end_to3;

			h_intersect23_to_2 = halfedge_2to3->opposite()->next_on_vertex();
			h_intersect23_to_3 = halfedge_2to3->prev_on_vertex();
			h_intersect31_to_3 = halfedge_3to1->opposite()->next_on_vertex();
			h_intersect31_to_1 = halfedge_3to1->prev_on_vertex();
			h_intersect12_to_1 = halfedge_1to2->opposite()->next_on_vertex();
			h_intersect12_to_2 = halfedge_1to2->prev_on_vertex();

			start_to2 = h_intersect12_to_2;
			end_to2 = h_intersect23_to_2;

			start_to3 = h_intersect23_to_3;
			end_to3 = h_intersect31_to_3;

			start_to1 = h_intersect31_to_1;
			end_to1 = h_intersect12_to_1;

			Mesh::Halfedge_iterator h_push = start_to1;
			do
			{
				if (h_push == end_to1)
				{
					break;
				}
				one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
				h_push = h_push->prev_on_vertex();
			} while (h_push != end_to1);
			h_push = start_to2;
			do
			{
				if (h_push == end_to2)
				{
					break;
				}
				one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
				h_push = h_push->prev_on_vertex();
			} while (h_push != end_to2);
			h_push = start_to3;
			do
			{
				if (h_push == end_to3)
				{
					break;
				}
				one_ring_cycle.push_back(h_push->opposite()->vertex()->vertex_index());
				h_push = h_push->prev_on_vertex();
			} while (h_push != end_to3);
		}
	}
}

void remove_same_elements(vector<int> &temp_set)
{
	if (temp_set.size() == 0)
	{
		return;
	}
	vector<int> new_set = temp_set;
	temp_set.clear();
	temp_set.push_back(new_set[0]);
	for (int k = 1; k < new_set.size(); k++)
	{
		bool is_insert = true;
		for (int i = 0; i < temp_set.size(); i++)
		{
			if (new_set[k] == temp_set[i])
			{
				is_insert = false;
				break;
			}
		}
		if (is_insert)
		{
			temp_set.push_back(new_set[k]);
		}
	}
}

void remove_same_elements(vector<Point_2> &temp_set)
{
	if (temp_set.size() == 0)
	{
		return;
	}
	vector<Point_2> new_set = temp_set;
	temp_set.clear();
	temp_set.push_back(new_set[0]);
	for (unsigned k = 1; k < new_set.size(); k++)
	{
		bool is_insert = true;
		for (unsigned i = 0; i < temp_set.size(); i++)
		{
			if (abs(new_set[k].x() - temp_set[i].x()) < 1e-8
				&&abs(new_set[k].y() - temp_set[i].y()) < 1e-8)
			{
				is_insert = false;
				break;
			}
		}
		if (is_insert)
		{
			temp_set.push_back(new_set[k]);
		}
	}
}

void remove_same_elements(vector<Point2withIndex> &temp_set)
{
	if (temp_set.size() == 0)
	{
		return;
	}
	vector<Point2withIndex> new_set = temp_set;
	temp_set.clear();
	temp_set.push_back(new_set[0]);
	for (unsigned k = 1; k < new_set.size(); k++)
	{
		bool is_insert = true;
		for (unsigned i = 0; i < temp_set.size(); i++)
		{
			if (new_set[k].index == temp_set[i].index)
			{
				is_insert = false;
				break;
			}
		}
		if (is_insert)
		{
			temp_set.push_back(new_set[k]);
		}
	}
}


double dis_between_p_line(Point_2 line_p1,Point_2 line_p2,Point_2 tempp)
{
	Vector_2 p_linef = tempp - line_p1;
	Vector_2 line_ = line_p2 - line_p1;
	double pseg_on_line = (p_linef * line_) / sqrt(line_.squared_length());
	double p_to_linef = sqrt(p_linef.squared_length());
	double dis = sqrt(pow(p_to_linef,2) - pow(pseg_on_line,2));
	return dis;
}

//std::pair<int, double> findMaximumDistance(vector<PointTriple>& Points, double &error) {
//	Point_2 firstpoint = Points[0].imagedomain;
//	Point_2 lastpoint = Points[Points.size() - 1].imagedomain;
//	int index = 0;  //index to be returned
//	double Mdist = -1; //the Maximum distance to be returned-consider curvature
//
//	//distance calculation
//	Vector_2 p = lastpoint - firstpoint;
//	for (int i = 1; i < Points.size() - 1; i++) { //traverse through second point to second last point
//		Vector_2 pp = Points[i].imagedomain - firstpoint;
//		double Dist = fabs(pp.x()*p.y() - pp.y()*p.x()) / sqrt(p.squared_length()); //formula for point-to-line distance
//		if (Dist > Mdist) {
//			Mdist = Dist;
//			index = i;
//		}
//	}
//	
//	bool do_consider_curveture = true;
//
//	if (Mdist < error)
//	{
//		if (do_consider_curveture)
//		{
//			//predicate curvature degree
//			bool is_change = false;
//			for (int i = 0; i < Points.size(); i++)
//			{
//				double d1 = 2.0*sin(PI_ / 8.0);
//				double d2 = 2.0*sin(PI_ / 4.0);
//				double d3 = 2.0*sin(3.0*PI_ / 8.0);
//				if (Points[i].curvature > d1 && Points[i].curvature < d2 && Points.size()>8)
//				{
//					error = Mdist * 0.9;
//					is_change = true;
//				}
//				else if (Points[i].curvature > d2 && Points[i].curvature < d3 && Points.size()>6)
//				{
//					error = Mdist * 0.9;
//					is_change = true;
//				}
//				else if (Points[i].curvature > d3 && Points.size() > 4)
//				{
//					error = Mdist * 0.9;
//					is_change = true;
//				}
//				else if (Points[i].curvature > 1.9 && Points.size() > 3)
//				{
//					error = Mdist * 0.9;
//					is_change = true;
//				}
//				else if (Points[i].curvature > 2.0 && Points.size() > 1)
//				{
//					error = Mdist * 0.9;
//					is_change = true;
//				}
//			}
//			if (is_change)
//			{
//				std::cout << "polyline error modify: curvature large-err-" << error << "\n";
//			}
//		}
//		
//		//project middle points onto segment
//		for (int i = 1;i<Points.size() - 1;i++)
//		{
//			Vector_2 pp = Points[i].imagedomain - firstpoint;
//			Vector_2 pp_temp = lastpoint - firstpoint;
//			double size_ = (pp*pp_temp) / sqrt(pp_temp.squared_length());
//			Point_2 new_p = firstpoint + (size_)*pp_temp / sqrt(pp_temp.squared_length());
//			Points[i].paradomain = new_p;
//		}
//		if (do_consider_curveture)
//		{
//			//predicate if projected points: counter-direction or too narrow
//			for (int i = 0; i < Points.size() - 1; i++)
//			{
//				double segdis = sqrt((lastpoint - firstpoint).squared_length()) / Points.size();
//				double size_ = sqrt((Points[i].paradomain - Points[i + 1].paradomain).squared_length());
//				if (size_ < segdis / 10.0)
//				{
//					error = Mdist * 0.9;
//					std::cout << "polyline error modify: too narrow" << "\n";
//				}
//
//				Vector_2 pp = Points[i + 1].paradomain - Points[i].paradomain;
//				Vector_2 pp_temp = lastpoint - firstpoint;
//				if (pp * pp_temp < 0)
//				{
//					error = Mdist * 0.9;
//					std::cout << "polyline error modify: counter" << "\n";
//				}
//			}
//			//predicate if projected points: out range
//			for (int i = 1; i < Points.size() - 1; i++)
//			{
//				if (Points[i].paradomain.x() < std::min(lastpoint.x(), firstpoint.x()) || Points[i].paradomain.x() > std::max(lastpoint.x(), firstpoint.x())
//					|| Points[i].paradomain.y() < std::min(lastpoint.y(), firstpoint.y()) || Points[i].paradomain.y() > std::max(lastpoint.y(), firstpoint.y()))
//				{
//					error = Mdist * 0.9;
//					std::cout << "polyline error modify: out range" << "\n";
//				}
//			}
//
//			//number of pixel no more than 30
//			/*if (Points.size() > 30)
//			{
//			error = Mdist * 0.9;
//			std::cout << "polyline error modify: number of pixel no more than 50" << "\n";
//			return std::make_pair(index, Mdist);
//			}*/
//
//			//predicate if rotation angle is large
//			Vector_2 start_ = Points[1].imagedomain - Points[0].imagedomain;
//			Vector_2 end_ = Points[Points.size() - 1].imagedomain - Points[Points.size() - 2].imagedomain;
//			double cos_ = (start_*end_) / sqrt(start_.squared_length()*end_.squared_length());
//			if (cos_ < 0 && Points.size()>5)
//			{
//				error = Mdist * 0.9;
//				std::cout << "polyline error modify: rotation angle large" << "\n";
//			}
//		}
//		
//
//		return std::make_pair(index, Mdist);
//	}
//
//	return std::make_pair(index, Mdist);
//}

std::pair<int, double> findMaximumDistance(vector<PointTriple>& Points, double &error) {
	Point_2 firstpoint = Points[0].imagedomain;
	Point_2 lastpoint = Points[Points.size() - 1].imagedomain;
	int index = 0;  //index to be returned
	double Mdist = -1; //the Maximum distance to be returned-consider curvature

#if 0
	//distance calculation
	Vector_2 p = lastpoint - firstpoint;
	for (int i = 1; i < Points.size() - 1; i++) { //traverse through second point to second last point
		Vector_2 pp = Points[i].imagedomain - firstpoint;
		double Dist = fabs(pp.x()*p.y() - pp.y()*p.x()) / sqrt(p.squared_length()); //formula for point-to-line distance
		if (Dist > Mdist) {
			Mdist = Dist;
			index = i;
		}
	}
#else
	//distance calculation
	Vector_2 p = lastpoint - firstpoint;
	vector<double> distance_;
	double Mcurveture = -1;
	for (int i = 1; i < Points.size() - 1; i++) { //traverse through second point to second last point
		Vector_2 pp = Points[i].imagedomain - firstpoint;
		double Dist = fabs(pp.x()*p.y() - pp.y()*p.x()) / sqrt(p.squared_length()); //formula for point-to-line distance
		distance_.push_back(Dist);
		if (Dist > Mdist) {
			Mdist = Dist;
			//index = i;
		}
		if (Points[i].curvature > Mcurveture)
		{
			Mcurveture = Points[i].curvature;
		}
	}

	vector<pair<int,double>> stardard_;
	for (int i = 1; i < Points.size() - 1; i++)
	{
		stardard_.push_back(pair<int,double>(i,0.7*distance_[i-1] / Mdist +0.3*Points[i].curvature/Mcurveture));
	}
	sort(stardard_.begin(), stardard_.end(), sortdoublePair_SecondSmaller);
	index = stardard_[0].first;
#endif


	if (Mdist < error)
	{
		//project middle points onto segment
		for (int i = 1; i<Points.size() - 1; i++)
		{
			Vector_2 pp = Points[i].imagedomain - firstpoint;
			Vector_2 pp_temp = lastpoint - firstpoint;
			double size_ = (pp*pp_temp) / sqrt(pp_temp.squared_length());
			Point_2 new_p = firstpoint + (size_)*pp_temp / sqrt(pp_temp.squared_length());
			Points[i].paradomain = new_p;
		}
#if 1
		//predicate if projected points: counter-direction or too narrow
		for (int i = 0; i < Points.size() - 1; i++)
		{
			double segdis = sqrt((lastpoint - firstpoint).squared_length()) / Points.size();
			double size_ = sqrt((Points[i].paradomain - Points[i + 1].paradomain).squared_length());
			if (size_ < segdis / 10.0)
			{
				error = Mdist * 0.9;
				std::cout << __FUNCTION__ << ": " << "polyline error modify: too narrow" << "\n";
			}

			Vector_2 pp = Points[i + 1].paradomain - Points[i].paradomain;
			Vector_2 pp_temp = lastpoint - firstpoint;
			if (pp * pp_temp < 0)
			{
				error = Mdist * 0.9;
				std::cout << __FUNCTION__ << ": " << "polyline error modify: counter" << "\n";
			}
		}
		//predicate if projected points: out range
		for (int i = 1; i < Points.size() - 1; i++)
		{
			if (Points[i].paradomain.x() < std::min(lastpoint.x(), firstpoint.x()) || Points[i].paradomain.x() > std::max(lastpoint.x(), firstpoint.x())
				|| Points[i].paradomain.y() < std::min(lastpoint.y(), firstpoint.y()) || Points[i].paradomain.y() > std::max(lastpoint.y(), firstpoint.y()))
			{
				error = Mdist * 0.9;
				std::cout << __FUNCTION__ << ": " << "polyline error modify: out range" << "\n";
			}
		}

		////predicate if rotation angle is large
		//Vector_2 start_ = Points[1].imagedomain - Points[0].imagedomain;
		//Vector_2 end_ = Points[Points.size() - 1].imagedomain - Points[Points.size() - 2].imagedomain;
		//double cos_ = (start_*end_) / sqrt(start_.squared_length()*end_.squared_length());
		//if (cos_ < 0 && Points.size()>5)
		//{
		//	error = Mdist * 0.9;
		//	std::cout << __FUNCTION__ << ": " << "polyline error modify: rotation angle large" << "\n";
		//}
#endif


		return std::make_pair(index, Mdist);
	}

	return std::make_pair(index, Mdist);
}


void simplifyWithRDP(vector<PointTriple>& Points, double epsilon,bool do_consider_curvature)
{
	if (Points.size() < 3)
	{  
		if (Points.size() == 1)
		{
			Points[0].is_fixed = true;
			Points[0].paradomain = Points[0].imagedomain;
		}
		if (Points.size() == 2)
		{
			Points[0].is_fixed = true;
			Points[0].paradomain = Points[0].imagedomain;
			Points[Points.size() - 1].is_fixed = true;
			Points[Points.size() - 1].paradomain = Points[Points.size() - 1].imagedomain;
		}
		return;
	}

	Points[0].is_fixed = true;
	Points[0].paradomain = Points[0].imagedomain;
	Points[Points.size() - 1].is_fixed = true;
	Points[Points.size() - 1].paradomain = Points[Points.size() - 1].imagedomain;

	if (do_consider_curvature)
	{
		if (Points.size() > 6 && 0)
		{
			bool do_split_head = Points[0].curvature > 80 || Points[1].curvature > 80 || Points[2].curvature > 80;
			bool do_split_tail = Points[Points.size() - 1].curvature > 80 || Points[Points.size() - 2].curvature > 80
				|| Points[Points.size() - 3].curvature > 80;
			int internal_ = 3;
			if (do_split_head && !do_split_tail)
			{
				vector<PointTriple>::iterator it = Points.begin();
				vector<PointTriple> path1(Points.begin(), it + internal_+1); //new path l1 from 0 to index
				vector<PointTriple> path2(it + internal_, Points.end()); // new path l2 from index to last

				simplifyWithRDP(path1, epsilon);
				simplifyWithRDP(path2, epsilon);

				//simplify path1 and path2 together
				Points.clear();
				Points.insert(Points.begin(), path1.begin(), path1.end());
				Points.pop_back();
				Points.insert(Points.end(), path2.begin(), path2.end());
				return;
			}
			if (do_split_head && do_split_tail)
			{
				if (Points.size() > 12)
				{
					vector<PointTriple>::iterator it = Points.begin();
					vector<PointTriple> path1(Points.begin(), it + internal_+1);
					vector<PointTriple> path2(it + internal_, it + Points.size() - internal_);
					vector<PointTriple> path3(it + Points.size() - internal_-1, Points.end());
					simplifyWithRDP(path1, epsilon);
					simplifyWithRDP(path2, epsilon);
					simplifyWithRDP(path3, epsilon);
					Points.clear();
					Points.insert(Points.begin(), path1.begin(), path1.end());
					Points.pop_back();
					Points.insert(Points.end(), path2.begin(), path2.end());
					Points.pop_back();
					Points.insert(Points.end(), path3.begin(), path3.end());
					return;
				}
			}
			if (!do_split_head && do_split_tail)
			{
				vector<PointTriple>::iterator it = Points.begin();
				vector<PointTriple> path1(Points.begin(), it + Points.size() - internal_); //new path l1 from 0 to index
				vector<PointTriple> path2(it + Points.size() - internal_-1, Points.end()); // new path l2 from index to last

				simplifyWithRDP(path1, epsilon);
				simplifyWithRDP(path2, epsilon);

				//simplify path1 and path2 together
				Points.clear();
				Points.insert(Points.begin(), path1.begin(), path1.end());
				Points.pop_back();
				Points.insert(Points.end(), path2.begin(), path2.end());
				return;
			}
		}


		int split_dis = -1;
		int split_index = -1;
		double Mcur = 0;
		for (int i = 3; i < Points.size() - 3; i++)
		{
			double d1 = 120;//45 degrees
			double d2 = 100;//90 degrees	
							//double d3 = sin(PI_ / 3.0);//120 degrees
			if (Points[i].curvature > d2)
			{
				if (Points[i].curvature > Mcur)
				{
					Mcur = Points[i].curvature;
					split_dis = 5; split_index = i;
				}
			}
			//else if (Points[i].curvature > d2 &&  Points[i].curvature <d1)
			//{
			//	split_dis = 5; split_index = i;
			//}
			//else if (Points[i].curvature >= d2 &&  Points[i].curvature < d3)
			//{
			//	split_dis = 6; split_index = i;
			//}
		}

		if (split_index != -1)
		{
			if (Points.size() > 2 * split_dis + 1)
			{
				vector<PointTriple> Points_new;
				vector<PointTriple>::iterator it = Points.begin();

				if (split_index > split_dis)
				{
					vector<PointTriple> path1(Points.begin(), it + split_index - split_dis + 1); //new path l1 from 0 to index
					simplifyWithRDP(path1, epsilon);
					Points_new.insert(Points_new.begin(), path1.begin(), path1.end());
					Points_new.pop_back();

					vector<PointTriple> path2(it + split_index - split_dis, it + split_index + 1); // new path l2 from index to last
					simplifyWithRDP(path2, epsilon);
					Points_new.insert(Points_new.end(), path2.begin(), path2.end());
					Points_new.pop_back();
				}
				else
				{
					vector<PointTriple> path1(Points.begin(), it + split_index + 1); //new path l1 from 0 to index
					simplifyWithRDP(path1, epsilon);
					Points_new.insert(Points_new.begin(), path1.begin(), path1.end());
					Points_new.pop_back();
				}

				if (split_index < Points.size() - split_dis)
				{
					vector<PointTriple> path3(it + split_index, it + split_index + split_dis + 1); //new path l1 from 0 to index
					simplifyWithRDP(path3, epsilon);
					Points_new.insert(Points_new.end(), path3.begin(), path3.end());
					Points_new.pop_back();

					vector<PointTriple> path4(it + split_index + split_dis, Points.end()); // new path l2 from index to last
					simplifyWithRDP(path4, epsilon);
					Points_new.insert(Points_new.end(), path4.begin(), path4.end());
				}
				else
				{
					vector<PointTriple> path3(it + split_index, Points.end()); //new path l1 from 0 to index
					simplifyWithRDP(path3, epsilon);
					Points_new.insert(Points_new.end(), path3.begin(), path3.end());
				}

				Points = Points_new;
				return;
			}
			else
			{
				vector<PointTriple>::iterator it = Points.begin();
				vector<PointTriple> path1(Points.begin(), it + split_index + 1); //new path l1 from 0 to index
				vector<PointTriple> path2(it + split_index, Points.end()); // new path l2 from index to last

				simplifyWithRDP(path1, epsilon);
				simplifyWithRDP(path2, epsilon);

				//simplify path1 and path2 together
				Points.clear();
				Points.insert(Points.begin(), path1.begin(), path1.end());
				Points.pop_back();
				Points.insert(Points.end(), path2.begin(), path2.end());
				return;
			}
		}
	}


	std::pair<int, double> maxDistance = findMaximumDistance(Points, epsilon);

	if (do_consider_curvature)
	{
		//re-estimate curvature
		for (int j = 0; j < Points.size(); j++)
		{
			if (Points.size() < 5)
			{
				Points[j].curvature = 0.1;
				continue;
			}
			if (j<3 || j> Points.size() - 4)
			{
				Points[j].curvature = 0.1;
				continue;
			}
			Point_2 p0 = Points[j - 3].image_newp;
			Point_2 p1 = Points[j].image_newp;
			Point_2 p2 = Points[j + 3].image_newp;
			double abc = sqrt((p0 - p1).squared_length()*(p0 - p2).squared_length()*(p1 - p2).squared_length());
			double discrete_cur = 1.0 / (abc / 4.0 / abs(area(p0, p1, p2)));
			Points[j].curvature = discrete_cur * Points[j].constant_size;
		}
	}

	if (maxDistance.second >= epsilon)
	{
		int index = maxDistance.first;
		vector<PointTriple>::iterator it = Points.begin();
		vector<PointTriple> path1(Points.begin(), it + index + 1); //new path l1 from 0 to index
		vector<PointTriple> path2(it + index, Points.end()); // new path l2 from index to last

		simplifyWithRDP(path1, epsilon);
		simplifyWithRDP(path2, epsilon);

		//simplify path1 and path2 together
		Points.clear();
		Points.insert(Points.begin(), path1.begin(), path1.end());
		Points.pop_back();
		Points.insert(Points.end(), path2.begin(), path2.end());
		return;
	}

	//if (sqrt((Points[0].imagedomain-Points[Points.size() - 1].imagedomain).squared_length()) < 0.01)

}
const std::pair<int, double> findMaximumDistance(const std::vector<Point2withIndex>& Points) {
	Point_2 firstpoint = Points[0].point2;
	Point_2 lastpoint = Points[Points.size() - 1].point2;
	int index = 0;  //index to be returned
	double Mdist = -1; //the Maximum distance to be returned

					   //distance calculation
	Vector_2 p = lastpoint - firstpoint;
	for (unsigned i = 1; i < Points.size() - 1; i++) { //traverse through second point to second last point
		Vector_2 pp = Points[i].point2 - firstpoint;
		Triangle_2 tri(firstpoint, lastpoint, Points[i].point2);
		double Dist = 2.0* abs(tri.area())/sqrt(p.squared_length()); //formula for point-to-line distance
		if (Dist > Mdist) {
			Mdist = Dist;
			index = i;
		}
	}
	return std::make_pair(index, Mdist);
}

std::vector<Point2withIndex> simplifyWithRDP(std::vector<Point2withIndex>& Points, double epsilon) {
	if (Points.size() < 3) {  //base case 1
		return Points;
	}
	std::pair<int, double> maxDistance = findMaximumDistance(Points);
	if (maxDistance.second >= epsilon) {
		int index = maxDistance.first;
		std::vector<Point2withIndex>::iterator it = Points.begin();
		std::vector<Point2withIndex> path1(Points.begin(), it + index + 1); //new path l1 from 0 to index
		std::vector<Point2withIndex> path2(it + index, Points.end()); // new path l2 from index to last

		std::vector<Point2withIndex> r1 = simplifyWithRDP(path1, epsilon);
		std::vector<Point2withIndex> r2 = simplifyWithRDP(path2, epsilon);

		//make simplified path1 and path2 together
		std::vector<Point2withIndex> rs(r1);
		rs.pop_back();
		rs.insert(rs.end(), r2.begin(), r2.end());
		return rs;
	}
	else { //base case 2, all points between are to be removed.
		std::vector<Point2withIndex> r(1, Points[0]);
		r.push_back(Points[Points.size() - 1]);
		return r;
	}
}

void search_neighboring_pixels(pair<int, int> center, vector<pair<int, int>> &region, int nfield,
	vector<pair<int, int>> select_ps)
{
	vector<pair<int, int>>::iterator bexist = find(region.begin(), region.end(), center);
	if (bexist == region.end())
		region.push_back(center);

	vector<pair<int, int>> new_centers;
	for (int i = center.first- nfield;i<center.first+ nfield+1;i++)
	{
		for (int j = center.second- nfield;j<center.second+ nfield+1;j++)
		{
			if (i == center.first && j == center.second)
			{
				continue;
			}
			pair<int, int> coord_now(i,j);
			vector<pair<int, int>>::iterator res = find(select_ps.begin(),select_ps.end(),coord_now);
			if (res != select_ps.end())
			{
				vector<pair<int, int>>::iterator bexist = find(region.begin(), region.end(), coord_now);
				if (bexist == region.end())
				{
					region.push_back(coord_now);
					new_centers.push_back(coord_now);
				}
			}
		}
	}

	//if (new_centers.size() < 2)
	//{
	//	return;
	//}

	for (int i = 0;i<new_centers.size();i++)
	{
		search_neighboring_pixels(new_centers[i],region,nfield, select_ps);
	}
}

void screen_controlps_outside_feature_polygon(CMInfo* cm_, vector<int> current_index_, 
	vector<vector<CMVertexInfo>>control_points_seqs, vector<vector<PointTriple>> collinear_knots,
	Mesh *knot_mesh, Mesh *fitted_mesh, OneStepInfo *fittingInf, QString output_file)
{
	int feature_line_index = -1;
	int index = 0;
	for (int i = 0; i < cm_->vertices.size(); i++)
	{
		if (cm_->vertices[i].flag == 1 && cm_->vertices[i].do_display)
		{
			if (cm_->vertices[i].is_on_feature)
			{
				vector<int>::iterator res1 = std::find(current_index_.begin(), current_index_.end(), index);
				if (res1 != current_index_.end())
				{
					feature_line_index = cm_->vertices[i].feacp_type;
					break;
				}
				index++;

				vector<int>::iterator res2 = std::find(current_index_.begin(), current_index_.end(), index);
				if (res2 != current_index_.end())
				{
					feature_line_index = cm_->vertices[i].feacp_type;
					break;
				}
				index++;
			}
			else
			{
				index++;
			}
		}
	}

	if (feature_line_index != -1)
	{
		int num_cp_outside = 0;
		int num_cp_onfea = 0;
		Polygon_2 polygon_;
		for (int i = 0; i < control_points_seqs[feature_line_index].size(); i++)
		{
			Point_2 p = control_points_seqs[feature_line_index][i].center;
			polygon_.push_back(p);
		}
		if (polygon_.is_simple())
		{
			//for display
			for (int i = 0; i < cm_->vertices.size(); i++)
			{
				if (cm_->vertices[i].flag == 1 && !cm_->vertices[i].is_on_feature)
				{
					if (polygon_.bounded_side(cm_->vertices[i].center) == CGAL::ON_UNBOUNDED_SIDE)
					{
						cm_->vertices[i].do_display = false;
						num_cp_outside++;
					}
				}
				if (cm_->vertices[i].is_on_feature)
				{
					num_cp_onfea++;
				}
			}			
		}

		Polygon_2 polygon_domain;
		for (int i = 0; i < collinear_knots[feature_line_index].size()-1; i++)
		{
			Point_2 p = collinear_knots[feature_line_index][i].paradomain;
			polygon_domain.push_back(p);
		}
		if (polygon_domain.is_simple())
		{
			//for knot
			int number_knots = 0,number_knotface = 0;//outside
			for (auto it = knot_mesh->vertices_begin(); it != knot_mesh->vertices_end(); it++)
			{
				it->vertex_flag().bshow = true;
				if (!it->is_feature())
				{
					if (polygon_.bounded_side(Point_2(it->point().x(), it->point().y())) == CGAL::ON_UNBOUNDED_SIDE)
					{
						number_knots++;
						it->vertex_flag().bshow = false;
					}
				}
			}
			//for knot face
			for (auto it = knot_mesh->facets_begin(); it != knot_mesh->facets_end(); it++)
			{
				bool is_out_polygon = false;
				Mesh::Halfedge_around_facet_circulator hc = it->facet_begin(), hs = hc;
				do
				{
					if (!hc->vertex()->is_feature())
					{
						if (!hc->vertex()->vertex_flag().bshow)
						{
							is_out_polygon = true;
							break;
						}
					}
				} while (++hc != hs);
				if (is_out_polygon)
				{
					number_knotface++;
				}
			}
			fittingInf->num_controlp = fittingInf->num_controlp - num_cp_outside - num_cp_onfea;
			fittingInf->num_knots = knot_mesh->size_of_vertices() - number_knots;
			fittingInf->num_knots_face = knot_mesh->size_of_facets() - number_knotface;
			fittingInf->size_representation =
				(fittingInf->num_controlp*20.0 +
					fittingInf->num_knots*8.0 + fittingInf->num_knots_face*6.0) / 1000.0;

			//for data
			double err_average_nfeature = 0.0;
			double mes = 0.0;
			double rmes = 0.0;
			for (auto vit = fitted_mesh->vertices_begin();vit != fitted_mesh->vertices_end();vit++)
			{
				vit->vertex_flag().bshow = true;
				if (!vit->is_feature())
				{
					if (polygon_.bounded_side(Point_2(vit->get_domain().x(), vit->get_domain().y())) == CGAL::ON_UNBOUNDED_SIDE)
					{
						vit->vertex_flag().bshow = false;
					}
				}
				if (vit->vertex_flag().bshow)
				{
					double error_255 = vit->vertex_error()*255.0;
					
					if (vit->tag() == -1)
					{
						rmes += pow(vit->vertex_error(), 2);
						mes += pow(error_255, 2);						
						err_average_nfeature += error_255;
					}
				}				
			}

			fittingInf->mean_err_nofeature = err_average_nfeature / double(fitted_mesh->size_of_vertices());
			fittingInf->MSE = mes / double(fitted_mesh->size_of_vertices());
			fittingInf->RMSE = rmes / double(fitted_mesh->size_of_vertices());
			fittingInf->psnr = 20.0*std::log10(255.0 / sqrt(fittingInf->MSE));

			std::cout << "implement information as follow: \n";
			std::cout << "number of control points  : " << fittingInf->num_controlp << "\n";
			std::cout << "number of knots           : " << fittingInf->num_knots << "\n";
			std::cout << "number of knot-face       : " << fittingInf->num_knots_face << "\n";
			std::cout << "size of representation    : " << fittingInf->size_representation << "\n";
			std::cout << "PSNR                      : " << fittingInf->psnr << "\n";
			std::cout << "mean errors               : " << fittingInf->mean_err_nofeature << "\n";
			std::cout << "RMSE		                : " << fittingInf->RMSE << "\n";
		}
	}

	CMInfo selectCm;
	QString name = output_file;
	name.append("/control_mesh_select.obj");
	extract_selected_control_mesh(*cm_, selectCm);
	control_mesh_output(selectCm, name);
}

void screen_controlps_outside_feature_polygon(CMInfo* cm_, vector<int> current_index_, vector<int> &selected_index_,
	vector<vector<CMVertexInfo>>control_points_seqs, vector<vector<PointTriple>> collinear_knots,
	Mesh *knot_mesh, Mesh *fitted_mesh, OneStepInfo *fittingInf, QString output_file)
{
	int feature_line_index = -1;
	int index = 0;
	for (int i = 0; i < cm_->vertices.size(); i++)
	{
		if (cm_->vertices[i].flag == 1 && cm_->vertices[i].do_display)
		{
			if (cm_->vertices[i].is_on_feature)
			{
				vector<int>::iterator res1 = std::find(current_index_.begin(), current_index_.end(), index);
				if (res1 != current_index_.end())
				{
					feature_line_index = cm_->vertices[i].feacp_type;
					break;
				}
				index++;

				vector<int>::iterator res2 = std::find(current_index_.begin(), current_index_.end(), index);
				if (res2 != current_index_.end())
				{
					feature_line_index = cm_->vertices[i].feacp_type;
					break;
				}
				index++;
			}
			else
			{
				index++;
			}
		}
	}

	if (feature_line_index != -1)
	{
		int num_cp_outside = 0;
		int num_cp_onfea = 0;
		Polygon_2 polygon_;
		for (int i = 0; i < control_points_seqs[feature_line_index].size(); i++)
		{
			Point_2 p = control_points_seqs[feature_line_index][i].center;
			polygon_.push_back(p);
		}
		if (polygon_.is_simple())
		{
			//for display
			int it = 0;
			for (int i = 0; i < cm_->vertices.size(); i++)
			{
				if (cm_->vertices[i].flag == 1)
				{
					if (cm_->vertices[i].is_on_feature)
					{
						double gray1 = cm_->vertices[i].control_point_pair.first.r*0.2989 +
							cm_->vertices[i].control_point_pair.first.g*0.5870 +
							cm_->vertices[i].control_point_pair.first.b*0.1140;
						double gray2 = cm_->vertices[i].control_point_pair.second.r*0.2989 +
							cm_->vertices[i].control_point_pair.second.g*0.5870 +
							cm_->vertices[i].control_point_pair.second.b*0.1140;
						if (cm_->vertices[i].feacp_type == feature_line_index)
						{
							if (cm_->vertices[i].control_point_pair.first.r >
								cm_->vertices[i].control_point_pair.second.r)
							{
								selected_index_.push_back(it + 1);
							}
							else
							{
								selected_index_.push_back(it);
							}
						}						
						it += 2;
					}
					else
					{
						if (polygon_.bounded_side(cm_->vertices[i].center) == CGAL::ON_UNBOUNDED_SIDE)
						{
							cm_->vertices[i].do_display = false;
							num_cp_outside++;
						}
						else
						{
							selected_index_.push_back(it);
						}
						it++;
					}
				}

				if (cm_->vertices[i].is_on_feature)
				{
					num_cp_onfea++;
				}
			}
		}

		Polygon_2 polygon_domain;
		for (int i = 0; i < collinear_knots[feature_line_index].size() - 1; i++)
		{
			Point_2 p = collinear_knots[feature_line_index][i].paradomain;
			polygon_domain.push_back(p);
		}
		if (polygon_domain.is_simple())
		{
			////for cps
			//for (int i = 0;i<cm_.vertices.size();i++)
			//{
			//	int index_ = cm_->vertices[i].control_point_pair.first.basis_index;
			//	bSplineBasis->basisConfigs[index_]
			//}

			//for knot
			int number_knots = 0, number_knotface = 0;//outside
			for (auto it = knot_mesh->vertices_begin(); it != knot_mesh->vertices_end(); it++)
			{
				it->vertex_flag().bshow = true;
				if (!it->is_feature())
				{
					if (polygon_.bounded_side(Point_2(it->point().x(), it->point().y())) == CGAL::ON_UNBOUNDED_SIDE)
					{
						number_knots++;
						it->vertex_flag().bshow = false;
					}
				}
			}
			//for knot face
			for (auto it = knot_mesh->facets_begin(); it != knot_mesh->facets_end(); it++)
			{
				bool is_out_polygon = false;
				Mesh::Halfedge_around_facet_circulator hc = it->facet_begin(), hs = hc;
				do
				{
					if (!hc->vertex()->is_feature())
					{
						if (!hc->vertex()->vertex_flag().bshow)
						{
							is_out_polygon = true;
							break;
						}
					}
				} while (++hc != hs);
				if (is_out_polygon)
				{
					number_knotface++;
				}
			}
			fittingInf->num_controlp = fittingInf->num_controlp - num_cp_outside - num_cp_onfea/2;
			fittingInf->num_knots = knot_mesh->size_of_vertices() - number_knots;
			fittingInf->num_knots_face = knot_mesh->size_of_facets() - number_knotface;
			fittingInf->size_representation =
				(fittingInf->num_controlp*20.0 +
					fittingInf->num_knots*8.0 + fittingInf->num_knots_face*6.0) / 1000.0;

			//for data
			double err_average_nfeature = 0.0;
			double mes = 0.0;
			double rmse = 0.0;
			for (auto vit = fitted_mesh->vertices_begin(); vit != fitted_mesh->vertices_end(); vit++)
			{
				vit->vertex_flag().bshow = true;
				if (!vit->is_feature())
				{
					if (polygon_.bounded_side(Point_2(vit->get_domain().x(), vit->get_domain().y())) == CGAL::ON_UNBOUNDED_SIDE)
					{
						vit->vertex_flag().bshow = false;
					}
				}
				if (vit->vertex_flag().bshow)
				{
					double error_255 = vit->vertex_error()*255.0;
					
					if (vit->tag() == -1)
					{
						rmse += pow(vit->vertex_error(), 2);
						mes += pow(error_255, 2);	
						err_average_nfeature += error_255;
					}
				}
			}

			fittingInf->mean_err_nofeature = err_average_nfeature / double(fitted_mesh->size_of_vertices());
			fittingInf->MSE = mes / double(fitted_mesh->size_of_vertices());
			fittingInf->RMSE = rmse / double(fitted_mesh->size_of_vertices());
			fittingInf->psnr = 20.0*std::log10(255.0 / sqrt(fittingInf->MSE));

			std::cout << "implement information as follow: \n";
			std::cout << "number of control points  : " << fittingInf->num_controlp << "\n";
			std::cout << "number of knots           : " << fittingInf->num_knots << "\n";
			std::cout << "number of knot-face       : " << fittingInf->num_knots_face << "\n";
			std::cout << "size of representation    : " << fittingInf->size_representation << "\n";
			std::cout << "PSNR                      : " << fittingInf->psnr << "\n";
			std::cout << "mean errors               : " << fittingInf->mean_err_nofeature << "\n";
			std::cout << "RMSE		                : " << fittingInf->RMSE << "\n";
		}
	}

	CMInfo selectCm;
	QString name = output_file;
	name.append("/control_mesh_select.obj");
	extract_selected_control_mesh(*cm_, selectCm);
	control_mesh_output(selectCm, name);
}

bool find_onering_pixels(cv::Point center, vector<cv::Point> ring, vector<cv::Point> & out_)
{
	out_.clear();
	for (int i = 0;i<ring.size();i++)
	{
		if (ring[i].x == center.x && ring[i].y == center.y)
		{
			continue;
		}
		if (abs(ring[i].x-center.x)<2 && abs(ring[i].y - center.y)<2)
		{
			out_.push_back(ring[i]);
		}
	}
	return (out_.size() == 2);
}

void restore_controlps_outside_feature_polygon(CMInfo* cm_)
{
	for (int i = 0; i < cm_->vertices.size(); i++)
	{
		if (cm_->vertices[i].flag == 1)
		{
			cm_->vertices[i].do_display = true;
		}
	}
}

bool is_singular(Link &link)
{
	if (link.sortEdges.empty())
		return true;

	list<unsigned>::iterator iter = link.sortEdges.begin();
	list<unsigned>::iterator last = link.sortEdges.end();
	last--;
	if (link.sortEdges.size()==3 && *iter==*last)
	{
		return true;
	}
	return false;
}

vector<Link>::iterator find_links(vector<Link> &allLinks, vector<unsigned> &interior)
{
	vector<Link>::iterator it = allLinks.begin(), end = allLinks.end();
	for (; it!=end; it++)
	{
		vector<unsigned int> scr_, temp_;
		scr_ = interior; temp_ = it->interior;
		sort(scr_.begin(), scr_.end());
		sort(temp_.begin(), temp_.end());
		if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
		{
			return it;
		}
	}
	return it;
}

void points_on_polygon(Mesh *mesh, ANNkd_tree *kdTree, vector<Point_2> &pts, FaceInfo &info)
{
	vector<Vertex_iterator> vIndices;
	points_in_bbox(mesh, kdTree, pts, vIndices);
	for (int i=0; i<vIndices.size(); i++)
	{
		if (!vIndices[i]->vertex_flag().bStatus)
		{
			int ret = CGAL::bounded_side_2(pts.begin(), pts.end(), vIndices[i]->get_domain(), K());
			if (ret!=CGAL::ON_UNBOUNDED_SIDE)
			{
				info.incInfos.push_back(vIndices[i]);
				vIndices[i]->vertex_flag().bStatus = 1;
			}
		}
	}
}

void points_on_polygon(Mesh *mesh, vector<Point_2> &pts, FaceInfo &info)
{
	vector<Vertex_iterator> vIndices;
	//points_in_bbox(mesh, kdTree, pts, vIndices);
	//a slow method
	double xmin = 1e100, ymin = 1e100;
	double xmax = -1e100, ymax = -1e100;
	for (int i = 0; i < pts.size(); i++)
	{
		if (xmin > pts[i].x())
			xmin = pts[i].x();

		if (xmax < pts[i].x())
			xmax = pts[i].x();

		if (ymin > pts[i].y())
			ymin = pts[i].y();

		if (ymax < pts[i].y())
			ymax = pts[i].y();
	}
	vector<double> x_v;
	vector<double> y_v;
	for (Mesh::Vertex_iterator vit = mesh->vertices_begin(); vit != mesh->vertices_end(); vit++)
	{
		if (vit->get_domain().x() >= (xmin - 1e-8) && vit->get_domain().x() <= (xmax + 1e-8)
			&& vit->get_domain().y() >= (ymin - 1e-8) && vit->get_domain().y() <= (ymax + 1e-8))
		{
			vIndices.push_back(vit);
			x_v.push_back(vit->get_domain().x());
			y_v.push_back(vit->get_domain().y());
		}
	}
	for (int i = 0; i < vIndices.size(); i++)
	{
		if (!vIndices[i]->vertex_flag().bStatus)
		{
			int ret = CGAL::bounded_side_2(pts.begin(), pts.end(), vIndices[i]->get_domain(), K());
			if (ret != CGAL::ON_UNBOUNDED_SIDE || is_on_polygon_convex_bound(vIndices[i]->get_domain(), pts))
			{
				info.incInfos.push_back(vIndices[i]);
				vIndices[i]->vertex_flag().bStatus = 1;
			}
		}
	}
}

void points_on_polygon(vector<Point_2> &pts, FaceInfo &dstInfo, FaceInfo &srcInfo)
{
	vector<Vertex_iterator>::iterator it = srcInfo.incInfos.begin();
	vector<Vertex_iterator>::iterator end = srcInfo.incInfos.end();
	int ret;
	for (; it!=end; it++)
	{
		ret = CGAL::bounded_side_2(pts.begin(), pts.end(), (*it)->get_domain(), K());
		if(ret!=CGAL::ON_UNBOUNDED_SIDE)
			dstInfo.incInfos.push_back((*it));
	}
}

void points_on_polygon(vector<Point_2> &fPts, FaceInfo &fInfo, vector<Point_2> &gPts, FaceInfo &gInfo, FaceInfo &srcInfo)
{
	vector<Vertex_iterator>::iterator it = srcInfo.incInfos.begin();
	vector<Vertex_iterator>::iterator end = srcInfo.incInfos.end();
	int ret;
	for (; it!=end; it++)
	{
		ret = CGAL::bounded_side_2(fPts.begin(), fPts.end(), (*it)->get_domain(), K());
		if(ret==CGAL::ON_BOUNDED_SIDE)
			fInfo.incInfos.push_back((*it));
		else if(ret==CGAL::ON_BOUNDARY)
		{
			fInfo.incInfos.push_back((*it));
			ret = CGAL::bounded_side_2(gPts.begin(), gPts.end(), (*it)->get_domain(), K());
			if(ret!=CGAL::ON_UNBOUNDED_SIDE)
				gInfo.incInfos.push_back((*it));
		}
		else
			gInfo.incInfos.push_back((*it));
	}
}

void points_on_polygon(Mesh *mesh, ANNkd_tree *kdTree, vector<Point_2> &pts, vector<Vertex_iterator> &indices)
{
	vector<Vertex_iterator> vIndices;
	points_in_bbox(mesh, kdTree, pts, vIndices);
	for (int i=0; i<vIndices.size(); i++)
	{
		int ret = CGAL::bounded_side_2(pts.begin(), pts.end(), vIndices[i]->get_domain(), K());
		if (ret!=CGAL::ON_UNBOUNDED_SIDE)
			indices.push_back(vIndices[i]);
	}
}

void points_on_polygon(Mesh *mesh, ANNkd_tree *kdTree, vector<Point_2> &pts, vector<SupportInfo> &supportInfo)
{
	vector<Vertex_iterator> vIndices;
	points_in_bbox(mesh, kdTree, pts, vIndices);
	for (int i=0; i<vIndices.size(); i++)
	{
		int ret = CGAL::bounded_side_2(pts.begin(), pts.end(), vIndices[i]->get_domain(), K());
		if (ret!=CGAL::ON_UNBOUNDED_SIDE)
		{
			SupportInfo info;
			info.index = vIndices[i]->vertex_index();
			info.pt = vIndices[i]->get_domain();
			supportInfo.push_back(info);
		}
	}
}

void points_on_polygon(Mesh *mesh, vector<Point_2> &pts, map<unsigned, DomainValue> &supports)
{
	vector<Vertex_iterator> vIndices;
	//points_in_bbox(mesh, kdTree, pts, vIndices);

	//a slow method
	double xmin = 1e100, ymin = 1e100;
	double xmax = -1e100, ymax = -1e100;
	for (int i = 0; i < pts.size(); i++)
	{
		if (xmin > pts[i].x())
			xmin = pts[i].x();

		if (xmax < pts[i].x())
			xmax = pts[i].x();

		if (ymin > pts[i].y())
			ymin = pts[i].y();

		if (ymax < pts[i].y())
			ymax = pts[i].y();
	}
	vector<double> x_v;
	vector<double> y_v;
	for (Mesh::Vertex_iterator vit = mesh->vertices_begin(); vit != mesh->vertices_end(); vit++)
	{
		if (vit->get_domain().x() >= (xmin - 1e-8) && vit->get_domain().x() <= (xmax + 1e-8)
			&& vit->get_domain().y() >= (ymin - 1e-8) && vit->get_domain().y() <= (ymax + 1e-8))
		{
			vIndices.push_back(vit);
			x_v.push_back(vit->get_domain().x());
			y_v.push_back(vit->get_domain().y());
		}
	}

	for (int i = 0; i<vIndices.size(); i++)
	{
		int ret = CGAL::bounded_side_2(pts.begin(), pts.end(), vIndices[i]->get_domain(), K());
		if (ret != CGAL::ON_UNBOUNDED_SIDE || is_on_polygon_convex_bound(vIndices[i]->get_domain(), pts))
		{
			//test
			/*if (abs(vIndices[i]->get_domain().x() - 1) < 1e-8 && vIndices[i]->get_domain().y() > 0.01 && vIndices[i]->get_domain().y() < 0.99)
			{
			double x = vIndices[i]->get_domain().x();
			double y = vIndices[i]->get_domain().y();
			int stop = 1;
			}*/

			/*SupportInfo info;
			info.pt = vIndices[i]->get_domain();
			info.index = vIndices[i]->vertex_index();
			supports.push_back(info);*/
			DomainValue dv;
			dv.pt = vIndices[i]->get_domain();
			supports[vIndices[i]->vertex_index()] = dv;
		}
	}
}

void points_on_polygon(vector<Point_2> &sample_point, vector<Point_2> &pts, map<unsigned, DomainValue> &supports)
{
	supports.clear();
	vector<int> vIndices;

	//a slow method
	double xmin = 1e100, ymin = 1e100;
	double xmax = -1e100, ymax = -1e100;
	for (int i = 0; i < pts.size(); i++)
	{
		if (xmin > pts[i].x())
			xmin = pts[i].x();

		if (xmax < pts[i].x())
			xmax = pts[i].x();

		if (ymin > pts[i].y())
			ymin = pts[i].y();

		if (ymax < pts[i].y())
			ymax = pts[i].y();
	}
	for (int i = 0;i<sample_point.size();i++)
	{
		if (sample_point[i].x() >= (xmin - 1e-8) && sample_point[i].x() <= (xmax + 1e-8)
			&& sample_point[i].y() >= (ymin - 1e-8) && sample_point[i].y() <= (ymax + 1e-8))
		{
			vIndices.push_back(i);
		}
	}

	for (int i = 0; i<vIndices.size(); i++)
	{
		int ret = CGAL::bounded_side_2(pts.begin(), pts.end(),sample_point[vIndices[i]], K());
		if (ret != CGAL::ON_UNBOUNDED_SIDE || is_on_polygon_convex_bound(sample_point[vIndices[i]], pts))
		{
			DomainValue dv;
			dv.pt = sample_point[vIndices[i]];
			supports[vIndices[i]] = dv;
		}
	}
}

void points_on_polygon(vector<SurfaceTangentForG1> &sample_point, vector<Point_2> &pts, map<unsigned, DomainValue> &supports)
{
	supports.clear();
	vector<int> vIndices;

	//a slow method
	double xmin = 1e100, ymin = 1e100;
	double xmax = -1e100, ymax = -1e100;
	for (int i = 0; i < pts.size(); i++)
	{
		if (xmin > pts[i].x())
			xmin = pts[i].x();

		if (xmax < pts[i].x())
			xmax = pts[i].x();

		if (ymin > pts[i].y())
			ymin = pts[i].y();

		if (ymax < pts[i].y())
			ymax = pts[i].y();
	}
	for (int i = 0; i < sample_point.size(); i++)
	{
		if (sample_point[i].domain_pos.x() >= (xmin - 1e-8) && sample_point[i].domain_pos.x() <= (xmax + 1e-8)
			&& sample_point[i].domain_pos.y() >= (ymin - 1e-8) && sample_point[i].domain_pos.y() <= (ymax + 1e-8))
		{
			vIndices.push_back(i);
		}
	}

	for (int i = 0; i < vIndices.size(); i++)
	{
		int ret = CGAL::bounded_side_2(pts.begin(), pts.end(), sample_point[vIndices[i]].domain_pos, K());
		if (ret != CGAL::ON_UNBOUNDED_SIDE || is_on_polygon_convex_bound(sample_point[vIndices[i]].domain_pos, pts))
		{
			DomainValue dv;
			dv.dir1 = sample_point[vIndices[i]].dir1;
			dv.dir2 = sample_point[vIndices[i]].dir2;
			dv.pt = sample_point[vIndices[i]].domain_pos;
			supports[vIndices[i]] = dv;
		}
	}
}

bool is_point_on_segment(Point_2 p, Segment_2 seg_, double precision)
{
	if (p.x() < std::min(seg_.vertex(0).x(),seg_.vertex(1).x())-precision ||
		p.x() > std::max(seg_.vertex(0).x(), seg_.vertex(1).x()) + precision ||
		p.y() < std::min(seg_.vertex(0).y(), seg_.vertex(1).y()) - precision ||
		p.y() > std::max(seg_.vertex(0).y(), seg_.vertex(1).y()) + precision)
	{
		return false;
	}

	if (abs(seg_.vertex(0).x() - seg_.vertex(1).x())<precision)
	{
		if (abs(seg_.vertex(0).x() - p.x()) < precision)
			return true;
		else
			return false;
	}
	else if (abs(seg_.vertex(0).y() - seg_.vertex(1).y())<precision)
	{
		if (abs(seg_.vertex(0).y() - p.y()) < precision)
			return true;
		else
			return false;
	}
	else
	{
		if (abs((p.x() - seg_.vertex(0).x()) / (seg_.vertex(0).x() - seg_.vertex(1).x()) -
			(p.y() - seg_.vertex(0).y()) / (seg_.vertex(0).y() - seg_.vertex(1).y())) < precision)
			return true;
		else
			return false;
	}
}

bool is_on_domain_bound(Point_2 a)
{
	bool on_edge = abs(a.x() - 1) < 1e-8 ||
		abs(a.y() - 1) < 1e-8 ||
		abs(a.y() - 0) < 1e-8 ||
		abs(a.x() - 0) < 1e-8;
	return on_edge ? true : false;
}

int domain_bound_type(Point_2 a)
{
	bool on_edge = abs(a.x() - 1) < 1e-8 ||
		abs(a.y() - 1) < 1e-8 ||
		abs(a.y() - 0) < 1e-8 ||
		abs(a.x() - 0) < 1e-8;

	if (on_edge)
	{
		if (abs(a.x() - 0) < 1e-8)
		{
			return 0;
		}
		else  if (abs(a.x() - 1.0) < 1e-8)
		{
			return 1;
		}
		else  if (abs(a.y() - 0) < 1e-8)
		{
			return 2;
		}
		else  if (abs(a.y() - 1.0) < 1e-8)
		{
			return 3;
		}
	}
	return -1;
}

bool is_on_domain_corner(Point_2 a)
{
	bool on_corner = (abs(a.x() - 0) < 1e-8 && abs(a.y() - 0) < 1e-8) ||
		(abs(a.x() - 0) < 1e-8 && abs(a.y() - 1) < 1e-8) ||
		(abs(a.x() - 1) < 1e-8 && abs(a.y() - 0) < 1e-8) ||
		(abs(a.x() - 1) < 1e-8 && abs(a.y() - 1) < 1e-8);
	return on_corner ? true : false;
}

bool is_on_polygon_convex_bound(Point_2 temp, const vector<Point_2> poly_convex)
{
	//note that the criterion of (==0) is 1e-6
	bool is_onbound = false;
	for (int i = 0; i < poly_convex.size(); i++)
	{
		Point_2 a = poly_convex[i];
		Point_2 b = poly_convex[(i + 1) % poly_convex.size()];

		if (is_point_on_segment(temp, Segment_2(a, b), 1e-6))
		{
			is_onbound = true;
			break;
		}
	}
	return is_onbound;
}

void points_on_polygon(Mesh *mesh, ANNkd_tree *kdTree, vector<Point_2> &pts, vector<SupportInfo> &supportInfo, bool flag)
{
	vector<Vertex_iterator> vIndices;
	points_in_bbox(mesh, kdTree, pts, vIndices);
	if (flag)
	{
		for (int i=0; i<vIndices.size(); i++)
		{
			int ret = CGAL::bounded_side_2(pts.begin(), pts.end(), vIndices[i]->get_domain(), K());
			if (ret!=CGAL::ON_UNBOUNDED_SIDE)
			{
				SupportInfo info;
				info.index = vIndices[i]->vertex_index();
				info.pt = vIndices[i]->get_domain();
				supportInfo.push_back(info);
			}
		}
	}
	else
	{
		for (int i=0; i<vIndices.size(); i++)
		{
			SupportInfo info;
			info.index = vIndices[i]->vertex_index();
			info.pt = vIndices[i]->get_domain();
			supportInfo.push_back(info);
		}
	}
}

void points_in_bbox(Mesh *mesh, ANNkd_tree *kdTree, vector<Point_2> &pts, vector<Vertex_iterator> &vIndices)
{
	double xmin = 1e100, ymin = 1e100;
	double xmax = -1e100, ymax = -1e100;
	for (int i = 0; i < pts.size(); i++)
	{
		if (xmin > pts[i].x())
			xmin = pts[i].x();

		if (xmax < pts[i].x())
			xmax = pts[i].x();

		if (ymin > pts[i].y())
			ymin = pts[i].y();

		if (ymax < pts[i].y())
			ymax = pts[i].y();
	}

	int indices[3];
	double dists[3];

	double queryPt[2];

	queryPt[0] = (xmin + xmax) / 2;
	queryPt[1] = (ymin + ymax) / 2;

	vector<Vertex_iterator> outSides;
	kdTree->annkSearch(						// search
		queryPt,						// query point
		3,								// number of near neighbors
		indices,							// nearest neighbors (returned)
		dists,							// distance (returned)
		0);							// error bound

	Mesh::Vertex_iterator vit;
	for (int i = 0; i < 3; i++)
	{
		vit = mesh->vertices_begin();
		std::advance(vit, indices[i]);
		if (vit->vertex_flag().bSelected)
			continue;

		if (vit->get_domain().x() >= xmin && vit->get_domain().x() <= xmax
			&& vit->get_domain().y() >= ymin && vit->get_domain().y() <= ymax)
			vIndices.push_back(vit);
		else
			outSides.push_back(vit);

		vit->vertex_flag().bSelected = 1;
	}

	int start = 0;
	while (start != vIndices.size())
	{
		Mesh::Halfedge_around_vertex_circulator pHalfEdge = vIndices[start]->vertex_begin();
		Mesh::Halfedge_around_vertex_circulator end = pHalfEdge;
		do
		{
			if (pHalfEdge != NULL && pHalfEdge->opposite() != NULL)
			{
				Mesh::Vertex_handle v_it = pHalfEdge->opposite()->vertex();
				if (!v_it->vertex_flag().bSelected)
				{
					if (v_it->get_domain().x() >= xmin && v_it->get_domain().x() <= xmax
						&& v_it->get_domain().y() >= ymin && v_it->get_domain().y() <= ymax)
						vIndices.push_back(v_it);
					else
						outSides.push_back(v_it);

					v_it->vertex_flag().bSelected = 1;
				}
			}
		} while (++pHalfEdge != end);
		start++;
	}
	for (int i = 0; i < vIndices.size(); i++)
		vIndices[i]->vertex_flag().bSelected = 0;

	for (int i = 0; i < outSides.size(); i++)
		outSides[i]->vertex_flag().bSelected = 0;
}

void compute_mesh_error(Mesh *dstMesh, Mesh *refMesh, OneStepInfo &timeInfo)
{
#if 0
	double max_posx = 0.0;
	double max_posy = 0.0;
	double max_colr = 0.0;
	double max_colg = 0.0;
	double max_colb = 0.0;
	for (auto vit = refMesh->vertices_begin(); vit != refMesh->vertices_end(); vit++)
	{
		double error_ = 0.0;
		int index_ = vit->vertex_index();
		Mesh::Vertex_iterator vnewit = dstMesh->get_vertex_iterator(index_);

		if (abs(vit->point().x()-vnewit->point().x()) > max_posx)
		{
			max_posx = abs(vit->point().x() - vnewit->point().x());
		}
		if (abs(vit->point().y() - vnewit->point().y()) > max_posy)
		{
			max_posy = abs(vit->point().y() - vnewit->point().y());
		}
		if (abs(vit->vertex_pixel_color().x()-vnewit->vertex_pixel_color().x()) > max_colr)
		{
			max_colr = abs(vit->vertex_pixel_color().x() - vnewit->vertex_pixel_color().x());
		}
		if (abs(vit->vertex_pixel_color().y() - vnewit->vertex_pixel_color().y()) > max_colg)
		{
			max_colg = abs(vit->vertex_pixel_color().y() - vnewit->vertex_pixel_color().y());
		}
		if (abs(vit->vertex_pixel_color().z() - vnewit->vertex_pixel_color().z())> max_colb)
		{
			max_colb = abs(vit->vertex_pixel_color().z() - vnewit->vertex_pixel_color().z());
		}
	}
#endif

#if 0
	for (auto vit = refMesh->vertices_begin();vit != refMesh->vertices_end();vit++)
	{
		double error_ = 0.0;
		int index_ = vit->vertex_index();
		Mesh::Vertex_iterator vnewit = dstMesh->get_vertex_iterator(index_);

		//if (!vit->is_feature())
		{
			//error_ += abs((vit->point().x() - vnewit->point().x())/max_posx);
			//error_ += abs((vit->point().y() - vnewit->point().y())/max_posy);
			error_ += pow(vit->vertex_input_color().x() - vnewit->vertex_pixel_color().x(),2);
			error_ += pow(vit->vertex_input_color().y() - vnewit->vertex_pixel_color().y(),2);
			error_ += pow(vit->vertex_input_color().z() - vnewit->vertex_pixel_color().z(),2);
		}
		error_ = sqrt(error_);

		if (error_<1e-10)
		{
			error_ = 0.0;
		}

		vit->vertex_error() = error_;
		vnewit->vertex_error() = error_;
	}
#endif

	double err_average = 0.0;
	double err_average_nfeature = 0.0;
	double num_nofea = 0.0;
	double mes = 0.0;
	double rmse = 0.0;
	double sum_snr = 0.0;
	for (int i = 0;i<refMesh->size_of_vertices();i++)
	{
		double error_ = 0.0;
		Mesh::Vertex_iterator vit = refMesh->get_vertex_iterator(i);
		Mesh::Vertex_iterator vnewit = dstMesh->get_vertex_iterator(i);

		error_ += pow(vit->vertex_pixel_color().x() - vnewit->vertex_pixel_color().x(), 2);
		error_ += pow(vit->vertex_pixel_color().y() - vnewit->vertex_pixel_color().y(), 2);
		error_ += pow(vit->vertex_pixel_color().z() - vnewit->vertex_pixel_color().z(), 2);
		
		error_ = error_ / 3.0;
		if (vit->tag() == -1)
			rmse += error_;

		double square_ = 0.0;
		square_ += pow(vit->vertex_pixel_color().x(), 2);
		square_ += pow(vit->vertex_pixel_color().y(), 2);
		square_ += pow(vit->vertex_pixel_color().z(), 2);
		square_ = square_ / 3.0;
		sum_snr += square_;

		if (error_ < 1e-16)
		{
			error_ = 0.0;
		}

		error_ = sqrt(error_);
		if (error_ < 1e-10)
		{
			error_ = 0.0;
		}

		err_average += 255.0*error_;
		if (vit->tag() == -1)
		{
			mes += pow(255.0*error_,2);		
			err_average_nfeature += 255.0*error_;
			num_nofea += 1.0;
		}
		vit->vertex_error() = error_;
		vnewit->vertex_error() = error_;
	}
	timeInfo.SNR = 10.0*std::log10(sum_snr / rmse);
	timeInfo.mean_error = err_average / double(refMesh->size_of_vertices());
	timeInfo.mean_err_nofeature = err_average_nfeature / double(refMesh->size_of_vertices());
	timeInfo.MSE = mes/ double(refMesh->size_of_vertices());
	timeInfo.RMSE = sqrt(rmse / double(refMesh->size_of_vertices()));
	timeInfo.psnr = 20.0*std::log10(255.0 / sqrt(timeInfo.MSE));

}

void update_density(Mesh *mesh)
{
	Vertex_iterator dstIt = mesh->vertices_begin();
	for (; dstIt!=mesh->vertices_end(); dstIt++)
		dstIt->vertex_density() = std::pow(dstIt->vertex_error(), 2);
}

void compute_convex_hull(Mesh *mesh, vector<Point_2> &convexhull)
{
	Vertex_iterator dstIt = mesh->vertices_begin();
	std::vector<Point_2> pts;
	for (; dstIt!=mesh->vertices_end(); dstIt++)
		pts.push_back(dstIt->get_domain());

	CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(convexhull));
}

void release_basis(SplineBasisConfigs &bConfig)
{
	bConfig.tconfigs.clear();
	bConfig.config.clear();
	bConfig.supports.clear();
}

void release_basis(BSplineBasis &bSplineBasis)
{
	for (int i=0; i<bSplineBasis.basisConfigs.size(); i++)
	{
		bSplineBasis.basisConfigs[i].tconfigs.clear();
		bSplineBasis.basisConfigs[i].config.clear();
		bSplineBasis.basisConfigs[i].supports.clear();
	}
	bSplineBasis.cpInfos.clear();
	bSplineBasis.basisConfigs.clear();
	bSplineBasis.basisMergeInfos.clear();
}

void restore_index(map<unsigned, unsigned> &correspondence, vector<TConfig1> &configsk)
{
	vector<TConfig1>::iterator c_it = configsk.begin(), c_end = configsk.end();
	for (; c_it != c_end; c_it++)
	{
		map<unsigned, unsigned>::iterator cor_it = correspondence.begin(), cor_end = correspondence.end();
		for (; cor_it != cor_end; cor_it++)
		{
			vector<unsigned>::iterator b_it = c_it->begin(), b_end = c_it->end();
			for (; b_it != b_end; b_it++)
			{
				if (*b_it == cor_it->first)
				{
					*b_it = cor_it->second;
				}
			}
		}
	}
	c_it = configsk.begin();
	for (; c_it != c_end; c_it++)
	{
		TConfig1::iterator it = c_it->begin();
		std::advance(it, 3);
		if (it != c_it->end())
		{
			std::sort(it, c_it->end());
		}
	}
}

void restore_index(map<unsigned, unsigned> &correspondence, vector<TConfig2> &configsk)
{
	vector<TConfig2>::iterator c_it = configsk.begin(), c_end = configsk.end();
	for (; c_it != c_end; c_it++)
	{
		map<unsigned, unsigned>::iterator cor_it = correspondence.begin(), cor_end = correspondence.end();
		for (; cor_it != cor_end; cor_it++)
		{
			vector<unsigned>::iterator b_it = c_it->tconfig.begin(), b_end = c_it->tconfig.end();
			for (; b_it != b_end; b_it++)
			{
				if (*b_it == cor_it->first)
				{
					*b_it = cor_it->second;
				}
			}
		}
	}
	c_it = configsk.begin();
	for (; c_it != c_end; c_it++)
	{
		TConfig1::iterator it = c_it->tconfig.begin();
		std::advance(it, 3);
		if (it != c_it->tconfig.end())
		{
			std::sort(it, c_it->tconfig.end());
		}
	}
}

void restore_index(map<unsigned, unsigned> &correspondence, vector<unsigned> &configsk)
{
	for (int i = 0; i < configsk.size(); i++)
	{
		map<unsigned, unsigned>::iterator cor_it = correspondence.begin(), cor_end = correspondence.end();
		for (; cor_it != cor_end; cor_it++)
		{
			if (configsk[i] == cor_it->first)
			{
				configsk[i] = cor_it->second;
			}
		}
	}

	std::sort(configsk.begin(), configsk.end());
}

void combine_near_all(std::vector<unsigned> a, std::vector<std::vector<unsigned>> &combs)
{
	int n = a.size();
	std::vector<unsigned> b = a;
	for (int i = 0; i < n - 1; i++)
		combine(a, n, i + 1, b, i + 1, combs);
}

void combine(std::vector<unsigned> a, int n, int m, std::vector<unsigned> &b, const int M, std::vector<std::vector<unsigned>> &combs)
{
	for (int i = n; i >= m; i--)
	{
		b[m - 1] = i - 1;
		if (m > 1)
			combine(a, i - 1, m - 1, b, M, combs);
		else
		{
			std::vector<unsigned> indices;
			for (int j = M - 1; j >= 0; j--)
				indices.push_back(a[b[j]]);
			combs.push_back(indices);
		}
	}
}

int number_multiple_knot(vector<unsigned> &config, unsigned index)
{
	int num = 0;
	for (int i = 0; i < config.size(); i++)
	{
		if (index == config[i])
			num++;
	}
	return num;
}


void compute_centroid_triangulation(const vector<vector<TConfig2>> &alltconfigs1, const vector<vector<TConfig2>> &alltconfigs2,
	map<unsigned, KnotData> &dataMaps, CMInfo &cmInfo)
{
	int deg = alltconfigs2.size() - 1;
	if (deg > 1)
	{
		double x, y;
		for (unsigned i = 0; i < alltconfigs2[deg - 2].size(); i++)
		{
			vector<unsigned> face;
			for (unsigned j = 0; j < BOUNDARYSIZE; j++)
			{
				vector<unsigned> indices1, indices2;
				indices1.push_back(alltconfigs1[deg - 2][i].tconfig[(j + 1) % BOUNDARYSIZE]);
				indices1.push_back(alltconfigs1[deg - 2][i].tconfig[(j + 2) % BOUNDARYSIZE]);
				indices2.push_back(alltconfigs2[deg - 2][i].tconfig[(j + 1) % BOUNDARYSIZE]);
				indices2.push_back(alltconfigs2[deg - 2][i].tconfig[(j + 2) % BOUNDARYSIZE]);

				for (unsigned k = BOUNDARYSIZE; k < alltconfigs2[deg - 2][i].tconfig.size(); k++)
				{
					indices1.push_back(alltconfigs1[deg - 2][i].tconfig[k]);
					indices2.push_back(alltconfigs2[deg - 2][i].tconfig[k]);
				}
				bool bExist = false;
				for (unsigned k = 0; k < cmInfo.vertices.size(); k++)
				{
					vector<unsigned int> scr_, temp_;
					scr_ = indices2; temp_ = cmInfo.vertices[k].interior;
					sort(scr_.begin(), scr_.end());
					sort(temp_.begin(), temp_.end());
					if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
					{
						face.push_back(k);
						bExist = true;
						break;
					}
				}
				if (!bExist)
				{
					face.push_back(cmInfo.vertices.size());
					x = 0;
					y = 0;
					for (unsigned k = 0; k < indices1.size(); k++)
					{
						x += dataMaps[indices1[k]].pt.x();
						y += dataMaps[indices1[k]].pt.y();
					}
					x /= indices1.size();
					y /= indices1.size();
					CMVertexInfo vertexInfo;
					vertexInfo.interior = indices2;
					vertexInfo.center = Point_2(x, y);
					vertexInfo.flag = -1;
					cmInfo.vertices.push_back(vertexInfo);
				}
			}
			CMFaceInfo faceInfo;
			faceInfo.face = face;
			faceInfo.flag = 2;
			cmInfo.faces.push_back(faceInfo);
		}


		for (unsigned i = 0; i < alltconfigs2[deg - 1].size(); i++)
		{
			vector<unsigned> face;
			for (unsigned j = 0; j < BOUNDARYSIZE; j++)
			{
				vector<unsigned> indices1;
				vector<unsigned> indices2;
				indices1.push_back(alltconfigs2[deg - 1][i].tconfig[j]);
				indices2.push_back(alltconfigs2[deg - 1][i].tconfig[j]);

				for (unsigned k = BOUNDARYSIZE; k < alltconfigs2[deg - 1][i].tconfig.size(); k++)
				{
					indices1.push_back(alltconfigs2[deg - 1][i].tconfig[k]);
					indices2.push_back(alltconfigs2[deg - 1][i].tconfig[k]);
				}
				bool bExist = false;
				for (unsigned k = 0; k < cmInfo.vertices.size(); k++)
				{
					vector<unsigned int> scr_, temp_;
					scr_ = indices2; temp_ = cmInfo.vertices[k].interior;
					sort(scr_.begin(), scr_.end());
					sort(temp_.begin(), temp_.end());
					if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
					{
						face.push_back(k);
						bExist = true;
						break;
					}
				}
				if (!bExist)
				{
					face.push_back(cmInfo.vertices.size());
					x = 0;
					y = 0;
					for (unsigned k = 0; k < indices1.size(); k++)
					{
						x += dataMaps[indices1[k]].pt.x();
						y += dataMaps[indices1[k]].pt.y();
					}
					x /= indices1.size();
					y /= indices1.size();
					CMVertexInfo vertexInfo;
					vertexInfo.interior = indices2;
					vertexInfo.center = Point_2(x, y);
					vertexInfo.flag = -1;
					cmInfo.vertices.push_back(vertexInfo);
				}
			}
			CMFaceInfo faceInfo;
			faceInfo.face = face;
			faceInfo.flag = 1;
			cmInfo.faces.push_back(faceInfo);
		}

		for (unsigned i = 0; i < cmInfo.faces.size(); i++)
		{
			for (unsigned j = 0; j < BOUNDARYSIZE; j++)
			{
				CMEdgeInfo edgeInfo(cmInfo.faces[i].face[j], cmInfo.faces[i].face[(j + 1) % BOUNDARYSIZE]);
				cmInfo.edges.insert(edgeInfo);
			}
		}
	}
}

int generate_control_mesh(BSplineBasis &bSplineBasis, CMInfo &cmInfo, 
	vector<pair<int, int>> split_edgebasis_pair,double translation_length)
{
	//real vertices
	int realCpNum = 0;
	for (unsigned i=0; i<bSplineBasis.basisMergeInfos.size(); i++)
	{
		if (bSplineBasis.basisMergeInfos[i].index!=-1)
		{
			bool is_onfeature = false;
			break;
		}
	}

	//auxiliary vertices
	int nProcessed = realCpNum;
	while (nProcessed<cmInfo.vertices.size())
	{
		for (unsigned k=0; k<cmInfo.vertices.size(); k++)
		{
			if (cmInfo.vertices[k].flag==-1)
			{
				vector<unsigned> neighbors;
				for(set<CMEdgeInfo>::iterator it=cmInfo.edges.begin(); it!=cmInfo.edges.end(); it++)
				{
					if (it->first()==k && cmInfo.vertices[it->second()].flag!=-1)
						neighbors.push_back(it->second());
					else if (it->second()==k && cmInfo.vertices[it->first()].flag!=-1)
						neighbors.push_back(it->first());
				}

				if (!neighbors.empty())
				{
					double x=0, y=0, r=0,g = 0,b = 0;
					for (unsigned i=0; i<neighbors.size(); i++)
					{
						x += cmInfo.vertices[neighbors[i]].vertex.x;
						y += cmInfo.vertices[neighbors[i]].vertex.y;
						r += cmInfo.vertices[neighbors[i]].vertex.r;
						g += cmInfo.vertices[neighbors[i]].vertex.g;
						b += cmInfo.vertices[neighbors[i]].vertex.b;
					}
					RGBPoint p = { x / neighbors.size(), y / neighbors.size(), r / neighbors.size(), g / neighbors.size(), b / neighbors.size() };
					cmInfo.vertices[k].vertex = p;
					cmInfo.vertices[k].flag = 0;
					nProcessed++;
				}
			}
		}
	}
#if 0
	for (unsigned k=0; k<cmInfo.vertices.size(); k++)
	{
		if (cmInfo.vertices[k].flag==0)
		{
			vector<unsigned> neighbors;
			for(set<CMEdgeInfo>::iterator it=cmInfo.edges.begin(); it!=cmInfo.edges.end(); it++)
			{
				if (it->first()==k && cmInfo.vertices[it->second()].flag==1)
					neighbors.push_back(it->second());
				else if (it->second()==k && cmInfo.vertices[it->first()].flag==1)
					neighbors.push_back(it->first());
			}

			if (neighbors.size()<2)
			{
				std::cout << __FUNCTION__ << ": " << neighbors.size() << " ";
				std::cout << __FUNCTION__ << ": " << ": it is a problem" << std::endl;

				vector<NearInfo> nearInfos;
				for (unsigned i=0; i<cmInfo.vertices.size(); i++)
				{
					if (i==k || cmInfo.vertices[i].flag==0)
						continue;

					NearInfo nearInfo = {i, (cmInfo.vertices[i].center-cmInfo.vertices[k].center).squared_length()};
					nearInfos.push_back(nearInfo);
				}

				std::sort(nearInfos.begin(), nearInfos.end(), compare_near_info);

				unsigned n = std::min(nearInfos.size(), (unsigned)3);

#if 0
				MatrixXd A(n, 3);
				for (unsigned i=0; i<n; i++)
				{
					A(i, 0) = cmInfo.vertices[nearInfos[i].index].center.x();
					A(i, 1) = cmInfo.vertices[nearInfos[i].index].center.y();
					A(i, 2) = 1;
				}
				MatrixXd b(n, 3);
				for (unsigned i=0; i<n; i++)
				{
					b(i, 0) = cmInfo.vertices[nearInfos[i].index].vertex.x();
					b(i, 1) = cmInfo.vertices[nearInfos[i].index].vertex.y();
					b(i, 2) = cmInfo.vertices[nearInfos[i].index].vertex.z();
				}

				MatrixXd coeffs = (A.transpose()*A).inverse()*(A.transpose()*b);

				double x = coeffs(0, 0)*cmInfo.vertices[k].center.x() + coeffs(1, 0)*cmInfo.vertices[k].center.y() + coeffs(2, 0);
				double y = coeffs(0, 1)*cmInfo.vertices[k].center.x() + coeffs(1, 1)*cmInfo.vertices[k].center.y() + coeffs(2, 1);
				double z = coeffs(0, 2)*cmInfo.vertices[k].center.x() + coeffs(1, 2)*cmInfo.vertices[k].center.y() + coeffs(2, 2);

				cmInfo.vertices[k].vertex = Point_3(x, y, z);
#else
				double x = 0, y = 0, z = 0;
				for (unsigned i=0; i<n; i++)
				{
					x += cmInfo.vertices[nearInfos[i].index].vertex.x();
					y += cmInfo.vertices[nearInfos[i].index].vertex.y();
					z += cmInfo.vertices[nearInfos[i].index].vertex.z();
				}

				cmInfo.vertices[k].vertex = Point_3(x/n, y/n, z/n);
#endif
			}
			else
			{
#if 0
				unsigned n = neighbors.size();
				MatrixXd A(n, 3);
				for (unsigned i=0; i<n; i++)
				{
					A(i, 0) = cmInfo.vertices[neighbors[i]].center.x();
					A(i, 1) = cmInfo.vertices[neighbors[i]].center.y();
					A(i, 2) = 1;
				}
				MatrixXd b(n, 3);
				for (unsigned i=0; i<n; i++)
				{
					b(i, 0) = cmInfo.vertices[neighbors[i]].vertex.x();
					b(i, 1) = cmInfo.vertices[neighbors[i]].vertex.y();
					b(i, 2) = cmInfo.vertices[neighbors[i]].vertex.z();
				}

				MatrixXd coeffs = (A.transpose()*A).inverse()*(A.transpose()*b);

				double x = coeffs(0, 0)*cmInfo.vertices[k].center.x() + coeffs(1, 0)*cmInfo.vertices[k].center.y() + coeffs(2, 0);
				double y = coeffs(0, 1)*cmInfo.vertices[k].center.x() + coeffs(1, 1)*cmInfo.vertices[k].center.y() + coeffs(2, 1);
				double z = coeffs(0, 2)*cmInfo.vertices[k].center.x() + coeffs(1, 2)*cmInfo.vertices[k].center.y() + coeffs(2, 2);

				cmInfo.vertices[k].vertex = Point_3(x, y, z);
#else
				double x=0, y=0, z=0;
				for (unsigned i=0; i<neighbors.size(); i++)
				{
					x += cmInfo.vertices[neighbors[i]].vertex.x();
					y += cmInfo.vertices[neighbors[i]].vertex.y();
					z += cmInfo.vertices[neighbors[i]].vertex.z();
				}
				cmInfo.vertices[k].vertex = Point_3(x/neighbors.size(), y/neighbors.size(), z/neighbors.size());
#endif
			}
		}
	}
#endif
	return realCpNum;
}

void control_mesh_output(const CMInfo &cmInfo, QString fileName)
{
	QFile file(fileName);
	if (file.open(QIODevice::WriteOnly))
	{
		QTextStream out(&file);
		for (unsigned i = 0; i < cmInfo.vertices.size(); i++)
		{
			out << "v " << cmInfo.vertices[i].vertex.x << " " << cmInfo.vertices[i].vertex.y <<" "<< 0.0 << "\n";
			out << "c " << cmInfo.vertices[i].vertex.r << " " << cmInfo.vertices[i].vertex.g << " " << cmInfo.vertices[i].vertex.b << "\r\n";
		}
			
		for (set<CMEdgeInfo>::iterator eit=cmInfo.edges.begin(); eit!=cmInfo.edges.end(); eit++)
			out << "l " << eit->first()+1 << " " << eit->second()+1 << "\r\n";

		file.close();
	}
}

void centroid_triangulation_output(const CMInfo &cmInfo, QString fileName)
{
	QFile file(fileName);
	if (file.open(QIODevice::WriteOnly))
	{
		QTextStream out(&file);
		for (unsigned i=0; i<cmInfo.vertices.size(); i++)
		{
			if (cmInfo.vertices[i].flag==1)
				out << "r ";
			else
				out << "d ";
			out << cmInfo.vertices[i].center.x() << " " << cmInfo.vertices[i].center.y()<<" "<<0.0<< "\r\n";
		}

		for (set<CMEdgeInfo>::iterator eit=cmInfo.edges.begin(); eit!=cmInfo.edges.end(); eit++)
			out << "l " << eit->first()+1 << " " << eit->second()+1 << "\r\n";

		file.close();
	}
}

void mesh_output(Mesh *mesh, QString fileName)
{
	QFile file(fileName);
	if (file.open(QIODevice::WriteOnly))
	{
		QTextStream out(&file);
		out << "####" << "\r\n";
		out << "#" << "\r\n";
		out << "# Fake OBJ File Generated by B-spline Surface Reconstruction" << "\r\n";
		out << "#" << "\r\n";
		out << "#" << "\r\n";
		out << "# Vertices: " << mesh->size_of_vertices() << "\r\n";
		out << "# Faces: " << mesh->size_of_facets() << "\r\n";
		out << "#" << "\r\n";
		out << "####" << "\r\n";

		for (Vertex_iterator vit = mesh->vertices_begin(); vit != mesh->vertices_end(); vit++)
		{
			out << "vt " << vit->get_domain().x() << " " << vit->get_domain().y() << "\r\n";
			out << "v " << vit->point().x() << " " << vit->point().y() << " " << vit->point().z() << "\r";
			Point_3 color = vit->vertex_color();
			out << "vc " << color.x() << " " << color.y() << " " << color.z() <<"\r\n";
		}

		int fvNum = 3;
		if (mesh->is_pure_quad())
			fvNum = 4;

		for (Facet_iterator fit = mesh->facets_begin(); fit != mesh->facets_end(); fit++)
		{
			out << "f ";
			Halfedge_around_facet_circulator pHalfedge = fit->facet_begin();
			int i = 0;
			do
			{
				out << pHalfedge->vertex()->vertex_index() + 1 << "/" << pHalfedge->vertex()->vertex_index() + 1 << " ";
				i++;
			} while (++pHalfedge != fit->facet_begin());

			out << "\n";
			out << "is_feature ";
			do
			{
				out << pHalfedge->vertex()->is_feature() << " ";
				i++;
			} while (++pHalfedge != fit->facet_begin());
			out << "\n";
		}

		out << "# End of File";
		file.close();
	}
}


void mesh_output(Mesh *mesh, QString fileName, bool bColor,bool bheight)
{
	QFile file(fileName);
	if (file.open(QIODevice::WriteOnly))
	{
		QTextStream out(&file);
		out << "####" << "\r\n";
		out << "#" << "\r\n";
		out << "# OBJ File Generated by B-spline Surface Reconstruction" << "\r\n";
		out << "#" << "\r\n";
		out << "# Object 0_Fitted.obj" << "\r\n";
		out << "#" << "\r\n";
		out << "# Vertices: " << mesh->size_of_vertices() << "\r\n";
		out << "# Faces: " << mesh->size_of_facets() << "\r\n";
		out << "#" << "\r\n";
		out << "####" << "\r\n";

		for (Vertex_iterator vit = mesh->vertices_begin(); vit != mesh->vertices_end(); vit++)
		{
			out << "vt " << vit->get_domain().x() << " " << vit->get_domain().y() << "\r\n";
			if (bheight)
			{
				out << "v " << vit->point().x() << " " << vit->point().y() << " " << vit->point().z();
			}
			else
			{
				out << "v " << vit->point().x() << " " << vit->point().y() << " " << 0.0;
			}

			if (bColor)
			{
				Point_3 color = vit->vertex_pixel_color();
				out << " " << color.x() << " " << color.y() << " " << color.z() << "\n";
			}
			else
				out << "\n";
		}

		int fvNum = 3;
		if (mesh->is_pure_quad())
			fvNum = 4;

		for (Facet_iterator fit = mesh->facets_begin(); fit != mesh->facets_end(); fit++)
		{
			out << "f ";
			Halfedge_around_facet_circulator pHalfedge = fit->facet_begin();
			int i = 0;
			do
			{
				if (i == fvNum - 1)
					out << pHalfedge->vertex()->vertex_index() + 1 << "/" << pHalfedge->vertex()->vertex_index() + 1 << "\r\n";
				else
					out << pHalfedge->vertex()->vertex_index() + 1 << "/" << pHalfedge->vertex()->vertex_index() + 1 << " ";
				i++;
			} while (++pHalfedge != fit->facet_begin());
		}

		out << "# End of File";
		file.close();
	}
}

void extract_real_control_mesh(const CMInfo &srcInfo, CMInfo &dstInfo)
{
	map<unsigned, unsigned> correspondence;

	for (unsigned i=0; i<srcInfo.vertices.size(); i++)
	{
		if (srcInfo.vertices[i].flag==1)
		{
			correspondence[i] = dstInfo.vertices.size();
			dstInfo.vertices.push_back(srcInfo.vertices[i]);
		}
	}

	map<unsigned, unsigned>::iterator it;
	for (set<CMEdgeInfo>::iterator eit=srcInfo.edges.begin(); eit!=srcInfo.edges.end(); eit++)
	{
		CMEdgeInfo edge = *eit;
		it = correspondence.find(edge.first());
		if (it!=correspondence.end())
		{
			edge.set_first(it->second);
			it = correspondence.find(edge.second());
			if (it!=correspondence.end())
			{
				edge.set_second(it->second);
				dstInfo.edges.insert(edge);
			}
		}
	}
}

void extract_selected_control_mesh(const CMInfo &srcInfo, CMInfo &dstInfo)
{
	map<unsigned, unsigned> correspondence;

	for (unsigned i = 0; i < srcInfo.vertices.size(); i++)
	{
		if (srcInfo.vertices[i].flag == 1 && srcInfo.vertices[i].do_display)
		{
			correspondence[i] = dstInfo.vertices.size();
			dstInfo.vertices.push_back(srcInfo.vertices[i]);
		}
	}

	map<unsigned, unsigned>::iterator it;
	for (set<CMEdgeInfo>::iterator eit = srcInfo.edges.begin(); eit != srcInfo.edges.end(); eit++)
	{
		CMEdgeInfo edge = *eit;
		it = correspondence.find(edge.first());
		if (it != correspondence.end())
		{
			edge.set_first(it->second);
			it = correspondence.find(edge.second());
			if (it != correspondence.end())
			{
				edge.set_second(it->second);
				dstInfo.edges.insert(edge);
			}
		}
	}
}

void knots_output(map<unsigned, KnotData> &dataMaps, QString fileName)
{
	QFile file(fileName);
	if (file.open(QIODevice::WriteOnly))
	{
		QTextStream out(&file);
		out.setRealNumberPrecision(16);
		map<unsigned, KnotData>::iterator it = dataMaps.begin();
		for (; it!=dataMaps.end(); it++)
		{
			if (it->second.flag.bMulti)
				out << "m ";
			else
				out << "r ";
			out << it->second.pt.x() << " " << it->second.pt.y() << "\r\n";
		}

		file.close();
	}
}

void coplanar_info_output(const BSplineBasis &bSplineBasis, map<unsigned, KnotData> &dataMaps, QString fileName)
{
	vector<Point_2> coplanarPts;
	for (unsigned i=0; i<bSplineBasis.cpInfos.size(); i++)
	{
		unsigned mergeLoc = bSplineBasis.cpInfos[i].cornerBasisId;
		for (unsigned j=0; j<bSplineBasis.basisMergeInfos[mergeLoc].basisMerge.size(); j++)
		{
			unsigned basisLoc = bSplineBasis.basisMergeInfos[mergeLoc].basisMerge[j];
			double x =0, y=0;
			for (unsigned k=0; k<bSplineBasis.basisConfigs[basisLoc].config.size(); k++)
			{
				x += dataMaps[bSplineBasis.basisConfigs[basisLoc].config[k]].pt.x();
				y += dataMaps[bSplineBasis.basisConfigs[basisLoc].config[k]].pt.y();
			}
			x /= bSplineBasis.basisConfigs[basisLoc].config.size();
			y /= bSplineBasis.basisConfigs[basisLoc].config.size();
			coplanarPts.push_back(Point_2(x, y));
		}

		for (unsigned l=0; l<bSplineBasis.cpInfos[i].c0Ids.size(); l++)
		{
			mergeLoc = bSplineBasis.cpInfos[i].c0Ids[l];
			for (unsigned j=0; j<bSplineBasis.basisMergeInfos[mergeLoc].basisMerge.size(); j++)
			{
				unsigned basisLoc = bSplineBasis.basisMergeInfos[mergeLoc].basisMerge[j];
				double x =0, y=0;
				for (unsigned k=0; k<bSplineBasis.basisConfigs[basisLoc].config.size(); k++)
				{
					x += dataMaps[bSplineBasis.basisConfigs[basisLoc].config[k]].pt.x();
					y += dataMaps[bSplineBasis.basisConfigs[basisLoc].config[k]].pt.y();
				}
				x /= bSplineBasis.basisConfigs[basisLoc].config.size();
				y /= bSplineBasis.basisConfigs[basisLoc].config.size();
				coplanarPts.push_back(Point_2(x, y));
			}
		}
	}

	QFile file(fileName);
	if (file.open(QIODevice::WriteOnly))
	{
		QTextStream out(&file);
		for (unsigned i=0; i<coplanarPts.size(); i++)
			out << coplanarPts[i].x() << " " << coplanarPts[i].y() << "\r\n";
		file.close();
	}
}

bool generate_output_directory(QString allDir)
{
	QStringList dirList = allDir.split("/");
	QString dir = ".";

	for (unsigned i=0; i<dirList.size(); i++)
	{
		bool ret = false;

		QDir outputDir(dir);
		dir.append("/");
		dir.append(dirList.at(i));

		QString dirName = dirList.at(i);

		if (outputDir.exists(dirName))
			ret = true;
		else
			ret = outputDir.mkdir(dirName);

		if (!ret)
			return false;
	}

	return true;
}

bool generate_output_directory(QString allDir, QString prefix_,int butlast)
{
	QStringList dirList = allDir.split("/");
	QString dir = prefix_;

	for (unsigned i = 0; i < dirList.size()-butlast; i++)
	{
		bool ret = false;

		QDir outputDir(dir);
		dir.append("/");
		dir.append(dirList.at(i));

		QString dirName = dirList.at(i);

		if (outputDir.exists(dirName))
			ret = true;
		else
			ret = outputDir.mkdir(dirName);

		if (!ret)
			return false;
	}

	return true;
}


void update_tconfig3s(std::vector<TConfig3> &tconfigs)
{
	std::vector<TConfig3>::iterator it = tconfigs.begin();
	for (; it!=tconfigs.end(); )
	{
		if (!it->bUsed)
			it = tconfigs.erase(it);
		else
			it++;
	}
}

void union_two_ccw_region(vector<Point_2> &poly1, vector<Point_2> &poly2, vector<Point_2> &out)
{
	out.clear();

	assert(poly1.size() > 0);
	assert(poly2.size() > 0);

	Polygon_e2 polygon_1, polygon_2;
	for (int k = 0; k < poly1.size(); k++)
	{
		double x = poly1[k].x();
		double y = poly1[k].y();
		polygon_1.push_back(Point_e2(x, y));
	}
	for (int k = 0; k < poly2.size(); k++)
	{
		double x = poly2[k].x();
		double y = poly2[k].y();
		polygon_2.push_back(Point_e2(x, y));
	}

	assert((CGAL::do_intersect(polygon_1, polygon_2)));

	//union
	Polygon_with_holes_e2  unionR;
	CGAL::join(polygon_1, polygon_2, unionR);
	if (unionR.has_holes())
	{
		std::cout << __FUNCTION__ << ": " << "region has holes during union_two_ccw_region()" << std::endl;
	}
	Polygon_e2 polygon_union;
	if (!unionR.is_unbounded())
	{
		polygon_union = unionR.outer_boundary();
	}

	vector<Point_2> result;
	for (int k = 0; k < polygon_union.size(); k++)
	{
		double x = CGAL::to_double(polygon_union[k].x());
		double y = CGAL::to_double(polygon_union[k].y());
		result.push_back(Point_2(x, y));
	}
	remove_same_elements(result);

	out = result;
}

void intersect_two_ccw_region(vector<Point_2> &poly1, vector<Point_2> &poly2, vector<Point_2> &out)
{
	out.clear();

	assert(poly1.size() > 0);
	assert(poly2.size() > 0);

	Polygon_e2 polygon_1, polygon_2;
	for (int k = 0; k < poly1.size(); k++)
	{
		double x = poly1[k].x();
		double y = poly1[k].y();
		polygon_1.push_back(Point_e2(x, y));
	}
	for (int k = 0; k < poly2.size(); k++)
	{
		double x = poly2[k].x();
		double y = poly2[k].y();
		polygon_2.push_back(Point_e2(x, y));
	}

	assert((CGAL::do_intersect(polygon_1, polygon_2)));

	//intersect
	vector<Polygon_with_holes_e2>	intR;
	CGAL::intersection(polygon_1, polygon_2, std::back_inserter(intR));

	Polygon_e2 polygon_intersection;
	if (!intR[0].is_unbounded())
	{
		polygon_intersection = intR[0].outer_boundary();
	}

	if (polygon_intersection.area() < 1e-16)
	{
		std::cout << __FUNCTION__ << ": " << "polygons maybe do not intersect" << std::endl;
		assert(0);
	}

	vector<Point_2> intersect_region;
	for (int k = 0; k < polygon_intersection.size(); k++)
	{
		double x = CGAL::to_double(polygon_intersection[k].x());
		double y = CGAL::to_double(polygon_intersection[k].y());
		intersect_region.push_back(Point_2(x, y));
	}
	//add perturbation
	remove_same_elements(intersect_region);

	out = intersect_region;
}