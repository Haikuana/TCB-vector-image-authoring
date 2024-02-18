#include "Corefuncs/knotstriangulation.h"
#include "Corefuncs/auxfunc.h"
#include <QtWidgets/QMessageBox>
#include <CGAL/convex_hull_2.h>
#include <math.h>

#define M_PI 3.14159265358979323846

CKnotsTriangulation::CKnotsTriangulation(Mesh *m, map<unsigned, KnotData> *data, int deg)
{
	mesh = m;
	pDataMaps = data;
	degree = deg;
	triType = DELAUNAY;
	kdTree = NULL;
}

CKnotsTriangulation::CKnotsTriangulation(map<unsigned, KnotData>* data, int deg)
{
	pDataMaps = data;
	degree = deg;
	triType = DELAUNAY;
	kdTree = NULL;
}

CKnotsTriangulation::CKnotsTriangulation()
{
	mesh = NULL;
	pDataMaps = NULL;
	degree = 2;
	triType = DDT;
	kdTree = NULL;
}

void CKnotsTriangulation::set_mesh(Mesh *m)
{
	mesh = m;
}

void CKnotsTriangulation::set_kdtree(ANNkd_tree *tree)
{
	kdTree = tree;
}

void  CKnotsTriangulation::set_knots(map<unsigned, KnotData> *pknots)
{
	pDataMaps = pknots;
}

void CKnotsTriangulation::set_deg(int d)
{
	degree = d;
}

void CKnotsTriangulation::set_triangulation_type(Triangulation_Type type)
{
	triType = type;
}

void CKnotsTriangulation::set_optimization_algorithm(TriangulationOptimizationAlg opt)
{
	optAlg = opt;
}

void CKnotsTriangulation::set_correspondence(map<unsigned, unsigned> *cor)
{
	correspondence = cor;
}

void CKnotsTriangulation::set_corner_indices(vector<unsigned> *cornerIds)
{
	chIdx = cornerIds;
}

void CKnotsTriangulation::set_collinear_knots_config(const vector<vector<PointTriple>>& collinear_k, 
	vector<vector<PointTriple>> feature_bound_corner,
	vector<RuptureKnotConfig>	rupture_edgebasis_config,int ncollinear)
{
	ncollinear_rate = ncollinear;
	collinear_knots_lines = collinear_k;
	feature_boundto_corner = feature_bound_corner;
	rupture_basis_config = rupture_edgebasis_config;
}

void CKnotsTriangulation::link_triangulation_procedure(CT &tri, vector<vector<TConfig2>> &alltconfigs)
{
	extract_convex_hull();
	multipleknots_perturbation_oncorner();
	multipleknots_perturbation_onfeature(tri);
	std::cout << __FUNCTION__ << ": " << "multiple knots finished" << std::endl;

	int nf0 = tri.number_of_faces();

	for (int i = tri.number_of_vertices();i<pDataMaps->size();i++)
	{
		CT::Vertex_handle v;
		v = tri.insert(pDataMaps->at(i).pt);
		v->set_associated_index(i);
	}
	//remove face on line
	for (CDT::Face_iterator fit = tri.faces_begin(); fit != tri.faces_end(); fit++)
	{
		bool x_online = abs(fit->vertex(0)->point().x() - fit->vertex(1)->point().x()) < 1e-8 &&
			abs(fit->vertex(0)->point().x() - fit->vertex(2)->point().x()) < 1e-8;
		bool y_online = abs(fit->vertex(0)->point().y() - fit->vertex(1)->point().y()) < 1e-8 &&
			abs(fit->vertex(0)->point().y() - fit->vertex(2)->point().y()) < 1e-8;
		if (x_online || y_online)
		{
			tri.delete_face(fit);
			std::cout << __FUNCTION__ << ": " << "warning: (may be wrong) delete face on line at knottriangulation.cpp 133" << std::endl;
		}
	}

	int nf = tri.number_of_faces();
	bool te = 1;

	//sort the face such that we can read data from file
	vector<FaceSort> faces;
	for (auto fit = tri.faces_begin();fit != tri.faces_end();fit++)
	{
		FaceSort facet;
		for (int j = 0;j<3;j++)
		{
			facet.face.push_back(fit->vertex(j)->get_associated_index());
		}
		sort(facet.face.begin(), facet.face.end());
		facet.fit = fit;
		faces.push_back(facet);
	}
	sort(faces.begin(), faces.end(),sortFaceGreater);
	for (int i = 0;i<faces.size();i++)
	{
		faces[i].fit->info().facetindex = i;
	}

	//test
	//std::ifstream fin_0;
	//fin_0.open("allface.txt");
	//vector<vector<unsigned>> allface;
	//if (fin_0.is_open())
	//{
	//	int size_;
	//	while (fin_0 >> size_)
	//	{
	//		vector<unsigned> face;
	//		for (int j = 0; j < size_; j++)
	//		{
	//			int v;
	//			fin_0 >> v;
	//			face.push_back(v);
	//		}
	//		allface.push_back(face);
	//	}
	//}
	//fin_0.close();
	//std::cout << __FUNCTION__ << ": " << "read test" << std::endl;
	////contrast
	//int is_same0 = -1;
	//if (allface.size() != tri.number_of_faces())
	//{
	//	is_same0 = 1000000;
	//}
	//else
	//{
	//	CT::Face_iterator fit = tri.faces_begin();
	//	int i = 0;
	//	for (; fit != tri.faces_end(); i++, fit++)
	//	{
	//		vector<unsigned> face_temp;
	//		face_temp.push_back(fit->vertex(0)->get_associated_index());
	//		face_temp.push_back(fit->vertex(1)->get_associated_index());
	//		face_temp.push_back(fit->vertex(2)->get_associated_index());
	//		if (!equal(face_temp.begin(), face_temp.end(), allface[i].begin()))
	//		{
	//			is_same0 = i;
	//			break;
	//		}
	//	}
	//}
	//std::cout << __FUNCTION__ << ": " << "---------------------------test: knot mesh face is same: " << is_same0 << "\n";
	////write
	//std::ofstream fout_0;
	//fout_0.open("allface.txt");
	//if (fout_0.is_open())
	//{
	//	CT::Face_iterator fit = tri.faces_begin();
	//	for (; fit != tri.faces_end();fit++)
	//	{
	//		fout_0 << 3 << " ";
	//		fout_0 << fit->vertex(0)->get_associated_index() << " ";
	//		fout_0 << fit->vertex(1)->get_associated_index() << " ";
	//		fout_0 << fit->vertex(2)->get_associated_index() << " ";
	//		fout_0 << "\n";
	//	}
	//}
	//fout_0.close();

	collect_configs(tri, alltconfigs);
	high_degree_triangulation(alltconfigs);
}

void  CKnotsTriangulation::initial_triangulation(CT &tri)
{
	CDT delaunay;
	map<unsigned, KnotData>::iterator it = pDataMaps->begin(), end = pDataMaps->end();
	for(; it!=end; it++)
	{
		CDT::Vertex_handle v;
		v = delaunay.insert(it->second.pt);
		v->set_associated_index(it->first);
	}
	tri = delaunay;
}

void CKnotsTriangulation::extract_convex_hull()
{
	for (map<unsigned, KnotData>::iterator it=pDataMaps->begin(); it!=pDataMaps->end(); it++)
	{
		if (it->second.flag.bCorner)
		{
			chIdx->push_back(it->first);
		}
	}
}

void CKnotsTriangulation::multipleknots_perturbation_oncorner()
{	
	int sz = chIdx->size();

	vector<pair<double, double>> distangles(sz);
	for(int i=0; i<sz; i++)
	{
		double dist = 1e100;
		for (int j=0; j<pDataMaps->size(); j++)
		{
			if (j==(*chIdx)[i])
				continue;

			double d = ((*pDataMaps)[j].pt-(*pDataMaps)[(*chIdx)[i]].pt).squared_length();
			if (d>std::numeric_limits<double>::epsilon() && dist>d)
			{
				dist = d;
			}
		}

		int k = (i+sz-1)%sz;

		dist = std::min(sqrt(dist)/10, 1e-6);
		Vector_2 vec1 = (*pDataMaps)[(*chIdx)[k]].pt-(*pDataMaps)[(*chIdx)[i]].pt;
		Vector_2 vec2 = (*pDataMaps)[(*chIdx)[(i+1)%sz]].pt-(*pDataMaps)[(*chIdx)[i]].pt;
		vec1  = vec1/sqrt(vec1.squared_length());
		vec2  = vec2/sqrt(vec2.squared_length());
		double angle = acos(vec1*vec2);
		distangles[i] = std::make_pair(dist, angle);
	}

	unsigned n = pDataMaps->size();
	double u, v;

	for(int k=0; k<sz; k++)
	{
		int j = (k+sz-1)%sz;
		Vector_2 vec1 = (*pDataMaps)[(*chIdx)[j]].pt-(*pDataMaps)[(*chIdx)[k]].pt;
		vec1 = vec1/sqrt(vec1.squared_length());
		Vector_2 vec2 = (*pDataMaps)[(*chIdx)[(k+1)%sz]].pt-(*pDataMaps)[(*chIdx)[k]].pt;
		vec2 = vec2/sqrt(vec2.squared_length());

		double m[2][2] = { {vec1.x(),vec1.y()}, {vec2.x(),vec2.y()}};
		cv::Mat A(2, 2, CV_64F,m);
		cv::Mat invA=A.inv();
		
		for (int i = 1; i < degree+1; i++)
		{
			double bd[2] = { cos(i * distangles[k].second / (degree + 1)) ,cos((degree + 1 - i) * distangles[k].second / (degree + 1)) };
			cv::Mat b(2, 1, CV_64F,bd);
			cv::Mat x(2, 1, CV_64F);
			cv::gemm(invA, b, 1, 0, 0, x);

			KnotData knot;

			// Get coordinates of warped pixel in coordinate frame of I.
			u = *(x.ptr<double>(0));
			v = *(x.ptr<double>(1));

			knot.pt = Point_2((*pDataMaps)[(*chIdx)[k]].pt.x()+distangles[k].first*u, (*pDataMaps)[(*chIdx)[k]].pt.y()+distangles[k].first*v);
			knot.index = n;
			knot.flag = (*pDataMaps)[(*chIdx)[k]].flag;
			knot.flag.bMulti = 1;
			(*pDataMaps)[n] = knot;
			(*correspondence)[n++] = (*chIdx)[k];
		}
	}

}

void CKnotsTriangulation::multipleknots_perturbation_onfeature(CT &tri)
{
	vector<vector<unsigned>>  feature_corners;
	for (int i = 0; i < collinear_knots_lines.size(); i++)
	{
		if (collinear_knots_lines[i].size() > 3*ncollinear_rate+2)
		{
			vector<unsigned int> feature_triple_circle;
			if (collinear_knots_lines[i][0].index == collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index)
			{
				feature_triple_circle.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 2].index);
				feature_triple_circle.push_back(collinear_knots_lines[i][0].index);
				feature_triple_circle.push_back(collinear_knots_lines[i][1].index);
				feature_corners.push_back(feature_triple_circle);
			}
		}
		if (ncollinear_rate > 0) {
			for (int j = 0; j < collinear_knots_lines[i].size();)
			{
				if (collinear_knots_lines[i].size() <= ncollinear_rate + 2)
				{
					break;
				}

				vector<unsigned int> feature_triple;
				j++;
				if (j > collinear_knots_lines[i].size() - 1 || !collinear_knots_lines[i][j].is_fixed)
				{
					continue;
				}
				if (j < collinear_knots_lines[i].size() - 1)
				{
					feature_triple.push_back(collinear_knots_lines[i][j - 1].index);
					feature_triple.push_back(collinear_knots_lines[i][j].index);
					feature_triple.push_back(collinear_knots_lines[i][j + 1].index);
					feature_corners.push_back(feature_triple);
				}
			}
		}
		else
		{
			for (int j = 1; j < collinear_knots_lines[i].size()-1; j++)
			{
				vector<unsigned int> feature_triple;
				feature_triple.push_back(collinear_knots_lines[i][j - 1].index);
				feature_triple.push_back(collinear_knots_lines[i][j].index);
				feature_triple.push_back(collinear_knots_lines[i][j + 1].index);
				feature_corners.push_back(feature_triple);
			}
		}
	}

	map<int, CT::Vertex_handle> ct_trihandle;
	for (auto vit = tri.vertices_begin();vit != tri.vertices_end();vit++)
	{
		ct_trihandle[vit->get_associated_index()] = vit;
	}

	int sz = feature_corners.size()+feature_boundto_corner.size();
	vector<pair<double, double>> distangles(sz);
	for (int i = 0; i < feature_corners.size(); i++)
	{
		double dist = 1e100;
		for (int j = 0; j < pDataMaps->size(); j++)
		{
			if (j == feature_corners[i][1])
				continue;

			double d = ((*pDataMaps)[j].pt - (*pDataMaps)[feature_corners[i][1]].pt).squared_length();
			if (d > std::numeric_limits<double>::epsilon() && dist > d)
			{
				dist = d;
			}
		}

		//test
		if (feature_corners[i][1] == 15)
		{
			int t = 1;
		}

		dist = std::min(sqrt(dist) / 10, 1e-3);

		//distance of one ring vertex to feature segments
		Point_2 pcenter = (*pDataMaps)[feature_corners[i][1]].pt;
		Point_2 pside0 = (*pDataMaps)[feature_corners[i][0]].pt;
		Point_2 pside2 = (*pDataMaps)[feature_corners[i][2]].pt;

		double dis_tri = abs(CGAL::area(pcenter, pside0, pside2)) / sqrt((pside0 - pside2).squared_length()) / 5.0;
		dist = std::min(sqrt(dist) / 10, dis_tri);

		double distside = 1e10;

		CT::Vertex_handle v_center = ct_trihandle.at(feature_corners[i][1]);
		CT::Vertex_circulator vit = v_center->incident_vertices();
		do 
		{
			int it = vit->get_associated_index();//test
			if (vit->get_associated_index() == feature_corners[i][0] ||
				vit->get_associated_index() == feature_corners[i][2])
			{
				continue;
			}
			Point_2 ponering = vit->point();
			double distnow = dis_between_p_line(pcenter, pside0, ponering);
			if (distnow < distside)
			{
				distside = distnow;
			}
			distnow = dis_between_p_line(pcenter, pside2, ponering);
			if (distnow < distside)
			{
				distside = distnow;
			}
		} while (++vit != v_center->incident_vertices());

		dist = std::min(dist,0.1*distside);

		Vector_2 vec1 = (*pDataMaps)[feature_corners[i][0]].pt - (*pDataMaps)[feature_corners[i][1]].pt;
		Vector_2 vec2 = (*pDataMaps)[feature_corners[i][2]].pt - (*pDataMaps)[feature_corners[i][1]].pt;
		vec1 = vec1 / sqrt(vec1.squared_length());
		vec2 = vec2 / sqrt(vec2.squared_length());
		double angle = acos(vec1*vec2);
		distangles[i] = std::make_pair(dist, angle);
	}
	
	for (int i = 0;i<feature_boundto_corner.size();i++)
	{
		double dist = 1e100;
		for (int j = 0; j < pDataMaps->size(); j++)
		{
			if (j == feature_boundto_corner[i][1].index)
				continue;

			double d = ((*pDataMaps)[j].pt - feature_boundto_corner[i][1].imagedomain).squared_length();
			if (d > std::numeric_limits<double>::epsilon() && dist > d)
			{
				dist = d;
			}
		}
		dist = std::min(sqrt(dist) / 10, 1e-6);

		Vector_2 vec1 = feature_boundto_corner[i][2].imagedomain - feature_boundto_corner[i][1].imagedomain;
		Vector_2 vec2 = feature_boundto_corner[i][0].imagedomain - feature_boundto_corner[i][1].imagedomain;
		vec1 = vec1 / sqrt(vec1.squared_length());
		vec2 = vec2 / sqrt(vec2.squared_length());
		double angle = acos(vec1*vec2);
		distangles[i+feature_corners.size()] = std::make_pair(dist, angle);
	}

	unsigned n = pDataMaps->size();
	double u, v;

	for (int k = 0; k < feature_corners.size(); k++)
	{
		Vector_2 vec1 = (*pDataMaps)[feature_corners[k][0]].pt - (*pDataMaps)[feature_corners[k][1]].pt;
		Vector_2 vec2 = (*pDataMaps)[feature_corners[k][2]].pt - (*pDataMaps)[feature_corners[k][1]].pt;
		vec1 = vec1 / sqrt(vec1.squared_length());
		vec2 = vec2 / sqrt(vec2.squared_length());

		double m[2][2] = { {vec1.x(),vec1.y()}, {vec2.x(),vec2.y()} };
		cv::Mat A(2, 2, CV_64F, m);
		cv::Mat invA = A.inv();

		for (int i = 1; i < degree + 1; i++)
		{
			double bd[2] = { cos(i * distangles[k].second / (degree + 1)) ,cos((degree + 1 - i) * distangles[k].second / (degree + 1)) };
			cv::Mat b(2, 1, CV_64F, bd);
			cv::Mat x(2, 1, CV_64F);
			cv::gemm(invA, b, 1, 0, 0, x);

			KnotData knot;

			// Get coordinates of warped pixel in coordinate frame of I.
			u = *(x.ptr<double>(0));
			v = *(x.ptr<double>(1));

			knot.pt = Point_2((*pDataMaps)[feature_corners[k][1]].pt.x() + distangles[k].first*u, (*pDataMaps)[feature_corners[k][1]].pt.y() + distangles[k].first*v);
			knot.index = n;
			knot.flag = (*pDataMaps)[feature_corners[k][1]].flag;
			knot.flag.bMulti = 1;
			(*pDataMaps)[n] = knot;
			(*correspondence)[n++] = feature_corners[k][1];
		}
	}

	for (int k = 0; k < feature_boundto_corner.size(); k++)
	{
		Vector_2 vec1 = feature_boundto_corner[k][2].imagedomain - feature_boundto_corner[k][1].imagedomain;
		Vector_2 vec2 = feature_boundto_corner[k][0].imagedomain - feature_boundto_corner[k][1].imagedomain;
		vec1 = vec1 / sqrt(vec1.squared_length());
		vec2 = vec2 / sqrt(vec2.squared_length());
		double m[2][2] = { {vec1.x(),vec1.y()}, {vec2.x(),vec2.y()} };
		cv::Mat A(2, 2, CV_64F, m);
		cv::Mat invA = A.inv();

		for (int i = 1; i < degree + 1; i++)
		{
			double bd[2] = { cos(i * distangles[k].second / (degree + 1)) ,cos((degree + 1 - i) * distangles[k].second / (degree + 1)) };
			cv::Mat b(2, 1, CV_64F, bd);
			cv::Mat x(2, 1, CV_64F);
			cv::gemm(invA, b, 1, 0, 0, x);

			KnotData knot;

			// Get coordinates of warped pixel in coordinate frame of I.
			u = *(x.ptr<double>(0));
			v = *(x.ptr<double>(1));

			knot.pt = Point_2(feature_boundto_corner[k][1].imagedomain.x() + distangles[k+feature_corners.size()].first*u,
				feature_boundto_corner[k][1].imagedomain.y() + distangles[k+feature_corners.size()].first*v);
			knot.index = n;
			knot.flag = (*pDataMaps)[feature_boundto_corner[k][1].index].flag;
			knot.flag.bMulti = 1;
			(*pDataMaps)[n] = knot;
			(*correspondence)[n++] = feature_boundto_corner[k][1].index;
		}
	}
}


void CKnotsTriangulation::collect_configs(CT &tri, vector<vector<TConfig2>> &alltconfigs)
{
	vector<TConfig2> configs;
	CT::Finite_faces_iterator f_it = tri.finite_faces_begin(), f_end = tri.finite_faces_end();
	map<int, CT::Finite_faces_iterator> facet_index;
	for (; f_it != f_end; f_it++)
	{
		facet_index[f_it->info().facetindex] = f_it;
	}
	//To get the Configurations Gamma_0
	for(int i = 0;i<facet_index.size();i++)
	{
		TConfig2 config;
		config.tconfig.push_back(facet_index.at(i)->vertex(0)->get_associated_index());
		config.tconfig.push_back(facet_index.at(i)->vertex(1)->get_associated_index());
		config.tconfig.push_back(facet_index.at(i)->vertex(2)->get_associated_index());
		configs.push_back(config);
	}
	alltconfigs.push_back(configs);
}

bool CKnotsTriangulation::flip_neighbour_four_edges(CT &tri, CT::Face_handle &f, int index, FaceInfo &orgFInfo, 
	FaceInfo &orgGInfo, std::vector<NeighbourFaceInfo> &neighbourInfos, std::map<EdgeInfo, Edge> &suspicionEdges)
{
	CT::Face_handle g = f->neighbor(index);
	int i = tri.mirror_index(f, index);
	//int i = f->mirror_index(f,index);

	FaceInfo fInfo = f->info();
	FaceInfo gInfo = g->info();
	double orgCost;

	if (neighbourInfos[0].bExist)
	{
#if 0
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[0].info.cost)/
			(orgFInfo.weight + orgGInfo.weight + neighbourInfos[0].info.weight + std::numeric_limits<double>::epsilon());
#else
		orgCost = orgFInfo.cost + orgGInfo.cost + neighbourInfos[0].info.cost;
#endif
		if(flip_one_edge(tri, f, f->ccw(index), orgCost, gInfo, suspicionEdges))
		{
			Edge e = std::make_pair(g->vertex(i), g->vertex(g->ccw(i)));
			add_suspicions(tri, e, suspicionEdges);
			e = std::make_pair(g->vertex(i), g->vertex(g->cw(i)));
			add_suspicions(tri, e, suspicionEdges);
			return true;
		}
	}

	if (neighbourInfos[1].bExist)
	{
#if 0
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[1].info.cost)/
			(orgFInfo.weight + orgGInfo.weight + neighbourInfos[1].info.weight + std::numeric_limits<double>::epsilon());
#else
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[1].info.cost);
#endif
		if(flip_one_edge(tri, f, f->cw(index), orgCost, gInfo, suspicionEdges))
		{
			Edge e = std::make_pair(g->vertex(i), g->vertex(g->ccw(i)));
			add_suspicions(tri, e, suspicionEdges);
			e = std::make_pair(g->vertex(i), g->vertex(g->cw(i)));
			add_suspicions(tri, e, suspicionEdges);
			return true;
		}
	}

	if (neighbourInfos[2].bExist)
	{
#if 0
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[2].info.cost)/
			(orgFInfo.weight + orgGInfo.weight + neighbourInfos[2].info.weight + std::numeric_limits<double>::epsilon());
#else
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[2].info.cost);
#endif
		if (flip_one_edge(tri, g, g->ccw(i), orgCost, fInfo, suspicionEdges))
		{
			Edge e = std::make_pair(f->vertex(index), f->vertex(f->ccw(index)));
			add_suspicions(tri, e, suspicionEdges);
			e = std::make_pair(f->vertex(index), f->vertex(f->cw(index)));
			add_suspicions(tri, e, suspicionEdges);
			return true;
		}
	}
	if (neighbourInfos[3].bExist)
	{
#if 0
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[3].info.cost)/
			(orgFInfo.weight + orgGInfo.weight + neighbourInfos[3].info.weight + std::numeric_limits<double>::epsilon());
#else
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[3].info.cost);
#endif
		if (flip_one_edge(tri, g, g->cw(i), orgCost, fInfo, suspicionEdges))
		{
			Edge e = std::make_pair(f->vertex(index), f->vertex(f->ccw(index)));
			add_suspicions(tri, e, suspicionEdges);
			e = std::make_pair(f->vertex(index), f->vertex(f->cw(index)));
			add_suspicions(tri, e, suspicionEdges);
			return true;
		}
	}
	return false;
}

bool CKnotsTriangulation::flip_one_edge(CT &tri, CT::Face_handle &f, int index, double orgCost, 
	FaceInfo &restInfo, std::map<EdgeInfo, Edge> &suspicionEdges)
{
	Edge e = std::make_pair(f->vertex(f->cw(index)), f->vertex(f->ccw(index)));
	if (!is_flippable(tri, e))
		return false;

	CT::Face_handle g = f->neighbor(index);
	CT::Vertex_handle v = tri.mirror_vertex(f, index);
	vector<Point_2> flipFpts, flipGpts;
	flipFpts.push_back(f->vertex(index)->point());
	flipFpts.push_back(f->vertex(f->ccw(index))->point());
	flipFpts.push_back(v->point());

	flipGpts.push_back(f->vertex(f->cw(index))->point());
	flipGpts.push_back(f->vertex(index)->point());
	flipGpts.push_back(v->point());

	FaceInfo flipFinfo, flipGinfo;
	flipFinfo.triangle = flipFpts;
	flipGinfo.triangle = flipGpts;
#if 0
	FaceInfo comInfo;
	combine_vertices(f, g, comInfo);
	points_on_polygon(flipFpts, flipFinfo, flipGpts, flipGinfo, comInfo);
#else
	int i = tri.mirror_index(f, index);
	//int i = f->mirror_index(index);
	Point_2 p1 = f->vertex(index)->point(), p2 = g->vertex(i)->point();
	Point_2 p3 = f->vertex(f->ccw(index))->point();
	Vector_2 vec1 = p1-p2, vec2 = p3-p1;
	if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>=0)
	{
		for (int i=0; i<f->info().incInfos.size(); i++)
		{
			vec2 = f->info().incInfos[i]->get_domain()-p1;
			if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>=0)
				flipFinfo.incInfos.push_back(f->info().incInfos[i]);
			else
				flipGinfo.incInfos.push_back(f->info().incInfos[i]);
		}

		for (int i=0; i<g->info().incInfos.size(); i++)
		{
			vec2 = g->info().incInfos[i]->get_domain()-p1;
			if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>=0)
				flipFinfo.incInfos.push_back(g->info().incInfos[i]);
			else
				flipGinfo.incInfos.push_back(g->info().incInfos[i]);
		}
	}
	else
	{
		for (int i=0; i<f->info().incInfos.size(); i++)
		{
			vec2 = f->info().incInfos[i]->get_domain()-p1;
			if (vec1.x()*vec2.y()-vec1.y()*vec2.x()<0)
				flipFinfo.incInfos.push_back(f->info().incInfos[i]);
			else
				flipGinfo.incInfos.push_back(f->info().incInfos[i]);
		}

		for (int i=0; i<g->info().incInfos.size(); i++)
		{
			vec2 = g->info().incInfos[i]->get_domain()-p1;
			if (vec1.x()*vec2.y()-vec1.y()*vec2.x()<0)
				flipFinfo.incInfos.push_back(g->info().incInfos[i]);
			else
				flipGinfo.incInfos.push_back(g->info().incInfos[i]);
		}
	}
#endif

	face_plane_deviation(flipFinfo);
	face_plane_deviation(flipGinfo);

#if 0
	double flipCost = (restInfo.cost +flipFinfo.cost + flipGinfo.cost)
		/(restInfo.weight + flipFinfo.weight+flipGinfo.weight+std::numeric_limits<double>::epsilon());
#else
	double flipCost = restInfo.cost +flipFinfo.cost + flipGinfo.cost;
#endif

	if(orgCost>flipCost)//flippable
	{
		tri.flip(f, index);
		f->info() = flipFinfo;
		f->neighbor(f->ccw(index))->info() = flipGinfo;
		add_suspicions(tri, f, index, suspicionEdges);
		return true;
	}
	return false;
}


bool CKnotsTriangulation::flip_neighbour_four_edges(CT &tri, CT::Face_handle &f, int index, vector<pair<unsigned, unsigned>> &convexLink, 
	FaceInfo &orgFInfo, FaceInfo &orgGInfo, std::vector<NeighbourFaceInfo> &neighbourInfos, std::map<EdgeInfo, Edge> &suspicionEdges)
{
	CT::Face_handle g = f->neighbor(index);
	//int i = f->mirror_index(index);
	int i = tri.mirror_index(f, index);

	FaceInfo fInfo = f->info();
	FaceInfo gInfo = g->info();
	double orgCost;

	if (neighbourInfos[0].bExist)
	{
#if 0
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[0].info.cost)/
			(orgFInfo.weight + orgGInfo.weight + neighbourInfos[0].info.weight + std::numeric_limits<double>::epsilon());
#else
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[0].info.cost);
#endif
		if(flip_one_edge(tri, f, f->ccw(index), convexLink, orgCost, gInfo, suspicionEdges))
		{
			Edge e = std::make_pair(g->vertex(i), g->vertex(g->ccw(i)));
			add_suspicions(tri, e, suspicionEdges);
			e = std::make_pair(g->vertex(i), g->vertex(g->cw(i)));
			add_suspicions(tri, e, suspicionEdges);
			return true;
		}
	}

	if (neighbourInfos[1].bExist)
	{
#if 0
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[1].info.cost)/
			(orgFInfo.weight + orgGInfo.weight + neighbourInfos[1].info.weight + std::numeric_limits<double>::epsilon());
#else
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[1].info.cost);
#endif
		if(flip_one_edge(tri, f, f->cw(index), convexLink, orgCost, gInfo, suspicionEdges))
		{
			Edge e = std::make_pair(g->vertex(i), g->vertex(g->ccw(i)));
			add_suspicions(tri, e, suspicionEdges);
			e = std::make_pair(g->vertex(i), g->vertex(g->cw(i)));
			add_suspicions(tri, e, suspicionEdges);
			return true;
		}
	}

	if (neighbourInfos[2].bExist)
	{
#if 0
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[2].info.cost)/
			(orgFInfo.weight + orgGInfo.weight + neighbourInfos[2].info.weight + std::numeric_limits<double>::epsilon());
#else
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[2].info.cost);
#endif
		if (flip_one_edge(tri, g, g->ccw(i), convexLink, orgCost, fInfo, suspicionEdges))
		{
			Edge e = std::make_pair(f->vertex(index), f->vertex(f->ccw(index)));
			add_suspicions(tri, e, suspicionEdges);
			e = std::make_pair(f->vertex(index), f->vertex(f->cw(index)));
			add_suspicions(tri, e, suspicionEdges);
			return true;
		}
	}
	if (neighbourInfos[3].bExist)
	{
#if 0
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[3].info.cost)/
			(orgFInfo.weight + orgGInfo.weight + neighbourInfos[3].info.weight + std::numeric_limits<double>::epsilon());
#else
		orgCost = (orgFInfo.cost + orgGInfo.cost + neighbourInfos[3].info.cost);
#endif
		if (flip_one_edge(tri, g, g->cw(i), convexLink, orgCost, fInfo, suspicionEdges))
		{
			Edge e = std::make_pair(f->vertex(index), f->vertex(f->ccw(index)));
			add_suspicions(tri, e, suspicionEdges);
			e = std::make_pair(f->vertex(index), f->vertex(f->cw(index)));
			add_suspicions(tri, e, suspicionEdges);
			return true;
		}
	}
	return false;
}

bool CKnotsTriangulation::flip_one_edge(CT &tri, CT::Face_handle &f, int index, vector<pair<unsigned, unsigned>> &convexLink, 
	double orgCost, FaceInfo &restInfo, std::map<EdgeInfo, Edge> &suspicionEdges)
{
	Edge e = std::make_pair(f->vertex(f->cw(index)), f->vertex(f->ccw(index)));

	if(!is_convex(tri, e) || is_constrained(convexLink, e))
		return false;

	CT::Face_handle g = f->neighbor(index);
	CT::Vertex_handle v = tri.mirror_vertex(f, index);
	vector<Point_2> flipFpts, flipGpts;
	flipFpts.push_back(f->vertex(index)->point());
	flipFpts.push_back(f->vertex(f->ccw(index))->point());
	flipFpts.push_back(v->point());

	flipGpts.push_back(f->vertex(f->cw(index))->point());
	flipGpts.push_back(f->vertex(index)->point());
	flipGpts.push_back(v->point());

	FaceInfo flipFinfo, flipGinfo;
	flipFinfo.triangle = flipFpts;
	flipGinfo.triangle = flipGpts;
#if 0
	FaceInfo comInfo;
	combine_vertices(f, g, comInfo);
	points_on_polygon(flipFpts, flipFinfo, flipGpts, flipGinfo, comInfo);
#else
	int i = tri.mirror_index(f, index);
	//int i = f->mirror_index(index);
	Point_2 p1 = f->vertex(index)->point(), p2 = g->vertex(i)->point();
	Point_2 p3 = f->vertex(f->ccw(index))->point();
	Vector_2 vec1 = p1-p2, vec2 = p3-p1;
	if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>=0)
	{
		for (int i=0; i<f->info().incInfos.size(); i++)
		{
			vec2 = f->info().incInfos[i]->get_domain()-p1;
			if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>=0)
				flipFinfo.incInfos.push_back(f->info().incInfos[i]);
			else
				flipGinfo.incInfos.push_back(f->info().incInfos[i]);
		}

		for (int i=0; i<g->info().incInfos.size(); i++)
		{
			vec2 = g->info().incInfos[i]->get_domain()-p1;
			if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>=0)
				flipFinfo.incInfos.push_back(g->info().incInfos[i]);
			else
				flipGinfo.incInfos.push_back(g->info().incInfos[i]);
		}
	}
	else
	{
		for (int i=0; i<f->info().incInfos.size(); i++)
		{
			vec2 = f->info().incInfos[i]->get_domain()-p1;
			if (vec1.x()*vec2.y()-vec1.y()*vec2.x()<0)
				flipFinfo.incInfos.push_back(f->info().incInfos[i]);
			else
				flipGinfo.incInfos.push_back(f->info().incInfos[i]);
		}

		for (int i=0; i<g->info().incInfos.size(); i++)
		{
			vec2 = g->info().incInfos[i]->get_domain()-p1;
			if (vec1.x()*vec2.y()-vec1.y()*vec2.x()<0)
				flipFinfo.incInfos.push_back(g->info().incInfos[i]);
			else
				flipGinfo.incInfos.push_back(g->info().incInfos[i]);
		}
	}
#endif

	face_plane_deviation(flipFinfo);
	face_plane_deviation(flipGinfo);
#if 0
	double flipCost = (restInfo.cost +flipFinfo.cost + flipGinfo.cost)
		/(restInfo.weight + flipFinfo.weight+flipGinfo.weight+std::numeric_limits<double>::epsilon());
#else
	double flipCost = (restInfo.cost +flipFinfo.cost + flipGinfo.cost);
#endif
	if(orgCost>flipCost)//flippable
	{
		tri.flip(f, index);
		f->info() = flipFinfo;
		f->neighbor(f->ccw(index))->info() = flipGinfo;
		add_suspicions(tri, f, index, suspicionEdges);
		return true;
	}
	return false;
}

void CKnotsTriangulation::get_neighbour_four_face_info(CT &tri, CT::Face_handle &f, int index, vector<NeighbourFaceInfo> &neighbourInfos)
{
	CT::Face_handle g = f->neighbor(index);
	//int i = f->mirror_index(index);
	int i = tri.mirror_index(f, index);

	NeighbourFaceInfo neighbourInfo;
	CT::Face_handle h = f->neighbor(f->cw(index));
	if (h==NULL || tri.is_infinite(h))
		neighbourInfo.bExist = false;
	else
	{
		neighbourInfo.bExist = true;
		//face_plane_deviation(h->info());
		neighbourInfo.info = h->info();
	}
	neighbourInfos.push_back(neighbourInfo);

	h = g->neighbor(g->ccw(i));
	if (h==NULL || tri.is_infinite(h))
		neighbourInfo.bExist = false;
	else
	{
		neighbourInfo.bExist = true;
		//face_plane_deviation(h->info());
		neighbourInfo.info = h->info();
	}
	neighbourInfos.push_back(neighbourInfo);

	h = g->neighbor(g->cw(i));
	if (h==NULL || tri.is_infinite(h))
		neighbourInfo.bExist = false;
	else
	{
		neighbourInfo.bExist = true;
		//face_plane_deviation(h->info());
		neighbourInfo.info = h->info();
	}
	neighbourInfos.push_back(neighbourInfo);

	h = f->neighbor(f->ccw(index));
	if (h==NULL || tri.is_infinite(h))
		neighbourInfo.bExist = false;
	else
	{
		neighbourInfo.bExist = true;
		//face_plane_deviation(h->info());
		neighbourInfo.info = h->info();
	}
	neighbourInfos.push_back(neighbourInfo);
}

void CKnotsTriangulation::look_ahead(CT &tri)
{
	std::map<EdgeInfo, Edge> suspicionEdges;
	add_original_suspicions(tri, suspicionEdges, true);

	int ncount = 0;
	std::map<EdgeInfo, Edge>::iterator mit = suspicionEdges.begin();
	double area;
	std::map<EdgeInfo, int> flipEdges;
	std::map<EdgeInfo,int>::iterator it;
	EdgeInfo ids(0, 0);

	while(mit!=suspicionEdges.end())
	{
		CT::Face_handle f, g;
		int index;
		ids = mit->first;
		Edge edge = mit->second;

		suspicionEdges.erase(mit);//delete the edge from the suspicions

		if (!is_flippable(tri, edge))
		{
			mit = suspicionEdges.begin();
			continue;
		}

		it = flipEdges.find(ids);
		if (it!=flipEdges.end())
		{
			if (it->second>5)
			{
				mit = suspicionEdges.begin();
				continue;
			}
			it->second++;
		}
		else
			flipEdges[ids] = 1;

		CT::Vertex_handle v1 = edge.first;
		CT::Vertex_handle v2 = edge.second;
		tri.is_edge(v1, v2, f, index);
		g = f->neighbor(index);
		CT::Vertex_handle v = tri.mirror_vertex(f, index);

#if 0
		face_plane_deviation(f->info());
		face_plane_deviation(g->info());

		double orgCost = (f->info().cost + g->info().cost)
			/(f->info().weight+g->info().weight+std::numeric_limits<double>::epsilon());
#else
		double orgCost = f->info().cost + g->info().cost;
#endif

		vector<Point_2> flipFpts(3), flipGpts(3);
		flipFpts[0] = f->vertex(index)->point();
		flipFpts[1] = f->vertex(f->ccw(index))->point();
		flipFpts[2] = v->point();

		flipGpts[0] = f->vertex(f->cw(index))->point();
		flipGpts[1] = f->vertex(index)->point();
		flipGpts[2] = v->point();

		FaceInfo flipFinfo, flipGinfo;
		flipFinfo.triangle = flipFpts;
		flipGinfo.triangle = flipGpts;

#if 0
		FaceInfo comInfo;
		combine_vertices(f, g, comInfo);
		points_on_polygon(flipFpts, flipFinfo, flipGpts, flipGinfo, comInfo);
#else
		int i = tri.mirror_index(f, index);
		//int i = f->mirror_index(index);
		Point_2 p1 = f->vertex(index)->point(), p2 = g->vertex(i)->point();
		Point_2 p3 = f->vertex(f->ccw(index))->point();
		Vector_2 vec1 = p1-p2, vec2 = p3-p1;
		if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>=0)
		{
			for (int i=0; i<f->info().incInfos.size(); i++)
			{
				vec2 = f->info().incInfos[i]->get_domain()-p1;
				if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>=0)
					flipFinfo.incInfos.push_back(f->info().incInfos[i]);
				else
					flipGinfo.incInfos.push_back(f->info().incInfos[i]);
			}

			for (int i=0; i<g->info().incInfos.size(); i++)
			{
				vec2 = g->info().incInfos[i]->get_domain()-p1;
				if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>=0)
					flipFinfo.incInfos.push_back(g->info().incInfos[i]);
				else
					flipGinfo.incInfos.push_back(g->info().incInfos[i]);
			}
		}
		else
		{
			for (int i=0; i<f->info().incInfos.size(); i++)
			{
				vec2 = f->info().incInfos[i]->get_domain()-p1;
				if (vec1.x()*vec2.y()-vec1.y()*vec2.x()<0)
					flipFinfo.incInfos.push_back(f->info().incInfos[i]);
				else
					flipGinfo.incInfos.push_back(f->info().incInfos[i]);
			}

			for (int i=0; i<g->info().incInfos.size(); i++)
			{
				vec2 = g->info().incInfos[i]->get_domain()-p1;
				if (vec1.x()*vec2.y()-vec1.y()*vec2.x()<0)
					flipFinfo.incInfos.push_back(g->info().incInfos[i]);
				else
					flipGinfo.incInfos.push_back(g->info().incInfos[i]);
			}
		}
#endif
		face_plane_deviation(flipFinfo);
		face_plane_deviation(flipGinfo);

#if 0
		double flipCost = (flipFinfo.cost + flipGinfo.cost)
			/(flipFinfo.weight+flipGinfo.weight+std::numeric_limits<double>::epsilon());
#else
		double flipCost = flipFinfo.cost + flipGinfo.cost;
#endif

		if(orgCost>flipCost)//flippable
		{
			tri.flip(f, index);
			f->info() = flipFinfo;
			f->neighbor(f->ccw(index))->info() = flipGinfo;
			add_suspicions(tri, f, index, suspicionEdges);
		}
		else
		{
			FaceInfo orgFInfo = f->info();
			FaceInfo orgGInfo = g->info();
			vector<NeighbourFaceInfo> neighbourInfos;
			get_neighbour_four_face_info(tri, f, index, neighbourInfos);
			tri.flip(f, index);
			f->info() = flipFinfo;
			f->neighbor(f->ccw(index))->info() = flipGinfo;
			int i = f->ccw(index);
			if (!flip_neighbour_four_edges(tri, f, i, orgFInfo, orgGInfo, neighbourInfos, suspicionEdges))
			{
				tri.flip(f, i);
				f->info() = orgGInfo;
				f->neighbor(f->ccw(i))->info() = orgFInfo;
			}
		}
		mit = suspicionEdges.begin();
	}
}

void CKnotsTriangulation::look_ahead(CT &tri, vector<pair<unsigned, unsigned>> &convexLink)
{
	std::map<EdgeInfo, Edge> suspicionEdges;
	add_original_suspicions(tri, suspicionEdges);

	for (CT::Finite_faces_iterator fit= tri.finite_faces_begin(); fit!=tri.finite_faces_end(); fit++)
	{
		vector<Point_2> pts;
		for (int i=0; i<3; i++)
			pts.push_back(fit->vertex(i)->point());
		
		fit->info().triangle = pts;
		points_on_polygon(mesh, kdTree, pts, fit->info());
		face_plane_deviation(fit->info());
	}

	for (CT::Finite_faces_iterator fit= tri.finite_faces_begin(); fit!=tri.finite_faces_end(); fit++)
		for (int i=0; i<fit->info().incInfos.size(); i++)
			fit->info().incInfos[i]->vertex_flag().bStatus = 0;

	int ncount = 0;
	std::map<EdgeInfo, Edge>::iterator mit = suspicionEdges.begin();
	double area;
	std::map<EdgeInfo, int> flipEdges;
	std::map<EdgeInfo,int>::iterator it;
	EdgeInfo ids(0, 0);

	while(mit!=suspicionEdges.end())
	{
		CT::Face_handle f, g;
		int index;
		ids = mit->first;
		Edge edge = mit->second;

		suspicionEdges.erase(mit);//delete the edge from the suspicions

		if(!is_convex(tri, edge) || is_constrained(convexLink, edge))
		{
			mit = suspicionEdges.begin();
			continue;
		}

		it = flipEdges.find(ids);
		if (it!=flipEdges.end())
		{
			if (it->second>5)
			{
				mit = suspicionEdges.begin();
				continue;
			}
			it->second++;
		}
		else
			flipEdges[ids] = 1;

		CT::Vertex_handle v1 = edge.first;
		CT::Vertex_handle v2 = edge.second;
		tri.is_edge(v1, v2, f, index);
		g = f->neighbor(index);
		CT::Vertex_handle v = tri.mirror_vertex(f, index);

#if 0
		face_plane_deviation(f->info());
		face_plane_deviation(g->info());

		double orgCost = (f->info().cost + g->info().cost)/(f->info().weight+g->info().weight+std::numeric_limits<double>::epsilon());
#else
		double orgCost = (f->info().cost + g->info().cost);
#endif
		vector<Point_2> flipFpts(3), flipGpts(3);
		flipFpts[0] = f->vertex(index)->point();
		flipFpts[1] = f->vertex(f->ccw(index))->point();
		flipFpts[2] = v->point();

		flipGpts[0] = f->vertex(f->cw(index))->point();
		flipGpts[1] = f->vertex(index)->point();
		flipGpts[2] = v->point();

		FaceInfo flipFinfo, flipGinfo;
		flipFinfo.triangle = flipFpts;
		flipGinfo.triangle = flipGpts;
#if 0
		FaceInfo comInfo;
		combine_vertices(f, g, comInfo);
		points_on_polygon(flipFpts, flipFinfo, comInfo);
		points_on_polygon(flipGpts, flipGinfo, comInfo);
#else
		//int i = f->mirror_index(index);
		int i = tri.mirror_index(f, index);

		Point_2 p1 = f->vertex(index)->point(), p2 = g->vertex(i)->point();
		Point_2 p3 = f->vertex(f->ccw(index))->point();
		Vector_2 vec1 = p1-p2, vec2 = p3-p1;
		if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>=0)
		{
			for (int i=0; i<f->info().incInfos.size(); i++)
			{
				vec2 = f->info().incInfos[i]->get_domain()-p1;
				if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>=0)
					flipFinfo.incInfos.push_back(f->info().incInfos[i]);
				else
					flipGinfo.incInfos.push_back(f->info().incInfos[i]);
			}

			for (int i=0; i<g->info().incInfos.size(); i++)
			{
				vec2 = g->info().incInfos[i]->get_domain()-p1;
				if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>=0)
					flipFinfo.incInfos.push_back(g->info().incInfos[i]);
				else
					flipGinfo.incInfos.push_back(g->info().incInfos[i]);
			}
		}
		else
		{
			for (int i=0; i<f->info().incInfos.size(); i++)
			{
				vec2 = f->info().incInfos[i]->get_domain()-p1;
				if (vec1.x()*vec2.y()-vec1.y()*vec2.x()<0)
					flipFinfo.incInfos.push_back(f->info().incInfos[i]);
				else
					flipGinfo.incInfos.push_back(f->info().incInfos[i]);
			}

			for (int i=0; i<g->info().incInfos.size(); i++)
			{
				vec2 = g->info().incInfos[i]->get_domain()-p1;
				if (vec1.x()*vec2.y()-vec1.y()*vec2.x()<0)
					flipFinfo.incInfos.push_back(g->info().incInfos[i]);
				else
					flipGinfo.incInfos.push_back(g->info().incInfos[i]);
			}
		}
#endif

		face_plane_deviation(flipFinfo);
		face_plane_deviation(flipGinfo);

#if 0
		double flipCost = (flipFinfo.cost + flipGinfo.cost)/(flipFinfo.weight+flipGinfo.weight+std::numeric_limits<double>::epsilon());
#else
		double flipCost = flipFinfo.cost + flipGinfo.cost;
#endif
		if(orgCost>flipCost)//flippable
		{
			tri.flip(f, index);
			f->info() = flipFinfo;
			f->neighbor(f->ccw(index))->info() = flipGinfo;
			add_suspicions(tri, f, index, suspicionEdges);
		}
		else
		{
			FaceInfo orgFInfo = f->info();
			FaceInfo orgGInfo = g->info();

			vector<NeighbourFaceInfo> neighbourInfos;
			get_neighbour_four_face_info(tri, f, index, neighbourInfos);
			tri.flip(f, index);
			f->info() = flipFinfo;
			f->neighbor(f->ccw(index))->info() = flipGinfo;
			int i = f->ccw(index);
			if (!flip_neighbour_four_edges(tri, f, i, convexLink, orgFInfo, orgGInfo, neighbourInfos, suspicionEdges))
			{
				tri.flip(f, i);
				f->info() = orgGInfo;
				f->neighbor(f->ccw(i))->info() = orgFInfo;
			}
		}
		mit = suspicionEdges.begin();
	}
}

void CKnotsTriangulation::face_plane_deviation(FaceInfo &info)
{
	typedef Mesh::Point Point;

	info.cost = 0;
	info.weight = 0;
	if (info.incInfos.size()<3)
	{
		info.weight = info.incInfos.size();
		return;
	}

	Point_2 p1 = info.incInfos[0]->get_domain();
	Point_2 p2 = info.incInfos[1]->get_domain();
	int k=2;
	for (; k<info.incInfos.size(); k++)
	{
		if (std::abs(CGAL::area(info.incInfos[k]->get_domain(), p1, p2))>std::numeric_limits<double>::epsilon())
			break;
	}
	
	if (k==info.incInfos.size())
	{	
		MatrixXd A;
		MatrixXd b;
		MatrixXd c;
		b = MatrixXd::Zero(info.incInfos.size(), 3);
		A = MatrixXd::Zero(info.incInfos.size(), 2);
		vector<double> ts(info.incInfos.size());
		for (int i=0; i<info.incInfos.size(); i++)
		{
			b(i, 0) = info.incInfos[i]->point().x();
			b(i, 1) = info.incInfos[i]->point().y();
			b(i, 2) = info.incInfos[i]->point().z();
			A(i, 0) = std::sqrt((info.incInfos[i]->get_domain() - p1).squared_length());
			ts[i] = A(i, 0);
			A(i, 1) = 1;
		}
		MatrixXd B = (A.transpose()*A);
		B = B + std::numeric_limits<double>::epsilon()*MatrixXd::Identity(B.rows(), B.cols());
		c = B.inverse()*A.transpose()*b;
		double x, y, z;
		for (int i=0; i<info.incInfos.size(); i++)
		{
			x = ts[i]*c(0, 0) + c(1, 0);
			y = ts[i]*c(0, 1) + c(1, 1);
			z = ts[i]*c(0, 2) + c(1, 2);
			info.cost += (Point(x, y, z) - info.incInfos[i]->point()).squared_length();
		}
		info.weight = info.incInfos.size();
		return;
	}
	else if (info.incInfos.size()==3)
	{
		info.weight = info.incInfos.size();
		return;
	}

	vector<pair<Point, Point_2>> vertcies;
	MatrixXd A;
	MatrixXd b;
	MatrixXd c;
	
	vector<Vertex_iterator>::iterator it = info.incInfos.begin();
	vector<Vertex_iterator>::iterator end = info.incInfos.end();
	for (; it!=end; it++)
		vertcies.push_back(std::make_pair((*it)->point(), (*it)->get_domain()));

	A = MatrixXd::Zero(vertcies.size(), 3);
	b = MatrixXd::Zero(vertcies.size(), 3);
	int row = 0;
	
	for (int i=0; i<vertcies.size(); i++)
	{
		A(row, 0) = vertcies[i].second.x();
		A(row, 1) = vertcies[i].second.y();
		A(row, 2) = 1;
		b(row, 0) = vertcies[i].first.x();
		b(row, 1) = vertcies[i].first.y();
		b(row, 2) = vertcies[i].first.z();
		row++;
	}
	

	MatrixXd B = (A.transpose()*A);
	/*if (abs(B.determinant()) < std::numeric_limits<double>::epsilon())
		B = std::numeric_limits<double>::epsilon()*MatrixXd::Identity(B.rows(), B.cols()) + B;*/

	c = B.inverse()*A.transpose()*b;
	double x, y, z;
	for (int i=0; i<vertcies.size(); i++)
	{
		x = c(0, 0)*vertcies[i].second.x()+c(1, 0)*vertcies[i].second.y() + c(2, 0);
		y = c(0, 1)*vertcies[i].second.x()+c(1, 1)*vertcies[i].second.y() + c(2, 1);
		z = c(0, 2)*vertcies[i].second.x()+c(1, 2)*vertcies[i].second.y() + c(2, 2);
		info.cost += (Point(x, y, z) - vertcies[i].first).squared_length();
	}
	info.weight = vertcies.size();
}


void CKnotsTriangulation::flipped_edge_cost_function(CT &tri, CT::Face_handle &f, int index, double &cost, FaceInfo &fInfo, FaceInfo &gInfo)
{
	CT::Face_handle g = f->neighbor(index);
	//int i = f->mirror_index(index);
	int i = tri.mirror_index(f, index);

	vector<Point_2> fPts(3);
	//assemble the indices of triangles
	fPts[0] = f->vertex(index)->point();
	fPts[1] = f->vertex(f->ccw(index))->point();
	fPts[2] = g->vertex(i)->point();

	vector<Point_2> gPts(3);
	gPts[0] = f->vertex(index)->point();
	gPts[1] = g->vertex(i)->point();
	gPts[2] = f->vertex(f->cw(index))->point();

	Point_2 p1 = f->vertex(index)->point(), p2 = g->vertex(i)->point();
	Point_2 p3 = f->vertex(f->ccw(index))->point();
	Vector_2 vec1 = p1-p2, vec2 = p3-p1;
	if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>1e-15)
	{
		for (int i=0; i<f->info().incInfos.size(); i++)
		{
			vec2 = f->info().incInfos[i]->get_domain()-p1;
			if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>1e-15)
				fInfo.incInfos.push_back(f->info().incInfos[i]);
			else
				gInfo.incInfos.push_back(f->info().incInfos[i]);
		}

		for (int i=0; i<g->info().incInfos.size(); i++)
		{
			vec2 = g->info().incInfos[i]->get_domain()-p1;
			if (vec1.x()*vec2.y()-vec1.y()*vec2.x()>1e-15)
				fInfo.incInfos.push_back(g->info().incInfos[i]);
			else
				gInfo.incInfos.push_back(g->info().incInfos[i]);
		}
	}
	else
	{
		for (int i=0; i<f->info().incInfos.size(); i++)
		{
			vec2 = f->info().incInfos[i]->get_domain()-p1;
			if (vec1.x()*vec2.y()-vec1.y()*vec2.x()<1e-15)
				fInfo.incInfos.push_back(f->info().incInfos[i]);
			else
				gInfo.incInfos.push_back(f->info().incInfos[i]);
		}

		for (int i=0; i<g->info().incInfos.size(); i++)
		{
			vec2 = g->info().incInfos[i]->get_domain()-p1;
			if (vec1.x()*vec2.y()-vec1.y()*vec2.x()<1e-15)
				fInfo.incInfos.push_back(g->info().incInfos[i]);
			else
				gInfo.incInfos.push_back(g->info().incInfos[i]);
		}
	}
	fInfo.triangle = fPts;
	gInfo.triangle = gPts;
	face_plane_deviation(fInfo);
	cost = fInfo.cost;
	double weight = fInfo.weight;
	face_plane_deviation(gInfo);
	cost += gInfo.cost;
	weight += gInfo.weight;
	//cost /= (weight+std::numeric_limits<double>::epsilon());
}

bool CKnotsTriangulation::edge_cost_function(CT::Face_handle &f, int index, double &cost)
{
	typedef Mesh::Vertex_iterator Vertex_iterator;
	CT::Face_handle g = f->neighbor(index);

	cost = f->info().cost + g->info().cost;
	return true;

	face_plane_deviation(f->info());
	face_plane_deviation(g->info());
	cost = (f->info().cost+g->info().cost)
		/(f->info().weight+g->info().weight+std::numeric_limits<double>::epsilon());

	return true;
}

void CKnotsTriangulation::simulated_annealing(CT &tri)
{
	std::map<EdgeInfo, Edge> suspicionEdges;
	add_original_suspicions(tri, suspicionEdges, true);

	std::map<EdgeInfo, Edge>::iterator mit = suspicionEdges.begin();
	vector<double> costs;
	while(mit!=suspicionEdges.end())
	{
		CT::Face_handle f, g;
		int index;
		Edge edge = mit->second;

		if(!is_flippable(tri, edge))
		{
			mit++;
			continue;
		}

		CT::Vertex_handle v1 = edge.first;
		CT::Vertex_handle v2 = edge.second;
		tri.is_edge(v1, v2, f, index);
		g = f->neighbor(index);
		CT::Vertex_handle v = tri.mirror_vertex(f, index);

		double orgCost;
		edge_cost_function(f, index, orgCost);

		double flipCost;
		FaceInfo fInfo, gInfo;

		flipped_edge_cost_function(tri,f, index, flipCost, fInfo, gInfo);

		if(orgCost<=flipCost)//nonflippable
			costs.push_back(flipCost-orgCost);
		mit++;
	}

	std::sort(costs.begin(), costs.end());
	double alpha = costs[costs.size()/2]/std::log(2.0);
	srand(0);
	double prob;
	for (int k=0; k<50; k++)
	{
		for (int l=0; l<10; l++)
		{
			std::map<EdgeInfo, int> flipEdges;
			std::map<EdgeInfo,int>::iterator it;
			EdgeInfo ids(0, 0);
			mit = suspicionEdges.begin();
			while(mit!=suspicionEdges.end())
			{
				CT::Face_handle f, g;
				int index;
				ids = mit->first;
				Edge edge = mit->second;

				suspicionEdges.erase(mit);//delete the edge from the suspicions
				if(!is_flippable(tri, edge))
				{
					mit = suspicionEdges.begin();
					continue;
				}

				it = flipEdges.find(ids);
				if (it!=flipEdges.end())
				{
					if (it->second>5)
					{
						mit = suspicionEdges.begin();
						continue;
					}
					it->second++;
				}
				else
					flipEdges[ids] = 1;

				CT::Vertex_handle v1 = edge.first;
				CT::Vertex_handle v2 = edge.second;
				tri.is_edge(v1, v2, f, index);
				g = f->neighbor(index);
				CT::Vertex_handle v = tri.mirror_vertex(f, index);

				double orgCost;
				edge_cost_function(f, index, orgCost);

				double flipCost;
				FaceInfo fInfo, gInfo;

				flipped_edge_cost_function(tri,f, index, flipCost, fInfo, gInfo);

				if(orgCost>flipCost)//flippable
				{
					tri.flip(f, index);
					f->info() = fInfo;
					f->neighbor(f->ccw(index))->info() = gInfo;
					add_suspicions(tri, f, index, suspicionEdges);
				}
				else
				{
					double difference = flipCost-orgCost;
					double value = std::exp(-difference/alpha);
					prob = (double)rand()/RAND_MAX; 
					//std::cout << __FUNCTION__ << ": " << prob << " " << value << std::endl;
					if (value>prob)
					{
						tri.flip(f, index);
						f->info() = fInfo;
						f->neighbor(f->ccw(index))->info() = gInfo;
						add_suspicions(tri, f, index, suspicionEdges);
					}
				}
				mit = suspicionEdges.begin();
			}
			add_original_suspicions(tri, suspicionEdges, true);
		}
		alpha = 0.95*alpha;
	}
}

void CKnotsTriangulation::lop(CT &tri, int niterations)
{
	std::map<EdgeInfo, Edge> suspicionEdges;
	add_original_suspicions(tri, suspicionEdges, true);

	int ncount = 0;
	std::map<EdgeInfo, Edge>::iterator mit = suspicionEdges.begin();
	double area;
	std::map<EdgeInfo, int> flipEdges;
	std::map<EdgeInfo,int>::iterator it;
	EdgeInfo ids(0, 0);
	while(mit!=suspicionEdges.end())
	{
		CT::Face_handle f, g;
		int index;
		ids = mit->first;
		Edge edge = mit->second;

		suspicionEdges.erase(mit);//delete the edge from the suspicions

		if(!is_flippable(tri, edge))
		{
			mit = suspicionEdges.begin();
			continue;
		}

		it = flipEdges.find(ids);
		if (it!=flipEdges.end())
		{
			if (it->second>5)
			{
				mit = suspicionEdges.begin();
				continue;
			}
			it->second++;
		}
		else
			flipEdges[ids] = 1;

		CT::Vertex_handle v1 = edge.first;
		CT::Vertex_handle v2 = edge.second;
		tri.is_edge(v1, v2, f, index);
		g = f->neighbor(index);
		CT::Vertex_handle v = tri.mirror_vertex(f, index);

		double orgCost;
		edge_cost_function(f, index, orgCost);

		double flipCost;
		FaceInfo fInfo, gInfo;

		flipped_edge_cost_function(tri,f, index, flipCost, fInfo, gInfo);

		if(orgCost>flipCost)//flippable
		{
			tri.flip(f, index);
			f->info() = fInfo;
			f->neighbor(f->ccw(index))->info() = gInfo;
			add_suspicions(tri, f, index, suspicionEdges);
		}
		mit = suspicionEdges.begin();
	}
}

void CKnotsTriangulation::lop(CT &tri, vector<pair<unsigned, unsigned>> &convexLink)
{
	std::map<EdgeInfo, Edge> suspicionEdges;
	add_original_suspicions(tri, suspicionEdges);

	for (CT::Finite_faces_iterator fit= tri.finite_faces_begin(); fit!=tri.finite_faces_end(); fit++)
	{
		vector<Point_2> pts(3);
		for (int i=0; i<3; i++)
			pts[i] = fit->vertex(i)->point();
		fit->info().triangle = pts;
		points_on_polygon(mesh, pts, fit->info());
		face_plane_deviation(fit->info());
	}

	for (CT::Finite_faces_iterator fit= tri.finite_faces_begin(); fit!=tri.finite_faces_end(); fit++)
		for (int i=0; i<fit->info().incInfos.size(); i++)
			fit->info().incInfos[i]->vertex_flag().bStatus = 0;

	int ncount = 0;
	std::map<EdgeInfo, Edge>::iterator mit = suspicionEdges.begin();
	std::map<EdgeInfo, int> flipEdges;
	std::map<EdgeInfo,int>::iterator it;
	EdgeInfo ids(0, 0);
	while(mit!=suspicionEdges.end())
	{
		CT::Face_handle f, g;
		int index;
		ids = mit->first;
		Edge edge = mit->second;

		suspicionEdges.erase(mit);//delete the edge from the suspicions


		if(!is_convex(tri, edge) || is_constrained(convexLink, edge))
		{
			mit = suspicionEdges.begin();
			continue;
		}

		it = flipEdges.find(ids);
		if (it!=flipEdges.end())
		{
			if (it->second>5)
			{
				mit = suspicionEdges.begin();
				continue;
			}
			it->second++;
		}
		else
			flipEdges[ids] = 1;

		CT::Vertex_handle v1 = edge.first;
		CT::Vertex_handle v2 = edge.second;
		tri.is_edge(v1, v2, f, index);
		g = f->neighbor(index);
		CT::Vertex_handle v = tri.mirror_vertex(f, index);

		double orgCost;
		edge_cost_function(f, index, orgCost);

		double flipCost;
		FaceInfo fInfo, gInfo;
		flipped_edge_cost_function(tri,f, index, flipCost, fInfo, gInfo);

		if(orgCost>flipCost)//flippable
		{
			tri.flip(f, index);
			f->info() = fInfo;
			f->neighbor(f->ccw(index))->info() = gInfo;
			add_suspicions(tri, f, index, suspicionEdges);
		}

		mit = suspicionEdges.begin();
	}
}


void CKnotsTriangulation::high_degree_triangulation(vector<vector<TConfig2>> &alltconfigs)
{
	std::cout << __FUNCTION__ << ": " << "ltp degree begin" << std::endl;
	for(int p=1; p<=degree; p++)
	{	
		vector<TConfig2> configs;
		//To get the Vertices of the Configurations Gammak_1
		vector<Link> linksvec;
		get_all_links(alltconfigs[alltconfigs.size()-1], linksvec);
		std::cout << __FUNCTION__ << ": " << "get all link finish" << std::endl;

		vector<vector<TConfig2>> alllink_config;
		alllink_config.resize(linksvec.size());
#pragma omp parallel for
		for (int it = 0; it < linksvec.size(); it++)
		{
#if 0
			///////////test
			bool is_test = false;
			vector<unsigned int> temp_p = {13,14};
			/*Point_2 pt = pDataMaps->at(289).pt;
			std::cout << __FUNCTION__ << ": " << "289:-" << pt.x() << "," << pt.y() << std::endl;*/
			int num_find = 0;
			for (int j = 0; j < temp_p.size(); j++)
			{
				/*list<unsigned int>::iterator res = find(linksvec[it].sortEdges.begin(), linksvec[it].sortEdges.end(), temp_p[j]);
				if (res != linksvec[it].sortEdges.end())
				{
					num_find++;
				}*/
				vector<unsigned int>::iterator res1 = find(linksvec[it].interior.begin(), linksvec[it].interior.end(), temp_p[j]);
				if (res1 != linksvec[it].interior.end())
				{
					num_find++;
				}
			}
			if (num_find >= 2)
			{
				is_test = true;
				std::cout << "inte: ";
				for (int j = 0; j < linksvec[it].interior.size(); j++)
				{
					std::cout << linksvec[it].interior[j] << " - ";
				}
				std::cout << "\n";
				for (auto itj = linksvec[it].sortEdges.begin(); itj != linksvec[it].sortEdges.end(); itj++)
				{
					std::cout << *itj << " - ";
				}
				std::cout << "\n";
			}

			///////////////
#endif
			if (p == 2)
			{
				//std::cout << "ltp tri :" << it <<"-inte: "<< linksvec[it].interior[0] <<"-"<< linksvec[it].interior[1] << std::endl;
			}

			vector<TConfig2> config_now;
			triangulate_configs(linksvec[it], config_now, p);
			alllink_config[it] = config_now;
			
#if 0
			if (is_test)
			{
				for (int i = 0;i<config_now.size();i++)
				{
					for (int j = 0;j<config_now[i].tconfig.size();j++)
					{
						int id_ = config_now[i].tconfig[j];
						if (correspondence->find(id_) != correspondence->end())
							id_ = correspondence->at(id_);
						std::cout << id_ << " ";
					}
					std::cout << "\n";
				}
			}
#endif
		}
//#pragma omp barrier
		for (int k = 0;k<alllink_config.size();k++)
		{
			for (int i = 0;i<alllink_config[k].size();i++)
			{
				configs.push_back(alllink_config[k][i]);
			}
		}
		alltconfigs.push_back(configs);
		std::cout << __FUNCTION__ << ": " << "ltp degree up" << std::endl;
	}
}

void CKnotsTriangulation::triangulate_configs(Link &link, vector<TConfig2> &configs, int p)
{
	if (!is_singular(link))
	{
		CDT tri;
		if (link.sortEdges.size()>3)
		{
			vector<pair<unsigned, unsigned>> convexLink;
			//get_convex_link(*it, convexLink);
			//insert_link(tri, convexLink);
			list<unsigned>::iterator end_ = link.sortEdges.end();
			end_--;
			for (auto ik = link.sortEdges.begin(); ik != end_; ik++)
			{
				CT::Vertex_handle v1 = tri.insert((*pDataMaps)[*ik].pt);
				v1->set_associated_index(*ik);
				v1->set_new_status(false);
				ik++;
				CT::Vertex_handle v2 = tri.insert((*pDataMaps)[*ik].pt);
				v2->set_associated_index(*ik);
				v2->set_new_status(false);
				ik--;
				if (ncollinear_rate > 0) {
					tri.insert_constraint(v1, v2);
				}
			}

			//insert collinear knots constrains
			if (ncollinear_rate > 0) {
				bool is_inte_same = true;
				RuptureKnotConfig config_candidate;
				if (link.interior.size() > 1)
				{
					for (int its = 1; its < link.interior.size(); its++)
					{
						if (link.interior[its] != link.interior[0])
						{
							is_inte_same = false;
							break;
						}
					}
				}
				else
				{
					is_inte_same = false;
				}
				bool do_exist_inte = false;
				if (!is_inte_same)
				{
					vector<unsigned> inte = link.interior;
					for (int t = 0; t < inte.size(); t++)
					{
						map<unsigned, unsigned>::iterator res = correspondence->find(inte[t]);
						if (res != correspondence->end())
						{
							inte[t] = correspondence->at(inte[t]);
						}
					}
					sort(inte.begin(), inte.end());
					for (int k = 0; k < rupture_basis_config.size(); k++)
					{
						vector<unsigned> temp = rupture_basis_config[k].inte;
						sort(temp.begin(), temp.end());
						if (inte.size() == temp.size() && equal(inte.begin(), inte.end(), temp.begin()))
						{
							do_exist_inte = true;
							config_candidate = rupture_basis_config[k];
							break;
						}
					}
				}

				if (do_exist_inte)
				{
					if (config_candidate.constrain.first == config_candidate.inte[0] &&
						config_candidate.constrain.second == config_candidate.inte[1])
					{
						map<int, CDT::Vertex_handle> trihandle;
						for (auto vit = tri.vertices_begin(); vit != tri.vertices_end(); vit++)
						{
							trihandle[vit->get_associated_index()] = vit->handle();
						}
						int k1 = config_candidate.constrain.first;
						int k2 = config_candidate.constrain.second;
						if (k1 != k2) {
							if (trihandle.find(k1) != trihandle.end() && trihandle.find(k2) != trihandle.end()) {
								trihandle[k1]->set_new_status(true);
								trihandle[k2]->set_new_status(true);
								tri.insert_constraint(trihandle[k1], trihandle[k2]);
							}
						}
					}
					else {
						vector<unsigned> two_side;
						two_side.push_back(config_candidate.constrain.first);
						two_side.push_back(config_candidate.constrain.second);
						list<unsigned int>::iterator res1 = find(link.sortEdges.begin(), link.sortEdges.end(), config_candidate.constrain.first);
						list<unsigned int>::iterator res2 = find(link.sortEdges.begin(), link.sortEdges.end(), config_candidate.constrain.second);
						bool do_exist_cornerside = false;
						if (res2 == link.sortEdges.end())
						{
							for (auto ik = link.sortEdges.begin(); ik != end_; ik++)
							{
								map<unsigned, unsigned>::iterator res = correspondence->find(*ik);
								if (res != correspondence->end())
								{
									if (correspondence->at(*ik) == config_candidate.constrain.second)
									{
										do_exist_cornerside = true;
										two_side[1] = *ik;
										break;
									}
								}
							}
						}

						if (res1 != link.sortEdges.end() && (res2 != link.sortEdges.end() || do_exist_cornerside))
						{
							map<int, CDT::Vertex_handle> trihandle;
							for (auto vit = tri.vertices_begin(); vit != tri.vertices_end(); vit++)
							{
								trihandle[vit->get_associated_index()] = vit->handle();
							}
							trihandle[two_side[0]]->set_new_status(true);
							trihandle[two_side[1]]->set_new_status(true);
							tri.insert_constraint(trihandle[two_side[0]], trihandle[two_side[1]]);
						}
					}
				}
			}
			else
			{
				map<int, CDT::Vertex_handle> trihandle;
				for (auto vit = tri.vertices_begin(); vit != tri.vertices_end(); vit++)
				{
					int id_ = vit->get_associated_index();
					if (correspondence->find(id_) != correspondence->end()) {
						id_ = correspondence->at(id_);
					}
					trihandle[id_] = vit->handle();
				}
				for (int m = 0; m < collinear_knots_lines.size(); m++)
				{
					for (int n = 0; n < collinear_knots_lines[m].size() - 1; n++)
					{
						int k1 = collinear_knots_lines[m][n].index;
						int k2 = collinear_knots_lines[m][n+1].index;
						if (trihandle.find(k1) != trihandle.end() && trihandle.find(k2) != trihandle.end()) {
							trihandle[k1]->set_new_status(true);
							trihandle[k2]->set_new_status(true);							
							tri.insert_constraint(trihandle[k1], trihandle[k2]);
						}
					}
				}
			}

			if (triType != DELAUNAY && p != degree)
			{
				if (optAlg == LOP)
					lop(tri, convexLink);
				else if (optAlg == LOOKAHEAD)
					look_ahead(tri, convexLink);
				else
					lop(tri, convexLink);
			}
#if 1
			//remove triangles outside polygon
			Polygon_2 poly_;
			list<unsigned>::iterator end = link.sortEdges.end();
			end--;
			for (auto ik = link.sortEdges.begin(); ik != end; ik++)
			{
				poly_.push_back((*pDataMaps)[*ik].pt);
			}
			end = link.sortEdges.end();
			end--;
			if (*(link.sortEdges.begin()) != *end)
			{
				poly_.push_back((*pDataMaps)[*end].pt);
			}
			//assert(poly_.is_simple());
			if (poly_.is_simple()) {
				//assert(poly_.is_counterclockwise_oriented());
				vector<vector<unsigned>> splittriangles;
				for (auto fit = tri.faces_begin(); fit != tri.faces_end(); fit++)
				{
					Point_2 p0 = fit->vertex(0)->point();
					Point_2 p1 = fit->vertex(1)->point();
					Point_2 p2 = fit->vertex(2)->point();

					unsigned index0 = fit->vertex(0)->get_associated_index();
					unsigned index1 = fit->vertex(1)->get_associated_index();
					unsigned index2 = fit->vertex(2)->get_associated_index();
					vector<unsigned> triangle = { index0,index1,index2 };
					splittriangles.push_back(triangle);

					double w1 = 0.495, w2 = 0.01;
					Point_2 pt1((w1 * p0.x() + w1 * p1.x() + w2 * p2.x()), (w1 * p0.y() + w1 * p1.y() + w2 * p2.y()));
					Point_2 pt2((w1 * p0.x() + w2 * p1.x() + w1 * p2.x()), (w1 * p0.y() + w2 * p1.y() + w1 * p2.y()));
					Point_2 pt3((w2 * p0.x() + w1 * p1.x() + w1 * p2.x()), (w2 * p0.y() + w1 * p1.y() + w1 * p2.y()));
					//if (!(abs(area(p0, p1, p2)) < 1e-12))
					{
						if (poly_.bounded_side(pt1) == CGAL::ON_UNBOUNDED_SIDE ||
							poly_.bounded_side(pt2) == CGAL::ON_UNBOUNDED_SIDE ||
							poly_.bounded_side(pt3) == CGAL::ON_UNBOUNDED_SIDE /*||
							poly_.bounded_side(pt4) == CGAL::ON_UNBOUNDED_SIDE ||
							poly_.bounded_side(pt5) == CGAL::ON_UNBOUNDED_SIDE ||
							poly_.bounded_side(pt6) == CGAL::ON_UNBOUNDED_SIDE*/)
						{
							//std::cout << "remove triangle during LTP: " << link.interior[0] << "-" << link.interior[1] << std::endl;
							tri.delete_face(fit);
						}
					}
				}

				//detect if some vertices remove from the former steps
				vector<unsigned> lost_ver;
				for (auto ik = link.sortEdges.begin(); ik != link.sortEdges.end(); ik++)
				{
					bool is_exist = false;
					for (int k = 0; k < splittriangles.size(); k++)
					{
						if (splittriangles[k][0] == (*ik) || splittriangles[k][1] == (*ik) || splittriangles[k][2] == (*ik))
						{
							is_exist = true;
							break;
						}
					}
					if (!is_exist)
					{
						std::cout << __FUNCTION__ << ": " << "knot lost during LTP: " << link.interior[0] << "-" << link.interior[1] << std::endl;
						lost_ver.push_back(*ik);
					}
				}
			}
#endif

		}
		else
		{
			list<unsigned>::iterator se_it = link.sortEdges.begin();
			for (; se_it != link.sortEdges.end(); se_it++)
			{
				CDT::Vertex_handle v1 = tri.insert((*pDataMaps)[*se_it].pt);
				v1->set_associated_index(*se_it);
			}
		}
		get_configs(tri, link, configs, p);
	}
}

void CKnotsTriangulation::get_configs(CT &tri, Link &link, vector<TConfig2> &configs, int deg)
{
	for (CT::Finite_faces_iterator fit=tri.finite_faces_begin(); fit!=tri.finite_faces_end(); ++fit)
	{
		TConfig2 config2;
		TConfig1 config(3+deg);
		//vector<unsigned> boundary(3);
		for (int i=0; i<3; i++)
			config[i] = fit->vertex(i)->get_associated_index();
		
		vector<unsigned> indices;
		for (int i=0; i<3; i++)
		{
			int index = 0;
			list<unsigned>::iterator it = link.sortEdges.begin();
			for (; it!=link.sortEdges.end(); it++)
			{
				if (*it==config[i])
				{
					indices.push_back(index);
					break;
				}
				index++;
			}
		}
		if (indices.size()<3)
			continue;

		int sign = (indices[1]-indices[0])*(indices[2]-indices[1])*(indices[0]-indices[2]);

		if (sign<0)
		{
			for (int i=0; i<link.interior.size(); i++)
				config[3+i] = link.interior[i];
			config2.tconfig = config;
			configs.push_back(config2);
		}
	}
}

void CKnotsTriangulation::get_all_links(vector<TConfig2> &configs, vector<Link> &allLinks)
{
	//Collect all the edges of every interiors
	vector<TConfig2>::iterator it = configs.begin();
	for (; it!=configs.end(); it++)
	{
		int deg = it->tconfig.size()-BOUNDARYSIZE;
		for (int j=0; j<BOUNDARYSIZE; j++)
		{
			vector<unsigned> interior(deg+1);
			int k=BOUNDARYSIZE;
			for (; k<it->tconfig.size(); k++)
				interior[k-BOUNDARYSIZE] = (*it).tconfig[k];

			interior[k-BOUNDARYSIZE] = (*it).tconfig[j];
			sort(interior.begin(), interior.end());

#if 0
			///////////test
			bool is_test = false;
			vector<unsigned int> temp_p = { 181,304 };
			vector<unsigned int> inte = interior;
			sort(inte.begin(), inte.end());
			if (std::equal(inte.begin(), inte.end(), temp_p.begin()))
			{
				int te = 1;
			}
			///////////////
#endif

			//Judge whether the interior exists in allLinks, if exist, return the index, otherwise -1
			vector<Link>::iterator aIt = find_links(allLinks, interior);

			if (aIt==allLinks.end())//not finded
			{
				Link link;
				link.interior = interior;
				link.orgEdges.push_back(std::make_pair((*it).tconfig[(j+1)%3], (*it).tconfig[(j+2)%3]));
				allLinks.push_back(link);
			}
			else//finded
			{
				aIt->orgEdges.push_back(std::make_pair((*it).tconfig[(j+1)%3], (*it).tconfig[(j+2)%3]));
			}
		}
	}
	//std::cout << "get all link collect" << std::endl;

	//Sort the links to make them link in order
	for (int lIt = 0; lIt<allLinks.size(); lIt++)
	{
#if 0
		///////////test
		vector<unsigned int> temp_p = { 181,304 };
		if (std::equal(allLinks[lIt].interior.begin(), allLinks[lIt].interior.end(),temp_p.begin()))
		{
			int te = 1;
		}
		///////////////
#endif

		//std::cout << "all link:number " << lIt <<"-";
		if (0)
		{
			std::cout << "inte: ";
			for (int m = 0;m<allLinks[lIt].interior.size();m++)
			{
				std::cout << allLinks[lIt].interior[m] << "-";
			}
			std::cout << "end \n";

			std::cout << "edge: ";
			for (auto mi = allLinks[lIt].orgEdges.begin(); mi != allLinks[lIt].orgEdges.end(); mi++)
			{
				std::cout << mi->first << "-"<<mi->second<<", ";
			}
			std::cout << "end \n";
		}


		allLinks[lIt].edges = allLinks[lIt].orgEdges;
		if (allLinks[lIt].edges.size()<=1)
			continue;
		list<pair<unsigned, unsigned>> edges = allLinks[lIt].edges;
		list<pair<unsigned, unsigned>>::iterator iter = edges.begin();

		//Judge whether the links contain isolated edge, if do, erase it
		for (; iter!=edges.end(); )
		{
			bool isIsolate = true;
			list<pair<unsigned, unsigned>>::iterator initer = edges.begin();
			for (; initer!=edges.end(); initer++)
			{
				if (iter->first==initer->second || iter->second==initer->first)
				{
					isIsolate = false;
					break;
				}
			}
			if (isIsolate)
			{
				iter = edges.erase(iter);
			}
			else
				iter++;
		}

		//Judge whether the links contain opposite edges, if do, erase them
		list<pair<unsigned, unsigned>> temp;
		for (iter = edges.begin(); iter!=edges.end(); iter++)
		{
			bool isOpposite = false;
			list<pair<unsigned, unsigned>>::iterator initer = edges.begin();
			for (; initer!=edges.end(); initer++)
			{
				if (iter->first==initer->second && iter->second==initer->first)
				{
					isOpposite = true;
					break;
				}
			}

			if (!isOpposite)
			{
				temp.push_back(*iter);
			}
		}
		edges = temp;
		allLinks[lIt].edges = edges;//To update the links, this links erase the opposite edges and isolated edges
		if(edges.empty())
			continue;

		iter = edges.begin();
		allLinks[lIt].sortEdges.push_back(iter->first);
		allLinks[lIt].sortEdges.push_back(iter->second);

		iter = edges.erase(iter);
		int index = 0;
		while(!edges.empty())
		{
			if (iter==edges.end())
			{
				iter=edges.begin();
			}

			list<unsigned>::iterator start = allLinks[lIt].sortEdges.begin();
			list<unsigned>::iterator last = allLinks[lIt].sortEdges.end();
			last--;
			if (*start==*last)
			{
				break;
			}

			if (*start==iter->second)
			{
				allLinks[lIt].sortEdges.insert(start, iter->first);
				iter = edges.erase(iter);
				continue;
			}
			last = allLinks[lIt].sortEdges.end();
			last--;
			if (*last==iter->first)
			{
				allLinks[lIt].sortEdges.push_back(iter->second);
				iter = edges.erase(iter);
				continue;
			}
			iter++;
			index++;

			if (index>100*allLinks[lIt].sortEdges.size())
			{
				std::cout << "wrong: " << std::endl;
				std::cout << "inte: ";
				for (int m = 0; m < allLinks[lIt].interior.size(); m++)
				{
					std::cout << allLinks[lIt].interior[m] << "-";
				}
				std::cout << "end \n";

				std::cout << "edge: ";
				for (auto mi = allLinks[lIt].orgEdges.begin(); mi != allLinks[lIt].orgEdges.end(); mi++)
				{
					std::cout << mi->first << "-" << mi->second << ", ";
				}
				std::cout << "end \n";
				break;
			}
		}
		if (!edges.empty())
		{
			list<pair<unsigned, unsigned>>::iterator iter = edges.begin();
			for (; iter!=edges.end(); iter++)
			{
				list<pair<unsigned, unsigned>>::iterator initer = allLinks[lIt].edges.begin();
				for (; initer!=allLinks[lIt].edges.end(); initer++)
				{
					if (initer->first == iter->first && initer->second == iter->second)
					{
						allLinks[lIt].edges.erase(initer);
						break;
					}
				}
			}

		}

		//std::cout << "end" << std::endl;


		//corner
		bool is_insert_corner = true;
		for (int i = 0; i < allLinks[lIt].interior.size(); i++)
		{
			if (!is_on_domain_corner(pDataMaps->at(allLinks[lIt].interior[i]).pt))
			{
				is_insert_corner = false;
			}
		}
		if (is_insert_corner)
		{
			allLinks[lIt].sortEdges.push_back(allLinks[lIt].interior[0]);
		}
	}
}

void  CKnotsTriangulation::add_original_suspicions(CT &tri, std::map<EdgeInfo, Edge> &suspicionEdges, bool flag)
{
	for(CT::Finite_edges_iterator eit=tri.finite_edges_begin(); eit!=tri.finite_edges_end(); eit++)
	{
		CT::Face_handle f = eit->first;
		int index = eit->second;
		if(tri.is_infinite(f))
			continue;

		CT::Face_handle g = f->neighbor(index);
		if (g==NULL)
			continue;

		if(tri.is_infinite(g))
			continue;

		if(flag)
		{
			if (!f->vertex(f->ccw(index))->get_new_status() && !f->vertex(f->cw(index))->get_new_status()
				&& !f->vertex(index)->get_new_status())
				continue;
		}

		Edge e = std::make_pair(f->vertex(f->ccw(index)), f->vertex(f->cw(index)));

		if(!is_convex(tri, e) || is_constrained(tri, e)/* && !(e.first->get_new_status()&& e.second->get_new_status())*/)
			continue;

		CT::Vertex_handle v1 = f->vertex(f->ccw(index));
		CT::Vertex_handle v2 = f->vertex(f->cw(index));
		unsigned id1 = v1->get_associated_index();
		unsigned id2 = v2->get_associated_index();
		unsigned minId = id1, maxId = id2;
		if(id1>id2)
		{
			minId = id2;
			maxId = id1;
		}
		EdgeInfo id(minId, maxId);
		suspicionEdges[id] = std::make_pair(v1, v2);
	}
}

void CKnotsTriangulation::add_suspicions(CT &tri, CT::Face_handle &f, int index, std::map<EdgeInfo, Edge> & suspicionEdges)
{
	CT::Face_handle g = f->neighbor(f->ccw(index));
	int i = tri.mirror_index(f, f->ccw(index));
	Edge e = std::make_pair(f->vertex(index), f->vertex(f->ccw(index)));

	if(is_finite(tri, e) && is_convex(tri, e) && !is_constrained(tri, e) && !(e.first->get_associated_index() && e.second->get_new_status()))
		add_suspicions(tri, e, suspicionEdges);

	e = std::make_pair(f->vertex(f->ccw(index)), f->vertex(f->cw(index)));
	if(is_finite(tri, e) && is_convex(tri, e) && !is_constrained(tri, e) && !(e.first->get_associated_index() && e.second->get_new_status()))
		add_suspicions(tri, e, suspicionEdges);

	e = std::make_pair(g->vertex(g->cw(i)), g->vertex(i));
	if(is_finite(tri, e) && is_convex(tri, e) && !is_constrained(tri, e) && !(e.first->get_associated_index() && e.second->get_new_status()))
		add_suspicions(tri, e, suspicionEdges);

	e = std::make_pair(g->vertex(i), g->vertex(g->ccw(i)));
	if(is_finite(tri, e) && is_convex(tri, e) && !is_constrained(tri, e) && !(e.first->get_associated_index()&&e.second->get_new_status()))
		add_suspicions(tri, e, suspicionEdges);
}

void CKnotsTriangulation::add_suspicions(CT &tri, Edge &e, std::map<EdgeInfo, Edge> &suspicionEdges)
{
	CT::Vertex_handle v1 = e.first;
	CT::Vertex_handle v2 = e.second;
	unsigned id1 = v1->get_associated_index();
	unsigned id2 = v2->get_associated_index();
	unsigned minId = id1;
	unsigned maxId = id2;
	if(id1>id2)
	{
		minId = id2;
		maxId = id1;
	}
	EdgeInfo id(minId, maxId);
	std::map<EdgeInfo, Edge>::iterator it = suspicionEdges.find(id);
	if(it==suspicionEdges.end())
	{
		suspicionEdges[id] = std::make_pair(v1, v2);
	}
	else
	{
		Edge edge = it->second;
		if(!is_convex(tri, edge))
			suspicionEdges.erase(it);
	}
}

bool CKnotsTriangulation::is_flippable(CT &tri, Edge &e)
{
	return (is_finite(tri, e) && is_convex(tri, e) && !is_constrained(tri, e));
}

bool CKnotsTriangulation::is_convex(CT &tri, Edge &e)//strict convex
{
	CT::Vertex_handle v1 = e.first;
	CT::Vertex_handle v2 = e.second;
	CT::Face_handle f;
	int index;

	if(!tri.is_edge(v1, v2, f, index))
		return false;

	CT::Vertex_handle v = tri.mirror_vertex(f, index);
	
	Point_2 pts[] = {pDataMaps->at(f->vertex(index)->get_associated_index()).pt,
		pDataMaps->at(f->vertex(f->ccw(index))->get_associated_index()).pt,
		pDataMaps->at(v->get_associated_index()).pt,
		pDataMaps->at(f->vertex(f->cw(index))->get_associated_index()).pt};

	//test
	//int e1 = e.first->get_associated_index();
	//int e2 = e.second->get_associated_index();
	//int in1 = f->vertex(index)->get_associated_index();
	//int in2 = f->vertex(f->ccw(index))->get_associated_index();
	//int in3 = v->get_associated_index();
	//int in4 = f->vertex(f->cw(index))->get_associated_index();

	//double area = std::abs(CGAL::area(pts[0], pts[1], pts[2]));
	double area = std::abs(CGAL::area(pts[0], pts[1], pts[2]));
	if (area<=1e-12)// || std::abs(area)<std::numeric_limits<double>::epsilon())
		return false;

	//area = std::abs(CGAL::area(pts[0], pts[2], pts[3]));
	area = std::abs(CGAL::area(pts[0], pts[2], pts[3]));
	if (area<=1e-12)// || std::abs(area)<std::numeric_limits<double>::epsilon())
		return false;

	vector<Point_2> out;
	CGAL::convex_hull_2(pts, pts+4,  back_inserter(out));
	if(out.size()<4)
		return false;
	return true;
}

bool CKnotsTriangulation::is_finite(CT &tri, Edge &e)
{
	CT::Vertex_handle v1 = e.first;
	CT::Vertex_handle v2 = e.second;
	CT::Face_handle f;
	int index;

	tri.is_edge(v1, v2, f, index);

	if (f==NULL)
		return false;

	if(tri.is_infinite(f))
		return false;

	CT::Face_handle g = f->neighbor(index);

	if (g==NULL)
		return false;

	if(tri.is_infinite(g))
		return false;
	return true;
}

bool CKnotsTriangulation::is_constrained(vector<pair<unsigned, unsigned>> &convexLink, Edge &e)
{
	unsigned first = e.first->get_associated_index();
	unsigned second = e.second->get_associated_index();
	bool isCons = false;
	for (int i=0; i<convexLink.size(); i++)
	{
		if ( (convexLink[i].first==first && convexLink[i].second==second )|| 
			 (convexLink[i].second==first && convexLink[i].first==second ) || 
			 (e.first->get_new_status() == 1 && e.second->get_new_status() == 1) )
		{
			isCons = true;
			break;
		}
	}
	return isCons;
}

bool CKnotsTriangulation::is_constrained(CT &tri, CT::Finite_edges_iterator &e)
{
	CT::Face_handle f = e->first;
	int index = e->second;

	return tri.is_constrained(std::make_pair(f, index));
}


void CKnotsTriangulation::get_convex_link(Link &link, vector<pair<unsigned, unsigned>> &convexLink)
{
	list<unsigned> edges = link.sortEdges;
	list<unsigned>::iterator it = edges.begin();
	list<unsigned>::iterator end = edges.end();
	end--;
	vector<Point_2> srcPts;
	vector<unsigned> indices;
	for (; it!=end; it++)
	{
		srcPts.push_back((*pDataMaps)[*it].pt);
		indices.push_back(*it);
	}
	it = edges.begin();
	if (*it!=*end)
	{
		srcPts.push_back((*pDataMaps)[*end].pt);
		indices.push_back(*end);
	}
	vector<Point_2> convexHull;
	CGAL::convex_hull_2(srcPts.begin(), srcPts.end(), back_inserter(convexHull));

	vector<unsigned> convexIndices;
	for (int i=0; i<convexHull.size(); i++)
	{
		for (int j=0; j<indices.size(); j++)
		{
			if ((convexHull[i]-(*pDataMaps)[indices[j]].pt).squared_length()<std::numeric_limits<double>::epsilon())
			{
				convexIndices.push_back(indices[j]);
				break;
			}

		}
	}
	int sz = convexIndices.size();
	for (int i=0; i<convexIndices.size(); i++)
	{
		convexLink.push_back(std::make_pair(convexIndices[i], convexIndices[(i+1)%sz]));
	}

	list<pair<unsigned, unsigned>>::iterator eit = link.edges.begin();
	list<pair<unsigned, unsigned>>::iterator eend = link.edges.end();
	sz = convexLink.size();
	for (; eit!=eend; eit++)
	{
		bool bExist = false;
		for(int i=0; i<sz; i++)
		{
			if (eit->first==convexLink[i].first && eit->second==convexLink[i].second)
			{
				bExist =true;
				break;
			}
			if (eit->first==convexLink[i].second && eit->second==convexLink[i].first)
			{
				bExist =true;
				break;
			}
		}
		if (!bExist)
			convexLink.push_back(*eit);
	}
}

void CKnotsTriangulation::insert_link(CDT &tri, vector<pair<unsigned, unsigned>> &convexLink)
{
	for (int i=0; i<convexLink.size(); i++)
	{
		CT::Vertex_handle v1 = tri.insert((*pDataMaps)[convexLink[i].first].pt);
		v1->set_associated_index(convexLink[i].first);
		CT::Vertex_handle v2 = tri.insert((*pDataMaps)[convexLink[i].second].pt);
		v2->set_associated_index(convexLink[i].second);
		tri.insert_constraint(v1, v2);
	}
}

bool CKnotsTriangulation::is_constrained(CT &tri, Edge &e)
{
	CT::Vertex_handle v1 = e.first;
	CT::Vertex_handle v2 = e.second;
	CT::Face_handle f;
	int index;

	tri.is_edge(v1, v2, f, index);

	return tri.is_constrained(std::make_pair(f, index));
}

bool CKnotsTriangulation::is_corners(Edge &e)
{
	CT::Vertex_handle v1 = e.first;
	CT::Vertex_handle v2 = e.second;
	if ((*pDataMaps)[v1->get_associated_index()].flag.bCorner || 
		(*pDataMaps)[v1->get_associated_index()].flag.bMulti)
		return true;
	if ((*pDataMaps)[v2->get_associated_index()].flag.bCorner || 
		(*pDataMaps)[v2->get_associated_index()].flag.bMulti)
		return true;

	return false;
}