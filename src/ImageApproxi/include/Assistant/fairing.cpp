#include "Assistant/fairing.h"

CFairing::CFairing(const std::map<unsigned, KnotData> knotmap, BSplineBasis *splinebasis, vector<TConfig3>*alltconfig3s, QString filename)
{
	OutFileName = filename;
	dataMaps = knotmap;
	bSplineBasis = splinebasis;

	std::copy(alltconfig3s->begin(), alltconfig3s->end(), std::back_inserter(allconfigs));

	u_direction[0] = 1.0; u_direction[1] = 0.0;
	v_direction[0] = 0.0; v_direction[1] = 1.0;

	nquadrature_sample = 2;

	bhas_compute = false;
}

CFairing::CFairing()
{

}

CFairing::~CFairing()
{

}


#if 0
double * CFairing::testFunction(int deg)
{
	double *quadrature_value = new double[4];
	quadrature_value[0] = 0.0;
	quadrature_value[1] = 0.0;
	quadrature_value[2] = 0.0;
	quadrature_value[3] = 0.0;
	Mesh::Facet_iterator f_it;
	for (f_it = fairingMesh->facets_begin(); f_it != fairingMesh->facets_end(); f_it++)
	{
		Mesh::Vertex_handle p1, p2, p3;
		Mesh::Halfedge_around_facet_circulator h = f_it->facet_begin();
		p1 = h->vertex(); h++; p2 = h->vertex(); h++; p3 = h->vertex();

		/*int num_knots = 12;
		double rnodes[24];
		double weights[12];
		triasymq(6,v1,v2,v3,rnodes,weights,num_knots);*/

		vector<Point_2> knots;
		vector<double> weights;
		int num_knots;
		dunavant_quadrature_rule(deg, p1->get_domain(), p2->get_domain(), p3->get_domain(), knots, weights, num_knots);

		for (int i = 0; i < num_knots; i++)
		{
			quadrature_value[0] += 13.0 * (pow(knots[i].x(), 4) + pow(knots[i].y(), 4)) * weights[i];
			quadrature_value[1] += pow(5, knots[i].x() + knots[i].y()) * weights[i];
			if (knots[i].x() > 0.5)
			{
				quadrature_value[2] += 3.0 *weights[i];
			}
			else
			{
				quadrature_value[2] -= weights[i];
			}
			quadrature_value[3] += exp(knots[i].x() + knots[i].y()) * weights[i];
		}
	}
	return quadrature_value;
}
#endif

SpMat & CFairing::get_Bx()
{
	return matrix_u_basis;
}

SpMat & CFairing::get_By()
{
	return matrix_v_basis;
}

SpMat & CFairing::get_Bxx()
{
	return matrix_uu_basis;
}

SpMat & CFairing::get_Bxy()
{
	return matrix_uv_basis;
}

SpMat & CFairing::get_Byy()
{
	return matrix_vv_basis;
}

bool &CFairing::has_compute()
{
	return bhas_compute;
}

void CFairing::compute_fairing_matrix()
{
	generate_output_dir();
	compute_simplex_derivate_innerproduct();
	compute_basis_derivate_innerproduct();
}

void CFairing::test_quadrature_precision(vector<int> simplex1, vector<int> simplex2)
{
	vector<Point_2> simplex_i, simplex_j;
	vector<Point2withIndex> simplex_in, simplex_jn;
	vector<Point_2> support_i, support_j;
	for (int i = 0; i < simplex1.size(); i++)
	{
		simplex_i.push_back(dataMaps[simplex1[i]].pt);
		simplex_j.push_back(dataMaps[simplex2[i]].pt);
		Point2withIndex pi, pj;
		pi.index = simplex1[i];
		pi.point2 = dataMaps[simplex1[i]].pt;
		simplex_in.push_back(pi);
		pj.index = simplex2[i];
		pj.point2 = dataMaps[simplex2[i]].pt;
		simplex_jn.push_back(pj);
	}
	CGAL::convex_hull_2(simplex_i.begin(), simplex_i.end(), back_inserter(support_i));
	CGAL::convex_hull_2(simplex_j.begin(), simplex_j.end(), back_inserter(support_j));

	Polygon_e2 polygon_i, polygon_j;
	for (int k = 0; k < support_i.size(); k++)
	{
		double x = support_i[k].x();
		double y = support_i[k].y();
		polygon_i.push_back(Point_e2(x, y));
	}
	for (int k = 0; k < support_j.size(); k++)
	{
		double x = support_j[k].x();
		double y = support_j[k].y();
		polygon_j.push_back(Point_e2(x, y));
	}

	if (!CGAL::do_intersect(polygon_i, polygon_j))
	{
		return;
	}

	//perturbation
	vector<Polygon_with_holes_e2>	intR;
	CGAL::intersection(polygon_i, polygon_j, std::back_inserter(intR));
	Polygon_e2 polygon_intersection;
	if (!intR[0].is_unbounded())
	{
		polygon_intersection = intR[0].outer_boundary();
	}
	if (polygon_intersection.area() < 1e-16)
	{
		return;
	}

	//triangulate intersection region
	vector<vector<Point_2>> facets;
	vector<Point_2> support_intersect;
	intersect_two_ccw_region(support_i, support_j, support_intersect);
	triangulate_two_simplex_support(simplex_in, simplex_jn, facets, support_intersect);

	for (int d = 1; d<6; d++)
	{
		//quadrature
		vector<Point_2>	quadrature_knots;
		vector<double>	quadrature_weights;

		compute_quadrature_knots_and_weights(d, facets, quadrature_knots, quadrature_weights);

		vector<DomainValue> values_i, values_j;
		compute_simplex_spline(simplex_i, quadrature_knots, values_i);
		compute_simplex_spline(simplex_j, quadrature_knots, values_j);

		double quad_value_dx = 0.0, quad_value_dy = 0.0, quad_value_dxx = 0.0, quad_value_dxy = 0.0, quad_value_dyy = 0.0;
		for (int k = 0; k < quadrature_knots.size(); k++)
		{
			quad_value_dx += values_i[k].dx * values_j[k].dx * quadrature_weights[k];
			quad_value_dy += values_i[k].dy * values_j[k].dy * quadrature_weights[k];
			quad_value_dxx += values_i[k].dxx * values_j[k].dxx * quadrature_weights[k];
			quad_value_dxy += values_i[k].dxy * values_j[k].dxy * quadrature_weights[k];
			quad_value_dyy += values_i[k].dyy * values_j[k].dyy * quadrature_weights[k];
		}
		std::cout << "dx: " << quad_value_dx << " ";
		std::cout << "dy: " << quad_value_dy << " ";
		std::cout << "dxx: " << quad_value_dxx << " ";
		std::cout << "dxy: " << quad_value_dxy << " ";
		std::cout << "dyy: " << quad_value_dyy << "\n";
	}
}

void CFairing::test_polygon_trigulation(vector<vector<Point_2>>& convex1, vector<vector<Point_2>>& convex2, vector<vector<vector<Point_2>>>& trifacets)
{
	std::cout << "test polygon triangulation begin"<< "\n";
	convex1.clear();
	convex2.clear();
	trifacets.clear();

	int numcompile = 0;
	for (int it = 0; it <allconfigs.size(); it++)
	{
		for (int jt = 0; jt <allconfigs.size(); jt++)
		{
			if (it > jt)
			{
				continue;
			}

			vector<Point_2> simplex_i, simplex_j;
			vector<Point2withIndex> simplex_in, simplex_jn;
			vector<Point_2> support_i, support_j;
			for (int i = 0; i < allconfigs[it].tconfig.size(); i++)
			{
				simplex_i.push_back(dataMaps[allconfigs[it].tconfig[i]].pt);
				simplex_j.push_back(dataMaps[allconfigs[jt].tconfig[i]].pt);
				Point2withIndex pi, pj;
				pi.index = allconfigs[it].tconfig[i];
				pi.point2 = dataMaps[allconfigs[it].tconfig[i]].pt;
				simplex_in.push_back(pi);
				pj.index = allconfigs[jt].tconfig[i];
				pj.point2 = dataMaps[allconfigs[jt].tconfig[i]].pt;
				simplex_jn.push_back(pj);
			}
			CGAL::convex_hull_2(simplex_i.begin(), simplex_i.end(), back_inserter(support_i));
			CGAL::convex_hull_2(simplex_j.begin(), simplex_j.end(), back_inserter(support_j));

			Polygon_e2 polygon_i, polygon_j;
			for (int k = 0; k < support_i.size(); k++)
			{
				double x = support_i[k].x();
				double y = support_i[k].y();
				polygon_i.push_back(Point_e2(x, y));
			}
			for (int k = 0; k < support_j.size(); k++)
			{
				double x = support_j[k].x();
				double y = support_j[k].y();
				polygon_j.push_back(Point_e2(x, y));
			}

			if (!CGAL::do_intersect(polygon_i, polygon_j))
			{
				continue;
			}

			//perturbation
			vector<Polygon_with_holes_e2>	intR;
			CGAL::intersection(polygon_i, polygon_j, std::back_inserter(intR));
			Polygon_e2 polygon_intersection;
			if (!intR[0].is_unbounded())
			{
				polygon_intersection = intR[0].outer_boundary();
			}
			if (polygon_intersection.area() < 1e-16)
			{
				continue;
			}

			//triangulate intersection region
			vector<vector<Point_2>> facets;
			vector<Point_2> support_intersect;
			intersect_two_ccw_region(support_i, support_j, support_intersect);
			triangulate_two_simplex_support(simplex_in, simplex_jn, facets, support_intersect);

			convex1.push_back(support_i);
			convex2.push_back(support_j);
			trifacets.push_back(facets);

			numcompile++;
			if (numcompile % 500 == 0)
			{
				std::cout << "number: " << numcompile << "\n";
				//return;
			}

			//quadrature
			vector<Point_2>	quadrature_knots;
			vector<double>	quadrature_weights;
			vector<DomainValue> values_i, values_j;
			compute_quadrature_knots_and_weights(2, facets, quadrature_knots, quadrature_weights);
			compute_simplex_spline(simplex_i, quadrature_knots, values_i);
			compute_simplex_spline(simplex_j, quadrature_knots, values_j);

			double quad_value_dxd2 = 0.0, quad_value_dyd2 = 0.0, quad_value_dxxd2 = 0.0, quad_value_dxyd2 = 0.0, quad_value_dyyd2 = 0.0;
			for (int k = 0; k < quadrature_knots.size(); k++)
			{
				quad_value_dxd2 += values_i[k].dx * values_j[k].dx * quadrature_weights[k];
				quad_value_dyd2 += values_i[k].dy * values_j[k].dy * quadrature_weights[k];
				quad_value_dxxd2 += values_i[k].dxx * values_j[k].dxx * quadrature_weights[k];
				quad_value_dxyd2 += values_i[k].dxy * values_j[k].dxy * quadrature_weights[k];
				quad_value_dyyd2 += values_i[k].dyy * values_j[k].dyy * quadrature_weights[k];
			}
			quadrature_weights.clear();
			quadrature_knots.clear();
			values_i.clear(); values_j.clear();

			compute_quadrature_knots_and_weights(3, facets, quadrature_knots, quadrature_weights);
			compute_simplex_spline(simplex_i, quadrature_knots, values_i);
			compute_simplex_spline(simplex_j, quadrature_knots, values_j);

			double quad_value_dxd3 = 0.0, quad_value_dyd3 = 0.0, quad_value_dxxd3 = 0.0, quad_value_dxyd3 = 0.0, quad_value_dyyd3 = 0.0;
			for (int k = 0; k < quadrature_knots.size(); k++)
			{
				quad_value_dxd3 += values_i[k].dx * values_j[k].dx * quadrature_weights[k];
				quad_value_dyd3 += values_i[k].dy * values_j[k].dy * quadrature_weights[k];
				quad_value_dxxd3 += values_i[k].dxx * values_j[k].dxx * quadrature_weights[k];
				quad_value_dxyd3 += values_i[k].dxy * values_j[k].dxy * quadrature_weights[k];
				quad_value_dyyd3 += values_i[k].dyy * values_j[k].dyy * quadrature_weights[k];
			}
			if (abs(quad_value_dxd2 - quad_value_dxd3) / abs(quad_value_dxd3) > 1e-5 && !(abs(quad_value_dxd2)< 1e-10 && abs(quad_value_dxd3) <1e-10) ||
				abs(quad_value_dyd2 - quad_value_dyd3) / abs(quad_value_dyd3) > 1e-5 && !(abs(quad_value_dyd2)< 1e-10 && abs(quad_value_dyd3) <1e-10) ||
				abs(quad_value_dxxd2 - quad_value_dxxd3) / abs(quad_value_dxxd3) > 1e-5 && !(abs(quad_value_dxxd2)< 1e-10 && abs(quad_value_dxxd3) <1e-10)||
				abs(quad_value_dxyd2 - quad_value_dxyd3) / abs(quad_value_dxyd3) > 1e-5 && !(abs(quad_value_dxyd2)< 1e-10 && abs(quad_value_dxyd3) <1e-10) ||
				abs(quad_value_dyyd2 - quad_value_dyyd3) / abs(quad_value_dyyd3) > 1e-5 && !(abs(quad_value_dyyd2)< 1e-10 && abs(quad_value_dyyd3) <1e-10) )
			{
				std::cout << "number: " << numcompile << "\n";
				std::cout << "dx: " << quad_value_dxd2 << " " << quad_value_dxd3 << "\n";
				std::cout << "dy: " << quad_value_dyd2 << " " << quad_value_dyd3 << "\n";
				std::cout << "dxx: " << quad_value_dxxd2 << " " << quad_value_dxxd3 << "\n";
				std::cout << "dxy: " << quad_value_dxyd2 << " " << quad_value_dxyd3 << "\n";
				std::cout << "dyy: " << quad_value_dyyd2 << " " << quad_value_dyyd3 << "\n\n";
			}

		}
	}
}

void CFairing::compute_simplex_derivate_innerproduct()
{
	std::cout << "all simplex config: " << allconfigs.size() << "\n";

	matrix_u_simplex.resize(allconfigs.size(), allconfigs.size());
	matrix_v_simplex.resize(allconfigs.size(), allconfigs.size());
	matrix_uu_simplex.resize(allconfigs.size(), allconfigs.size());
	matrix_uv_simplex.resize(allconfigs.size(), allconfigs.size());
	matrix_vv_simplex.resize(allconfigs.size(), allconfigs.size());
	matrix_u_simplex.setZero();
	matrix_v_simplex.setZero();
	matrix_uu_simplex.setZero();
	matrix_uv_simplex.setZero();
	matrix_vv_simplex.setZero();


#if DO_COMPUTE_INSTEAD_READFILE

#pragma omp parallel for
	//int numcompile = 0;
	for (int it = 0; it < allconfigs.size(); it++)
	{
		for (int jt = 0; jt < allconfigs.size(); jt++)
		{
			if (it > jt)
			{
				continue;
			}

			vector<Point_2> simplex_i, simplex_j;
			vector<Point2withIndex> simplex_in, simplex_jn;
			vector<Point_2> support_i, support_j;
			for (int i = 0; i < allconfigs[it].tconfig.size(); i++)
			{
				simplex_i.push_back(dataMaps[allconfigs[it].tconfig[i]].pt);
				simplex_j.push_back(dataMaps[allconfigs[jt].tconfig[i]].pt);
				Point2withIndex pi, pj;
				pi.index = allconfigs[it].tconfig[i];
				pi.point2 = dataMaps[allconfigs[it].tconfig[i]].pt;
				simplex_in.push_back(pi);
				pj.index = allconfigs[jt].tconfig[i];
				pj.point2 = dataMaps[allconfigs[jt].tconfig[i]].pt;
				simplex_jn.push_back(pj);
			}
			CGAL::convex_hull_2(simplex_i.begin(), simplex_i.end(), back_inserter(support_i));
			CGAL::convex_hull_2(simplex_j.begin(), simplex_j.end(), back_inserter(support_j));

			Polygon_e2 polygon_i, polygon_j;
			for (int k = 0; k < support_i.size(); k++)
			{
				double x = support_i[k].x();
				double y = support_i[k].y();
				polygon_i.push_back(Point_e2(x, y));
			}
			for (int k = 0; k < support_j.size(); k++)
			{
				double x = support_j[k].x();
				double y = support_j[k].y();
				polygon_j.push_back(Point_e2(x, y));
			}

			if (!CGAL::do_intersect(polygon_i, polygon_j))
			{
				continue;
			}

			//perturbation
			vector<Polygon_with_holes_e2>	intR;
			CGAL::intersection(polygon_i, polygon_j, std::back_inserter(intR));
			Polygon_e2 polygon_intersection;
			if (!intR[0].is_unbounded())
			{
				polygon_intersection = intR[0].outer_boundary();
			}
			if (polygon_intersection.area() < 1e-16)
			{
				continue;
			}

			//triangulate intersection region
			vector<vector<Point_2>> facets;
			vector<Point_2> support_intersect;
			intersect_two_ccw_region(support_i, support_j, support_intersect);
			triangulate_two_simplex_support(simplex_in, simplex_jn, facets, support_intersect);
			//std::cout << "number of triangles which union two d1rc-spline:" << cdt.number_of_faces() << std::endl;

			//quadrature
			vector<Point_2>	quadrature_knots;
			vector<double>	quadrature_weights;
			compute_quadrature_knots_and_weights(nquadrature_sample, facets, quadrature_knots, quadrature_weights);

			vector<DomainValue> values_i, values_j;
			compute_simplex_spline(simplex_i, quadrature_knots, values_i);
			compute_simplex_spline(simplex_j, quadrature_knots, values_j);

			double quad_value_dx = 0.0, quad_value_dy = 0.0, quad_value_dxx = 0.0, quad_value_dxy = 0.0, quad_value_dyy = 0.0;
			for (int k = 0; k < quadrature_knots.size(); k++)
			{
				quad_value_dx += values_i[k].dx * values_j[k].dx * quadrature_weights[k];
				quad_value_dy += values_i[k].dy * values_j[k].dy * quadrature_weights[k];
				quad_value_dxx += values_i[k].dxx * values_j[k].dxx * quadrature_weights[k];
				quad_value_dxy += values_i[k].dxy * values_j[k].dxy * quadrature_weights[k];
				quad_value_dyy += values_i[k].dyy * values_j[k].dyy * quadrature_weights[k];
			}

			if (it == jt)
			{
				matrix_u_simplex.coeffRef(it, jt) = quad_value_dx;
				matrix_v_simplex.coeffRef(it, jt) = quad_value_dy;
				matrix_uu_simplex.coeffRef(it, jt) = quad_value_dxx;
				matrix_uv_simplex.coeffRef(it, jt) = quad_value_dxy;
				matrix_vv_simplex.coeffRef(it, jt) = quad_value_dyy;
			}
			else
			{
				matrix_u_simplex.coeffRef(it, jt) = quad_value_dx; 
				matrix_v_simplex.coeffRef(it, jt) = quad_value_dy;
				matrix_uu_simplex.coeffRef(it, jt) = quad_value_dxx;
				matrix_uv_simplex.coeffRef(it, jt) = quad_value_dxy;
				matrix_vv_simplex.coeffRef(it, jt) = quad_value_dyy;

				matrix_u_simplex.coeffRef(jt, it) = quad_value_dx;
				matrix_v_simplex.coeffRef(jt, it) = quad_value_dy;
				matrix_uu_simplex.coeffRef(jt, it) = quad_value_dxx;
				matrix_uv_simplex.coeffRef(jt, it) = quad_value_dxy;
				matrix_vv_simplex.coeffRef(jt, it) = quad_value_dyy;
			}

			/*numcompile++;
			if (numcompile % 500 == 0)
			{
				std::cout << numcompile << std::endl;
			}*/
		}
	}
	//std::cout << "simplex quadrature end: " << numcompile - 1 << std::endl;

	//write to file
	if (quad_file.size() < 5)
	{
		return;
	}
	std::ofstream fout_du;
	fout_du.open(quad_file[0].toStdString());
	if (fout_du.is_open())
	{
		for (int i =0;i<matrix_u_simplex.rows();i++)
		{
			for (int j = 0;j<matrix_u_simplex.cols();j++)
			{
				if (abs(matrix_u_simplex.coeff(i,j)) > 1e-10)
				{
					fout_du << i << " " << j << " " << std::setprecision(18) << matrix_u_simplex.coeff(i, j) << "\n";
				}
			}
		}
	}
	fout_du.close();

	std::ofstream fout_dv;
	fout_dv.open(quad_file[1].toStdString());
	if (fout_dv.is_open())
	{
		for (int i = 0; i < matrix_v_simplex.rows(); i++)
		{
			for (int j = 0; j < matrix_v_simplex.cols(); j++)
			{
				if (abs(matrix_v_simplex.coeff(i, j)) > 1e-10)
				{
					fout_dv << i << " " << j << " " << std::setprecision(18) << matrix_v_simplex.coeff(i, j) << "\n";
				}
			}
		}
	}
	fout_dv.close();

	std::ofstream fout_duu;
	fout_duu.open(quad_file[2].toStdString());
	if (fout_duu.is_open())
	{
		for (int i = 0; i < matrix_uu_simplex.rows(); i++)
		{
			for (int j = 0; j < matrix_uu_simplex.cols(); j++)
			{
				if (abs(matrix_uu_simplex.coeff(i, j)) > 1e-10)
				{
					fout_duu << i << " " << j << " " << std::setprecision(18) << matrix_uu_simplex.coeff(i, j) << "\n";
				}
			}
		}
	}
	fout_duu.close();

	std::ofstream fout_duv;
	fout_duv.open(quad_file[3].toStdString());
	if (fout_duv.is_open())
	{
		for (int i = 0; i < matrix_uv_simplex.rows(); i++)
		{
			for (int j = 0; j < matrix_uv_simplex.cols(); j++)
			{
				if (abs(matrix_uv_simplex.coeff(i, j)) > 1e-10)
				{
					fout_duv << i << " " << j << " " << std::setprecision(18) << matrix_uv_simplex.coeff(i, j) << "\n";
				}
			}
		}
	}
	fout_duv.close();

	std::ofstream fout_dvv;
	fout_dvv.open(quad_file[4].toStdString());
	if (fout_dvv.is_open())
	{
		for (int i = 0; i < matrix_vv_simplex.rows(); i++)
		{
			for (int j = 0; j < matrix_vv_simplex.cols(); j++)
			{
				if (abs(matrix_vv_simplex.coeff(i, j)) > 1e-10)
				{
					fout_dvv << i << " " << j << " " << std::setprecision(18) << matrix_vv_simplex.coeff(i, j) << "\n";
				}

			}
		}
	}
	fout_dvv.close();

	std::cout << "write done" << std::endl;

#else

	vector<Tri> tripletlists_u, tripletlists_v, tripletlists_uu, tripletlists_uv, tripletlists_vv;

	//read file
	if (quad_file.size() < 5)
	{
		return;
	}
	std::ifstream fin_du;
	fin_du.open(quad_file[0].toStdString());
	if (fin_du.is_open())
	{
		int row_;
		int col_;
		double value_;
		while (fin_du >> row_)
		{
			fin_du >> col_;
			fin_du >> value_;
			tripletlists_u.push_back(Tri(row_, col_, value_));
		}
	}
	fin_du.close();

	std::ifstream fin_dv;
	fin_dv.open(quad_file[1].toStdString());
	if (fin_dv.is_open())
	{
		int row_;
		int col_;
		double value_;
		while (fin_dv >> row_)
		{
			fin_dv >> col_;
			fin_dv >> value_;
			tripletlists_v.push_back(Tri(row_, col_, value_));
		}
	}
	fin_dv.close();

	std::ifstream fin_duu;
	fin_duu.open(quad_file[2].toStdString());
	if (fin_duu.is_open())
	{
		int row_;
		int col_;
		double value_;
		while (fin_duu >> row_)
		{
			fin_duu >> col_;
			fin_duu >> value_;
			tripletlists_uu.push_back(Tri(row_, col_, value_));
		}
	}
	fin_duu.close();

	std::ifstream fin_duv;
	fin_duv.open(quad_file[3].toStdString());
	if (fin_duv.is_open())
	{
		int row_;
		int col_;
		double value_;
		while (fin_duv >> row_)
		{
			fin_duv >> col_;
			fin_duv >> value_;
			tripletlists_uv.push_back(Tri(row_, col_, value_));
		}
	}
	fin_duv.close();

	std::ifstream fin_dvv;
	fin_dvv.open(quad_file[4].toStdString());
	if (fin_dvv.is_open())
	{
		int row_;
		int col_;
		double value_;
		while (fin_dvv >> row_)
		{
			fin_dvv >> col_;
			fin_dvv >> value_;
			tripletlists_vv.push_back(Tri(row_, col_, value_));
		}
	}
	fin_dvv.close();

	for (int i = 0; i < tripletlists_u.size(); i++)
	{
		matrix_u_simplex.coeffRef(tripletlists_u[i].row(), tripletlists_u[i].col()) = tripletlists_u[i].value();
	}
	for (int i = 0; i < tripletlists_v.size(); i++)
	{
		matrix_v_simplex.coeffRef(tripletlists_v[i].row(), tripletlists_v[i].col()) = tripletlists_v[i].value();
	}
	for (int i = 0; i < tripletlists_uu.size(); i++)
	{
		matrix_uu_simplex.coeffRef(tripletlists_uu[i].row(), tripletlists_uu[i].col()) = tripletlists_uu[i].value();
	}
	for (int i = 0; i < tripletlists_uv.size(); i++)
	{
		matrix_uv_simplex.coeffRef(tripletlists_uv[i].row(), tripletlists_uv[i].col()) = tripletlists_uv[i].value();
	}
	for (int i = 0; i < tripletlists_vv.size(); i++)
	{
		matrix_vv_simplex.coeffRef(tripletlists_vv[i].row(), tripletlists_vv[i].col()) = tripletlists_vv[i].value();
	}

#endif

	std::ofstream fout_;
	QString dir = OutFileName;
	dir.append("du_r.txt");
	fout_.open(dir.toStdString());
	if (fout_.is_open())
	{
		for (int i = 0; i < matrix_u_simplex.rows(); i++)
		{
			for (int j = 0; j < matrix_u_simplex.cols(); j++)
			{
				if (abs(matrix_u_simplex.coeff(i, j)) > 1e-10)
				{
					fout_ << i << " " << j << " " << matrix_u_simplex.coeff(i, j) << "\n";
				}

			}
		}
	}
	fout_.close();

}

void CFairing::compute_basis_derivate_innerproduct()
{
	vector<Tri> tripletlists_u, tripletlists_v, tripletlists_uu, tripletlists_uv, tripletlists_vv;

	struct Btobs
	{
		int index;
		vector<int> simplex_index;
		vector<double> area_weight;
	};

	vector<Btobs> all_basis;

	for (int i = 0; i < bSplineBasis->basisMergeInfos.size(); i++)
	{
		Btobs basis;
		basis.index = bSplineBasis->basisMergeInfos[i].index-1;
		if (bSplineBasis->basisMergeInfos[i].index != -1)
		{
			for (int k = 0; k<bSplineBasis->basisMergeInfos[i].basisMerge.size(); k++)
			{
				int indexconfig = bSplineBasis->basisMergeInfos[i].basisMerge[k];
				for (int j = 0; j<bSplineBasis->basisConfigs[indexconfig].tconfigs.size(); j++)
				{
					basis.simplex_index.push_back(bSplineBasis->basisConfigs[indexconfig].tconfigs[j].allconfigs_index);
					basis.area_weight.push_back(bSplineBasis->basisConfigs[indexconfig].tconfigs[j].tri_area);
				}
			}
			all_basis.push_back(basis);
		}
	}

	//test
	/*std::ofstream fout_d1;
	fout_d1.open("d1.txt");
	if (fout_d1.is_open())
	{
		for (int k = 0; k < d1_values.size(); k++)
		{
			fout_d1 << d1_values[k] << "\n";
		}
	}
	fout_d1.close();*/

	for (int i = 0; i < all_basis.size(); i++)
	{
		for (int j = 0; j < all_basis.size(); j++)
		{
			int index_i = all_basis[i].index;
			int index_j = all_basis[j].index;

			assert(index_i < all_basis.size());
			assert(index_j < all_basis.size());

			if (index_i < index_j)
			{
				continue;
			}

			double integrate_value_u = 0.0;
			double integrate_value_v = 0.0;
			double integrate_value_uu = 0.0;
			double integrate_value_uv = 0.0;
			double integrate_value_vv = 0.0;
			for (int m = 0; m < all_basis[i].simplex_index.size(); m++)
			{
				for (int n = 0; n < all_basis[j].simplex_index.size(); n++)
				{
					int mi = all_basis[i].simplex_index[m];
					int nj = all_basis[j].simplex_index[n];
					double di = all_basis[i].area_weight[m];
					double dj = all_basis[j].area_weight[n];
					//assert(di > 0);
					//assert(dj > 0);
					integrate_value_u += di * dj * matrix_u_simplex.coeff(mi, nj);
					integrate_value_v += di * dj * matrix_v_simplex.coeff(mi, nj);
					integrate_value_uu += di * dj * matrix_uu_simplex.coeff(mi, nj);
					integrate_value_uv += di * dj * matrix_uv_simplex.coeff(mi, nj);
					integrate_value_vv += di * dj * matrix_vv_simplex.coeff(mi, nj);
				}
			}

			if (index_i == index_j)
			{
				tripletlists_u.push_back(Tri(index_i, index_i, integrate_value_u));
				tripletlists_v.push_back(Tri(index_i, index_i, integrate_value_v));
				tripletlists_uu.push_back(Tri(index_i, index_i, integrate_value_uu));
				tripletlists_uv.push_back(Tri(index_i, index_i, integrate_value_uv));
				tripletlists_vv.push_back(Tri(index_i, index_i, integrate_value_vv));
			}
			else
			{
				tripletlists_u.push_back(Tri(index_i, index_j, integrate_value_u));
				tripletlists_v.push_back(Tri(index_i, index_j, integrate_value_v));
				tripletlists_uu.push_back(Tri(index_i, index_j, integrate_value_uu));
				tripletlists_uv.push_back(Tri(index_i, index_j, integrate_value_uv));
				tripletlists_vv.push_back(Tri(index_i, index_j, integrate_value_vv));

				tripletlists_u.push_back(Tri(index_j, index_i, integrate_value_u));
				tripletlists_v.push_back(Tri(index_j, index_i, integrate_value_v));
				tripletlists_uu.push_back(Tri(index_j, index_i, integrate_value_uu));
				tripletlists_uv.push_back(Tri(index_j, index_i, integrate_value_uv));
				tripletlists_vv.push_back(Tri(index_j, index_i, integrate_value_vv));
			}
		}
	}
	std::cout << "quadrature of basis derivate begin " << all_basis.size() << std::endl;
	matrix_u_basis.resize(all_basis.size(), all_basis.size());
	matrix_v_basis.resize(all_basis.size(), all_basis.size());
	matrix_uu_basis.resize(all_basis.size(), all_basis.size());
	matrix_uv_basis.resize(all_basis.size(), all_basis.size());
	matrix_vv_basis.resize(all_basis.size(), all_basis.size());
	matrix_u_basis.setFromTriplets(tripletlists_u.begin(), tripletlists_u.end());
	matrix_v_basis.setFromTriplets(tripletlists_v.begin(), tripletlists_v.end());
	matrix_uu_basis.setFromTriplets(tripletlists_uu.begin(), tripletlists_uu.end());
	matrix_uv_basis.setFromTriplets(tripletlists_uv.begin(), tripletlists_uv.end());
	matrix_vv_basis.setFromTriplets(tripletlists_vv.begin(), tripletlists_vv.end());

	std::ofstream fout_;
	QString dir = OutFileName;
	dir.append("basis_du_r.txt");
	fout_.open(dir.toStdString());
	if (fout_.is_open())
	{
		for (int i = 0; i < tripletlists_u.size(); i++)
		{
			if (abs(tripletlists_u[i].value()) > 1e-10)
			{
				fout_ << tripletlists_u[i].row() << " " << tripletlists_u[i].col() << " " << tripletlists_u[i].value() << "\n";
			}
		}
	}
	fout_.close();

	std::cout << "quadrature of basis derivate finished" << std::endl;
}

void CFairing::triangulate_two_simplex_support(
	vector<Point2withIndex> simplex1, vector<Point2withIndex> simplex2,
	vector<vector<Point_2>> &facets, vector<Point_2> support_join)
{
	//insert interior struct first, and then insert boundary struct
	CDT_Intersect cdt_temp;

	struct InsertSets
	{
		std::pair<Point2withIndex, Point2withIndex> constraint_edge;
		bool do_inserting;
	};

	vector<InsertSets> insert_queue;

	//remove same interior points
	remove_same_elements(simplex1);
	remove_same_elements(simplex2);

	//is same
	vector<int> simp1, simp2;
	for (int i = 0; i < simplex1.size(); i++)
	{
		simp1.push_back(simplex1[i].index);
	}
	for (int i = 0; i<simplex2.size(); i++)
	{
		simp2.push_back(simplex2[i].index);
	}
	bool is_same_config = false;
	if (simp1.size() == simp2.size())
	{
		sort(simp1.begin(), simp1.end());
		sort(simp2.begin(), simp2.end());
		is_same_config = std::equal(simp1.begin(), simp1.end(), simp2.begin());
	}

	for (int i = 0; i<support_join.size(); i++)
	{
		CDT_Intersect::Vertex_handle v_h_1, v_h_2;
		v_h_1 = cdt_temp.insert(Point_Intersect(support_join[i].x(), support_join[i].y()));
		v_h_2 = cdt_temp.insert(Point_Intersect(support_join[(i + 1)%support_join.size()].x(), support_join[(i + 1) % support_join.size()].y()));
		cdt_temp.insert_constraint(v_h_1, v_h_2);
	}

	//the key is the standard of predicate narrow triangle
	double narrow_pred = 0.995;
	double area_pred = 1e-10;
	for (int i = 0; i < simplex1.size() - 1; i++)
	{
		for (int j = i + 1; j < simplex1.size(); j++)
		{
			//remove triangle with narrow angle
			bool do_insert_narrow = true;
			Point2withIndex p_start = simplex1[i];
			Point2withIndex p_end = simplex1[j];
			if (i == j)
			{
				continue;
			}
			for (int k = 0; k < simplex1.size(); k++)
			{
				if (k == 1 || k == j)
				{
					continue;
				}
				Point2withIndex p_other = simplex1[k];
				if ((p_other.point2 - p_start.point2).squared_length() > 1e-10 &&
					(p_end.point2 - p_other.point2).squared_length() > 1e-10)
				{
					double cose1 = (p_start.point2 - p_end.point2) * (p_other.point2 - p_end.point2) / sqrt((p_start.point2 - p_end.point2).squared_length() *
						(p_other.point2 - p_end.point2).squared_length());
					double cose2 = (p_end.point2 - p_start.point2) * (p_other.point2 - p_start.point2) / sqrt((p_end.point2 - p_start.point2).squared_length() *
						(p_other.point2 - p_start.point2).squared_length());
					double cose3 = (p_start.point2 - p_other.point2) * (p_end.point2 - p_other.point2) / sqrt((p_start.point2 - p_other.point2).squared_length() *
						(p_end.point2 - p_other.point2).squared_length());

					double testarea = abs(area(p_start.point2, p_end.point2, p_other.point2));
					if ((cose1 < -narrow_pred || cose2 < -narrow_pred || cose3 < -narrow_pred) &&
						(p_start.point2 - p_end.point2).squared_length() > (p_other.point2 - p_start.point2).squared_length() &&
						(p_start.point2 - p_end.point2).squared_length() > (p_other.point2 - p_end.point2).squared_length()
						&& testarea < area_pred)
					{
						do_insert_narrow = false;
						break;
					}
				}
			}
			bool do_insert_now = true;
			for (int k = 0; k < insert_queue.size(); k++)
			{
				if (insert_queue[k].constraint_edge.first.index == simplex1[i].index &&
					insert_queue[k].constraint_edge.second.index == simplex1[j].index
					|| insert_queue[k].constraint_edge.first.index == simplex1[j].index &&
					insert_queue[k].constraint_edge.second.index == simplex1[i].index)
				{
					do_insert_now = false;
					break;
				}
			}
			if (do_insert_now)
			{
				InsertSets edge;
				edge.constraint_edge = { simplex1[i], simplex1[j] };
				//remove itself
				if ((simplex1[i].point2 - simplex1[j].point2).squared_length() < 1e-16)
				{
					edge.do_inserting = false;
				}
				else
				{
					edge.do_inserting = do_insert_narrow;
				}
				insert_queue.push_back(edge);
			}
		}
	}

	if (!is_same_config)
	{
		for (int i = 0; i < simplex2.size() - 1; i++)
		{
			for (int j = i + 1; j < simplex2.size(); j++)
			{
				//remove triangle with narrow angle
				bool do_insert_narrow = true;
				Point2withIndex p_start = simplex2[i];
				Point2withIndex p_end = simplex2[j];
				if (i == j)
				{
					continue;
				}
				for (int k = 0; k < simplex1.size(); k++)
				{
					Point2withIndex p_other = simplex1[k];
					if ((p_other.point2 - p_start.point2).squared_length() > 1e-10 &&
						(p_end.point2 - p_other.point2).squared_length() > 1e-10)
					{
						double cose1 = (p_start.point2 - p_end.point2) * (p_other.point2 - p_end.point2) / sqrt((p_start.point2 - p_end.point2).squared_length() *
							(p_other.point2 - p_end.point2).squared_length());
						double cose2 = (p_end.point2 - p_start.point2) * (p_other.point2 - p_start.point2) / sqrt((p_end.point2 - p_start.point2).squared_length() *
							(p_other.point2 - p_start.point2).squared_length());
						double cose3 = (p_start.point2 - p_other.point2) * (p_end.point2 - p_other.point2) / sqrt((p_start.point2 - p_other.point2).squared_length() *
							(p_end.point2 - p_other.point2).squared_length());

						double testarea = abs(area(p_start.point2, p_end.point2, p_other.point2));
						if ((cose1 < -narrow_pred || cose2 < -narrow_pred || cose3 < -narrow_pred) &&
							(p_start.point2 - p_end.point2).squared_length() >(p_other.point2 - p_start.point2).squared_length() &&
							(p_start.point2 - p_end.point2).squared_length() >(p_other.point2 - p_end.point2).squared_length()
							&& testarea < area_pred)
						{
							do_insert_narrow = false;
							break;
						}
					}
				}
				if (!is_same_config)
				{
					if (do_insert_narrow)
					{
						for (int k = 0; k < simplex2.size(); k++)
						{
							if (k == i || k == j)
							{
								continue;
							}
							Point2withIndex p_other = simplex2[k];
							if ((p_other.point2 - p_start.point2).squared_length() > 1e-10 &&
								(p_end.point2 - p_other.point2).squared_length() > 1e-10)
							{
								double cose1 = (p_start.point2 - p_end.point2) * (p_other.point2 - p_end.point2) / sqrt((p_start.point2 - p_end.point2).squared_length() *
									(p_other.point2 - p_end.point2).squared_length());
								double cose2 = (p_end.point2 - p_start.point2) * (p_other.point2 - p_start.point2) / sqrt((p_end.point2 - p_start.point2).squared_length() *
									(p_other.point2 - p_start.point2).squared_length());
								double cose3 = (p_start.point2 - p_other.point2) * (p_end.point2 - p_other.point2) / sqrt((p_start.point2 - p_other.point2).squared_length() *
									(p_end.point2 - p_other.point2).squared_length());

								double testarea = abs(area(p_start.point2, p_end.point2, p_other.point2));
								if ((cose1 < -narrow_pred || cose2 < -narrow_pred || cose3 < -narrow_pred) &&
									(p_start.point2 - p_end.point2).squared_length() >(p_other.point2 - p_start.point2).squared_length() &&
									(p_start.point2 - p_end.point2).squared_length() >(p_other.point2 - p_end.point2).squared_length()
									&& testarea < area_pred)
								{
									do_insert_narrow = false;
									break;
								}
							}

						}
					}
				}

				bool do_insert_now = true;
				for (int k = 0; k < insert_queue.size(); k++)
				{
					if (insert_queue[k].constraint_edge.first.index == simplex2[i].index &&
						insert_queue[k].constraint_edge.second.index == simplex2[j].index
						|| insert_queue[k].constraint_edge.first.index == simplex2[j].index &&
						insert_queue[k].constraint_edge.second.index == simplex2[i].index)
					{
						do_insert_now = false;
						break;
					}
				}
				if (do_insert_now)
				{
					InsertSets edge;
					edge.constraint_edge = { simplex2[i], simplex2[j] };
					//remove itself
					if ((simplex2[i].point2 - simplex2[j].point2).squared_length() < 1e-16)
					{
						edge.do_inserting = false;
					}
					else
					{
						edge.do_inserting = do_insert_narrow;
					}
					insert_queue.push_back(edge);
				}
			}
		}
	}

	for (int num = 0; num < insert_queue.size(); num++)
	{
		if (insert_queue[num].do_inserting)
		{
			CDT_Intersect::Vertex_handle v_h_1, v_h_2;
			v_h_1 = cdt_temp.insert(Point_Intersect(insert_queue[num].constraint_edge.first.point2.x(),
				insert_queue[num].constraint_edge.first.point2.y()));
			v_h_2 = cdt_temp.insert(Point_Intersect(insert_queue[num].constraint_edge.second.point2.x(),
				insert_queue[num].constraint_edge.second.point2.y()));
			cdt_temp.insert_constraint(v_h_1, v_h_2);
		}
	}

	//remove triangle outside support
	Polygon_2 support;
	for (int i = 0; i < support_join.size(); i++)
	{
		support.push_back(support_join[i]);
	}
	CDT_Intersect::Face_iterator f_it;
	for (f_it = cdt_temp.faces_begin(); f_it != cdt_temp.faces_end(); f_it++)
	{
		Point_2 p1, p2, p3;
		p1 = Point_2(CGAL::to_double(f_it->vertex(0)->point().x()), CGAL::to_double(f_it->vertex(0)->point().y()));
		p2 = Point_2(CGAL::to_double(f_it->vertex(1)->point().x()), CGAL::to_double(f_it->vertex(1)->point().y()));
		p3 = Point_2(CGAL::to_double(f_it->vertex(2)->point().x()), CGAL::to_double(f_it->vertex(2)->point().y()));

		Point_2 testp1((p1.x() + p2.x() + p3.x()) / 3.0, (p1.y() + p2.y() + p3.y()) / 3.0);
		Point_2 testp2((0.4 * p1.x() + 0.4 * p2.x() + 0.2 * p3.x()), (0.4 * p1.y() + 0.4 * p2.y() + 0.2 * p3.y()));
		Point_2 testp3((0.2 * p1.x() + 0.2 * p2.x() + 0.6 * p3.x()), (0.2 * p1.y() + 0.2 * p2.y() + 0.6 * p3.y()));
		if (support.bounded_side(testp1) == CGAL::ON_UNBOUNDED_SIDE ||
			support.bounded_side(testp2) == CGAL::ON_UNBOUNDED_SIDE ||
			support.bounded_side(testp3) == CGAL::ON_UNBOUNDED_SIDE)
		{
			//cdt_temp.delete_face(f_it);
		}
		else
		{
			vector<Point_2> face_ = { p1,p2,p3 };
			facets.push_back(face_);
		}
	}
}


void CFairing::compute_quadrature_knots_and_weights(int qua_degree, vector<vector<Point_2>> &facets, vector<Point_2> &quadrature_knots, vector<double> &quadrature_weights)
{
	quadrature_knots.clear();
	quadrature_weights.clear();

	for (int i = 0; i<facets.size(); i++)
	{
		vector<Point_2> knots;
		vector<double> weights;
		int num_knots;
		dunavant_quadrature_rule(qua_degree, facets[i][0], facets[i][1], facets[i][2], knots, weights, num_knots);

		for (int i = 0; i < num_knots; i++)
		{
			quadrature_knots.push_back(knots[i]);
			quadrature_weights.push_back(weights[i]);
		}
	}
}

void CFairing::dunavant_quadrature_rule(int degree, Point_2 v1, Point_2 v2, Point_2 v3, vector<Point_2> &knots, vector<double> &weights, int &num_knots)
{
	knots.clear();
	weights.clear();

	double node_xy2[2 * 3] = {
		v1.x(), v1.y(),
		v2.x(), v2.y(),
		v3.x(), v3.y() };
	int rule = degree;
	double *w;
	double *xy;
	double *xy2;

	num_knots = dunavant_order_num(rule);

	xy = new double[2 * num_knots];
	xy2 = new double[2 * num_knots];
	w = new double[num_knots];

	dunavant_rule(rule, num_knots, xy, w);

	reference_to_physical_t3(node_xy2, num_knots, xy, xy2);
	double area2 = abs(triangle_area(node_xy2));

	for (int i = 0; i < num_knots; i++)
	{
		Point_2 p(xy2[2 * i + 0], xy2[2 * i + 1]);
		knots.push_back(p);
		weights.push_back(w[i] * area2);
	}

	delete[] w;
	delete[] xy;
	delete[] xy2;
}

void CFairing::generate_output_dir()
{
	QString dir = OutFileName;

	QString dirdx = dir;
	dirdx.append("simplexquadrature_du.txt");

	QString dirdy = dir;
	dirdy.append("simplexquadrature_dv.txt");

	QString dirdxx = dir;
	dirdxx.append("simplexquadrature_duu.txt");

	QString dirdxy = dir;
	dirdxy.append("simplexquadrature_duv.txt");

	QString dirdyy = dir;
	dirdyy.append("simplexquadrature_dvv.txt");

	quad_file = {dirdx,dirdy,dirdxx,dirdxy,dirdyy};

}

