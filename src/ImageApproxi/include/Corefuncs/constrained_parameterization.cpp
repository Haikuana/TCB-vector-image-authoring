#include "Corefuncs/constrained_parameterization.h"

#define PARA_UNIFORM 1
#define PARA_SQUARE 2
#define PARA_PRESERVE_SHAPE 3
#define PARA_METHOD 1

#define CONSIDER_FEATURE 1

#define CONSIDER_SURFACE_Z 0

ConstrainedPara::ConstrainedPara(Mesh *surface_, int object_typ)
{
	surface = surface_;
	type_object = object_typ;
}

ConstrainedPara::~ConstrainedPara()
{

}

void ConstrainedPara::compute_para_domain()
{
	if (surface == NULL)
	{
		return;
	}

	num_vertex_ = surface->size_of_vertices();
	num_bound_vertex = 0;
	for (auto vit = surface->vertices_begin();vit != surface->vertices_end();vit++)
	{
#if CONSIDER_FEATURE
		if (vit->is_feature() || vit->is_border())
#else
		if (vit->is_border())
#endif
		{
			num_bound_vertex++;
		}
	}
	num_inner_vertex = num_vertex_ - num_bound_vertex;

	//std::cout << "CalculateSpraseMatrix start: inner" << num_inner_vertex << " boundary:" << num_bound_vertex << std::endl;

	int id_interior = 0;
	int id_boundary = num_inner_vertex;
	for (auto vit = surface->vertices_begin(); vit != surface->vertices_end(); vit++)
	{
#if CONSIDER_FEATURE
		if (vit->is_feature() || vit->is_border())
#else
		if (vit->is_border())
#endif
		{
			vit->vertex_standby_index() = id_boundary;
			id_boundary++;
		}
		else
		{
			vit->vertex_standby_index() = id_interior;
			id_interior++;
		}
	}
	surface->map_vertex_standby_index_to_iterator();

	AssembleMatrixCoeff();

	std::cout << __FUNCTION__ << ": " << "CalculateSpraseMatrix finished" << std::endl;

	SloveEquation();

	std::cout << __FUNCTION__ << ": " << "solving equation finished" << std::endl;

}

void ConstrainedPara::AssembleMatrixCoeff(void)
{
	matrix_assemble.resize(num_vertex_,num_vertex_);
	vector<Tri> triplete;

	if (PARA_METHOD == PARA_PRESERVE_SHAPE)
	{
		//shape preserve para
		for (auto v_it = surface->vertices_begin(); v_it != surface->vertices_end(); v_it++)
		{
#if CONSIDER_FEATURE
			if (v_it->is_feature() || v_it->is_border())
#else
			if (v_it->is_border())
#endif
			{
				continue;
			}
			//calculate row index 
			int index_row = v_it->vertex_standby_index();
			CalculateAlpha(v_it);
			int		k = 0;
			Mesh::Halfedge_around_vertex_circulator h_it = v_it->vertex_begin();
			do
			{
				int	index_col = h_it->opposite()->vertex()->vertex_standby_index();
				triplete.push_back(Tri(index_row, index_col, alpha_(0, k)));
				k++;
			} while (++h_it != v_it->vertex_begin());
			triplete.push_back(Tri(index_row, index_row, -1.0f));
		}
	}

	if (PARA_METHOD == PARA_SQUARE)
	{
		//square method
		for (auto v_it = surface->vertices_begin(); v_it != surface->vertices_end(); ++v_it)
		{
#if CONSIDER_FEATURE
			if (v_it->is_feature() || v_it->is_border())
#else
			if (v_it->is_border())
#endif
			{
				continue;
			}
			//calculate row index 
			int		index_row = v_it->vertex_standby_index();
			double	alphaplus = 0;
			Mesh::Halfedge_around_vertex_circulator h_it = v_it->vertex_begin();
			do
			{
				//calculate col index 
				int		index_col = h_it->opposite()->vertex()->vertex_standby_index();
				double dis_square;
				if (CONSIDER_SURFACE_Z)
				{
					dis_square = (v_it->point() - h_it->opposite()->vertex()->point()).squared_length();
				}
				else
				{
					Point_2 p1(v_it->point().x(), v_it->point().y());
					Point_2 p2(h_it->opposite()->vertex()->point().x(), h_it->opposite()->vertex()->point().y());
					dis_square = (p1 - p2).squared_length();
				}

				
				float	alpha = 1.0f / dis_square;

				triplete.push_back(Tri(index_row, index_col, alpha));
				alphaplus += alpha;
			} while (++h_it != v_it->vertex_begin());
			triplete.push_back(Tri(index_row, index_row, -alphaplus));
		}
	}

	if (PARA_METHOD == PARA_UNIFORM)
	{
		//uniform
		for (auto v_it = surface->vertices_begin(); v_it != surface->vertices_end(); ++v_it)
		{
#if CONSIDER_FEATURE
			if (v_it->is_feature() || v_it->is_border())
#else
			if (v_it->is_border())
#endif
			{
				continue;
			}
			//calculate row index 
			int	index_row = v_it->vertex_standby_index();
			int degree = v_it->vertex_degree();
			Mesh::Halfedge_around_vertex_circulator h_it = v_it->vertex_begin();
			do
			{
				//calculate col index 
				int		index_col = h_it->opposite()->vertex()->vertex_standby_index();
				triplete.push_back(Tri(index_row, index_col, 1.0f));
			} while (++h_it != v_it->vertex_begin());
			triplete.push_back(Tri(index_row, index_row, -double(degree)));
		}
	}

	matrix_assemble.setFromTriplets(triplete.begin(),triplete.end());
}

void ConstrainedPara::SloveEquation()
{
	MatrixXd	x_solve(num_inner_vertex, 2);
	MatrixXd	b(num_inner_vertex, 2);
	b.setZero();
	SpMat	matrix_A(num_inner_vertex, num_inner_vertex);

	//assemble A
	for (int k = 0; k < matrix_assemble.outerSize(); ++k)
	{
		for (SpMat::InnerIterator it(matrix_assemble, k); it; ++it)
		{
			if (it.row() < num_inner_vertex && it.col() < num_inner_vertex)
			{
				matrix_A.insert(it.row(), it.col()) =  it.value();
			}
		}
	}
	//assemble b
	for (int k = 0; k < matrix_assemble.outerSize(); ++k)
	{
		for (SpMat::InnerIterator it(matrix_assemble, k); it; ++it)
		{
			if (it.row() < num_inner_vertex && it.col() >= num_inner_vertex)
			{
				if (type_object == 0)
				{
					b.coeffRef(it.row(), 0) -= it.value() * surface->get_vertex_standby_iterator(it.col())->get_domain().x();
					b.coeffRef(it.row(), 1) -= it.value() * surface->get_vertex_standby_iterator(it.col())->get_domain().y();
				}
				else if (type_object == 1)
				{
					b.coeffRef(it.row(), 0) -= it.value() * surface->get_vertex_standby_iterator(it.col())->point().x();
					b.coeffRef(it.row(), 1) -= it.value() * surface->get_vertex_standby_iterator(it.col())->point().y();
				}
			}
		}
	}

	//solve
	clock_t start_ = clock(), end_;
	if (PARA_METHOD == PARA_PRESERVE_SHAPE)
	{
		//LU decomposition;
		QR decomposition;
		matrix_A.makeCompressed();
		decomposition.compute(matrix_A);
		x_solve = decomposition.solve(b);

		end_ = clock();
		std::cout << __FUNCTION__ << ": " << "solving equation by QR spend time: " << end_ - start_ << std::endl;
	}
	else
	{
		SimplicialLDLT decomposition;
		matrix_A.makeCompressed();
		decomposition.compute(matrix_A);
		x_solve = decomposition.solve(b);

		end_ = clock();
		std::cout << __FUNCTION__ << ": " << "solving equation by LDLT spend time: " << end_ - start_ << std::endl;
	}
	

	//update domain value
	for (auto v_it = surface->vertices_begin(); v_it != surface->vertices_end(); ++v_it)
	{
#if CONSIDER_FEATURE
		if (v_it->is_feature() || v_it->is_border())
#else
		if (v_it->is_border())
#endif
		{
			continue;
		}
		if (type_object == 0)
		{
			v_it->get_domain() = Point_2(x_solve.coeff(v_it->vertex_standby_index(), 0), x_solve.coeff(v_it->vertex_standby_index(), 1));
		}
		else if (type_object == 1)
		{
			v_it->point() = Point(x_solve.coeff(v_it->vertex_standby_index(), 0), x_solve.coeff(v_it->vertex_standby_index(), 1),0.0);
		}
		
	}
}

void ConstrainedPara::CalculateAlpha(const Mesh::Vertex_iterator v_it)
{
	Vertex_iterator vert_center_ = v_it;

	//calculate the max distance of point-point and degree
	double max_dis_ = 0;
	int degree = v_it->vertex_degree();
	Mesh::Halfedge_around_vertex_circulator h_it = v_it->vertex_begin();
	do
	{
		Vertex_iterator vv = h_it->opposite()->vertex();
		double nor_dis_;
		if (CONSIDER_SURFACE_Z)
		{
			nor_dis_ = sqrt((v_it->point() - vv->point()).squared_length());
		}
		else
		{
			Point_2 p1(v_it->point().x(), v_it->point().y());
			Point_2 p2(vv->point().x(), vv->point().y());
			nor_dis_ = sqrt((p1 - p2).squared_length());
		}
		if (nor_dis_ > max_dis_)
		{
			max_dis_ = nor_dis_;
		}
	} while (++h_it != v_it->vertex_begin());

	if (degree < 3)
	{
		std::cout << __FUNCTION__ << ": " << "degree < 3 " << std::endl;
		return;
	}

	//calculate plane point
	std::vector<Point_2> plane_point;
	plane_point.clear();
	double dis_squ ;
	if (CONSIDER_SURFACE_Z)
	{
		dis_squ = (v_it->point() - v_it->vertex_begin()->opposite()->vertex()->point()).squared_length();
	}
	else
	{
		Point_2 p1(v_it->point().x(), v_it->point().y());
		Point_2 p2(v_it->vertex_begin()->opposite()->vertex()->point().x(), v_it->vertex_begin()->opposite()->vertex()->point().y());
		dis_squ = (p1 - p2).squared_length();
	}
	Point_2 p0(0, 0), p1(sqrt(dis_squ), 0);
	plane_point.push_back(p0);
	plane_point.push_back(p1);

	double angle = 0, angle2 = 0;
	h_it = v_it->vertex_begin();
	do
	{
		double cosangle3;
		Vertex_iterator v1, v2;
		v1 = h_it->opposite()->vertex();
		v2 = h_it->next_on_vertex()->opposite()->vertex();

		if (CONSIDER_SURFACE_Z)
		{
			cosangle3 = (v1->point() - v_it->point())*(v2->point() - v_it->point()) / sqrt((v1->point() - v_it->point()).squared_length())
				/ sqrt((v2->point() - v_it->point()).squared_length());
		}
		else
		{
			Point_2 p1(v1->point().x(), v1->point().y());
			Point_2 p2(v2->point().x(), v2->point().y());
			Point_2 p0(v_it->point().x(), v_it->point().y());
			cosangle3 = (p1-p0)*(p2-p0) / sqrt((p1 - p0).squared_length())
				/ sqrt((p2 - p0).squared_length());
		}
		
		angle += abs(acos(cosangle3));
	} while (++h_it != v_it->vertex_begin());

	h_it = v_it->vertex_begin();
	do
	{
		double cosangle3;
		Vertex_iterator v1, v2;
		v1 = h_it->opposite()->vertex();
		v2 = h_it->next_on_vertex()->opposite()->vertex();
		if (CONSIDER_SURFACE_Z)
		{
			cosangle3 = (v1->point() - v_it->point())*(v2->point() - v_it->point()) / sqrt((v1->point() - v_it->point()).squared_length())
				/ sqrt((v2->point() - v_it->point()).squared_length());
		}
		else
		{
			Point_2 p1(v1->point().x(), v1->point().y());
			Point_2 p2(v2->point().x(), v2->point().y());
			Point_2 p0(v_it->point().x(), v_it->point().y());
			cosangle3 = (p1 - p0)*(p2 - p0) / sqrt((p1 - p0).squared_length())
				/ sqrt((p2 - p0).squared_length());
		}
		
		angle2 = angle2 + 2 * PI * abs(acos(cosangle3)) / angle;

		Vertex_iterator next = h_it->next_on_vertex()->opposite()->vertex();
		double x, y;
		if (CONSIDER_SURFACE_Z)
		{
			x = sqrt((v_it->point() - next->point()).squared_length()) * cos(angle2);
			y = sqrt((v_it->point() - next->point()).squared_length()) * sin(angle2);
		}
		else
		{
			Point_2 p1(v_it->point().x(), v_it->point().y());
			Point_2 p2(next->point().x(), next->point().y());
			x = sqrt((p2-p1).squared_length()) * cos(angle2);
			y = sqrt((p2-p1).squared_length()) * sin(angle2);
		}
		plane_point.push_back(Point_2(x,y));
	} while (++h_it != v_it->vertex_begin());
	plane_point.pop_back();


	//calculate alpha_
	MatrixXd	alpha_alpha, alpha_pre;
	alpha_alpha.setZero(plane_point.size() - 1, plane_point.size() - 1);
	alpha_pre.setOnes(1, plane_point.size() - 1);
	alpha_.setZero(1, plane_point.size() - 1);

	double	allarea = 0;
	for (size_t i = 1; i != plane_point.size(); i++)
	{
		//calculate r(l)
		int					r_l_ = -1, r_l_1 = -1;
		Point_2				p1, p2, p2p, p3, p4;

		p1 = plane_point[0];
		p2p = plane_point[i];
		p2 = p1 + (p1 - p2p) * 1000 * max_dis_;

		for (size_t j = 1; j != plane_point.size(); j++)
		{
			p3 = plane_point[j];
			if (j == plane_point.size() - 1)
			{
				r_l_1 = 1;
			}
			else
			{
				r_l_1 = j + 1;
			}
			p4 = plane_point[r_l_1];
			pair<double, double> d1(p1[0], p1[1]);
			pair<double, double> d2(p2[0], p2[1]);
			pair<double, double> d3(p3[0], p3[1]);
			pair<double, double> d4(p4[0], p4[1]);
			if (SegmentIntersect(d1, d2, d3, d4))
			{
				r_l_ = j;
				break;
			}
		}

		Point_2 tri1, tri2, tri3;
		tri1 = plane_point[i];
		tri2 = plane_point[r_l_];
		tri3 = plane_point[r_l_1];

		double	s023 = 0, s012 = 0, s013 = 0, s123 = 0;
		s023 = (1.0 / 2) * (tri2[0] * tri3[1] - tri3[0] * tri2[1]);
		s023 = sqrt(s023*s023);
		s013 = (1.0 / 2) * (tri1[0] * tri3[1] - tri3[0] * tri1[1]);
		s013 = sqrt(s013*s013);
		s012 = (1.0 / 2) * (tri2[0] * tri1[1] - tri1[0] * tri2[1]);
		s012 = sqrt(s012*s012);
		s123 = s012 + s013 + s023;
		alpha_alpha(i - 1, i - 1) = s023 / s123;
		alpha_alpha(i - 1, r_l_ - 1) = s013 / s123;
		alpha_alpha(i - 1, r_l_1 - 1) = s012 / s123;
	}
	alpha_ = alpha_pre * alpha_alpha;
	alpha_ = alpha_ / double(degree);
}

//计算p1p2和p1p3叉积
double ConstrainedPara::direction(pair<double, double> p1, pair<double, double> p2, pair<double, double> p3)
{
	pair<double, double> d1 = std::make_pair(p3.first - p1.first, p3.second - p1.second);
	pair<double, double> d2 = std::make_pair(p2.first - p1.first, p2.second - p1.second);
	return d1.first*d2.second - d1.second*d2.first;
}

//当p3在直线p1p2上时，OnSegment函数用于确认p3在上，还是在的延长线上
bool ConstrainedPara::OnSegment(pair<double, double> p1, pair<double, double> p2, pair<double, double> p3)
{
	double x_min, x_max, y_min, y_max;
	if (p1.first < p2.first) {
		x_min = p1.first;
		x_max = p2.first;
	}
	else {
		x_min = p2.first;
		x_max = p1.first;
	}
	if (p1.second < p2.second) {
		y_min = p1.second;
		y_max = p2.second;
	}
	else {
		y_min = p2.second;
		y_max = p1.second;
	}
	if (p3.first<x_min || p3.first>x_max || p3.second<y_min || p3.second>y_max)
		return false;
	else
		return true;
}

bool ConstrainedPara::SegmentIntersect(pair<double, double> p1, pair<double, double> p2,
	pair<double, double> p3, pair<double, double> p4)
{
	double d1 = direction(p3, p4, p1);
	double d2 = direction(p3, p4, p2);
	double d3 = direction(p1, p2, p3);
	double d4 = direction(p1, p2, p4);

	if (d1*d2 < 0 && d3*d4 < 0)
		return true;
	else if (d1 == 0 && OnSegment(p3, p4, p1))
		return true;
	else if (d2 == 0 && OnSegment(p3, p4, p2))
		return true;
	else if (d3 == 0 && OnSegment(p1, p2, p3))
		return true;
	else if (d4 == 0 && OnSegment(p1, p2, p4))
		return true;
	else
		return false;
}



void ConstrainedPara::mesh_deformation()
{
	typedef CGAL::Simple_cartesian<double>										KernelDeform;
	typedef CGAL::Polyhedron_3<KernelDeform, CGAL::Polyhedron_items_with_id_3>	PolyhedronDeform;
	typedef boost::graph_traits<PolyhedronDeform>::vertex_descriptor			vertex_descriptor;
	typedef boost::graph_traits<PolyhedronDeform>::vertex_iterator				vertex_iterator;
	typedef CGAL::Surface_mesh_deformation<PolyhedronDeform>					Surface_mesh_deformation;

	const QString off_name = "./KnotsConstrainOut/deformation_domain.off";
	std::ofstream fout_off;
	fout_off.open(off_name.toStdString());
	if (fout_off.is_open())
	{
		fout_off << "COFF" << "\n";
		fout_off << surface->size_of_vertices() << " " << surface->size_of_facets() << " " << 0 << "\n";
		for (auto vit = surface->vertices_begin(); vit != surface->vertices_end(); vit++)
		{
			double x = vit->get_old_domain().x();
			double y = vit->get_old_domain().y();
			double z = 0.0;
			fout_off << x << " " << y << " " << z << "\n";
		}
		for (auto fit = surface->facets_begin(); fit != surface->facets_end(); fit++)
		{
			fout_off << 3<<" ";
			Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
			do
			{
				fout_off << hc->vertex()->vertex_index() <<" ";
			} while (++hc != hs);
			fout_off << "\n";
		}
	}
	fout_off.close();

	std::cout << __FUNCTION__ << ": " << "write off finished" << std::endl;

	PolyhedronDeform mesh_temp;
	clock_t start_ = clock(), end_;

	std::ifstream input("./KnotsConstrainOut/deformation_domain.off");
	if (!input || !(input >> mesh_temp) || mesh_temp.empty()) {
		std::cerr << "Cannot open  data/plane.off" << std::endl;
		return;
	}
	// Init the indices of the halfedges and the vertices.
	set_halfedgeds_items_id(mesh_temp);
	// Create a deformation object
	Surface_mesh_deformation deform_mesh(mesh_temp);
	// Definition of the region of interest (use the whole mesh_temp)
	vertex_iterator vb, ve;
	boost::tie(vb, ve) = vertices(mesh_temp);
	deform_mesh.insert_roi_vertices(vb, ve);

	map<int, vertex_descriptor > control_pmap;
	for (auto vit = surface->vertices_begin();vit != surface->vertices_end();vit++)
	{
		if (vit->is_feature() || vit->is_border())
		{
			vertex_descriptor control_ = *CGAL::cpp11::next(vb, vit->vertex_index());
			deform_mesh.insert_control_vertex(control_);
			control_pmap[vit->vertex_index()] = control_;
		}
	}

	bool is_matrix_factorization_OK = deform_mesh.preprocess();
	if (!is_matrix_factorization_OK) {
		std::cerr << "Error in preprocessing, check documentation of preprocess()" << std::endl;
		return;
	}

	for (auto vit = surface->vertices_begin(); vit != surface->vertices_end(); vit++)
	{
		if (vit->is_feature() || vit->is_border())
		{
			vertex_descriptor control_ = control_pmap.at(vit->vertex_index());
			Surface_mesh_deformation::Point constrained_pos_(vit->get_domain().x(), vit->get_domain().y(),0.0);
			deform_mesh.set_target_position(control_, constrained_pos_);
		}
	}

	// iterate 10 times and do not use energy based termination criterion
	deform_mesh.deform(10, 0.0);

	Mesh::Vertex_iterator sur_it = surface->vertices_begin();
	for (PolyhedronDeform::Vertex_iterator vit = mesh_temp.vertices_begin(); vit != mesh_temp.vertices_end(); vit++,sur_it++)
	{
		sur_it->get_domain() = Point_2(vit->point().x(), vit->point().y());
	}

	const QString obj_name = "./KnotsConstrainOut/deformation_domain.obj";
	std::ofstream fout_obj;
	fout_obj.open(obj_name.toStdString());
	if (fout_obj.is_open())
	{
		for (auto vit = mesh_temp.vertices_begin(); vit != mesh_temp.vertices_end(); vit++)
		{
			double x = vit->point().x();
			double y = vit->point().y();
			double z = vit->point().z();
			fout_obj << "v" << " " << x << " " << y << " " << z << "\n";
		}
		for (auto fit = mesh_temp.facets_begin(); fit != mesh_temp.facets_end(); fit++)
		{
			fout_obj << "f";

			PolyhedronDeform::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
			do
			{
				int v = hc->vertex()->id();
				fout_obj << " " << v + 1;
			} while (++hc != hs);
			fout_obj << "\n";
		}
	}
	fout_obj.close();

	end_ = clock();
	std::cout << __FUNCTION__ << ": " << "deformation domain spend time: " << end_ - start_ << std::endl;
}

