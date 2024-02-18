#include "Corefuncs/ImageApproximation.h"
#include <QtWidgets/QMessageBox>

/////////////////////////////////////////////////////////////
//test basis
void Image_Approximation::MakeTestKnotsDomain()
{
	vector<Point_2> points;
	points.clear();
	points.push_back(Point_2(0.0, 0.0));
	points.push_back(Point_2(0.0, 1.0));
	points.push_back(Point_2(1.0, 1.0));
	points.push_back(Point_2(1.0, 0.0));
	for (int i = 0; i < points.size(); i++)
	{
		CDT_Refine::Vertex_handle v_h_1, v_h_2;
		v_h_1 = cdt_domain.insert(points[i]);
		v_h_2 = cdt_domain.insert(points[(i + 1) % points.size()]);
		cdt_domain.insert_constraint(v_h_1, v_h_2);
	}
#if 1
	CDT_Refine::Vertex_handle v1, v2, v3, v4, v5;
	v1 = cdt_domain.insert(Point_2(0.2, 0.2));
	v2 = cdt_domain.insert(Point_2(0.3, 0.3));
	v3 = cdt_domain.insert(Point_2(0.4, 0.4));
	v4 = cdt_domain.insert(Point_2(0.5, 0.5));
	v5 = cdt_domain.insert(Point_2(0.6, 0.6));
	cdt_domain.insert_constraint(v1, v2);
	cdt_domain.insert_constraint(v2, v3);
	cdt_domain.insert_constraint(v3, v4);
	cdt_domain.insert_constraint(v4, v5);
	//CGAL::refine_Delaunay_mesh_2(cdt_domain, Criteria(0.125, 0.4));
#endif
	CGAL::refine_Delaunay_mesh_2(cdt_domain, Criteria(0.125, 0.3));
	int index_ = 0;
	for (auto vit = cdt_domain.vertices_begin(); vit != cdt_domain.vertices_end(); vit++)
	{
		vit->set_associated_index(index_);
		index_++;
	}

	knots_config = new CDT;
	map<int, CDT::Vertex_handle> knotmap;
	//cdt domain transfer
	for (auto fit = cdt_domain.faces_begin(); fit != cdt_domain.faces_end(); fit++)
	{
		CDT::Vertex_handle v0 = knots_config->insert(fit->vertex(0)->point());
		v0->set_associated_index(fit->vertex(0)->get_associated_index());

		CDT::Vertex_handle v1 = knots_config->insert(fit->vertex(1)->point());
		v1->set_associated_index(fit->vertex(1)->get_associated_index());

		CDT::Vertex_handle v2 = knots_config->insert(fit->vertex(2)->point());
		v2->set_associated_index(fit->vertex(2)->get_associated_index());

		knotmap[fit->vertex(0)->get_associated_index()] = v0;
		knotmap[fit->vertex(1)->get_associated_index()] = v1;
		knotmap[fit->vertex(2)->get_associated_index()] = v2;
	}
	knots_config->insert_constraint(knotmap[0], knotmap[1]);
	knots_config->insert_constraint(knotmap[1], knotmap[2]);
	knots_config->insert_constraint(knotmap[2], knotmap[3]);
	knots_config->insert_constraint(knotmap[3], knotmap[0]);

#if 0
	CDT::Vertex_handle v11, v12;
	for (auto vit = knots_config->vertices_begin(); vit != knots_config->vertices_end(); vit++)
	{
		if (vit->get_associated_index() == 11)
		{
			v11 = vit->handle();
		}
		if (vit->get_associated_index() == 12)
		{
			v12 = vit->handle();
		}
	}
	knots_config->insert_constraint(v11, v12);
#endif

	int num_ = cdt_domain.number_of_vertices();

	const QString obj_name = "./test_out/testkontsconfig.obj";
	std::ofstream fout_obj;
	fout_obj.open(obj_name.toStdString());
	if (fout_obj.is_open())
	{
		for (auto vit = cdt_domain.vertices_begin(); vit != cdt_domain.vertices_end(); vit++)
		{
			double x = vit->point().x();
			double y = vit->point().y();
			double z = 0.0;
			fout_obj << "v" << " " << x << " " << y << " " << z << "\n";
		}
		for (auto fit = cdt_domain.faces_begin(); fit != cdt_domain.faces_end(); fit++)
		{
			fout_obj << "f";

			int v0 = fit->vertex(0)->get_associated_index();
			fout_obj << " " << v0 + 1;
			int v1 = fit->vertex(1)->get_associated_index();
			fout_obj << " " << v1 + 1;
			int v2 = fit->vertex(2)->get_associated_index();
			fout_obj << " " << v2 + 1;
			fout_obj << "\n";
		}
	}
	fout_obj.close();

}

void Image_Approximation::compute_test_basis()
{
	mesh_basis = NULL;

#if 1

#if 1
	MakeTestKnotsDomain();

	for (auto vit = knots_config->vertices_begin(); vit != knots_config->vertices_end(); vit++)
	{
		KnotData data;
		data.pt = Point_2(vit->point().x(), vit->point().y());
		data.index = vit->get_associated_index();
		data.flag.bBoundary = is_on_domain_bound(vit->point());
		data.flag.bCorner = is_on_domain_corner(vit->point());
		dataMaps[vit->get_associated_index()] = data;
	}
	std::cout << "knots done: "<<cdt_domain.number_of_vertices() << std::endl;
#endif

#if 1
#if 0
	vector<int> select_ = { 0,1 };
	if (int(select_.size()) != int(nDeg))
	{
		int corner_ = select_[0];
		select_.clear();
		for (int i = 0; i<nDeg; i++)
		{
			select_.push_back(corner_);
		}
	}
	vector<SplineBasisConfigs> basisConfigs = bSplineBasis.basisConfigs;
	for (int k = 0; k<basisConfigs.size(); k++)
	{
		vector<unsigned> sel;
		sel.push_back(select_[0]); sel.push_back(select_[1]);
		sort(sel.begin(), sel.end());
		vector<unsigned> temp = basisConfigs[k].config;
		sort(temp.begin(), temp.end());
		if (equal(sel.begin(), sel.end(), temp.begin()))
		{
			basis_config = basisConfigs[k];
			break;
		}
		if (k == basisConfigs.size() - 1)
		{
			basis_config = basisConfigs[0];
		}
	}

	vector<TConfig3>		basistconfig3s;
	for (int i = 0; i < basis_config.tconfigs.size(); i++)
	{
		TConfig3 c_;
		c_.tconfig = basis_config.tconfigs[i].tconfig;
		basistconfig3s.push_back(c_);

		std::cout << "configs: " << std::endl;
		for (int j = 0; j<basis_config.tconfigs[i].tconfig.size(); j++)
		{
			std::cout << basis_config.tconfigs[i].tconfig[j] << "-";
		}
		std::cout << "\n";
	}
#else
	vector<TConfig3>		basistconfig3s;
	TConfig3 c_;
	//supplementary F3
	//c_.tconfig = { 4,26,7,5,6 };
	//basistconfig3s.push_back(c_);
	//c_.tconfig = { 21,7,26,5,6 };
	//basistconfig3s.push_back(c_);
	//c_.tconfig = { 23,28,7,5,6 };
	//basistconfig3s.push_back(c_);
	//c_.tconfig = { 4,7,28,5,6 };
	//basistconfig3s.push_back(c_);

	//F3
	//c_.tconfig = { 4,5,28,4,4 };
	//basistconfig3s.push_back(c_);
	//c_.tconfig = { 7,8,26,8,8};
	//basistconfig3s.push_back(c_);

	//F12
	/*c_.tconfig = { 4,6,28,5 };
	basistconfig3s.push_back(c_);
	c_.tconfig = { 23,28,6,5 };
	basistconfig3s.push_back(c_);
	c_.tconfig = { 4,6,26,5 };
	basistconfig3s.push_back(c_);*/

	/*c_.tconfig = { 23,28,5,4,6 };
	basistconfig3s.push_back(c_);*/
	//d1
	/*c_.tconfig = { 31,34,28,17 };
	basistconfig3s.push_back(c_);*/
	/*c_.tconfig = { 31,28,29,17 };
	basistconfig3s.push_back(c_);*/
	/*c_.tconfig = { 31,25,29,17 };
	basistconfig3s.push_back(c_);*/
	//d2
	//c_.tconfig = { 17,25,8,31 };
	//basistconfig3s.push_back(c_);
	//c_.tconfig = { 17,34,8,31 };
	//basistconfig3s.push_back(c_);
	//c_.tconfig = { 20,25,8,31 };
	//basistconfig3s.push_back(c_);
	//d2
	//c_.tconfig = { 31,25,29,17 };
	//basistconfig3s.push_back(c_);
	//c_.tconfig = {31,28,29,17};
	//basistconfig3s.push_back(c_);
	//c_.tconfig = { 31,34,28,17 };
	//basistconfig3s.push_back(c_);
	//d3
	/*c_.tconfig = {8,25,34,17,31};
	basistconfig3s.push_back(c_);*/
	/*c_.tconfig = { 34,28,25,17,31 };
	basistconfig3s.push_back(c_);*/
	/*c_.tconfig = { 28,29,25,17,31 };
	basistconfig3s.push_back(c_);*/

	//bound-d3
	//c_.tconfig = { 26,0,1,7,9,12 };
	//basistconfig3s.push_back(c_);

	//corner
	//c_.tconfig = { 11,12,1,1,1 };
	//basistconfig3s.push_back(c_);
#endif

#else
	std::cout << "basis begin" << std::endl;

	//basis sample
	vector<Point_2> data_domain;
	int size_ = 3000;
	for (int i = 0;i<size_;i++)
	{
		for (int j = 0;j<size_;j++)
		{
			double x_ = double(i) / (size_ - 1);
			double y_ = double(j) / (size_ - 1);
			Point_2 p(x_, y_);
			data_domain.push_back(p);
		}
	}

	
	vector<SplineBasisConfigs> basisConfigs = bSplineBasis.basisConfigs;
	vector<TConfig3>	basisConfigs3;
//#pragma omp parallel for
	for (int k = 0; k < basisConfigs.size(); k++)
	{
		for (auto cit = basisConfigs[k].tconfigs.begin(); cit != basisConfigs[k].tconfigs.end(); cit++)
		{
			TConfig3 c_;
			c_.tconfig = cit->tconfig;			

			//convex
			vector<Point_2> pts, convex_hull;
			for (int j = 0; j<cit->tconfig.size(); j++)
			{
				pts.push_back(dataMaps[cit->tconfig[j]].pt);
			}
			CGAL::convex_hull_2(pts.begin(), pts.end(), back_inserter(convex_hull));
			//box predicate
			vector<double> box_ = return_box2d_from_vector_point(pts);
			for (unsigned int in = 0;in<data_domain.size();in++)
			{
				if (data_domain[in].x() >= (box_[0] - 1e-8) && data_domain[in].x() <= (box_[1] + 1e-8)
					&& data_domain[in].y() >= (box_[2] - 1e-8) && data_domain[in].y() <= (box_[3] + 1e-8))
				{
					int ret = CGAL::bounded_side_2(convex_hull.begin(), convex_hull.end(), data_domain[in], K());
					if (ret != CGAL::ON_UNBOUNDED_SIDE || is_on_polygon_convex_bound(data_domain[in], convex_hull))
					{
						DomainValue dv;
						dv.pt = data_domain[in];
						c_.supports[in] = dv;
					}
				}
			}
			basisConfigs3.push_back(c_);
		}
	}
	clock_t start = clock();
	for (auto cit = basisConfigs3.begin(); cit != basisConfigs3.end(); cit++)
	{
		//convex
		vector<Point_2> pts;
		for (int j = 0; j < cit->tconfig.size(); j++)
		{
			pts.push_back(dataMaps[cit->tconfig[j]].pt);
		}
		//simplex spline
		double** dataPts = new double*[pts.size()];//[2];
		for (int i = 0; i < pts.size(); i++)
			dataPts[i] = new double[2];
		for (int i = 0; i < pts.size(); i++)
		{
			dataPts[i][0] = pts[i].x();
			dataPts[i][1] = pts[i].y();
		}
		Simplex_Spline_n spline(dataPts, pts.size() - 3, 1, 1);

		for (auto itn = cit->supports.begin(); itn != cit->supports.end(); itn++)
		{
			double pt[2] = { itn->second.pt.x(),itn->second.pt.y() };
			double value_ = spline.computeSimplexSpline(pt);
			itn->second.value = value_;
		}
		for (int i = 0; i < pts.size(); i++)
			delete[]dataPts[i];
		delete[]dataPts;
	}
	
	clock_t finish = clock();
	double t_basis_process = (double(finish - start)) / CLOCKS_PER_SEC;
	std::cout << "timing:" << t_basis_process << "\n";
#endif

#if 0
	//basis sample
	CDT_Refine cdt_sample;
	vector<Point_2> points, convex_;
	//insert convex
	vector<int> pre_points;
	for (auto cit = basistconfig3s.begin(); cit != basistconfig3s.end(); cit++)
	{
		for (int i = 0; i < cit->tconfig.size(); i++)
		{
			pre_points.push_back(dataMaps[cit->tconfig[i]].index);
		}
	}
	remove_same_elements(pre_points);
	for (int i = 0; i < pre_points.size(); i++)
	{
		points.push_back(dataMaps[pre_points[i]].pt);
	}
	CGAL::convex_hull_2(points.begin(), points.end(), back_inserter(convex_));

	for (int i = 0; i < convex_.size(); i++)
	{
		CDT_Refine::Vertex_handle v_h_1, v_h_2;
		v_h_1 = cdt_sample.insert(convex_[i]);
		v_h_2 = cdt_sample.insert(convex_[(i + 1) % convex_.size()]);
		cdt_sample.insert_constraint(v_h_1, v_h_2);
	}

	/*CDT_Refine::Vertex_handle v_h_1, v_h_2;
	v_h_1 = cdt_sample.insert(dataMaps[22].pt);
	v_h_2 = cdt_sample.insert(dataMaps[21].pt);
	cdt_sample.insert_constraint(v_h_1, v_h_2);

	v_h_1 = cdt_sample.insert(dataMaps[21].pt);
	v_h_2 = cdt_sample.insert(dataMaps[20].pt);
	cdt_sample.insert_constraint(v_h_1, v_h_2);

	v_h_1 = cdt_sample.insert(dataMaps[20].pt);
	v_h_2 = cdt_sample.insert(dataMaps[69].pt);
	cdt_sample.insert_constraint(v_h_1, v_h_2);

	v_h_1 = cdt_sample.insert(dataMaps[69].pt);
	v_h_2 = cdt_sample.insert(dataMaps[22].pt);
	cdt_sample.insert_constraint(v_h_1, v_h_2);*/

	CGAL::refine_Delaunay_mesh_2(cdt_sample, Criteria(0.125, 0.0008));
	std::cout << "basis sample points: " << cdt_sample.number_of_vertices() << std::endl;

	//test
	/*Polygon_2 poly_;
	poly_.push_back(dataMaps[22].pt);
	poly_.push_back(dataMaps[21].pt);
	poly_.push_back(dataMaps[20].pt);
	poly_.push_back(dataMaps[69].pt);
	for (auto fit = cdt_sample.faces_begin(); fit != cdt_sample.faces_end(); fit++)
	{
	Point_2 p0 = fit->vertex(0)->point();
	Point_2 p1 = fit->vertex(1)->point();
	Point_2 p2 = fit->vertex(2)->point();

	Point_2 pt1((p0.x() + p1.x() + p2.x()) / 3.0, (p0.y() + p1.y() + p2.y()) / 3.0);
	Point_2 pt2((p0.x() + 2 * p1.x() + 2 * p2.x()) / 5.0, (p0.y() + 2 * p1.y() + 2 * p2.y()) / 5.0);
	Point_2 pt3((2 * p0.x() + p1.x() + p2.x()) / 4.0, (2 * p0.y() + p1.y() + p2.y()) / 4.0);
	if (poly_.bounded_side(pt1) == CGAL::ON_UNBOUNDED_SIDE ||
	poly_.bounded_side(pt2) == CGAL::ON_UNBOUNDED_SIDE ||
	poly_.bounded_side(pt3) == CGAL::ON_UNBOUNDED_SIDE)
	{
	cdt_sample.delete_face(fit);
	}
	}*/

	int index_ = 0;
	for (auto vit = cdt_sample.vertices_begin(); vit != cdt_sample.vertices_end(); vit++)
	{
		vit->set_associated_index(index_);
		index_++;
	}

	for (auto cit = basistconfig3s.begin(); cit != basistconfig3s.end(); cit++)
	{
		//convex
		vector<Point_2> pts, convex_hull;
		for (int j = 0; j<cit->tconfig.size(); j++)
		{
			pts.push_back(dataMaps[cit->tconfig[j]].pt);
		}
		CGAL::convex_hull_2(pts.begin(), pts.end(), back_inserter(convex_hull));

		//simplex spline
		double** dataPts = new double*[pts.size()];//[2];
		for (int i = 0; i < pts.size(); i++)
			dataPts[i] = new double[2];
		for (int i = 0; i < pts.size(); i++)
		{
			dataPts[i][0] = pts[i].x();
			dataPts[i][1] = pts[i].y();
		}
		Simplex_Spline_n spline(dataPts, pts.size() - 3, 1, 1);

		//box predicate
		vector<CDT_Refine::Vertex_iterator> vIndices;
		vector<double> box_ = return_box2d_from_vector_point(pts);
		for (auto vit = cdt_sample.vertices_begin(); vit != cdt_sample.vertices_end(); vit++)
		{
			if (vit->point().x() >= (box_[0] - 1e-8) && vit->point().x() <= (box_[1] + 1e-8)
				&& vit->point().y() >= (box_[2] - 1e-8) && vit->point().y() <= (box_[3] + 1e-8))
			{
				vIndices.push_back(vit);
			}
		}
		//convex predicate and compute simplex
		for (int i = 0; i<vIndices.size(); i++)
		{
			int ret = CGAL::bounded_side_2(convex_hull.begin(), convex_hull.end(), vIndices[i]->point(), K());
			if (ret != CGAL::ON_UNBOUNDED_SIDE || is_on_polygon_convex_bound(vIndices[i]->point(), convex_hull))
			{
				DomainValue dv;
				dv.pt = vIndices[i]->point();
				double pt[2] = { dv.pt.x(),dv.pt.y() };
				double value_ = spline.computeSimplexSpline(pt);
				dv.value = value_;
				cit->supports[vIndices[i]->get_associated_index()] = dv;
			}
		}

		for (int i = 0; i < pts.size(); i++)
			delete[]dataPts[i];
		delete[]dataPts;
	}

	//compute assemble value
	map<int, DomainValue> allsupport;
	for (auto cit = basistconfig3s.begin(); cit != basistconfig3s.end(); cit++)
	{
		for (auto sit = cit->supports.begin(); sit != cit->supports.end(); sit++)
		{
			allsupport.insert(*sit);
		}
	}
	for (auto sit = allsupport.begin(); sit != allsupport.end(); sit++)
	{
		sit->second.value = 0.0;
	}
	for (auto cit = basistconfig3s.begin(); cit != basistconfig3s.end(); cit++)
	{
		Point_2 p1 = dataMaps[cit->tconfig[0]].pt;
		Point_2 p2 = dataMaps[cit->tconfig[1]].pt;
		Point_2 p3 = dataMaps[cit->tconfig[2]].pt;
		double absarea = abs(CGAL::area(p1, p2, p3));
		for (auto sit = cit->supports.begin(); sit != cit->supports.end(); sit++)
		{
			allsupport[sit->first].value += sit->second.value * absarea;
		}
	}

	vector<double> basis_value;
	basis_value.resize(cdt_sample.number_of_vertices());
	for (int i = 0; i < basis_value.size(); i++)
	{
		double value_ = 0.0;
		map<int, DomainValue>::iterator res = allsupport.find(i);
		if (res != allsupport.end())
		{
			value_ = res->second.value;
		}
		basis_value[i] = value_;
	}

	mesh_basis = new Mesh;
	from_triangulationdata_to_mesh(cdt_sample, &mesh_basis, basis_value);

#else

	//test to MatLab
#if 0
map<int, CDT::Vertex_iterator> knotmap;
for (auto vit = knots_config->vertices_begin(); vit != knots_config->vertices_end(); vit++)
{
	knotmap[vit->get_associated_index()] = vit;
}
//output to MatLab
const QString trigu_filename = "./test_out/knotlines.txt";
std::ofstream fout_tri;
fout_tri.open(trigu_filename.toStdString());
if (fout_tri.is_open())
{
	for (auto fit = knots_config->faces_begin(); fit != knots_config->faces_end(); fit++)
	{
		double x1 = fit->vertex(0)->point().x();
		double y1 = fit->vertex(0)->point().y();
		double x2 = fit->vertex(1)->point().x();
		double y2 = fit->vertex(1)->point().y();
		double x3 = fit->vertex(2)->point().x();
		double y3 = fit->vertex(2)->point().y();
		fout_tri << x1 << " " << x2 << " " << x3 << " " << y1 << " " << y2 << " " << y3 << "\n";
	}
}
fout_tri.close();

//save obj
QString knot_config_obj = "./test_out/knot_obj.obj";;
const string obj_name = knot_config_obj.toStdString();
std::ofstream fout_obj;
fout_obj.open(obj_name);
if (fout_obj.is_open())
{
	std::cout << "number of all knots now: " << knots_config->number_of_vertices() << std::endl;
	for (int i = 0; i < knots_config->number_of_vertices(); i++)
	{
		CDT::Vertex_handle vh = knotmap.at(i);
		if (vh->get_associated_index() == i)
		{
			double x = vh->point().x();
			double y = vh->point().y();
			double z = 0.0;
			fout_obj << "v" << " " << x << " " << y << " " << z << "\n";
		}
	}
	for (auto fit = knots_config->faces_begin(); fit != knots_config->faces_end(); fit++)
	{
		fout_obj << "f";

		int v0 = fit->vertex(0)->get_associated_index();
		fout_obj << " " << v0 + 1;
		int v1 = fit->vertex(1)->get_associated_index();
		fout_obj << " " << v1 + 1;
		int v2 = fit->vertex(2)->get_associated_index();
		fout_obj << " " << v2 + 1;
		fout_obj << "\n";
	}
}
fout_obj.close();

//output config to MatLab
const QString basis_points_file = "./test_out/points.txt";
std::ofstream fout_bp;
fout_bp.open(basis_points_file.toStdString());
if (fout_bp.is_open())
{
	//inte
	vector<unsigned> inte;
	for (int i = 3; i < basistconfig3s[0].tconfig.size(); i++)
	{
		inte.push_back(basistconfig3s[0].tconfig[i]);
	}

	for (auto vit = knots_config->vertices_begin(); vit != knots_config->vertices_end(); vit++)
	{
		int points_type = 0;//normal-0, interior-1, outer-2	
		bool is_inte = false;
		for (int i = 0; i < inte.size(); i++)
		{
			if (inte[i] == vit->get_associated_index())
			{
				is_inte = true;
				break;
			}
		}
		if (is_inte)
		{
			points_type = 1;
		}
		//outer points
		bool is_outer = false;
		for (int i = 0; i < basistconfig3s.size(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (correspondence.find(vit->get_associated_index()) != correspondence.end())
				{
					if (correspondence.at(vit->get_associated_index()) == basistconfig3s[i].tconfig[j])
					{
						is_outer = true;
						break;
					}
				}
				if (basistconfig3s[i].tconfig[j] == vit->get_associated_index())
				{
					is_outer = true;
					break;
				}
			}
			if (is_outer)
			{
				break;
			}
		}
		if (is_outer)
		{
			if (!is_inte)
			{
				points_type = 2;
			}
		}
		//std::cout<< vit->get_associated_index() << "-" << points_type << "\n";
		fout_bp << vit->point().x() << " " << vit->point().y() << " " << points_type << "\n";
	}
}
fout_bp.close();

//output support to MatLab
const QString basis_support = "./test_out/support.txt";
std::ofstream fout_su;
fout_su.open(basis_support.toStdString());
if (fout_su.is_open())
{
	for (int i = 0; i < basistconfig3s.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			fout_su << dataMaps[basistconfig3s[i].tconfig[j]].pt.x() << " " << dataMaps[basistconfig3s[i].tconfig[j]].pt.y() << " ";
		}
		fout_su << "\n";
	}
}
fout_su.close();

//output LT-triangle to MatLab
const QString basis_tri = "./test_out/LT-tri.txt";
std::ofstream fout_lt;
fout_lt.open(basis_tri.toStdString());
if (fout_lt.is_open())
{
	for (auto it = basistconfig3s.begin(); it != basistconfig3s.end(); it++)
	{
		for (int i = 0; i < 3; i++)
		{
			double x = dataMaps[it->tconfig[i]].pt.x();
			fout_lt << x << " ";
		}
		for (int i = 0; i < 3; i++)
		{
			double y = dataMaps[it->tconfig[i]].pt.y();
			fout_lt << y << " ";
		}
		fout_lt << "\n";
	}
}
fout_lt.close();
#endif // 0

	//sample
	CDT_Refine cdt_sample;
	vector<Point_2> points, convex_;
	//insert convex
	vector<int> pre_points;
	for (auto cit = basistconfig3s.begin(); cit != basistconfig3s.end(); cit++)
	{
		for (int i = 0; i < cit->tconfig.size(); i++)
		{
			pre_points.push_back(dataMaps[cit->tconfig[i]].index);
		}
	}
	remove_same_elements(pre_points);
	for (int i = 0; i < pre_points.size(); i++)
	{
		points.push_back(dataMaps[pre_points[i]].pt);
	}
	CGAL::convex_hull_2(points.begin(), points.end(), back_inserter(convex_));

	for (int i = 0; i < convex_.size(); i++)
	{
		CDT_Refine::Vertex_handle v_h_1, v_h_2;
		v_h_1 = cdt_sample.insert(convex_[i]);
		v_h_2 = cdt_sample.insert(convex_[(i + 1) % convex_.size()]);
		cdt_sample.insert_constraint(v_h_1, v_h_2);
	}
	CGAL::refine_Delaunay_mesh_2(cdt_sample, Criteria(0.125, 0.0008));
	std::cout << "basis sample points: " << cdt_sample.number_of_vertices() << std::endl;

	int index_ = 0;
	for (auto vit = cdt_sample.vertices_begin(); vit != cdt_sample.vertices_end(); vit++)
	{
		vit->set_associated_index(index_);
		index_++;
	}

	for (auto cit = basistconfig3s.begin(); cit != basistconfig3s.end(); cit++)
	{
		//convex
		vector<Point_2> pts, convex_hull;
		for (int j = 0; j < cit->tconfig.size(); j++)
		{
			pts.push_back(dataMaps[cit->tconfig[j]].pt);
		}
		CGAL::convex_hull_2(pts.begin(), pts.end(), back_inserter(convex_hull));

		//simplex spline
		double** dataPts = new double*[pts.size()];//[2];
		for (int i = 0; i < pts.size(); i++)
			dataPts[i] = new double[2];
		for (int i = 0; i < pts.size(); i++)
		{
			dataPts[i][0] = pts[i].x();
			dataPts[i][1] = pts[i].y();
		}
		Simplex_Spline_n spline(dataPts, pts.size() - 3, 1, 1);

		//box predicate
		vector<CDT_Refine::Vertex_iterator> vIndices;
		vector<double> box_ = return_box2d_from_vector_point(pts);
		for (auto vit = cdt_sample.vertices_begin(); vit != cdt_sample.vertices_end(); vit++)
		{
			if (vit->point().x() >= (box_[0] - 1e-8) && vit->point().x() <= (box_[1] + 1e-8)
				&& vit->point().y() >= (box_[2] - 1e-8) && vit->point().y() <= (box_[3] + 1e-8))
			{
				vIndices.push_back(vit);
			}
		}
		//convex predicate and compute simplex
		for (int i = 0; i < vIndices.size(); i++)
		{
			int ret = CGAL::bounded_side_2(convex_hull.begin(), convex_hull.end(), vIndices[i]->point(), K());
			if (ret != CGAL::ON_UNBOUNDED_SIDE || is_on_polygon_convex_bound(vIndices[i]->point(), convex_hull))
			{
				DomainValue dv;
				dv.pt = vIndices[i]->point();
				double pt[2] = { dv.pt.x(),dv.pt.y() };
				double value_ = spline.computeSimplexSpline(pt);
				//double dir[2] = {1,0};
				//double value_ = spline.firstdirectionalderivate(pt,dir);
				dv.value = value_;
				cit->supports[vIndices[i]->get_associated_index()] = dv;
			}
		}

		for (int i = 0; i < pts.size(); i++)
			delete[]dataPts[i];
		delete[]dataPts;
	}

	//compute assemble value
	map<int, DomainValue> allsupport;
	for (auto cit = basistconfig3s.begin(); cit != basistconfig3s.end(); cit++)
	{
		for (auto sit = cit->supports.begin(); sit != cit->supports.end(); sit++)
		{
			allsupport.insert(*sit);
		}
	}
	for (auto sit = allsupport.begin(); sit != allsupport.end(); sit++)
	{
		sit->second.value = 0.0;
	}
	for (auto cit = basistconfig3s.begin(); cit != basistconfig3s.end(); cit++)
	{
		Point_2 p1 = dataMaps[cit->tconfig[0]].pt;
		Point_2 p2 = dataMaps[cit->tconfig[1]].pt;
		Point_2 p3 = dataMaps[cit->tconfig[2]].pt;
		double absarea = abs(CGAL::area(p1, p2, p3));
		for (auto sit = cit->supports.begin(); sit != cit->supports.end(); sit++)
		{
			allsupport[sit->first].value += sit->second.value * absarea;
		}
	}

	vector<double> basis_value;
	basis_value.resize(cdt_sample.number_of_vertices());
	double max_v = 0;
	for (int i = 0; i < basis_value.size(); i++)
	{
		double value_ = 0.0;
		map<int, DomainValue>::iterator res = allsupport.find(i);
		if (res != allsupport.end())
		{
			value_ = res->second.value;
		}
		if (value_ > max_v)
		{
			max_v = value_;
		}
		basis_value[i] = value_;
	}
	std::cout << "max value of basis function: " << max_v << std::endl;

	mesh_basis = new Mesh;
	from_triangulationdata_to_mesh(cdt_sample, &mesh_basis, basis_value);
	mesh_basis->write_obj("./splinef.obj",false);
	
#endif
#endif
	std::cout << "compute basis down" << std::endl;
}
//////////////////////////////////////////////////////////////////////////////////////

vector<Point_2>	Image_Approximation::get_basis_normalize_wrongp()
{
	return wrong_points;
}

void Image_Approximation::extract_image_feature()
{
	compute_image_feature();
	//smooth_and_sample_feature_curve_points();
}

vector<cv::Point2i> Image_Approximation::return_image_anchor()
{
	return image_anchor;
}

void Image_Approximation::build_collinear_knots_test()
{
	out_file_name = "Results/";
	out_file_name.append(fileName.split("/").last().split(".").at(0));
	out_file_name.append(QString("/ite%1-d%2").arg(Nite_now).arg(nDeg));
	generate_output_directory(out_file_name);

	compute_image_feature();

	smooth_and_sample_feature_curve_points();

	project_featureps_onto_smoothps();

	//test
	//points with color
	const QString mesh_p = "./test_out/mesh_p.txt";
	std::ofstream fout_p;
	fout_p.open(mesh_p.toStdString());
	if (fout_p.is_open())
	{
		for (auto vit = originalMesh->vertices_begin(); vit != originalMesh->vertices_end(); vit++)
		{
			fout_p << vit->point().x() << " " << vit->point().y() << " ";
			if (vit->is_feature())
			{
				fout_p << 0 << " " << 0 << " " << 0.5 << "\n";
			}
			else
			{
				fout_p << vit->vertex_pixel_color().x() << " "
					<< vit->vertex_pixel_color().y() << " " << vit->vertex_pixel_color().z() << "\n";
			}
		}
	}
	fout_p.close();
	//edge with color
	const QString mesh_ = "./test_out/mesh_f.txt";
	std::ofstream fout_;
	fout_.open(mesh_.toStdString());
	if (fout_.is_open())
	{
		for (auto fit = originalMesh->facets_begin(); fit != originalMesh->facets_end(); fit++)
		{
			Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
			do
			{
				fout_ << hc->vertex()->vertex_index() + 1 << " ";
			} while (++hc != hs);
			fout_ << "\n";
		}
	}
	fout_.close();
	//feature edge with color
	const QString mesh_ef = "./test_out/mesh_e.txt";
	std::ofstream fout_ef;
	fout_ef.open(mesh_ef.toStdString());
	if (fout_ef.is_open())
	{
		for (auto eit = originalMesh->edges_begin(); eit != originalMesh->edges_end(); eit++)
		{
			if (eit->vertex()->is_feature() && eit->opposite()->vertex()->is_feature())
			{
				fout_ef << eit->vertex()->point().x() << " " << eit->vertex()->point().y() << " "
					<< eit->opposite()->vertex()->point().x() << " " << eit->opposite()->vertex()->point().y() << "\n";
			}
		}
	}
	fout_ef.close();

	compute_constrained_parameterization();
	create_kd_tree();
	remedy_color_of_one_neighbor();

	resample_domain_image_on_mesh();

	build_collinear_knots();
}

void Image_Approximation::matching_feature_with_polyline_test(double e)
{
	feature_lines_modify.clear();
	image_feature_lines_set.clear();
	has_compute_feature = false;
	bsimplyfeature = false;

	//construct file
	out_file_name = "Results/";
	out_file_name.append(fileName.split("/").last().split(".").at(0));
	out_file_name.append(QString("/ite%1-d%2").arg(Nite_now).arg(nDeg));
	generate_output_directory(out_file_name);

	compute_image_feature();
	smooth_and_sample_feature_curve_points();
	project_featureps_onto_smoothps();

	matching_feature_with_polyline(e);
}

void Image_Approximation::compute_constrained_parameterization_test()
{
	compute_constrained_parameterization();
}

void Image_Approximation::Init_knots_triangulation_test()
{
	//construct file
	out_file_name = "Results/";
	out_file_name.append(fileName.split("/").last().split(".").at(0));

	compute_image_feature();
	smooth_and_sample_feature_curve_points();
	project_featureps_onto_smoothps();
	compute_constrained_parameterization();
	create_kd_tree();
	remedy_color_of_one_neighbor();
	sample_feature_region(featureregion_sampleps, featureregion_originalps);
	sample_feature_curve_domainps();//after ploy-line, the number of circle feature points plus 1
	resample_domain_image_on_mesh();
	
	QString dir_featureknot_cdt;
	QString dir_featureknot_edge;
	QString dir_number_fixed_knots;

	QString dir_allknot = out_file_name;
	dir_allknot.append("/midprocess_optiknots.obj");

	build_collinear_knots();
	if (Nite_now != 0)
	{
		QString	outputDir = last_file_name;
		dir_featureknot_cdt = outputDir;
		dir_featureknot_cdt.append("/forIte_knot_mesh.obj");

		dir_featureknot_edge = outputDir;
		dir_featureknot_edge.append("/forIte_featureknotsedge.txt");

		dir_number_fixed_knots = outputDir;
		dir_number_fixed_knots.append("/forIte_numberfixedknots.txt");
	}
	else
	{
		dir_featureknot_cdt = out_file_name;
		dir_featureknot_cdt.append("/midprocess_featureknotscdt.obj");

		dir_featureknot_edge = out_file_name;
		dir_featureknot_edge.append("/midprocess_featureknotsedge.txt");
	}
	std::cout << "/////////////////////////////optimize knots mesh begin " << std::endl;
	//by Yanyang
	if (opti_knottri == NULL)
	{
		opti_knottri = new OptiKnotTri;
	}
	opti_knottri->clear();
	opti_knottri->set_accelerator(OptiKnotTri::Accelerator_None);

	QString gray_file_name = out_file_name;
	gray_file_name.append("/image_domain_gray.png");

	//image
	int w, h, c;
	stbi_set_flip_vertically_on_load(true);
	unsigned char* data = stbi_load(gray_file_name.toLatin1(), &w, &h, &c, 0);
	if (data)
	{
		if (c > 3)
		{
			printf("%s: not support image with channel > 3\n", __FUNCTION__);
		}
		else
		{
			printf("%s: success, width = %d, height = %d, channel = %d\n", __FUNCTION__, w, h, c);
			opti_knottri->set_image(data, w, h, c);
		}
		stbi_image_free(data);
	}
	else
	{
		printf("%s: failed\n", __FUNCTION__);
	}
	if (Nite_now == 0)
	{
		nfixedknots = Norepeat_knot.size();
	}
	else
	{
		std::ifstream file(dir_number_fixed_knots.toLatin1());
		if (!file)
			return;
		file >> nfixedknots;
		file.close();
	}
	opti_knottri->load_mesh(dir_featureknot_cdt.toLatin1(), nfixedknots);
	opti_knottri->load_fixed_edge(dir_featureknot_edge.toLatin1());
	opti_knottri->set_degree(1);
	if (Nite_now == 0)
	{
		QString dir_initknot = out_file_name;
		dir_initknot.append("/");
		dir_initknot.append("Initknots_tri.obj");
		opti_knottri->init_greedy_mesh(Norepeat_knot.size() + nknotinsert_opti, dir_initknot.toLatin1());
	}
}

void Image_Approximation::optimize_knots_triangulation_test()
{
	//construct file
	out_file_name = "Results/";
	out_file_name.append(fileName.split("/").last().split(".").at(0));
	out_file_name.append(QString("/ite%1-d%2").arg(Nite_now).arg(nDeg));
	generate_output_directory(out_file_name);
	if (Nite_now > 0)
	{
		last_file_name = "Results/";
		last_file_name.append(fileName.split("/").last().split(".").at(0));
		last_file_name.append(QString("/ite%1-d%2").arg(Nite_now - 1).arg(nDeg));
		generate_output_directory(last_file_name);
	}

	compute_image_feature();
	smooth_and_sample_feature_curve_points();
	project_featureps_onto_smoothps();
	compute_constrained_parameterization();
	create_kd_tree();
	remedy_color_of_one_neighbor();
	sample_feature_region(featureregion_sampleps, featureregion_originalps);
	sample_feature_curve_domainps();//after ploy-line, the number of circle feature points plus 1
	resample_domain_image_on_mesh();
	optimize_knots_triangulation();
}

void Image_Approximation::get_resampled_image_and_knotstri(QImage & image_, Mesh&mesh_)
{
	QString file_name = out_file_name;
	file_name.append("/image_domainrgb.png");
	image_.load(file_name);

	//mesh_ = *knot_mesh;
}

void Image_Approximation::postprocess_feature_corner_controlps()
{

}

double &Image_Approximation::get_error_threshold()
{
	return err_thres;
}

void Image_Approximation::get_inserted_regions(vector<pair<int, int>>&pixels, vector<vector<pair<int, int>>> &regions)
{
	pixels = insert_pixels;
	regions = insert_regions;
}

//sample
void Image_Approximation::sample_feature_line(vector<vector<RGBPoint>> &sampleps)
{
	vector<vector<Point_2>> feature_sampleps;
	for (int i = 0; i < collinear_knots_lines.size(); i++)
	{
		vector<Point_2> samplelines;
		for (int j = 0; j < collinear_knots_lines[i].size() - 1; j++)
		{
			vector<Point_2> sample2seg;
			Point_2 pstart, pend;
			pstart = collinear_knots_lines[i][j].paradomain;
			pend = collinear_knots_lines[i][j + 1].paradomain;
			SamplingPoint2dInLineUniformly(pstart, pend, 100, sample2seg);
			sample2seg.pop_back();
			samplelines.insert(samplelines.end(), sample2seg.begin(), sample2seg.end());
		}
		feature_sampleps.push_back(samplelines);
	}

	vector<Point_2> sampleallp;
	for (int i = 0; i < feature_sampleps.size(); i++)
	{
		for (int j = 0; j < feature_sampleps[i].size(); j++)
		{
			sampleallp.push_back(feature_sampleps[i][j]);
		}
	}

	//compute basis value
	BSplineBasis new_bsplinebasis = bSplineBasis;
	vector<TConfig3> new_alltconfig3s = alltconfig3s;
	new_alltconfig3s.clear();
	compute_sample_basis(sampleallp, new_bsplinebasis, dataMaps, new_alltconfig3s);

	//combine basis value and control point
	vector<RGBPoint> color_value;
	vector<double> verify_basis;
	color_value.resize(sampleallp.size());
	verify_basis.resize(sampleallp.size());
	for (int key = 0; key < color_value.size(); key++)
	{
		verify_basis[key] = 0.0;
		color_value[key] = RGBPoint{ 0.0, 0.0, 0.0 ,0.0,0.0};
		for (int i = 0; i < new_bsplinebasis.basisMergeInfos.size(); i++)
		{
			int flag = new_bsplinebasis.basisMergeInfos[i].index;
			if (flag != -1)
			{
				for (int j = 0; j < new_bsplinebasis.basisMergeInfos[i].basisMerge.size(); j++)
				{
					int index_basis = new_bsplinebasis.basisMergeInfos[i].basisMerge[j];
					map<unsigned int, DomainValue>::iterator ite = new_bsplinebasis.basisConfigs[index_basis].supports.find(key);
					if (ite != new_bsplinebasis.basisConfigs[index_basis].supports.end())
					{
						verify_basis[key] = verify_basis[key] + ite->second.value;
						color_value[key].x = color_value[key].x + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.x;
						color_value[key].y = color_value[key].y + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.y;
						color_value[key].r = color_value[key].r + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.r;
						color_value[key].g = color_value[key].g + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.g;
						color_value[key].b = color_value[key].b + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.b;
					}
				}
			}
		}
	}
	sampleps.clear();
	int index_ = 0;
	for (int i = 0; i < feature_sampleps.size(); i++)
	{
		vector<RGBPoint> samplelinep3s;
		for (int j = 0; j < feature_sampleps[i].size(); j++)
		{
			samplelinep3s.push_back(color_value[index_]);
			index_++;
		}
		sampleps.push_back(samplelinep3s);
		std::cout << "number of feature line sample: " << samplelinep3s.size() << std::endl;
	}

	std::cout << "feature sample finished" << std::endl;
}

void Image_Approximation::sample_local_image(Point_2 psta, Point_2 pend, CDT &cdt_, vector<RGBPoint> &values_)
{
	values_.clear();
	if (cdt_.number_of_vertices() > 0) cdt_.clear();

	double insert_rate = 2.0;
	double insert_rate_feature = 2.0;

	Point_2 p1(psta.x() + imagemesh_dx, (psta.y() + imagemesh_dy) / (2 * imagemesh_dy));
	Point_2 p2(pend.x() + imagemesh_dx, (pend.y() + imagemesh_dy) / (2 * imagemesh_dy));
	vector<Point_2> points;
	points.clear();
	points.push_back(p1);
	points.push_back(Point_2(p1.x(), p2.y()));
	points.push_back(p2);
	points.push_back(Point_2(p2.x(), p1.y()));

	//sample segment
	for (auto eit = fittedMesh->edges_begin(); eit != fittedMesh->edges_end(); eit++)
	{
		Mesh::Vertex_iterator mv1, mv2;
		mv1 = eit->vertex();
		mv2 = eit->opposite()->vertex();

		int res1 = CGAL::bounded_side_2(points.begin(), points.end(), mv1->get_domain());
		int res2 = CGAL::bounded_side_2(points.begin(), points.end(), mv2->get_domain());
		if (!(res1 == CGAL::ON_BOUNDED_SIDE || res2 == CGAL::ON_BOUNDED_SIDE))
		{
			continue;
		}

		vector<Point_2> segps;
		CDT::Vertex_handle vbeg, vend;
		vbeg = cdt_.insert(mv1->get_domain());
		vbeg->set_new_status(mv1->is_feature());
		vend = cdt_.insert(mv2->get_domain());
		vend->set_new_status(mv2->is_feature());
		if (mv1->is_feature() || mv2->is_feature())
		{
			SamplingPoint2dInLineUniformly(mv1->get_domain(), mv2->get_domain(), insert_rate_feature, segps);
		}
		else
		{
			SamplingPoint2dInLineUniformly(mv1->get_domain(), mv2->get_domain(), insert_rate, segps);
		}
		if (segps.size() == 0)
		{
			cdt_.insert_constraint(vbeg, vend);
			continue;
		}
		for (int i = 0; i < segps.size() - 1; i++)
		{
			CDT::Vertex_handle v1, v2;
			v1 = cdt_.insert(segps[i]);
			v2 = cdt_.insert(segps[(i + 1) % segps.size()]);

			if (i == 0)
			{
				if (mv1->is_feature() && mv2->is_feature())
				{
					v1->set_new_status(true);
				}
				else
				{
					v1->set_new_status(false);
				}
				cdt_.insert_constraint(vbeg, v1);
			}
			if (segps.size() > 1)
			{
				if (mv1->is_feature() && mv2->is_feature())
				{
					v1->set_new_status(true);
				}
				else
				{
					v1->set_new_status(false);
				}
				cdt_.insert_constraint(v1, v2);
			}

			if (i == segps.size() - 2)
			{
				if (mv1->is_feature() && mv2->is_feature())
				{
					v2->set_new_status(true);
				}
				else
				{
					v2->set_new_status(false);
				}
				cdt_.insert_constraint(v2, vend);
			}
		}
	}

	//sample inte-triangle
	insert_rate += 2.0;
	insert_rate_feature += 2.0;
	for (auto fit = fittedMesh->facets_begin(); fit != fittedMesh->facets_end(); fit++)
	{
		vector<Point_2> triangle;
		bool bhas_feature = false;
		Mesh::Halfedge_around_facet_circulator hfir = fit->facet_begin(), hend = hfir;
		bool is_in_quad = false;
		do
		{
			int res = CGAL::bounded_side_2(points.begin(), points.end(), hfir->vertex()->get_domain());
			if (res == CGAL::ON_BOUNDED_SIDE)
				is_in_quad = true;
			if (hfir->vertex()->is_feature())
			{
				bhas_feature = true;
			}
			triangle.push_back(hfir->vertex()->get_domain());
		} while (++hfir != hend);
		if (!is_in_quad)
		{
			continue;
		}

		vector<Point_2> trips;
		if (bhas_feature)
		{
			SamplingPoint2dInTriangleUniformly(triangle, insert_rate_feature, trips);
		}
		else
		{
			SamplingPoint2dInTriangleUniformly(triangle, insert_rate, trips);
		}
		for (int i = 0; i < trips.size(); i++)
		{
			CDT::Vertex_handle v = cdt_.insert(trips[i]);
			v->set_new_status(false);
		}
	}

	std::cout << "sample surface: " << cdt_.number_of_vertices() << std::endl;

	int index_ = 0;
	vector<Point_2> sample_ps;
	for (auto vit = cdt_.vertices_begin(); vit != cdt_.vertices_end(); vit++)
	{
		sample_ps.push_back(Point_2(vit->point().x(), vit->point().y()));
		vit->set_associated_index(index_);
		index_++;
	}

	//compute basis value
	BSplineBasis new_bsplinebasis = bSplineBasis;
	vector<TConfig3> new_alltconfig3s = alltconfig3s;
	new_alltconfig3s.clear();
	compute_sample_basis(sample_ps, new_bsplinebasis, dataMaps, new_alltconfig3s);

	//combine basis value and control point-verify
	map<unsigned int, DomainValue> cdt_data_map;
	int idn = 0;
	for (auto vit = cdt_.vertices_begin(); vit != cdt_.vertices_end(); vit++)
	{
		DomainValue p;
		p.pt = vit->point();
		cdt_data_map[idn] = p;
		idn++;
	}
	map<unsigned int, DomainValue>::iterator mit;
	for (mit = cdt_data_map.begin(); mit != cdt_data_map.end(); mit++)
	{
		int index = mit->first;
		double basis_value_sum_ = 0.0;
		RGBPoint pt = { 0.0, 0.0, 0.0,0.0,0.0 };
		for (int k = 0; k < new_bsplinebasis.basisConfigs.size(); k++)
		{
			if (new_bsplinebasis.basisConfigs[k].supports.size() != NULL)
			{
				map<unsigned int, DomainValue>::iterator res = new_bsplinebasis.basisConfigs[k].supports.find(index);
				if (res != new_bsplinebasis.basisConfigs[k].supports.end())
				{
					basis_value_sum_ += res->second.value;
					double x = pt.x + res->second.value * new_bsplinebasis.basisConfigs[k].controlPt.x;
					double y = pt.y + res->second.value * new_bsplinebasis.basisConfigs[k].controlPt.y;
					double r = pt.r + res->second.value * new_bsplinebasis.basisConfigs[k].controlPt.r;
					double g = pt.g + res->second.value * new_bsplinebasis.basisConfigs[k].controlPt.g;
					double b = pt.b + res->second.value * new_bsplinebasis.basisConfigs[k].controlPt.b;
					pt = RGBPoint{x,y,r,g,b};
				}
			}
		}
		mit->second.value = basis_value_sum_;
		mit->second.surfacep = pt;
	}
	int nwe4 = 0;
	int nwe8 = 0;
	for (mit = cdt_data_map.begin(); mit != cdt_data_map.end(); mit++)
	{
		if (abs(mit->second.value - 1) > 1e-4)
		{
			nwe4++;
			std::cout << "sample basis value sum!=1- " << mit->second.value << " wrong points index:(" << mit->second.pt.x() << "," << mit->second.pt.y() << ")" << std::endl;
		}
		if (abs(mit->second.value - 1) > 1e-8)
		{
			nwe8++;
		}
	}
	std::cout << "sample wrong number (error > 1e-4): " << nwe4 << std::endl;
	std::cout << "sample wrong number (error > 1e-8): " << nwe8 << std::endl;

	values_.clear();
	for (int i = 0; i < cdt_data_map.size(); i++)
	{
		values_.push_back(cdt_data_map[i].surfacep);
	}
	std::cout << "image local sample finished" << std::endl;
}

void Image_Approximation::sample_feature_region(vector<Point_2> &sample_domain,vector<RGBPoint> &image_values)
{
	double sample_rate = 4.0;

	//sample inte-triangle
	sample_rate += 2.0;//insert rate must >= 1.0
	vector<pair<Point_2, vector<int>>> sample_points_withtriangle;

	for (auto fit = originalMesh->facets_begin(); fit != originalMesh->facets_end(); fit++)
	{
		vector<Mesh::Vertex_iterator> triangle;
		bool bhas_feature = false;
		Mesh::Halfedge_around_facet_circulator hfir = fit->facet_begin(), hend = hfir;
		do
		{
			if (hfir->vertex()->is_feature())
			{
				bhas_feature = true;
			}
			triangle.push_back(hfir->vertex());
		} while (++hfir != hend);

		if (!bhas_feature)
		{
			continue;
		}

		Point_2 p0 = triangle[0]->get_domain(), p1 = triangle[1]->get_domain(), p2 = triangle[2]->get_domain();
		vector<int> triangle_int = {triangle[0]->vertex_index(),triangle[1]->vertex_index() ,triangle[2]->vertex_index() };
		double d_j = 1.0;
		for (int j = 0; j < int(sample_rate + 0.5)-1; j++)
		{
			double lamda0 = d_j / sample_rate;
			double d_k = 1.0;
			for (int k = 0; k < int(sample_rate + 0.5) - j -2; k++)
			{
				double lamda1 = d_k / sample_rate;
				double lamda2 = 1.0 - lamda0 - lamda1;
				if (lamda2 < 1e-10)
				{
					lamda2 = 0.0;
				}
				double x = lamda0 * p0.x() + lamda1 * p1.x() + lamda2 * p2.x();
				double y = lamda0 * p0.y() + lamda1 * p1.y() + lamda2 * p2.y();
				Point_2 temp(x, y);
				sample_domain.push_back(temp);
				sample_points_withtriangle.push_back(pair<Point_2,vector<int>>(temp,triangle_int));
				d_k = d_k + 1.0;
			}
			d_j += 1.0;
		}
	}
	for (int i = 0;i<sample_points_withtriangle.size();i++)
	{
		Point_2 pt = sample_points_withtriangle[i].first;
		vector<int> triangle = sample_points_withtriangle[i].second;
		RGBPoint pout;
		compute_originalcolor_at_domainpoint(pt,triangle,pout);
		image_values.push_back(pout);
	}
}

void Image_Approximation::sample_feature_curve_domainps()
{
#if 1
	vector<pair<bool, vector<Point_2>>> smooth_samplesps;
	vector<pair<bool, vector<PointTriple>>> smooth_originalps;

	int size_add = sample_feature_curve_rate;

	//Laplace uniform
	for (int k = 0; k < feature_lines_modify.size(); k++)
	{
		vector<Point_2> samplesps;
		bool is_circle = false;
		if (abs(feature_lines_modify[k][0].index_x - feature_lines_modify[k][feature_lines_modify[k].size() - 1].index_x) < 2 &&
			abs(feature_lines_modify[k][0].index_y - feature_lines_modify[k][feature_lines_modify[k].size() - 1].index_y) < 2)
		{
			is_circle = true;
		}
		for (int i = 0; i < feature_lines_modify[k].size(); i++)
		{
			if (is_circle && i == feature_lines_modify[k].size()-1)
			{
				break;
			}
			Point_2 pt(feature_lines_modify[k][i].paradomain);
			samplesps.push_back(pt);
		}
		smooth_samplesps.push_back(pair<bool, vector<Point_2>>(is_circle, samplesps));
	}
	for (int n = 0; n < smooth_samplesps.size(); n++)
	{
		vector<Point_2> new_points;
		if (smooth_samplesps[n].first)
		{
			for (int k = 0; k < size_add; k++)
			{
				new_points.clear();
				for (int i = 0; i < smooth_samplesps[n].second.size(); i++)
				{
					new_points.push_back(smooth_samplesps[n].second[i]);
					double midx = (smooth_samplesps[n].second[(i + 1) % smooth_samplesps[n].second.size()].x() + smooth_samplesps[n].second[i].x()) / 2.0;
					double midy = (smooth_samplesps[n].second[(i + 1) % smooth_samplesps[n].second.size()].y() + smooth_samplesps[n].second[i].y()) / 2.0;
					Point_2 pc(midx, midy);
					new_points.push_back(pc);
				}
				smooth_samplesps[n].second = new_points;
			}
		}
		else
		{
			for (int k = 0; k < size_add; k++)
			{
				new_points.clear();
				for (int i = 0; i < smooth_samplesps[n].second.size(); i++)
				{
					new_points.push_back(smooth_samplesps[n].second[i]);
					if (i == smooth_samplesps[n].second.size() - 1)
					{
						break;
					}
					double midx = (smooth_samplesps[n].second[(i + 1) % smooth_samplesps[n].second.size()].x() + smooth_samplesps[n].second[i].x()) / 2.0;
					double midy = (smooth_samplesps[n].second[(i + 1) % smooth_samplesps[n].second.size()].y() + smooth_samplesps[n].second[i].y()) / 2.0;
					Point_2 pc(midx, midy);
					new_points.push_back(pc);
				}
				smooth_samplesps[n].second = new_points;
			}
		}
	}

	for (int i = 0; i < smooth_samplesps.size(); i++)
	{
		for (int j = 0; j < smooth_samplesps[i].second.size(); j++)
		{
			featurecurve_sampleps.push_back(smooth_samplesps[i].second[j]);
		}
	}

#endif
	std::cout << "feature curve size: " << featurecurve_sampleps.size() << "-" << featurecurve_originalps.size() << std::endl;
}


void Image_Approximation::sample_local_image(Point_2 psta, Point_2 pend, vector<vector<pair<int, bool>>> &facets, vector<RGBPoint> &values_)
{
	facets.clear();
	values_.clear();

	CDT_Intersect cdt_;
	double insert_rate = local_refine_rate;
	double insert_rate_feature = local_refine_rate_fea;

	Point_2 p1(psta.x() + imagemesh_dx, (psta.y() + imagemesh_dy) / (2 * imagemesh_dy));
	Point_2 p2(pend.x() + imagemesh_dx, (pend.y() + imagemesh_dy) / (2 * imagemesh_dy));
	vector<Point_2> points;
	points.clear();
	points.push_back(p1);
	points.push_back(Point_2(p1.x(), p2.y()));
	points.push_back(p2);
	points.push_back(Point_2(p2.x(), p1.y()));

#if 1
	//sample inte-triangle
	insert_rate += 2.0;
	insert_rate_feature += 2.0;
	for (auto fit = fittedMesh->facets_begin(); fit != fittedMesh->facets_end(); fit++)
	{
		vector<Mesh::Vertex_iterator> triangle;
		bool bhas_feature = false;
		Mesh::Halfedge_around_facet_circulator hfir = fit->facet_begin(), hend = hfir;
		bool is_in_quad = false;
		do
		{
			int res = CGAL::bounded_side_2(points.begin(), points.end(), hfir->vertex()->get_domain());
			if (res == CGAL::ON_BOUNDED_SIDE)
				is_in_quad = true;
			if (hfir->vertex()->is_feature())
			{
				bhas_feature = true;
			}
			triangle.push_back(hfir->vertex());
		} while (++hfir != hend);
		if (!is_in_quad)
		{
			continue;
		}

		//insert rate must >= 1.0
		double sample_rate;
		if (bhas_feature)
		{
			sample_rate = insert_rate_feature;
		}
		else
		{
			sample_rate = insert_rate;
		}
		vector<BaryPoint> triangle_interior_points;
		Point_2 p0 = triangle[0]->get_domain(), p1 = triangle[1]->get_domain(), p2 = triangle[2]->get_domain();
		double d_j = 0.0;
		int numcdt = cdt_.number_of_vertices();
		int index_ = 0;
		for (int j = 0; j < int(sample_rate + 0.5) + 1; j++)
		{
			double lamda0 = d_j / sample_rate;
			double d_k = 0.0;
			for (int k = 0; k < int(sample_rate + 0.5) - j + 1; k++)
			{
				double lamda1 = d_k / sample_rate;
				double lamda2 = 1.0 - lamda0 - lamda1;
				if (lamda2 < 1e-10)
				{
					lamda2 = 0.0;
				}
				double x = lamda0 * p0.x() + lamda1 * p1.x() + lamda2 * p2.x();
				double y = lamda0 * p0.y() + lamda1 * p1.y() + lamda2 * p2.y();
				Point_2 temp(x, y);
				BaryPoint newp;
				newp.lamda0 = lamda0; newp.lamda1 = lamda1; newp.lamda2 = lamda2; newp.p = temp;
				newp.index = index_;
				CDT_Intersect::Vertex_handle vh = cdt_.insert(CDT_Intersect::Point(temp.x(), temp.y()));
				newp.vhandle = vh;
				triangle_interior_points.push_back(newp);
				index_++;
				d_k = d_k + 1.0;
			}
			d_j += 1.0;
		}

		//point 1-2
		for (double va = 0; va < 1 + 1e-8; va += 1.0 / sample_rate)
		{
			vector<BaryPoint> line_points;
			for (int i = 0; i < triangle_interior_points.size(); i++)
			{
				if (abs(triangle_interior_points[i].lamda0 - va) < 1e-8)
				{
					line_points.push_back(triangle_interior_points[i]);
				}
			}
			if (line_points.size() < 2)
			{
				continue;
			}
			sort(line_points.begin(), line_points.end(), SortFunction1);

			for (int i = 0; i < line_points.size() - 1; i++)
			{
				cdt_.insert_constraint(line_points[i].vhandle, line_points[i + 1].vhandle);
			}
		}
		//point 0-2
		for (double va = 0; va < 1 + 1e-8; va += 1.0 / sample_rate)
		{
			vector<BaryPoint> line_points;
			for (int i = 0; i < triangle_interior_points.size(); i++)
			{
				if (abs(triangle_interior_points[i].lamda1 - va) < 1e-8)
				{
					line_points.push_back(triangle_interior_points[i]);
				}
			}
			if (line_points.size() < 2)
			{
				continue;
			}
			sort(line_points.begin(), line_points.end(), SortFunction0);
			for (int i = 0; i < line_points.size() - 1; i++)
			{
				cdt_.insert_constraint(line_points[i].vhandle, line_points[i + 1].vhandle);
			}
		}
		//point 0-1
		for (double va = 0; va < 1 + 1e-8; va += 1.0 / sample_rate)
		{
			vector<BaryPoint> line_points;
			for (int i = 0; i < triangle_interior_points.size(); i++)
			{
				if (abs(triangle_interior_points[i].lamda2 - va) < 1e-8)
				{
					line_points.push_back(triangle_interior_points[i]);
				}
			}
			if (line_points.size() < 2)
			{
				continue;
			}
			sort(line_points.begin(), line_points.end(), SortFunction0);
			for (int i = 0; i < line_points.size() - 1; i++)
			{
				cdt_.insert_constraint(line_points[i].vhandle, line_points[i + 1].vhandle);
			}
		}

		BaryPoint tri0, tri1, tri2;
		for (int i = 0; i < triangle_interior_points.size(); i++)
		{
			if (abs(triangle_interior_points[i].lamda0 - 1) < 1e-8)
			{
				tri0 = triangle_interior_points[i];
				tri0.vhandle->set_new_status(triangle[0]->is_feature());
			}
			if (abs(triangle_interior_points[i].lamda1 - 1) < 1e-8)
			{
				tri1 = triangle_interior_points[i];
				tri1.vhandle->set_new_status(triangle[1]->is_feature());
			}
			if (abs(triangle_interior_points[i].lamda2 - 1) < 1e-8)
			{
				tri2 = triangle_interior_points[i];
				tri2.vhandle->set_new_status(triangle[2]->is_feature());
			}
		}
		Point_2 pfeature1, pfeature2;
		if (triangle[0]->is_feature() && triangle[1]->is_feature())
		{
			pfeature1 = triangle[0]->get_domain();
			pfeature2 = triangle[1]->get_domain();
		}
		else if (triangle[2]->is_feature() && triangle[1]->is_feature())
		{
			pfeature1 = triangle[2]->get_domain();
			pfeature2 = triangle[1]->get_domain();
		}
		else if (triangle[0]->is_feature() && triangle[2]->is_feature())
		{
			pfeature1 = triangle[0]->get_domain();
			pfeature2 = triangle[2]->get_domain();
		}
		int index_now = 0;
		for (auto vi = cdt_.vertices_begin(); vi != cdt_.vertices_end(); vi++, index_now++)
		{
			if (index_now >= numcdt)
			{
				Point_2 pt(CGAL::to_double(vi->point().x()), CGAL::to_double(vi->point().y()));
				if (abs(area(pt, pfeature1, pfeature2)) < 1e-16)
				{
					vi->set_new_status(true);
				}
				else
				{
					vi->set_new_status(false);
				}
			}
		}
	}

#endif

#if 0
	insert_rate += 2.0;
	insert_rate_feature += 2.0;
	Mesh::Facet_iterator f_it;
	for (f_it = fittedMesh->facets_begin(); f_it != fittedMesh->facets_end(); f_it++)
	{
		vector<Point_2> triangle;
		bool bhas_feature = false;
		Mesh::Halfedge_around_facet_circulator hfir = f_it->facet_begin(), hend = hfir;
		bool is_in_quad = false;
		do
		{
			int res = CGAL::bounded_side_2(points.begin(), points.end(), hfir->vertex()->get_domain());
			if (res == CGAL::ON_BOUNDED_SIDE)
				is_in_quad = true;
			if (hfir->vertex()->is_feature())
			{
				bhas_feature = true;
			}
			triangle.push_back(hfir->vertex()->get_domain());
		} while (++hfir != hend);
		if (!is_in_quad)
		{
			continue;
		}

		//insert rate must >= 1.0
		double sample_rate;
		if (bhas_feature)
		{
			sample_rate = insert_rate_feature;
		}
		else
		{
			sample_rate = insert_rate;
		}

		vector<BaryPoint> triangle_interior_points;
		Point_2 p1 = triangle[0], p0 = triangle[1], p2 = triangle[2];
		double d_j = 0.0;
		for (int j = 0; j < int(sample_rate + 0.5) + 1; j++)
		{
			double lamda0 = d_j / sample_rate;
			double d_k = 0.0;
			for (int k = 0; k < int(sample_rate + 0.5) - j + 1; k++)
			{
				double lamda1 = d_k / sample_rate;
				double lamda2 = 1.0 - lamda0 - lamda1;
				double x = lamda0 * p0.x() + lamda1 * p1.x() + lamda2 * p2.x();
				double y = lamda0 * p0.y() + lamda1 * p1.y() + lamda2 * p2.y();

				//double bx = floor(x * 100000.000f + 0.5) / 100000.000f;
				//double by = floor(y * 100000.000f + 0.5) / 100000.000f;
				Point_2 temp(x, y);
				BaryPoint newp;
				newp.lamda0 = lamda0; newp.lamda1 = lamda1; newp.lamda2 = lamda2; newp.p = temp;
				triangle_interior_points.push_back(newp);
				d_k = d_k + 1.0;
			}
			d_j += 1.0;
		}

		for (double va = 0; va < 1 + 1e-8;)
		{
			vector<BaryPoint> line_points;
			for (int i = 0; i < triangle_interior_points.size(); i++)
			{
				if (abs(triangle_interior_points[i].lamda0 - va) < 1e-8)
				{
					line_points.push_back(triangle_interior_points[i]);
				}
			}
			sort(line_points.begin(), line_points.end(), SortFunction1);
			for (int i = 0; i < line_points.size() - 1; i++)
			{
				CDT_Intersect::Vertex_handle v_h_1, v_h_2;
				v_h_1 = cdt_.insert(line_points[i].p);
				v_h_2 = cdt_.insert(line_points[i + 1].p);
				cdt_.insert_constraint(v_h_1, v_h_2);
			}
			va += 1.0 / sample_rate;
		}

		for (double va = 0; va < 1 + 1e-8;)
		{
			vector<BaryPoint> line_points;
			for (int i = 0; i < triangle_interior_points.size(); i++)
			{
				if (abs(triangle_interior_points[i].lamda1 - va) < 1e-8)
				{
					line_points.push_back(triangle_interior_points[i]);
				}
			}
			sort(line_points.begin(), line_points.end(), SortFunction0);
			for (int i = 0; i < line_points.size() - 1; i++)
			{
				CDT_Intersect::Vertex_handle v_h_1, v_h_2;
				v_h_1 = cdt_.insert(line_points[i].p);
				v_h_2 = cdt_.insert(line_points[i + 1].p);
				cdt_.insert_constraint(v_h_1, v_h_2);
			}
			va += 1.0 / sample_rate;
		}

		for (double va = 0; va < 1 + 1e-8;)
		{
			vector<BaryPoint> line_points;
			for (int i = 0; i < triangle_interior_points.size(); i++)
			{
				if (abs(triangle_interior_points[i].lamda2 - va) < 1e-8)
				{
					line_points.push_back(triangle_interior_points[i]);
				}
			}
			sort(line_points.begin(), line_points.end(), SortFunction0);
			for (int i = 0; i < line_points.size() - 1; i++)
			{
				CDT_Intersect::Vertex_handle v_h_1, v_h_2;
				v_h_1 = cdt_.insert(line_points[i].p);
				v_h_2 = cdt_.insert(line_points[i + 1].p);
				cdt_.insert_constraint(v_h_1, v_h_2);
			}
			va += 1.0 / sample_rate;
		}
	}
#endif

	std::cout << "sample surface: " << cdt_.number_of_vertices() << std::endl;

	int index_ = 0;
	vector<Point_2> sample_ps;
	for (auto vit = cdt_.vertices_begin(); vit != cdt_.vertices_end(); vit++)
	{
		sample_ps.push_back(Point_2(CGAL::to_double(vit->point().x()),
			CGAL::to_double(vit->point().y())));
		vit->set_associated_index(index_);
		index_++;
	}

	for (auto fi = cdt_.faces_begin(); fi != cdt_.faces_end(); fi++)
	{
		pair<int, bool> v1(fi->vertex(0)->get_associated_index(), fi->vertex(0)->get_new_status());
		pair<int, bool> v2(fi->vertex(1)->get_associated_index(), fi->vertex(1)->get_new_status());
		pair<int, bool> v3(fi->vertex(2)->get_associated_index(), fi->vertex(2)->get_new_status());
		vector<pair<int, bool>> face = { v1,v2,v3 };
		facets.push_back(face);
	}

	//compute basis value
	BSplineBasis new_bsplinebasis = bSplineBasis;
	vector<TConfig3> new_alltconfig3s = alltconfig3s;
	new_alltconfig3s.clear();
	compute_sample_basis(sample_ps, new_bsplinebasis, dataMaps, new_alltconfig3s);

	//combine basis value and control point
	vector<RGBPoint> color_value;
	color_value.resize(sample_ps.size());
	vector<double> verify_basis;
	verify_basis.resize(sample_ps.size());
#pragma omp parallel for
	for (int key = 0; key < color_value.size(); key++)
	{
		verify_basis[key] = 0.0;
		color_value[key] = RGBPoint{ 0.0, 0.0, 0.0 ,0.0,0.0 };
		for (int i = 0; i < new_bsplinebasis.basisMergeInfos.size(); i++)
		{
			int flag = new_bsplinebasis.basisMergeInfos[i].index;
			if (flag != -1)
			{
				for (int j = 0; j < new_bsplinebasis.basisMergeInfos[i].basisMerge.size(); j++)
				{
					int index_basis = new_bsplinebasis.basisMergeInfos[i].basisMerge[j];
					map<unsigned int, DomainValue>::iterator ite = new_bsplinebasis.basisConfigs[index_basis].supports.find(key);
					if (ite != new_bsplinebasis.basisConfigs[index_basis].supports.end())
					{
						verify_basis[key] += ite->second.value;
						color_value[key].x = color_value[key].x + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.x;
						color_value[key].y = color_value[key].y + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.y;
						color_value[key].r = color_value[key].r + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.r;
						color_value[key].g = color_value[key].g + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.g;
						color_value[key].b = color_value[key].b + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.b;
					}
				}
			}
		}
	}
	int Nnonormal = 0;
	wrong_points.clear();
	for (int i = 0; i < color_value.size(); i++)
	{
		values_.push_back(color_value[i]);
		if (abs(verify_basis[i]-1.0)>1e-8)
		{
			Nnonormal++;
			wrong_points.push_back(sample_ps[i]);
		}
	}
	std::cout << "image local sample - basis non-normal >1e-8: " << Nnonormal << std::endl;
	std::cout << "image local sample finished" << std::endl;

	//write to file
	QString local_refine = out_file_name;
	local_refine.append("/fitted_mesh_local_refine.obj");
	QFile file(local_refine);
	if (file.open(QIODevice::WriteOnly))
	{
		QTextStream out(&local_refine);
		out << "####" << "\r\n";
		out << "#" << "\r\n";
		out << "# OBJ File Generated by B-spline Surface Reconstruction" << "\r\n";
		out << "#" << "\r\n";
		out << "# Object 0_Fitted.obj" << "\r\n";
		out << "#" << "\r\n";
		out << "# Vertices: " << values_.size() << "\r\n";
		out << "# Faces: " << facets.size() << "\r\n";
		out << "#" << "\r\n";
		out << "####" << "\r\n";

		for (int i = 0; i < values_.size(); i++)
		{
			out << "v " << values_[i].x << " " << values_[i].y << " " << 0.0<<"\r\n" ;
			out << "c " << values_[i].r << " " << values_[i].g << " " << values_[i].b << "\r\n";
		}
		for (auto i = 0; i < facets.size(); i++)
		{
			out << "f " << facets[i][0].first + 1 << " " << facets[i][1].first + 1 << " " << facets[i][2].first + 1<<"\r\n";
			out << "is_feature " << facets[i][0].second << " " << facets[i][1].second << " " << facets[i][2].second << "\r\n";
		}
		out << "# End of File";
		file.close();
	}
}

void Image_Approximation::resample_image(vector<vector<pair<int, bool>>> &facets, vector<RGBPoint> &values_)
{
	facets.clear();
	values_.clear();

	CDT_Intersect cdt_;

	double insert_rate = whole_refine_rate;
	double insert_rate_feature = whole_refine_rate_fea;

#if 1
	//sample inte-triangle
	insert_rate += 2.0;
	insert_rate_feature += 2.0;
	for (auto fit = fittedMesh->facets_begin(); fit != fittedMesh->facets_end(); fit++)
	{
		vector<Mesh::Vertex_iterator> triangle;
		bool bhas_feature = false;
		Mesh::Halfedge_around_facet_circulator hfir = fit->facet_begin(), hend = hfir;
		bool is_in_quad = false;
		do
		{
			if (hfir->vertex()->is_feature())
			{
				bhas_feature = true;
			}
			triangle.push_back(hfir->vertex());
		} while (++hfir != hend);

		//insert rate must >= 1.0
		double sample_rate;
		if (bhas_feature)
		{
			sample_rate = insert_rate_feature;
		}
		else
		{
			sample_rate = insert_rate;
		}
		vector<BaryPoint> triangle_interior_points;
		Point_2 p0 = triangle[0]->get_domain(), p1 = triangle[1]->get_domain(), p2 = triangle[2]->get_domain();
		double d_j = 0.0;
		int numcdt = cdt_.number_of_vertices();
		int index_ = 0;
		for (int j = 0; j < int(sample_rate + 0.5) + 1; j++)
		{
			double lamda0 = d_j / sample_rate;
			double d_k = 0.0;
			for (int k = 0; k < int(sample_rate + 0.5) - j + 1; k++)
			{
				double lamda1 = d_k / sample_rate;
				double lamda2 = 1.0 - lamda0 - lamda1;
				if (lamda2 < 1e-10)
				{
					lamda2 = 0.0;
				}
				double x = lamda0 * p0.x() + lamda1 * p1.x() + lamda2 * p2.x();
				double y = lamda0 * p0.y() + lamda1 * p1.y() + lamda2 * p2.y();
				Point_2 temp(x, y);
				BaryPoint newp;
				newp.lamda0 = lamda0; newp.lamda1 = lamda1; newp.lamda2 = lamda2; newp.p = temp;
				newp.index = index_;
				CDT_Intersect::Vertex_handle vh = cdt_.insert(CDT_Intersect::Point(temp.x(), temp.y()));
				newp.vhandle = vh;
				triangle_interior_points.push_back(newp);
				index_++;
				d_k = d_k + 1.0;
			}
			d_j += 1.0;
		}

		//point 1-2
		for (double va = 0; va < 1 + 1e-8; va += 1.0 / sample_rate)
		{
			vector<BaryPoint> line_points;
			for (int i = 0; i < triangle_interior_points.size(); i++)
			{
				if (abs(triangle_interior_points[i].lamda0 - va) < 1e-8)
				{
					line_points.push_back(triangle_interior_points[i]);
				}
			}
			if (line_points.size() < 2)
			{
				continue;
			}
			sort(line_points.begin(), line_points.end(), SortFunction1);

			for (int i = 0; i < line_points.size() - 1; i++)
			{
				cdt_.insert_constraint(line_points[i].vhandle, line_points[i + 1].vhandle);
			}
		}
		//point 0-2
		for (double va = 0; va < 1 + 1e-8; va += 1.0 / sample_rate)
		{
			vector<BaryPoint> line_points;
			for (int i = 0; i < triangle_interior_points.size(); i++)
			{
				if (abs(triangle_interior_points[i].lamda1 - va) < 1e-8)
				{
					line_points.push_back(triangle_interior_points[i]);
				}
			}
			if (line_points.size() < 2)
			{
				continue;
			}
			sort(line_points.begin(), line_points.end(), SortFunction0);
			for (int i = 0; i < line_points.size() - 1; i++)
			{
				cdt_.insert_constraint(line_points[i].vhandle, line_points[i + 1].vhandle);
			}
		}
		//point 0-1
		for (double va = 0; va < 1 + 1e-8; va += 1.0 / sample_rate)
		{
			vector<BaryPoint> line_points;
			for (int i = 0; i < triangle_interior_points.size(); i++)
			{
				if (abs(triangle_interior_points[i].lamda2 - va) < 1e-8)
				{
					line_points.push_back(triangle_interior_points[i]);
				}
			}
			if (line_points.size() < 2)
			{
				continue;
			}
			sort(line_points.begin(), line_points.end(), SortFunction0);
			for (int i = 0; i < line_points.size() - 1; i++)
			{
				cdt_.insert_constraint(line_points[i].vhandle, line_points[i + 1].vhandle);
			}
		}

		BaryPoint tri0, tri1, tri2;
		for (int i = 0; i < triangle_interior_points.size(); i++)
		{
			if (abs(triangle_interior_points[i].lamda0 - 1) < 1e-8)
			{
				tri0 = triangle_interior_points[i];
				tri0.vhandle->set_new_status(triangle[0]->is_feature());
			}
			if (abs(triangle_interior_points[i].lamda1 - 1) < 1e-8)
			{
				tri1 = triangle_interior_points[i];
				tri1.vhandle->set_new_status(triangle[1]->is_feature());
			}
			if (abs(triangle_interior_points[i].lamda2 - 1) < 1e-8)
			{
				tri2 = triangle_interior_points[i];
				tri2.vhandle->set_new_status(triangle[2]->is_feature());
			}
		}
		Point_2 pfeature1, pfeature2;
		if (triangle[0]->is_feature() && triangle[1]->is_feature())
		{
			pfeature1 = triangle[0]->get_domain();
			pfeature2 = triangle[1]->get_domain();
		}
		else if (triangle[2]->is_feature() && triangle[1]->is_feature())
		{
			pfeature1 = triangle[2]->get_domain();
			pfeature2 = triangle[1]->get_domain();
		}
		else if (triangle[0]->is_feature() && triangle[2]->is_feature())
		{
			pfeature1 = triangle[0]->get_domain();
			pfeature2 = triangle[2]->get_domain();
		}
		int index_now = 0;
		for (auto vi = cdt_.vertices_begin(); vi != cdt_.vertices_end(); vi++, index_now++)
		{
			if (index_now >= numcdt)
			{
				Point_2 pt(CGAL::to_double(vi->point().x()), CGAL::to_double(vi->point().y()));
				if (abs(area(pt, pfeature1, pfeature2)) < 1e-16)
				{
					vi->set_new_status(true);
				}
				else
				{
					vi->set_new_status(false);
				}
			}
		}
	}

#endif

	std::cout << "sample surface: " << cdt_.number_of_vertices() << std::endl;

	int index_ = 0;
	vector<Point_2> sample_ps;
	for (auto vit = cdt_.vertices_begin(); vit != cdt_.vertices_end(); vit++)
	{
		sample_ps.push_back(Point_2(CGAL::to_double(vit->point().x()),
			CGAL::to_double(vit->point().y())));
		vit->set_associated_index(index_);
		index_++;
	}

	for (auto fi = cdt_.finite_faces_begin(); fi != cdt_.finite_faces_end(); fi++)
	{
		pair<int, bool> v1(fi->vertex(0)->get_associated_index(), fi->vertex(0)->get_new_status());
		pair<int, bool> v2(fi->vertex(1)->get_associated_index(), fi->vertex(1)->get_new_status());
		pair<int, bool> v3(fi->vertex(2)->get_associated_index(), fi->vertex(2)->get_new_status());
		vector<pair<int, bool>> face = { v1,v2,v3 };
		facets.push_back(face);
	}

	//compute basis value
	BSplineBasis new_bsplinebasis = bSplineBasis;
	vector<TConfig3> new_alltconfig3s = alltconfig3s;
	new_alltconfig3s.clear();
	compute_sample_basis(sample_ps, new_bsplinebasis, dataMaps, new_alltconfig3s);

	//combine basis value and control point
	vector<RGBPoint> color_value;
	color_value.resize(sample_ps.size());
	vector<double> verify_basis;
	verify_basis.resize(sample_ps.size());
#pragma omp parallel for
	for (int key = 0; key < color_value.size(); key++)
	{
		verify_basis[key] = 0.0;
		color_value[key] = RGBPoint{ 0.0, 0.0, 0.0 ,0.0,0.0 };
		for (int i = 0; i < new_bsplinebasis.basisMergeInfos.size(); i++)
		{
			int flag = new_bsplinebasis.basisMergeInfos[i].index;
			if (flag != -1)
			{
				for (int j = 0; j < new_bsplinebasis.basisMergeInfos[i].basisMerge.size(); j++)
				{
					int index_basis = new_bsplinebasis.basisMergeInfos[i].basisMerge[j];
					map<unsigned int, DomainValue>::iterator ite = new_bsplinebasis.basisConfigs[index_basis].supports.find(key);
					if (ite != new_bsplinebasis.basisConfigs[index_basis].supports.end())
					{
						verify_basis[key] += ite->second.value;
						color_value[key].x = color_value[key].x + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.x;
						color_value[key].y = color_value[key].y + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.y;
						color_value[key].r = color_value[key].r + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.r;
						color_value[key].g = color_value[key].g + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.g;
						color_value[key].b = color_value[key].b + ite->second.value * new_bsplinebasis.basisConfigs[index_basis].controlPt.b;
					}
				}
			}
		}
	}
	int Nnonormal = 0;
	for (int i = 0; i < color_value.size(); i++)
	{
		values_.push_back(color_value[i]);
		if (abs(verify_basis[i] - 1.0) > 1e-8)
		{
			Nnonormal++;
		}
	}
	std::cout << "image local sample - basis non-normal >1e-8: " << Nnonormal << std::endl;
	std::cout << "image re-sample finished" << std::endl;

	//write to file
	QString whole_refine = out_file_name;
	whole_refine.append("/fitted_mesh_while_refine.obj");
	QFile file(whole_refine);
	if (file.open(QIODevice::WriteOnly))
	{
		QTextStream out(&file);
		out << "####" << "\r\n";
		out << "#" << "\r\n";
		out << "# OBJ File Generated by B-spline Surface Reconstruction" << "\r\n";
		out << "#" << "\r\n";
		out << "# Object 0_Fitted.obj" << "\r\n";
		out << "#" << "\r\n";
		out << "# Vertices: " << values_.size() << "\r\n";
		out << "# Faces: " << facets.size() << "\r\n";
		out << "#" << "\r\n";
		out << "####" << "\r\n";

		for (int i = 0; i < values_.size(); i++)
		{
			out << "v " << values_[i].x << " " << values_[i].y << " " << 0.0 << "\r\n";;
			out << "c " << values_[i].r << " " << values_[i].g << " " << values_[i].b << "\r\n";
		}
		for (auto i = 0; i < facets.size(); i++)
		{
			out << "f " << facets[i][0].first + 1 << " " << facets[i][1].first + 1 << " " << facets[i][2].first + 1 << "\r\n";
			out << "is_feature " << facets[i][0].second << " " << facets[i][1].second << " " << facets[i][2].second << "\r\n";
		}
		out << "# End of File";
		file.close();
	}
}

void Image_Approximation::sample_local_image(vector<pair<MatrixXd, MatrixXd>> &mtin, vector<MatrixXd> &mtout)
{
	if (mtin.size() == 0)
	{
		return;
	}

	// build old domain KD-tree
	int				numnear = 3;							// number of nearest neighbors
	int				dim = 2;								// dimension
	double			eps = 0;								// error bound
	int				maxPts = originalMesh->size_of_vertices();// maximum number of data points

	int				nPts = originalMesh->size_of_vertices();// actual number of data points
	ANNpointArray	dataPts_temp;							// data points
	ANNkd_tree*		kdTree_temp;							// search structure

	dataPts_temp = annAllocPts(originalMesh->size_of_vertices(), dim);
	Vertex_iterator dstIt = originalMesh->vertices_begin();
	int i = 0;
	for (; dstIt != originalMesh->vertices_end(); dstIt++)
	{
		dataPts_temp[i][0] = dstIt->get_old_domain().x();
		dataPts_temp[i][1] = dstIt->get_old_domain().y();
		i++;
	}
	kdTree_temp = new ANNkd_tree(							// build search structure
		dataPts_temp,										// the data points
		nPts,												// number of points
		dim);												// dimension of space

	vector<Point_2> sample_domain_ps;
	for (int ite = 0; ite < 5; ite++)
	{
		sample_domain_ps.clear();
		sample_domain_ps.reserve(mtin[ite].first.rows() *mtin[ite].first.cols());
		for (int i = 0; i < mtin[ite].first.rows(); i++)
		{
			for (int j = 0; j < mtin[ite].first.cols(); j++)
			{
				Point_2 olddomain_p(mtin[ite].first.coeff(i, j), mtin[ite].second.coeff(i, j));

				//search
				ANNpoint			queryPt_temp;				// query point
				ANNidxArray			nnIdx_temp;					// near neighbor indices
				ANNdistArray		dists_temp;					// near neighbor distances

				queryPt_temp = annAllocPt(dim);					// allocate query point
				nnIdx_temp = new ANNidx[numnear];						// allocate near neigh indices
				dists_temp = new ANNdist[numnear];					// allocate near neighbor dist

				queryPt_temp[0] = olddomain_p.x();
				queryPt_temp[1] = olddomain_p.y();

				kdTree_temp->annkSearch(						// search
					queryPt_temp,								// query point
					numnear,											// number of near neighbors
					nnIdx_temp,									// nearest neighbors (returned)
					dists_temp,									// distance (returned)
					eps);										// error bound

				bool is_find = false;
				Point_2 domain_point;
				for (int it = 0; it < numnear; it++)
				{
					Mesh::Vertex_iterator v = originalMesh->get_vertex_iterator(nnIdx_temp[it]);
					vector<int> inte = { nnIdx_temp[it] };
					vector<int> one_ring;
					from_interior_to_ccw_one_ring_cycle(inte, one_ring, &originalMesh);

					for (int it_ring = 0; it_ring < one_ring.size(); it_ring++)
					{
						Point_2 p0 = v->get_old_domain();
						Point_2 p1 = originalMesh->get_vertex_iterator(one_ring[it_ring])->get_old_domain();
						Point_2 p2 = originalMesh->get_vertex_iterator(one_ring[(it_ring + 1) % one_ring.size()])->get_old_domain();
						Point_2 p0new = v->get_domain();
						Point_2 p1new = originalMesh->get_vertex_iterator(one_ring[it_ring])->get_domain();
						Point_2 p2new = originalMesh->get_vertex_iterator(one_ring[(it_ring + 1) % one_ring.size()])->get_domain();

						//on points
						if (abs(olddomain_p.x() - p0.x()) < 1e-8&&abs(olddomain_p.y() - p0.y()) < 1e-8)
						{
							is_find = true;
							domain_point = p0new;
							break;
						}
						if (abs(olddomain_p.x() - p1.x()) < 1e-8&&abs(olddomain_p.y() - p1.y()) < 1e-8)
						{
							is_find = true;
							domain_point = p1new;
							break;
						}
						if (abs(olddomain_p.x() - p2.x()) < 1e-8&&abs(olddomain_p.y() - p2.y()) < 1e-8)
						{
							is_find = true;
							domain_point = p2new;
							break;
						}
						//on lines
						if (area(p0, p1, p2) < 1e-16)
						{
							Point_2 minp, maxp, min_pt, max_pt;
							if ((p0 - p2).squared_length() > (p0 - p1).squared_length() && (p0 - p2).squared_length() > (p1 - p2).squared_length())
							{
								minp = p0; maxp = p2; min_pt = p0new; max_pt = p2new;
							}
							else if ((p0 - p1).squared_length() > (p1 - p2).squared_length() && (p0 - p1).squared_length() > (p0 - p2).squared_length())
							{
								minp = p0; maxp = p1; min_pt = p0new; max_pt = p1new;
							}
							else if ((p1 - p2).squared_length() > (p0 - p1).squared_length() && (p1 - p2).squared_length() > (p0 - p2).squared_length())
							{
								minp = p1; maxp = p2; min_pt = p1new; max_pt = p2new;
							}
							if (olddomain_p.x() > std::min(minp.x(), maxp.x()) && olddomain_p.x() < std::max(minp.x(), maxp.x()))
							{
								is_find = true;
								double dis_1 = sqrt((olddomain_p - minp).squared_length());
								double dis_mm = sqrt((maxp - minp).squared_length());
								double x_test = dis_1 / dis_mm * maxp.x() + (1.0 - dis_1 / dis_mm)*minp.x();
								assert(abs(x_test - olddomain_p.x()) < 1e-8);

								double x_ = dis_1 / dis_mm * max_pt.x() + (1.0 - dis_1 / dis_mm)*min_pt.x();
								double y_ = dis_1 / dis_mm * max_pt.y() + (1.0 - dis_1 / dis_mm)*min_pt.y();
								domain_point = Point_2(x_, y_);

								break;
							}
						}
						else //on triangles
						{
							vector<Point_2> triangle = { p0,p1,p2 };
							int ret = CGAL::bounded_side_2(triangle.begin(), triangle.end(), olddomain_p);
							if (ret != CGAL::ON_UNBOUNDED_SIDE)
							{
								is_find = true;

								double centc0 = area(olddomain_p, p1, p2) / area(p0, p1, p2);
								double centc1 = area(p0, olddomain_p, p2) / area(p0, p1, p2);
								double centc2 = 1.0 - centc0 - centc1;
								assert(abs(centc0*p0.x() + centc1*p1.x() + centc2*p2.x() - olddomain_p.x()) < 1e-8);
								assert(abs(centc0*p0.y() + centc1*p1.y() + centc2*p2.y() - olddomain_p.y()) < 1e-8);

								domain_point = Point_2(centc0*p0new.x() + centc1*p1new.x() + centc2*p2new.x(), centc0*p0new.y() + centc1*p1new.y() + centc2*p2new.y());

								break;
							}
						}
					}
					if (is_find)
					{
						break;
					}
				}
				if (!is_find)
				{
					std::cout << "KD-tree search wrong: no finding" << olddomain_p.x() << " " << olddomain_p.y() << std::endl;
				}

				delete[] nnIdx_temp;							// clean things up
				delete[] dists_temp;

				sample_domain_ps.push_back(domain_point);
			}
		}

		//compute basis value
		BSplineBasis new_bsplinebasis = bSplineBasis;
		vector<TConfig3> new_alltconfig3s = alltconfig3s;
		new_alltconfig3s.clear();
		compute_sample_basis(sample_domain_ps, new_bsplinebasis, dataMaps, new_alltconfig3s);

		//combine basis value and control point
		vector<double> color_value;
		color_value.resize(sample_domain_ps.size());
		for (int key = 0; key < color_value.size(); key++)
		{
			color_value[key] = 0;
			for (int i = 0; i < new_bsplinebasis.basisConfigs.size(); i++)
			{
				map<unsigned int, DomainValue>::iterator ite = new_bsplinebasis.basisConfigs[i].supports.find(key);
				if (ite != new_bsplinebasis.basisConfigs[i].supports.end())
				{
					color_value[key] += ite->second.value * new_bsplinebasis.basisConfigs[i].controlPt.r;
				}
			}
		}

		MatrixXd color_m(mtin[ite].first.rows(), mtin[ite].first.cols());
		int k = 0;
		for (int i = 0; i < color_m.rows(); i++)
		{
			for (int j = 0; j < color_m.cols(); j++)
			{
				color_m.coeffRef(i, j) = color_value[k];
				k++;
			}
		}

		mtout.push_back(color_m);
	}

	delete kdTree_temp;
}

//model view
void  Image_Approximation::GetCameraPosAndOrientation(vector<vector<double>> &pos_orient)
{
	if (model_position_orientation.size() != 0)
	{
		pos_orient = model_position_orientation;
	}
	else
	{
		//std::cout << "read from txt" << std::endl;
		pos_orient.resize(2);
		pos_orient[0].resize(3);
		pos_orient[1].reserve(4);
		std::ifstream fin;
		fin.open("C:\\Users\\hk\\Desktop\\Project\\TcbsplineApproxImage\\TcbsplineApproxImage\\Resources\\model_camera_pos_orientation.txt");
		if (fin.is_open())
		{
			for (int i = 0; i < 3; i++)
			{
				fin >> pos_orient[0][i];
			}
			for (int i = 0; i < 4; i++)
			{
				fin >> pos_orient[1][i];
			}
		}
		fin.close();
	}
}

void  Image_Approximation::SetCameraPosAndOrientation(vector<vector<double>> pos_orient)
{
	model_position_orientation = pos_orient;
}

bool & Image_Approximation::is_fixed_model_view()
{
	return is_fixed_modelview;
}

void Image_Approximation::set_model_view(bool bv)
{
	is_fixed_modelview = bv;
}
