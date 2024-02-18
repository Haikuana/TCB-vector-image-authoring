#include "SubApproximation.h"

void Sub_Approximation::simplify_(double level_)
{
	std::string infile_mesh = "initialmesh.obj";
	std::string infile_feature= "feature.txt";

	SUB_SIMPLIFY::simplify(infile_mesh, infile_feature, mesh_sub, level_);

	mesh_simplified = mesh_sub;
	mat_sub.resize(mesh_sub.vertices.size(), mesh_sub.vertices.size());
	mat_sub.setIdentity();

	SUB_SIMPLIFY::get_new_featurelines(new_feature);
	SUB_SIMPLIFY::get_ori_featurelines(in_feature);
	SUB_SIMPLIFY::get_feature_e_pairs(simpli_fea_e_pair);
	SUB_SIMPLIFY::get_feature_v_pairs(fea_v_pair);

	sub_fea_e_pair = simpli_fea_e_pair;

	for (int i = 0; i < mesh_simplified.vertices.size(); i++)
	{
		mesh_simplified.vertices[i]->fitted_color = Eigen::Vector3f(1, 1, 1);
	}
#if 0
	//polygon screening
	int id_max_fea = -1;
	int size_ = 0;
	for (int i = 0;i<new_feature.size();i++)
	{
		if (new_feature[i].size()>size_)
		{
			size_ = new_feature[i].size();
			id_max_fea = i;
		}
		//std::cout << "size_: " << new_feature[i].size() << "\n";
	}
	//std::cout << "selected feature size_: " << new_feature[id_max_fea].size() << "\n";
	Polygon_2 polygon_domain;
	for (int i = 0; i < new_feature[id_max_fea].size(); i++)
	{
		Eigen::Vector3f loc_ = mesh_simplified.vertices[new_feature[id_max_fea][i]]->loc;
		Point_2 p(loc_[0],loc_[1]);
		polygon_domain.push_back(p);
	}
	if (polygon_domain.is_simple())
	{
		//for faces
		for (auto fit = mesh_simplified.faces.rbegin(); fit != mesh_simplified.faces.rend(); fit++)
		{			
			Vector_2 v1((*fit)->verts[0]->loc[0], (*fit)->verts[0]->loc[1]);
			Vector_2 v2((*fit)->verts[1]->loc[0], (*fit)->verts[1]->loc[1]);
			Vector_2 v3((*fit)->verts[2]->loc[0], (*fit)->verts[2]->loc[1]);
			Vector_2 vc = (v1 + v2 + v3) / 3.0;
			if (polygon_domain.bounded_side(Point_2(vc.x(),vc.y())) == CGAL::ON_UNBOUNDED_SIDE)
			{
				(*fit)->bshow = false;
			}
		}
	}
#endif

	std::cout << "simplification finished!" << std::endl;
	std::cout << "number of faces:" << mesh_simplified.faces.size() << std::endl;
}

void Sub_Approximation::load_image_mesh()
{
	std::string infile_mesh = "initialmesh.obj";
	//std::string infile_feature = "feature.txt";

	std::vector<std::vector<int>> in_feature_lines;
	//SUB_SIMPLIFY::loadFeatureFile(infile_feature, in_feature_lines);

	std::map<int, int> fea_pairs;
	SUB_SIMPLIFY::loadObjFile(infile_mesh, mesh_image, in_feature_lines, fea_pairs);

	for (int i = 0;i<mesh_image.faces.size();i++)
	{
		SUB_SIMPLIFY::face *f = mesh_image.faces[i];
		double dis1 = sqrt(pow(f->verts[0]->loc[0] - f->verts[1]->loc[0], 2) 
			+ pow(f->verts[0]->loc[1] - f->verts[1]->loc[1], 2));
		double dis2 = sqrt(pow(f->verts[0]->loc[0] - f->verts[2]->loc[0], 2) 
			+ pow(f->verts[0]->loc[1] - f->verts[2]->loc[1], 2));
		DisPixel = std::min(dis1, dis2);
		break;
	}
}

void Sub_Approximation::subdivision_once()
{	
	SUB_SIMPLIFY::MeshSimplify mesh;
	SUBDIVISION::LOOP Cloop(&mesh_sub,&mesh);

	Cloop.set_fea_e_pair(sub_fea_e_pair);
	Cloop.set_fea_v_pair(&fea_v_pair);
	Cloop.set_feature_line(&new_feature);

	Cloop.subdivide();
	Cloop.get_fea_e_pair(sub_fea_e_pair);

	mesh_sub = mesh;
	//SUB_SIMPLIFY::saveObjFile("sub.obj", meshout);
}

void Sub_Approximation::subdivision(int times)
{
	for (int i = 0;i<times;i++)
	{
		SUB_SIMPLIFY::MeshSimplify mesh;
		SUBDIVISION::LOOP Cloop(&mesh_sub, &mesh);

		if (i == times-1)
		{
			Cloop.set_blimit(true);
		}
		
		Cloop.set_fea_e_pair(sub_fea_e_pair);
		Cloop.set_fea_v_pair(&fea_v_pair);
		Cloop.set_feature_line(&new_feature);

		Cloop.subdivide();
		Cloop.get_fea_e_pair(sub_fea_e_pair);

		mesh_sub = mesh;

#if 1
		SpMat new_mat;
		Cloop.get_subdivision_mat(new_mat);

		if (i == 0)
			mat_sub = new_mat;
		else
			mat_sub = mat_sub*new_mat;	
#endif
	}

	//back-up
	for (int i = 0; i < mesh_sub.vertices.size(); i++)
	{
		mesh_sub.vertices[i]->fitted_color = Eigen::Vector3f(1, 1, 1);
	}

	std::cout << "subdivision finished!" << std::endl;
	std::cout << "number of faces:"<<mesh_sub.faces.size() << std::endl;
	//std::cout << "real timing:" << timing_sub_s << "\n";
}

void Sub_Approximation::classify_subdivied_vs(std::vector<bool> &bSafe)
{
	std::vector<bool> bsafe(mesh_sub.vertices.size(),true);
	for (int i = 0;i<mesh_sub.vertices.size();i++)
	{
		if (mesh_sub.vertices[i]->type.feature_type != -1)
		{
			bsafe[i] = false;
			mesh_sub.vertices[i]->color = Eigen::Vector3f(0, 0, 0);
			continue;
		}

		Eigen::Vector3f pt = mesh_sub.vertices[i]->loc;
		for (int m = 0;m<new_feature.size();m++)
		{
			for (int n = 0;n<new_feature[m].size();n++)
			{
				Eigen::Vector3f pc = mesh_sub.vertices[new_feature[m][n]]->loc;
				Point_2 p1(pt[0], pt[1]);
				Point_2 p2(pc[0], pc[1]);
				if (sqrt((p1-p2).squared_length()) < MAX_DIS_TO_FEATURE_EDGES*DisPixel)
				{
					bsafe[i] = false;					
					break;
				}
			}
			if (!bsafe[i])
			{
				break;
			}
		}

		if (!bsafe[i])
		{
			mesh_sub.vertices[i]->color = Eigen::Vector3f(0, 0, 0);
		}
	}
	bSafe = bsafe;
}

void Sub_Approximation::cal_target_color()
{
	load_image_mesh();

	for (int k = 0; k < mesh_sub.vertices.size(); k++)
	{
		mesh_sub.vertices[k]->color = Eigen::Vector3f(0,0,0);
	}

	std::vector<bool> bsafe;
	classify_subdivied_vs(bsafe);

	ANNpointArray	dataPtsold;
	dataPtsold = annAllocPts(mesh_image.vertices.size(), 2);
	for (int i = 0;i<mesh_image.vertices.size();i++)
	{
		dataPtsold[i][0] = mesh_image.vertices[i]->loc[0];
		dataPtsold[i][1] = mesh_image.vertices[i]->loc[1];
		i++;
	}
	kdTree_ori_image = new ANNkd_tree(// build search structure
		dataPtsold,					// the data points
		mesh_image.vertices.size(),	// number of points
		2);						// dimension of space

	//for safe v
	for (int k = 0;k<mesh_sub.vertices.size();k++)
	{
		if (bsafe[k])
		{
			//suppose has build KD-tree already
			int				nnei = 4;								// number of nearest neighbors
			int				dim = 2;								// dimension
			double			eps = 0;								// error bound
			int				maxPts = mesh_image.vertices.size();// maximum number of data points

			ANNpoint			queryPt_temp;						// query point
			ANNidxArray			nnIdx_temp;							// near neighbor indices
			ANNdistArray		dists_temp;							// near neighbor distances

			queryPt_temp = annAllocPt(dim);							// allocate query point
			nnIdx_temp = new ANNidx[nnei];								// allocate near neigh indices
			dists_temp = new ANNdist[nnei];							// allocate near neighbor dists

			queryPt_temp[0] = mesh_sub.vertices[k]->loc[0];
			queryPt_temp[1] = mesh_sub.vertices[k]->loc[1];

			kdTree_ori_image->annkSearch(										// search
				queryPt_temp,										// query point
				nnei,												// number of near neighbors
				nnIdx_temp,											// nearest neighbors (returned)
				dists_temp,											// distance (returned)
				eps);												// error bound

			Eigen::Vector3f new_color;
			new_color.setZero();
			double count_ = 0.0;
			for (int it = 0; it < nnei; it++)
			{
				if (mesh_image.vertices[nnIdx_temp[it]]->type.feature_type == -1)
				{
					new_color += mesh_image.vertices[nnIdx_temp[it]]->color;
					count_ += 1.0;
				}
			}
			new_color *= 1.0 / count_;

			if (count_ < 1.0)
			{
				std::cout << "find nothing\n";
				bsafe[k] = false;
			}

			mesh_sub.vertices[k]->color = mesh_image.vertices[nnIdx_temp[0]]->color;
		}
	}

//#pragma omp parallel for
	for (int i = 0; i < mesh_sub.faces.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int v = mesh_sub.faces[i]->verts[j]->id;
			int vp = mesh_sub.faces[i]->verts[(j + 1) % 3]->id;
			int vc = mesh_sub.faces[i]->verts[(j + 2) % 3]->id;
			mesh_sub.vertices[v]->neighbors.insert(vp);
			mesh_sub.vertices[v]->neighbors.insert(vc);
		}
	}

	//for unsafe v
	set<int> candidates2process, newProcessed;
	for (int k = 0; k<mesh_sub.vertices.size(); k++)
	{
		if (bsafe[k])
		{
			for (auto it = mesh_sub.vertices[k]->neighbors.begin();
				it != mesh_sub.vertices[k]->neighbors.end();it++)
			{
				if (!bsafe[*it])
				{
					candidates2process.insert(*it);
				}
			}
		}
	}
	do 
	{
		newProcessed.clear();
		for (auto it = candidates2process.begin();it != candidates2process.end();it++)
		{			
			if (bsafe[*it])
			{
				continue;
			}
			Eigen::Vector3f new_color;
			new_color.setZero();
			double count_ = 0.0;
			for (auto nei = mesh_sub.vertices[*it]->neighbors.begin();
				nei != mesh_sub.vertices[*it]->neighbors.end();nei++)
			{
				if (bsafe[*nei])
				{
					new_color += mesh_sub.vertices[*nei]->color;
					count_ += 1.0;
				}
				else
				{
					newProcessed.insert(*nei);
				}
			}
			if (count_ < 1.0)
			{
				newProcessed.insert(*it);
			}
			else
			{
				new_color *= 1.0 / count_;
				mesh_sub.vertices[*it]->color = new_color;
				bsafe[*it] = true;
			}	
		}
		candidates2process = newProcessed;
	} while (!candidates2process.empty());

	//test
	for (int i = 0;i<bsafe.size();i++)
	{
		if (!bsafe[i])
		{
			std::cout << "not processed\n";
		}
	}
}

void Sub_Approximation::optimize_color()
{
	cal_target_color();
	std::cout << "cal_target_color finished!" << std::endl;

	FittingColor optimizer(&mesh_sub,&mesh_simplified);
	optimizer.set_At(mat_sub);
	optimizer.solve_equation();

	std::cout << "optimize_color finished!" << std::endl;

	//calculate errors
	double rmse = 0.0;
#if 0
	for (int k = 0; k < mesh_sub.vertices.size(); k++)
	{
		//suppose has build KD-tree already
		int				nnei = 4;								// number of nearest neighbors
		int				dim = 2;								// dimension
		double			eps = 0;								// error bound
		int				maxPts = mesh_image.vertices.size();// maximum number of data points

		ANNpoint			queryPt_temp;						// query point
		ANNidxArray			nnIdx_temp;							// near neighbor indices
		ANNdistArray		dists_temp;							// near neighbor distances

		queryPt_temp = annAllocPt(dim);							// allocate query point
		nnIdx_temp = new ANNidx[nnei];								// allocate near neigh indices
		dists_temp = new ANNdist[nnei];							// allocate near neighbor dists

		queryPt_temp[0] = mesh_sub.vertices[k]->loc[0];
		queryPt_temp[1] = mesh_sub.vertices[k]->loc[1];

		kdTree_ori_image->annkSearch(										// search
			queryPt_temp,										// query point
			nnei,												// number of near neighbors
			nnIdx_temp,											// nearest neighbors (returned)
			dists_temp,											// distance (returned)
			eps);												// error bound

		Eigen::Vector3f new_color;
		new_color.setZero();
		double count_ = 0.0;
		for (int it = 0; it < nnei; it++)
		{
			//if (mesh_image.vertices[nnIdx_temp[it]]->type.feature_type == -1)
			{
				new_color += mesh_image.vertices[nnIdx_temp[it]]->color;
				count_ += 1.0;
			}
		}
		new_color *= 1.0 / count_;

		if (count_ < 1.0)
		{
			std::cout << "error!!!!!! find nothing\n";
		}

		if (mesh_sub.vertices[k]->type.feature_type != -1)
		{
			rmse += (pow(mesh_sub.vertices[k]->fitted_color[0] - new_color[0], 2) +
				pow(mesh_sub.vertices[k]->fitted_color[1] - new_color[1], 2) +
				pow(mesh_sub.vertices[k]->fitted_color[2] - new_color[2], 2)) / 3.0;
		}	
	}
#else
	for (int k = 0; k < mesh_sub.vertices.size(); k++)
	{
		double error_ = (pow(mesh_sub.vertices[k]->fitted_color[0] - mesh_sub.vertices[k]->color[0], 2) +
			pow(mesh_sub.vertices[k]->fitted_color[1] - mesh_sub.vertices[k]->color[1], 2) +
			pow(mesh_sub.vertices[k]->fitted_color[2] - mesh_sub.vertices[k]->color[2], 2)) / 3.0;

		//error_ = error_ < 1e-16? 0:sqrt(error_);

		//if (mesh_sub.vertices[k]->type.feature_type == -1)
		{
			rmse += error_;
		}
	}

#endif
	std::cout << "rmse:" << rmse << "\n";
	double RMSE_ = sqrt(rmse / double(mesh_sub.vertices.size()));

	save_subdivision_mesh("vectorimage.obj", mesh_sub);

	double size_representation =
		(mesh_simplified.vertices.size() * 20.0 + mesh_simplified.faces.size() * 6.0) / 1000.0;
	std::cout << "storage: " << size_representation << std::endl;
	std::cout << "RMSE: "<<RMSE_ << std::endl;
}

void Sub_Approximation::mesh_fitting_via_section6()
{
	//construct file
	out_file_name = "Results/";
	out_file_name.append(fileName.split("/").last().split(".").at(0));
	filename_foraddcollinear_knots = out_file_name;
	out_file_name.append(QString("/ite%1-d%2").arg(Nite_now).arg(nDeg));
	generate_output_directory(out_file_name);

	std::cout << __FUNCTION__ << ": " << fileName.toStdString() << std::endl;
	std::cout << __FUNCTION__ << ": " << out_file_name.toStdString() << std::endl;

	std::cout << "\n" << "/////////////////////////////fitting start,iterate: " << Nite_now << std::endl;
	statisticsInfos.allTime = 0.0;
	statisticsInfos.infos.resize(1);
	clock_t compute_startc = clock();
	clock_t start, finish;

	//extract image feature
	start = clock();
	compute_image_feature();

	//process intersections
	bool do_find_wrong;
	do
	{
		do_find_wrong = false;
		for (int i = 0; i < feature_lines_modify.size(); i++)
		{
			vector<PointTriple> front_tail = { feature_lines_modify[i][0],
				feature_lines_modify[i][feature_lines_modify[i].size() - 1] };

			if (abs(front_tail[0].index_x - front_tail[1].index_x) <= 1 &&
				abs(front_tail[0].index_y - front_tail[1].index_y) <= 1)
			{
				continue;
			}
			for (int k = 0; k < front_tail.size(); k++)
			{
				bool do_find = false;
				for (int j = 0; j < feature_lines_modify.size(); j++)
				{
					if (i == j)
					{
						for (int n = 2; n < feature_lines_modify[j].size() - 2; n++)
						{
							if (abs(feature_lines_modify[j][n].index_x - front_tail[k].index_x) <= 1 &&
								abs(feature_lines_modify[j][n].index_y - front_tail[k].index_y) <= 1)
							{
								do_find = true;
								break;
							}
						}
					}
					else {
						for (auto it = feature_lines_modify[j].begin(); it != feature_lines_modify[j].end(); it++)
						{
							if (abs(it->index_x - front_tail[k].index_x) <= 1 &&
								abs(it->index_y - front_tail[k].index_y) <= 1)
							{
								do_find = true;
								break;
							}
						}
					}
					if (do_find)
					{
						break;
					}
				}
				if (do_find)
				{
					std::cout << "found intersection\n";
					if (k == 0)
					{
						auto it = feature_lines_modify[i].begin();
						feature_lines_modify[i].erase(it);
					}
					if (k == 1)
					{
						feature_lines_modify[i].pop_back();
					}
					do_find_wrong = do_find;
				}
			}
		}
	} while (do_find_wrong);

	//smooth position
	smooth_and_sample_feature_curve_points();
	project_featureps_onto_smoothps();
	finish = clock();
	statisticsInfos.infos[0].t_feature_process = (double(finish - start)) / CLOCKS_PER_SEC;

	QString outfile("initialmesh.obj");
	originalMesh->write_obj_with_vtype(outfile);

	//domain
	start = clock();
	compute_constrained_parameterization();
	create_kd_tree();
	finish = clock();
	statisticsInfos.infos[0].t_para = (double(finish - start)) / CLOCKS_PER_SEC;

	resample_domain_image_on_mesh();

	start = clock();
	optimize_knots_triangulation_revised();
	finish = clock();
	statisticsInfos.infos[0].t_knots_generation = (double(finish - start)) / CLOCKS_PER_SEC;

	mesh_simplified = mesh_sub;
	simpli_fea_e_pair = sub_fea_e_pair;

	//extract image feature
	start = clock();
	subdivision(4);
	finish = clock();
	double t_subdivision_process = (double(finish - start)) / CLOCKS_PER_SEC;
	std::cout << "timing:" << t_subdivision_process << "\n";
	std::cout << "real timing:" << timing_sub_s << "\n";
	std::cout << "size of faces:" << mesh_sub.faces.size() << "\n";

	optimize_color();
	std::cout << "/////////////////////////////fitting finished " << std::endl;
}

void Sub_Approximation::optimize_knots_triangulation_revised()
{
	/***************************************************by Yanyang**********************************************/
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
	opti_knottri->init_greedy_mesh(Norepeat_knot.size() + nknotinsert_opti);
	opti_knottri->optimize_mesh(niterate_opti, dir_allknot.toLatin1(), 1);

	//knot mesh
	knot_mesh = new Mesh;
	read_mesh(&knot_mesh, dir_allknot);
	nknot = knot_mesh->size_of_vertices();
	std::cout << "num knots: " << nknot << std::endl;

	/***************************************************pre-process**********************************************/
	//assign attributes
	double _dx = 0.5;
	double _dy = 0.5 / input_image->width() * input_image->height();
	double paradx = _dx - 0.5 / input_image->width();
	double parady = _dy - 0.5 / input_image->width();
	for (auto vit = knot_mesh->vertices_begin(); vit != knot_mesh->vertices_end(); vit++)
	{
		vit->is_feature() = false;

		bool is_find = false;
		for (int i = 0; i < Norepeat_knot.size(); i++)
		{
			if (Norepeat_knot[i].index == vit->vertex_index())
			{
				is_find = true;
				vit->get_domain() = Norepeat_knot[i].paradomain;
				vit->is_feature() = true;
				break;
			}
		}
		if (!is_find)
		{
			double x = vit->point().x();
			double y = vit->point().y();
			vit->get_domain() = Point_2((x + imagemesh_dx) / (2.0*imagemesh_dx), (y + imagemesh_dy) / (2.0*imagemesh_dy));
		}
	}
	//handle boundary
	bool does_exist_changing = false;
	do
	{
		does_exist_changing = false;

		for (auto vit = knot_mesh->vertices_begin(); vit != knot_mesh->vertices_end(); vit++)
		{
			if (vit->vertex_index() < Norepeat_knot.size())
			{
				continue;
			}

			//tackle boundary perturbation procedure
			double pixel_perturb = 3.0 * imagemesh_dx / double(input_image->width());

			if (vit->vertex_index() == 1474)
			{
				int s = 1;
			}

			Point_2 p = vit->get_domain(), pt = p;

#if 1
			Mesh::Halfedge_around_vertex_circulator h_push = vit->vertex_begin(), push_start = h_push, h_pushplus;
			if (abs(p.x() - 0.0) < pixel_perturb)
			{
				if (vit->is_border())
				{
					p = Point_2(0.0, p.y());
				}
				else
				{
					do
					{
						h_pushplus = h_push;
						h_pushplus++;
						Point_2 p1, p2;
						p1 = h_push->opposite()->vertex()->get_domain();
						p2 = h_pushplus->opposite()->vertex()->get_domain();
						if (abs(p1.x() - 0.0) < 1e-12 &&abs(p2.x() - 0.0) < 1e-12)
						{
							if (vit->get_domain().y() < std::max(p1.y(), p2.y()) &&
								vit->get_domain().y() > std::min(p1.y(), p2.y()))
							{
								p = Point_2(0.0, p.y());
								break;
							}
						}
					} while (++h_push != push_start);
				}
			}
			if (abs(p.x() - 1.0) < pixel_perturb)
			{
				if (vit->is_border())
				{
					p = Point_2(1.0, p.y());
				}
				else
				{
					do
					{
						h_pushplus = h_push;
						h_pushplus++;
						Point_2 p1, p2;
						p1 = h_push->opposite()->vertex()->get_domain();
						p2 = h_pushplus->opposite()->vertex()->get_domain();
						if (abs(p1.x() - 1.0) < 1e-12 &&abs(p2.x() - 1.0) < 1e-12)
						{
							if (vit->get_domain().y() < std::max(p1.y(), p2.y()) &&
								vit->get_domain().y() > std::min(p1.y(), p2.y()))
							{
								p = Point_2(1.0, p.y());
								break;
							}
						}
					} while (++h_push != push_start);
				}
			}
			if (abs(p.y() - 0.0) < pixel_perturb)
			{
				if (vit->is_border())
				{
					p = Point_2(p.x(), 0.0);
				}
				else
				{
					do
					{
						h_pushplus = h_push;
						h_pushplus++;
						Point_2 p1, p2;
						p1 = h_push->opposite()->vertex()->get_domain();
						p2 = h_pushplus->opposite()->vertex()->get_domain();
						if (abs(p1.y() - 0.0) < 1e-12 &&abs(p2.y() - 0.0) < 1e-12)
						{
							if (vit->get_domain().x() < std::max(p1.x(), p2.x()) &&
								vit->get_domain().x() > std::min(p1.x(), p2.x()))
							{
								p = Point_2(p.x(), 0.0);
								break;
							}
						}
					} while (++h_push != push_start);
				}
			}
			if (abs(p.y() - 1.0) < pixel_perturb)
			{
				if (vit->is_border())
				{
					p = Point_2(p.x(), 1.0);
				}
				else
				{
					do
					{
						h_pushplus = h_push;
						h_pushplus++;
						Point_2 p1, p2;
						p1 = h_push->opposite()->vertex()->get_domain();
						p2 = h_pushplus->opposite()->vertex()->get_domain();
						if (abs(p1.y() - 1.0) < 1e-12 &&abs(p2.y() - 1.0) < 1e-12)
						{
							if (vit->get_domain().x() < std::max(p1.x(), p2.x()) &&
								vit->get_domain().x() > std::min(p1.x(), p2.x()))
							{
								p = Point_2(p.x(), 1.0);
								break;
							}
						}
					} while (++h_push != push_start);
				}
			}
			vit->get_domain() = p;

			if (abs(p.x() - pt.x()) > 1e-8 || abs(p.y() - pt.y()) > 1e-8)
			{
				does_exist_changing = true;
			}
#endif
		}
	} while (does_exist_changing);

	knot_mesh->computeCornerVerticesType();
	knot_mesh->compute_all_face_domain_area();

	/**********************************************make control mesh***************************************************/
	//from knot mesh to cdt
	knots_config = new CDT;
	map<int, CDT::Vertex_handle> vertex_handle_cdt;
	vector<Point_2> domain_bound = { Point_2(0,0),Point_2(0,1), Point_2(1,1), Point_2(1,0) };
	/////////////first step/////////////////
	//add feature and boundary knots
	for (auto vit = knot_mesh->vertices_begin(); vit != knot_mesh->vertices_end(); vit++)
	{
		if (vit->vertex_index() < Norepeat_knot.size())
		{
			CDT::Vertex_handle v = knots_config->insert(vit->get_domain());
			v->set_associated_index(vit->vertex_index());
			vertex_handle_cdt[vit->vertex_index()] = v;
		}
		//knots pretty close to boundary
		for (int i = 0; i < domain_bound.size(); i++)
		{
			Point_2 v1 = domain_bound[i], v2 = domain_bound[(i + 1) % 4];
			if (dis_between_p_line(v1, v2, vit->get_domain()) < 0.5*remove_criteria_closeto_featureedge / input_image->width())
			{
				double x = vit->get_domain().x(), y = vit->get_domain().y();
				if (abs(0 - vit->get_domain().x()) < 1e-8)
				{
					x = 0;
				}
				if (abs(1 - vit->get_domain().x()) < 1e-8)
				{
					x = 1;
				}
				if (abs(0 - vit->get_domain().y()) < 1e-8)
				{
					y = 0;
				}
				if (abs(1 - vit->get_domain().y()) < 1e-8)
				{
					y = 1;
				}
				vit->get_domain() = Point_2(x, y);
			}
		}
	}
	//add feature segments constrains
	for (int i = 0; i < collinear_knots_lines.size(); i++)
	{
		for (int j = 0; j < collinear_knots_lines[i].size() - 1; j++)
		{
			int edge_f = collinear_knots_lines[i][j].index;
			int edge_t = collinear_knots_lines[i][j + 1].index;
			knots_config->insert_constraint(vertex_handle_cdt.at(edge_f), vertex_handle_cdt.at(edge_t));
		}
	}

	/////////////second step/////////////////
	vector<int> knots_closeto_feature;
	for (auto vit = knot_mesh->vertices_begin(); vit != knot_mesh->vertices_end(); vit++)
	{
		bool is_remove = false;
		if (vit->get_domain().x() < 0|| vit->get_domain().x() >1||
			vit->get_domain().y() < 0|| vit->get_domain().y() >1 )
		{
			is_remove = true;
		}
		if (vit->vertex_index() >= Norepeat_knot.size())
		{
			//remove knots pretty close to feature segments
			for (int i = 0; i < collinear_knots_lines.size(); i++)
			{
				int former;
				for (int j = 0; j < collinear_knots_lines[i].size(); )
				{
					Point_2 left_, right_;
					if (collinear_knots_lines[i][j].is_fixed)
					{
						former = j;
						left_ = collinear_knots_lines[i][former].paradomain;
					}
					j++;
					if (j > collinear_knots_lines[i].size() - 1 || !collinear_knots_lines[i][j].is_fixed)
					{
						continue;
					}
					right_ = collinear_knots_lines[i][j].paradomain;
					if (vit->get_domain().x() > std::min(left_.x(), right_.x()) && vit->get_domain().x() < std::max(left_.x(), right_.x()) ||
						vit->get_domain().y() > std::min(left_.y(), right_.y()) && vit->get_domain().y() < std::max(left_.y(), right_.y()))
					{
						if (dis_between_p_line(left_, right_, vit->get_domain()) < remove_criteria_closeto_featureedge / input_image->width())
						{
							is_remove = true;
							knots_closeto_feature.push_back(vit->vertex_index());
							std::cout << __FUNCTION__ << ": " << "new knot: " << vit->vertex_index() << "-coord:(" << vit->get_domain().x() << "," << vit->get_domain().y()
								<< ") is almost on feature line, area: "
								<< abs(CGAL::area(vit->get_domain(), left_, right_)) << ". so, remove it" << std::endl;
							break;
						}
					}
				}
				if (is_remove)
				{
					break;
				}
			}			
		}

		if (!is_remove)
		{
			CDT::Vertex_handle v = knots_config->insert(vit->get_domain());
			v->set_associated_index(vit->vertex_index());
			vertex_handle_cdt[vit->vertex_index()] = v;
		}
		else
		{
			std::cout << __FUNCTION__ << ": " << "knot remove" << std::endl;
		}
	}

	std::cout << __FUNCTION__ << ": " << "before all edge constraints" << std::endl;

	//remove turnover triangles
	vector<pair<int, int>> turnover_edges;
	for (Mesh::Facet_iterator fit = knot_mesh->facets_begin(); fit != knot_mesh->facets_end(); fit++)
	{
		Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin();
		int index0 = hc->vertex()->vertex_index();
		Point_2 p0 = hc->vertex()->get_domain();
		hc++;
		int index1 = hc->vertex()->vertex_index();
		Point_2 p1 = hc->vertex()->get_domain();
		hc++;
		int index2 = hc->vertex()->vertex_index();
		Point_2 p2 = hc->vertex()->get_domain();

		if (area(p0, p1, p2) < 0)
		{
			pair<int, int> edge1(index0, index1);
			pair<int, int> edge2(index0, index2);
			pair<int, int> edge3(index2, index1);
			turnover_edges.push_back(edge1);
			turnover_edges.push_back(edge2);
			turnover_edges.push_back(edge3);
			std::cout << __FUNCTION__ << ": " << "triangle turnover: " << area(p0, p1, p2) << ": " << index0 << "-" << index1 << "-" << index2 << "\n";
		}
	}
	//add optimization edge constrains
	for (Mesh::Facet_iterator fit = knot_mesh->facets_begin(); fit != knot_mesh->facets_end(); fit++)
	{
		if (fit->domain_area() < 1e-8)
		{
			continue;
		}

		Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin();
		int index0 = hc->vertex()->vertex_index();
		hc++;
		int index1 = hc->vertex()->vertex_index();
		hc++;
		int index2 = hc->vertex()->vertex_index();

		vector<int>::iterator res0_closeto = find(knots_closeto_feature.begin(), knots_closeto_feature.end(), index0);
		vector<int>::iterator res1_closeto = find(knots_closeto_feature.begin(), knots_closeto_feature.end(), index1);
		vector<int>::iterator res2_closeto = find(knots_closeto_feature.begin(), knots_closeto_feature.end(), index2);

		//std::cout << __FUNCTION__ << ": " << "current constraint: " << index0 << "-" << index1 << "-" << index2 << "\n";

		if (res0_closeto == knots_closeto_feature.end() && res1_closeto == knots_closeto_feature.end()
			)
		{
			//knots_config->insert_constraint(vertex_handle_cdt[index0], vertex_handle_cdt[index1]);
			bool is_turnover = false;
			for (int k = 0; k < turnover_edges.size(); k++)
			{
				if (turnover_edges[k].first == index0 && turnover_edges[k].second == index1 ||
					turnover_edges[k].first == index1 && turnover_edges[k].second == index0)
				{
					is_turnover = true;
					break;
				}
			}
			if (!is_turnover)
			{
				knots_config->insert_constraint(vertex_handle_cdt[index0], vertex_handle_cdt[index1]);
			}
		}
		if (res0_closeto == knots_closeto_feature.end() && res2_closeto == knots_closeto_feature.end()
			)
		{
			//knots_config->insert_constraint(vertex_handle_cdt[index0], vertex_handle_cdt[index2]);
			bool is_turnover = false;
			for (int k = 0; k < turnover_edges.size(); k++)
			{
				if (turnover_edges[k].first == index0 && turnover_edges[k].second == index2 ||
					turnover_edges[k].first == index2 && turnover_edges[k].second == index0)
				{
					is_turnover = true;
					break;
				}
			}
			if (!is_turnover)
			{
				knots_config->insert_constraint(vertex_handle_cdt[index0], vertex_handle_cdt[index2]);
			}
		}
		if (res2_closeto == knots_closeto_feature.end() && res1_closeto == knots_closeto_feature.end()
			)
		{
			//knots_config->insert_constraint(vertex_handle_cdt[index2], vertex_handle_cdt[index1]);
			bool is_turnover = false;
			for (int k = 0; k < turnover_edges.size(); k++)
			{
				if (turnover_edges[k].first == index2 && turnover_edges[k].second == index1 ||
					turnover_edges[k].first == index1 && turnover_edges[k].second == index2)
				{
					is_turnover = true;
					break;
				}
			}
			if (!is_turnover)
			{
				knots_config->insert_constraint(vertex_handle_cdt[index2], vertex_handle_cdt[index1]);
			}
		}
	}

	/**********************************************update***************************************************/
	//renew index because of removing knots
	int i = 0;
	for (auto vi = knots_config->vertices_begin(); vi != knots_config->vertices_end(); vi++)
	{
		vi->set_associated_index(i);
		vi->set_new_status(0);
		i++;
	}

	//renew knot mesh
	if (knot_mesh != NULL)
	{
		delete knot_mesh;
		knot_mesh = new Mesh;
	}
	from_cdt2mesh(*knots_config, &knot_mesh);
	for (auto vit = knot_mesh->vertices_begin(); vit != knot_mesh->vertices_end(); vit++)
	{
		vit->get_domain() = Point_2(vit->point().x(), vit->point().y());
		double x = vit->get_domain().x() - imagemesh_dx;
		double y = vit->get_domain().y()*imagemesh_dy * 2 - imagemesh_dy;
		vit->point() = Mesh::Point(x, y, 0);
	}

	std::cout << "/////////////////////////////optimize knots mesh finished, number of knots " << nknot << "-" << knots_config->number_of_vertices() << std::endl;

	vector<SUB_SIMPLIFY::vertex*> vertices;
	std::vector<SUB_SIMPLIFY::face*> faces;

	for (auto vit = knot_mesh->vertices_begin(); vit != knot_mesh->vertices_end(); vit++)
	{
		SUB_SIMPLIFY::vertex *v = new SUB_SIMPLIFY::vertex;
		v->loc= Eigen::Vector3f(vit->point().x(), vit->point().y(), vit->point().z() );
		v->loc_back = v->loc;
		v->id = vit->vertex_index();
		v->added = false;
		v->type.feature_type = -1;
		v->type.is_fea_corner = false;
		v->type.bound_type = domain_bound_type(vit->get_domain());
		v->type.is_corner = is_on_domain_corner(vit->get_domain());
		v->type_back = v->type;
		vertices.push_back(v);
	}

	for (int m = 0; m < collinear_knots_lines.size(); m++)
	{
		vertices[collinear_knots_lines[m][0].index]->type.is_fea_corner = true;
		if (collinear_knots_lines[m][0].index != collinear_knots_lines[m][collinear_knots_lines[m].size() - 1].index)
			vertices[collinear_knots_lines[m][collinear_knots_lines[m].size() - 1].index]->type.is_fea_corner = true;

		for (int n = 0; n < collinear_knots_lines[m].size(); n++)
		{
			vertices[collinear_knots_lines[m][n].index]->type.feature_type = m;
		}
	}

	int id_v = knot_mesh->size_of_vertices();
	int id_f = 0;
	for (Mesh::Facet_iterator fit = knot_mesh->facets_begin(); fit != knot_mesh->facets_end(); fit++)
	{	
		Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin();
		int index0 = hc->vertex()->vertex_index();
		Point_2 p0(hc->vertex()->point().x(), hc->vertex()->point().y());
		hc++;
		int index1 = hc->vertex()->vertex_index();
		Point_2 p1(hc->vertex()->point().x(), hc->vertex()->point().y());
		hc++;
		int index2 = hc->vertex()->vertex_index();
		Point_2 p2(hc->vertex()->point().x(), hc->vertex()->point().y());
		vector<int> facet_vid = {index0,index1,index2};
		vector<Point_2> facet_vp = {p0,p1,p2};

		SUB_SIMPLIFY::face *f = new SUB_SIMPLIFY::face();
		f->id = id_f; id_f++;
		for (int j = 0; j < 3; j++) {
			f->verts[j] = vertices[facet_vid[j]];
		}

		for (int it = 0;it<3;it++)
		{
			bool is_find = false;
			int id_m = 0, id_n = 0;
			for (int m = 0; m < collinear_knots_lines.size(); m++)
			{
				for (int n = 0; n < collinear_knots_lines[m].size(); n++)
				{
					if (facet_vid[it] == collinear_knots_lines[m][n].index)
					{
						id_m = m;
						id_n = n;
						is_find = true; break;
					}
				}
			}
			if (!is_find)
			{
				continue;
			}
			Point_2 ptest((p0.x()+p1.x()+p2.x())/3.0, (p0.y() + p1.y() + p2.y()) / 3.0);
			int v_id_fea1 = -1;
			int v_id_fea2 = -1;

			if (id_n < collinear_knots_lines[id_m].size() - 1 ||
				collinear_knots_lines[id_m][collinear_knots_lines[id_m].size() - 1].index ==
				collinear_knots_lines[id_m][0].index)
			{
				v_id_fea2 = collinear_knots_lines[id_m][(id_n + 1) % collinear_knots_lines[id_m].size()].index;
			}
			if (id_n > 0 ||
				collinear_knots_lines[id_m][collinear_knots_lines[id_m].size() - 1].index ==
				collinear_knots_lines[id_m][0].index)
			{
				v_id_fea1 = collinear_knots_lines[id_m][(id_n - 1 +
					collinear_knots_lines[id_m].size()) % collinear_knots_lines[id_m].size()].index;
			}
			Point_2 pt0(vertices[facet_vid[it]]->loc[0], vertices[facet_vid[it]]->loc[1]);
			Point_2 pt1(pt0); Point_2 pt2(pt0);
			if (v_id_fea1 != -1)
			{
				pt1 = Point_2(vertices[v_id_fea1]->loc[0], vertices[v_id_fea1]->loc[1]);
			}
			if (v_id_fea2 != -1)
			{
				pt2 = Point_2(vertices[v_id_fea2]->loc[0], vertices[v_id_fea2]->loc[1]);
			}

			bool bcorrect_side = false;
			if (int(CGAL::area(pt1, pt2, ptest) > 1e-8) +
				int(CGAL::area(pt1, pt0, ptest) > 1e-8) +
				int(CGAL::area(pt0, pt2, ptest) > 1e-8) > 1)
			{
				bcorrect_side = true;
			}

			if (bcorrect_side)
			{
				if (vertices[facet_vid[it]]->type.is_fea_corner)
				{
					fea_v_pair.insert(pair<int, int>(facet_vid[it], facet_vid[it]));
				}
				else
				{
					if (fea_v_pair.find(facet_vid[it]) != fea_v_pair.end())
					{
						f->verts[it] = vertices[fea_v_pair.at(facet_vid[it])];
					}
					else
					{
						SUB_SIMPLIFY::vertex *v = new SUB_SIMPLIFY::vertex;
						*v = *(vertices[facet_vid[it]]);
						v->loc[2] = 0.1;
						v->id = id_v;
						v->added = true;
						vertices.push_back(v);
						f->verts[it] = v;

						fea_v_pair.insert(pair<int, int>(id_v, facet_vid[it]));
						fea_v_pair.insert(pair<int, int>(facet_vid[it], id_v));

						id_v++;
					}
				}
			}
		}

		mesh_sub.faces.push_back(f);
	}
	mesh_sub.vertices = vertices;

	QString test_file_name = out_file_name;
	test_file_name.append("/split_mesh.obj");
	SUB_SIMPLIFY::saveObjFile(test_file_name.toStdString(), mesh_sub);

	for (int i = 0; i < collinear_knots_lines.size(); i++)
	{
		for (int j = 0; j < collinear_knots_lines[i].size() - 1; j++)
		{
			/*if ((j == 0|| j == collinear_knots_lines[i].size() - 2) && collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index !=
				collinear_knots_lines[i][0].index)
			{
				continue;
			}*/
			int v1 = collinear_knots_lines[i][j].index;
			int v2 = collinear_knots_lines[i][j + 1].index;
			SUB_SIMPLIFY::vertpair e1 = SUB_SIMPLIFY::makeVertpair(v1,v2);
			SUB_SIMPLIFY::vertpair e2 = SUB_SIMPLIFY::makeVertpair
			(fea_v_pair.at(v1), fea_v_pair.at(v2));
			sub_fea_e_pair[e1] = e2;
			sub_fea_e_pair[e2] = e1;
		}
	}

	new_feature.clear();
	for (int i = 0; i < collinear_knots_lines.size(); i++)
	{
		std::vector<int> fea;
		for (int j = 0; j < collinear_knots_lines[i].size() - 1; j++)
		{
			fea.push_back(collinear_knots_lines[i][j].index);
		}
		new_feature.push_back(fea);
	}
}

void Sub_Approximation::save_subdivision_mesh(std::string outfile, const SUB_SIMPLIFY::MeshSimplify& mesh)
{
	ofstream fout;
	fout.open(outfile);
	if (fout.is_open())
	{
		for (auto v = mesh.vertices.begin(); v != mesh.vertices.end(); v++)
		{
			double z_ = 0.2989*(*v)->fitted_color[0] + 0.5870*(*v)->fitted_color[1] + 0.1140*(*v)->fitted_color[2];
			fout << "v " << (*v)->loc[0] << " " << (*v)->loc[1] << " " << z_<< " "
				<< (*v)->fitted_color[0] << " " << (*v)->fitted_color[1] << " " << (*v)->fitted_color[2] << "\n";
		}
		for (auto f = mesh.faces.begin(); f != mesh.faces.end(); f++)
		{
			fout << "f ";
			for (auto fv = (*f)->verts.begin(); fv != (*f)->verts.end(); fv++)
			{
				fout << (*fv)->id + 1 << " ";
			}
			fout << "\n";
		}
		fout.close();

		cout << "Save mesh: " << endl
			<< "  " << mesh.vertices.size() << " vertices." << endl
			<< "  " << mesh.faces.size() << " faces in OBJ file." << endl;
	}
}

