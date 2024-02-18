#include "Corefuncs/ImageApproximation.h"
#include <QtWidgets/QMessageBox>

Image_Approximation::Image_Approximation()
{
	//!!!!!do not update knots automatically

	///////////////////////////adjustable parameter
	//feature points
	gradient_threshold = 30;
	anchor_threshold =8;
	anchor_width = 4;
	min_legth_feature = 12;
	do_consider_cirlcewane = true;
	degree_ofsmoothing_feaP = 1;
	//pepper3 45-12-4-12
	//durian 45-12-4-8
	//eggflower5 36-8-4-15
	//eggflowerl1 40-10-4-12

	//initial number of knots
	nknotinsert_opti = 150;

	//add auxiliary knot near feature
	do_insert_feature_neighbor_knots = true;	//important parameter for knots //one p added currently
	//related to REMOVABLE_ONE_ONLY

	//increase knots iterated
	nknotinsert_ite = 10;
	err_thres = 20.0;

	//iteration for collinear knots
	Nite_add_collinear_knots = 0;
	threshold_error = 1.0;

	///////////////////////////default parameter
	originalMesh = NULL;
	input_imageMesh = NULL;
	input_image = NULL;
	imagemesh_dx = IMAGEWIDTHSIZE;
	model_position_orientation.clear();
	is_fixed_modelview = false;

	//polylines
	simply_error = 4;							//the threshold to simplify the (pixel segments) feature lines; (value*pixel-width); the larger, the more compact
	bsimplyfeature = false;
	has_compute_feature = false;

	//sample rate
	sample_feature_curve_rate = 3;

	//make up knots
	nknotinsert_collinear = 2;					//!!! number of collinear knots; default: 2; To get more compact result: set 1 or 0
	do_compute_collinear_knots = false;

	//optimize knot
	opti_knottri = NULL;
	niterate_opti = 10;							//initial number of knots
	remove_criteria_closeto_featureedge = 2.0;	//!!! the threshold that is used for removing knots near the feature

	knot_mesh = NULL;
	knots_config = NULL;
	new_knot_config = NULL;

	bparameterization = false;
	kdTree_newdomain = NULL;
	kdTree_olddomain = NULL;
	is_build_domain_kd_tree = false;

	ltp = NULL;

	fitting = NULL;
	nDeg = 2;								//!!! degree of TCB-splines
	Nite_now = 0;

	//fitting weight
	image_bound_weight = 50.0;
	feature_pos_weight = 2.0;
	pos_fair_firder_wei = 5.0;
	pos_fair_sedder_wei = 5.0;
	feature_color_1band_weight = 0.5;

	//refinement which will plus 2
	local_refine_rate_fea = 5.0;
	local_refine_rate = 1.0;
	whole_refine_rate_fea = 3.0;
	whole_refine_rate = 0.0;

	triType = DDT;
	optAlg = LOP;
	fittedMesh = NULL;
	fitted_image = NULL;
}

Image_Approximation::~Image_Approximation()
{
	annClose();
	if (kdTree_newdomain != NULL)
	{
		delete kdTree_newdomain;
		kdTree_newdomain = NULL;
	}
	if (kdTree_olddomain != NULL)
	{
		delete kdTree_olddomain;
		kdTree_olddomain = NULL;
	}
	if (fitting != NULL)
	{
		delete fitting;
		fitting = NULL;
	}

	release_basis(bSplineBasis);

	if (ltp)
	{
		delete ltp;
		ltp = NULL;
	}

	if (originalMesh != NULL)
	{
		delete originalMesh;
		originalMesh = NULL;
	}
	if (input_imageMesh)
	{
		delete input_imageMesh;
		input_imageMesh = NULL;
	}
	if (fittedMesh != NULL)
	{
		delete fittedMesh;
		fittedMesh = NULL;
	}
	if (input_image)
	{
		delete input_image;
		input_image = NULL;
	}
	if (fitted_image)
	{
		delete fitted_image;
		fitted_image = NULL;
	}
	if (opti_knottri)
	{
		delete opti_knottri;
		opti_knottri = NULL;
	}
	if (knot_mesh)
	{
		delete knot_mesh;
		knot_mesh = NULL;
	}
	if (new_knot_config)
	{
		delete new_knot_config;
		new_knot_config = NULL;
	}
	if (knots_config)
	{
		delete knots_config;
		knots_config = NULL;
	}
}

//file IO
bool Image_Approximation::read_surface(const QString &name)
{
	if (originalMesh)
	{
		originalMesh = new Mesh;
	}
	read_mesh(&originalMesh, name);

	return true;
}

bool Image_Approximation::write_surface(const QString &name)
{
	write_mesh(originalMesh, name, false, false, false);
	return true;
}

bool Image_Approximation::read_feature_lines_set(const QString &name)
{
	//normalize
	int lengthw = input_image->width();
	int lengthh = input_image->height();
	double step_ = 2.0*IMAGEWIDTHSIZE / (lengthw - 1);

#if 0
	///////////////////precision test
	std::cout << "///////////////////precision test \n";
	double x1=265.0, x2=266.0, x3=267.0, y1=265.0, y2=266.0, y3=267.0;
	Point_2 p1(step_ * x1, step_ * y1 / (2.0*imagemesh_dy));
	Point_2 p2(step_ * x2, step_ * y2 / (2.0*imagemesh_dy));
	Point_2 p3(step_ * x3, step_ * y3 / (2.0*imagemesh_dy));
	double tarea = abs(area(p1,p2,p3));

	double v1[2] = { x1,y1 };
	double v2[2] = { x2,y2 };
	double v3[2] = { x3,y3 };
	double parea = orient2d(v1,v2,v3);

	double ld = abs((y3-y1)*(x2-x1) - (y2-y1)*(x3-x1));
	std::cout << "precision test: pixel(265,265)-(266,266)-(267,267) \n" << "CGAL-area: " << setprecision(16) << tarea
		<< "\n gp-area: " << setprecision(16) << parea << "\n slope-dis: " << setprecision(16) << ld << "\n";
	std::cout << "///////////////////precision test finished\n";
	////////////////////
#endif

	std::string filename = name.toStdString();
	std::ifstream finfea;
	finfea.open(filename);
	if (finfea.is_open())
	{
		vector<vector<PointTriple>> lines_;
		int line_size;
		while (finfea >> line_size)
		{
			vector<PointTriple> line_;
			for (int i = 0; i < line_size; i++)
			{
				double x;
				double y;
				finfea >> x;
				finfea >> y;
				y = double(lengthh) - 1 - y;
				Point_2 pimage_(x, y);

				//the y-index of input txt is reverse
				int index_x = int(x + 0.5);
				int index_y = int(y + 0.5);

				//image index from 0 to image-w/h-1
				double x_1 = step_ * x;
				double y_1 = step_ * y / (2.0*imagemesh_dy);
				Point_2 pdomain_(x_1, y_1);

				PointTriple p_temp;
				p_temp.index_x = index_x;
				p_temp.index_y = index_y;
				p_temp.imagedomain = pdomain_;
				p_temp.imagep = pimage_;
				p_temp.is_fixed = false;
				line_.push_back(p_temp);
			}
			lines_.push_back(line_);
		}
		//plus boundary
		for (int i = 0;i<lines_.size();i++)
		{
			//head
			PointTriple p_temp;
			if (lines_[i][0].index_x == 1)
			{
				p_temp.index_y = lines_[i][0].index_y;
				p_temp.index_x = 0;
				p_temp.imagedomain = Point_2(0.0,lines_[i][0].imagedomain.y());
				p_temp.imagep = Point_2(0.0,lines_[i][0].imagep.y());
				p_temp.is_fixed = false;
				lines_[i].insert(lines_[i].begin(),p_temp);
			}
			else if (lines_[i][0].index_y == 1)
			{
				p_temp.index_y = 0;
				p_temp.index_x = lines_[i][0].index_x;
				p_temp.imagedomain = Point_2(lines_[i][0].imagedomain.x(),0.0);
				p_temp.imagep = Point_2(lines_[i][0].imagep.x(),0.0);
				p_temp.is_fixed = false;
				lines_[i].insert(lines_[i].begin(), p_temp);
			}
			else if (lines_[i][0].index_x == input_image->width() - 2)
			{
				p_temp.index_y = lines_[i][0].index_y;
				p_temp.index_x = input_image->width()-1;
				p_temp.imagedomain = Point_2(1.0, lines_[i][0].imagedomain.y());
				p_temp.imagep = Point_2(lines_[i][0].imagep.x()+1.0, lines_[i][0].imagep.y());
				p_temp.is_fixed = false;
				lines_[i].insert(lines_[i].begin(), p_temp);
			}
			else if (lines_[i][0].index_y == input_image->width() - 2)
			{
				p_temp.index_y = input_image->width() - 1;
				p_temp.index_x = lines_[i][0].index_x;
				p_temp.imagedomain = Point_2(lines_[i][0].imagedomain.x(),1.0);
				p_temp.imagep = Point_2(lines_[i][0].imagep.x(), lines_[i][0].imagep.y() + 1.0);
				p_temp.is_fixed = false;
				lines_[i].insert(lines_[i].begin(), p_temp);
			}
			//end
			if (lines_[i][lines_[i].size() - 1].index_x == 1)
			{
				p_temp.index_y = lines_[i][lines_[i].size() - 1].index_y;
				p_temp.index_x = 0;
				p_temp.imagedomain = Point_2(0.0, lines_[i][lines_[i].size() - 1].imagedomain.y());
				p_temp.imagep = Point_2(0.0, lines_[i][lines_[i].size() - 1].imagep.y());
				p_temp.is_fixed = false;
				lines_[i].push_back(p_temp);
			}
			else if (lines_[i][lines_[i].size() - 1].index_y == 1)
			{
				p_temp.index_y = 0;
				p_temp.index_x = lines_[i][lines_[i].size() - 1].index_x;
				p_temp.imagedomain = Point_2(lines_[i][lines_[i].size() - 1].imagedomain.x(), 0.0);
				p_temp.imagep = Point_2(lines_[i][lines_[i].size() - 1].imagep.x(), 0.0);
				p_temp.is_fixed = false;
				lines_[i].push_back(p_temp);
			}
			else if (lines_[i][lines_[i].size() - 1].index_x == input_image->width() - 2)
			{
				p_temp.index_y = lines_[i][lines_[i].size() - 1].index_y;
				p_temp.index_x = input_image->width() - 1;
				p_temp.imagedomain = Point_2(1.0, lines_[i][lines_[i].size() - 1].imagedomain.y());
				p_temp.imagep = Point_2(lines_[i][lines_[i].size() - 1].imagep.x() + 1.0, lines_[i][lines_[i].size() - 1].imagep.y());
				p_temp.is_fixed = false;
				lines_[i].push_back(p_temp);
			}
			else if (lines_[i][lines_[i].size() - 1].index_y == input_image->width() - 2)
			{
				p_temp.index_y = input_image->width() - 1;
				p_temp.index_x = lines_[i][lines_[i].size() - 1].index_x;
				p_temp.imagedomain = Point_2(lines_[i][lines_[i].size() - 1].imagedomain.x(), 1.0);
				p_temp.imagep = Point_2(lines_[i][lines_[i].size() - 1].imagep.x(), lines_[i][lines_[i].size() - 1].imagep.y() + 1.0);
				p_temp.is_fixed = false;
				lines_[i].push_back( p_temp);
			}

		}
		image_feature_lines_set = lines_;

		//feature flag
		// build KD-tree
		int				k = 1;									// number of nearest neighbors
		int				dim = 2;								// dimension
		double			eps = 0;								// error bound
		int				maxPts=originalMesh->size_of_vertices();// maximum number of data points

		int				nPts = originalMesh->size_of_vertices();// actual number of data points
		ANNpointArray	dataPts_temp;							// data points
		ANNkd_tree*		kdTree_temp;							// search structure

		dataPts_temp = annAllocPts(originalMesh->size_of_vertices(), dim);
		Vertex_iterator dstIt = originalMesh->vertices_begin();
		int i = 0;
		for (; dstIt != originalMesh->vertices_end(); dstIt++)
		{
			dataPts_temp[i][0] = dstIt->vertex_coordinate().first;
			dataPts_temp[i][1] = dstIt->vertex_coordinate().second;
			i++;
		}
		kdTree_temp = new ANNkd_tree(							// build search structure
			dataPts_temp,										// the data points
			nPts,												// number of points
			dim);												// dimension of space

		//search
		for (int i = 0; i < image_feature_lines_set.size(); i++)
		{
			for (int j = 0; j < image_feature_lines_set[i].size(); j++)
			{
				ANNpoint			queryPt_temp;				// query point
				ANNidxArray			nnIdx_temp;					// near neighbor indices
				ANNdistArray		dists_temp;					// near neighbor distances

				queryPt_temp = annAllocPt(dim);					// allocate query point
				nnIdx_temp = new ANNidx[k];						// allocate near neigh indices
				dists_temp = new ANNdist[k];					// allocate near neighbor dists

				queryPt_temp[0] = image_feature_lines_set[i][j].index_x;
				queryPt_temp[1] = image_feature_lines_set[i][j].index_y;

				kdTree_temp->annkSearch(						// search
					queryPt_temp,								// query point
					k,											// number of near neighbors
					nnIdx_temp,									// nearest neighbors (returned)
					dists_temp,									// distance (returned)
					eps);										// error bound

				Mesh::Vertex_iterator vit = originalMesh->get_vertex_iterator(nnIdx_temp[0]);
				if (vit->vertex_coordinate().first == image_feature_lines_set[i][j].index_x &&
					vit->vertex_coordinate().second == image_feature_lines_set[i][j].index_y)
				{
					vit->is_feature() = true;
					image_feature_lines_set[i][j].vit = vit;
				}
				else
				{
					std::cout << __FUNCTION__ << ": " << "KD-tree search wrong: no finding" << std::endl;
				}

				delete[] nnIdx_temp;							// clean things up
				delete[] dists_temp;
			}
		}

		delete kdTree_temp;
	}
	else
	{
		std::cout << __FUNCTION__ << ": " << "read feature lines wrong" << std::endl;
		return false;
	}
	finfea.close();

	feature_lines_modify = image_feature_lines_set;

	return true;
}

bool Image_Approximation::read_image(const QString &name)
{
	fileName = name;
	QString filename = name;
	if (!filename.isNull())
	{
		input_image = new QImage;
		if (input_image->load(filename))
		{
			cout << "image open down - size: " << input_image->width() << ' ' << input_image->height() << endl;
		}
		else
		{
			cout << "image open wrong" << endl;
			return false;
		}
	}
#if 1
	input_imageMesh = new Mesh;
	image2mesh(input_image, &input_imageMesh);
	originalMesh = new Mesh;
	*originalMesh = *input_imageMesh;
	originalMesh->map_vertex_index_to_iterator();
	cout << "original mesh constructed" << endl;
#endif

	//construct file
	out_file_name = "Results/";
	out_file_name.append(fileName.split("/").last().split(".").at(0));
	generate_output_directory(out_file_name);
	if (Nite_now > 0)
	{
		last_file_name = "Results/";
		last_file_name.append(fileName.split("/").last().split(".").at(0));
		generate_output_directory(last_file_name);
	}
	simply_error = simply_error / double(input_image->width() - 1);
	//simply_error = 0.0197;

	return true;
}

bool Image_Approximation::write_image(const QString &name)
{
	QString filename;
	filename = name;
	fitted_image->save(filename);
	return true;
}

bool Image_Approximation::read_mesh(Mesh **pmesh, const QString &name)
{
	QString strName = name;
	if (*pmesh == NULL)
		*pmesh = new Mesh;
	else
		(*pmesh)->clear();

	if (name.isEmpty())
	{
		QMessageBox::critical(NULL, "Failed to open", "Did not select any file", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
		return false;
	}
	QString extension = name.right(4);
	if (extension == ".off")
	{
		std::ifstream f(name.toLatin1());
		if (!f)
		{
			QMessageBox::critical(NULL, "Failed to open", "Unable to open file", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
			return false;
		}
		f >> **pmesh;
	}
	else if (extension == ".obj")
	{
		Parser_obj<Kernel3, Enriched_items> parser;
		if (parser.read_surface_with_curvature_domain_color(strName, *pmesh))
		{
			std::cout << __FUNCTION__ << ": " << "read mesh ";
		}
	}
	else
	{
		QMessageBox::critical(NULL, "Unknown extension", "Unknown extension", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
		return false;
	}

	(*pmesh)->compute_type();
	(*pmesh)->compute_normals();
	(*pmesh)->compute_index();
	(*pmesh)->initial_mesh_status();
	(*pmesh)->map_vertex_index_to_iterator();
	(*pmesh)->computeBorderVerticesType();
	std::cout << __FUNCTION__ << ": " << "success\n";
	return true;
}

bool Image_Approximation::write_mesh(Mesh *mesh_, const QString &name, bool bcurvature, bool bdoamin, bool bcolor)
{
	QString strName = name;
	if (name.isEmpty())
	{
		QMessageBox::critical(NULL, "Failed to write", "Did not select any file", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
		return false;
	}
	mesh_->write_obj_with_curvature_domain_color(name, bcurvature, bcurvature, bdoamin, bcolor);
	return true;
}

bool Image_Approximation::read_editdata(const QString name, EditFM &fm_,
	CPs &cp_seqs, EditCM &cm_, QPoint &image_size) {

	QFile file(name);
	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QTextStream in(&file);
		QString line = in.readLine();
		if (line.isNull())
			return false;

		while (!line.isNull())
		{
			QStringList list = line.split(" ");
			if (!list.at(0).compare("End"))
			{
				line = in.readLine();
				break;
			}
			if (!list.at(0).compare("InputImage"))
			{
				image_size = QPoint(list[1].toInt(), list[2].toInt());
			}
			line = in.readLine();
		}

		//control mesh
		int nver = 0;
		while (!line.isNull())
		{
			QStringList list = line.split(" ");
			if (!list.at(0).compare("End"))
			{
				line = in.readLine();
				break;
			}
			if (!list.at(0).compare("vertex"))
			{
				EditVertex cmvi;
				RGBPoint cp;
				cp.x = list[1].toDouble(); cp.y = list[2].toDouble(); cp.r = list[3].toDouble();
				cp.g = list[4].toDouble(); cp.b = list[5].toDouble();
				cp.r = std::max(cp.r, 0.0); cp.r = std::min(cp.r, 1.0);
				cp.g = std::max(cp.g, 0.0); cp.g = std::min(cp.g, 1.0);
				cp.b = std::max(cp.b, 0.0); cp.b = std::min(cp.b, 1.0);
				cmvi.cp = cp;
				cm_.vertices.push_back(cmvi);
			}
			if (!list.at(0).compare("support"))
			{
				map<unsigned, double> support_;
				for (int j = 1; j < list.size() - 1; j = j + 2)
				{
					unsigned si = list[j].toUInt();
					double val_ = list[j + 1].toDouble();
					support_[si] = val_;
				}
				cm_.vertices[nver].supports = support_;
			}
			if (!list.at(0).compare("cppair"))
			{
				RGBPoint cp1;
				cp1.x = list[1].toDouble(); cp1.y = list[2].toDouble(); cp1.r = list[3].toDouble();
				cp1.g = list[4].toDouble(); cp1.b = list[5].toDouble();
				cp1.r = std::max(cp1.r, 0.0); cp1.r = std::min(cp1.r, 1.0);
				cp1.g = std::max(cp1.g, 0.0); cp1.g = std::min(cp1.g, 1.0);
				cp1.b = std::max(cp1.b, 0.0); cp1.b = std::min(cp1.b, 1.0);
				RGBPoint cp2;
				cp2.x = list[6].toDouble(); cp2.y = list[7].toDouble(); cp2.r = list[8].toDouble();
				cp2.g = list[9].toDouble(); cp2.b = list[10].toDouble();
				cp2.r = std::max(cp2.r, 0.0); cp2.r = std::min(cp2.r, 1.0);
				cp2.g = std::max(cp2.g, 0.0); cp2.g = std::min(cp2.g, 1.0);
				cp2.b = std::max(cp2.b, 0.0); cp2.b = std::min(cp2.b, 1.0);
				cm_.vertices[nver].cp_pair = pair<RGBPoint, RGBPoint>(cp1, cp2);
			}
			if (!list.at(0).compare("other"))
			{
				cm_.vertices[nver].bCCWorientation = list[1].toInt();
				cm_.vertices[nver].is_on_feature = list[2].toInt();	
				cm_.vertices[nver].is_bound = list[3].toInt();
				cm_.vertices[nver].domain_midEdge = Point_2(list[4].toDouble(), list[5].toDouble());
				if (list[2].toInt())
				{
					cm_.vertices[nver].previous_pair = cm_.vertices[nver].cp_pair;
				}
				else
				{
					cm_.vertices[nver].previous_cp = cm_.vertices[nver].cp;
				}
				nver++;
			}
			if (!list.at(0).compare("edge"))
			{
				CMEdgeInfo cmei(list[1].toUInt(), list[2].toUInt());
				cm_.edges.insert(cmei);
			}
			line = in.readLine();
		}
		
		//control points seq
		while (!line.isNull())
		{
			QStringList list = line.split(" ");
			if (!list.at(0).compare("End"))
			{
				line = in.readLine();
				break;
			}
			if (!list.at(0).compare("cps"))
			{
				vector<Point2withIndex> seq_;
				for (int j = 1; j < list.size()-1; j = j + 4)
				{
					Point2withIndex pnow;
					pnow.point2 = Point_2(list[j].toDouble(), list[j + 1].toDouble());
					pnow.index = list[j + 2].toInt();
					pnow.spare_index = list[j + 3].toInt();
					seq_.push_back(pnow);
				}
				cp_seqs.push_back(seq_);
			}
			line = in.readLine();
		}

		//fitted mesh
		while (!line.isNull())
		{
			QStringList list = line.split(" ");
			if (!list.at(0).compare("End"))
			{
				line = in.readLine();
				break;
			}
			if (!list.at(0).compare("v"))
			{
				RGBPoint cp;
				cp.x = list[1].toDouble(); cp.y = list[2].toDouble(); cp.r = list[3].toDouble();
				cp.g = list[4].toDouble(); cp.b = list[5].toDouble();
				cp.r = std::max(cp.r, 0.0); cp.r = std::min(cp.r, 1.0);
				cp.g = std::max(cp.g, 0.0); cp.g = std::min(cp.g, 1.0);
				cp.b = std::max(cp.b, 0.0); cp.b = std::min(cp.b, 1.0);				
				cp.is_on_feature = list[6].toInt();
				fm_.vertices.push_back(cp);
			}
			if (!list.at(0).compare("f"))
			{
				vector<unsigned> facet;
				for (int j = 1;j<list.size()-1;j++)
				{
					facet.push_back(list[j].toUInt());
				}
				CMFaceInfo cmfi; cmfi.face = facet;
				fm_.faces.push_back(cmfi);
			}
			line = in.readLine();
		}

		file.close();
		return 1;
	}

	return 0;
}

bool Image_Approximation::write_editdata(const QString outname)
{
	vector<EditVertex> new_cp_data;
	map<int, pair<int,int>> idpair_trans;
	map<unsigned, int> id_trans;
	int index = 0;
	for (int i = 0; i < cmInfo.vertices.size(); i++)
	{
		if (cmInfo.vertices[i].flag == 1)
		{
			if (cmInfo.vertices[i].is_on_feature)
			{
				idpair_trans[i] = pair<int, int>(index, index + 1);

				EditVertex cp_data;
				cp_data.is_bound = is_on_domain_bound(cmInfo.vertices[i].center) ? true : false;
				cp_data.is_on_feature = cmInfo.vertices[i].is_on_feature;
				cp_data.domain_midEdge = cmInfo.vertices[i].center;
				cp_data.cp = cmInfo.vertices[i].vertex;
				cp_data.cp_pair = pair<RGBPoint, RGBPoint>(cmInfo.vertices[i].control_point_pair.first,
					cmInfo.vertices[i].cpp_little_trans.first);
				int loc = cmInfo.vertices[i].feature_mergebasis_id.first;
				map<unsigned, double> support_;
				for (auto jt = bSplineBasis.basisMergeInfos[loc].basisMerge.begin();
					jt != bSplineBasis.basisMergeInfos[loc].basisMerge.end(); jt++)
				{
					cp_data.bCCWorientation = bSplineBasis.basisConfigs[*jt].bCCWorientation;
					for (auto kt = bSplineBasis.basisConfigs[*jt].supports.begin();
						kt != bSplineBasis.basisConfigs[*jt].supports.end(); kt++)
					{
						if (support_.find(kt->first) == support_.end())
						{
							support_[kt->first] = kt->second.value;
						}
						else
						{
							support_.at(kt->first) += kt->second.value;
						}
					}
				}
				cp_data.supports = support_;
				new_cp_data.push_back(cp_data);
				id_trans[i] = index;
				index++;

				EditVertex cp_data1;
				cp_data1.is_bound = is_on_domain_bound(cmInfo.vertices[i].center) ? true : false;
				cp_data1.is_on_feature = cmInfo.vertices[i].is_on_feature;
				cp_data1.domain_midEdge = cmInfo.vertices[i].center;
				cp_data1.cp = cmInfo.vertices[i].vertex;
				cp_data1.cp_pair = pair<RGBPoint, RGBPoint>(cmInfo.vertices[i].control_point_pair.second,
					cmInfo.vertices[i].cpp_little_trans.second);
				loc = cmInfo.vertices[i].feature_mergebasis_id.second;
				map<unsigned, double> support_1;
				for (auto jt = bSplineBasis.basisMergeInfos[loc].basisMerge.begin();
					jt != bSplineBasis.basisMergeInfos[loc].basisMerge.end(); jt++)
				{
					cp_data1.bCCWorientation = bSplineBasis.basisConfigs[*jt].bCCWorientation;
					for (auto kt = bSplineBasis.basisConfigs[*jt].supports.begin();
						kt != bSplineBasis.basisConfigs[*jt].supports.end(); kt++)
					{
						if (support_1.find(kt->first) == support_1.end())
						{
							support_1[kt->first] = kt->second.value;
						}
						else
						{
							support_1.at(kt->first) += kt->second.value;
						}
					}
				}
				cp_data1.supports = support_1;
				new_cp_data.push_back(cp_data1);
				index++;
			}
			else
			{
				EditVertex cp_data;
				cp_data.is_bound = is_on_domain_bound(cmInfo.vertices[i].center) ? true : false;
				cp_data.is_on_feature = cmInfo.vertices[i].is_on_feature;
				cp_data.domain_midEdge = cmInfo.vertices[i].center;
				cp_data.cp = cmInfo.vertices[i].vertex;
				int loc = cmInfo.vertices[i].merge_basis_id;
				map<unsigned, double> support_;
				for (auto jt = bSplineBasis.basisMergeInfos[loc].basisMerge.begin();
					jt != bSplineBasis.basisMergeInfos[loc].basisMerge.end(); jt++)
				{
					cp_data.bCCWorientation = bSplineBasis.basisConfigs[*jt].bCCWorientation;
					for (auto kt = bSplineBasis.basisConfigs[*jt].supports.begin();
						kt != bSplineBasis.basisConfigs[*jt].supports.end(); kt++)
					{
						if (support_.find(kt->first) == support_.end())
						{
							support_[kt->first] = kt->second.value;
						}
						else
						{
							support_.at(kt->first) += kt->second.value;
						}
					}
				}
				cp_data.supports = support_;
				new_cp_data.push_back(cp_data);
				id_trans[i] = index;
				index++;
			}
		}
	}

	QFile file(outname);
	if (file.open(QIODevice::WriteOnly))
	{
		QTextStream out(&file);
		out.setRealNumberPrecision(10);
		
		out << "InputImage" << " "<<input_image->width() << " " << input_image->height() << "\nEnd\n";

		//control mesh
		out << "ControlMesh\n";
		for (int i = 0; i < new_cp_data.size(); i++)
		{
			out << "vertex " << new_cp_data[i].cp.x << " " << new_cp_data[i].cp.y << " "
				<< new_cp_data[i].cp.r << " " << new_cp_data[i].cp.g << " "
				<< new_cp_data[i].cp.b << "\n";
			out << "cppair " << new_cp_data[i].cp_pair.first.x << " "
				<< new_cp_data[i].cp_pair.first.y << " "
				<< new_cp_data[i].cp_pair.first.r << " "
				<< new_cp_data[i].cp_pair.first.g << " "
				<< new_cp_data[i].cp_pair.first.b << " ";				
			out << new_cp_data[i].cp_pair.second.x << " "
				<< new_cp_data[i].cp_pair.second.y << " "
				<< new_cp_data[i].cp_pair.second.r << " "
				<< new_cp_data[i].cp_pair.second.g << " "
				<< new_cp_data[i].cp_pair.second.b << "\n";
			out << "support ";
			for (auto jt = new_cp_data[i].supports.begin();
				jt != new_cp_data[i].supports.end(); jt++)
			{
				if (jt->first<fittedMesh->size_of_vertices()&&
					jt->second>1e-10)
				{
					out << jt->first << " " << jt->second << " ";
				}		
			}
			out << "\n";
			out << "other " << new_cp_data[i].bCCWorientation << " " <<
				new_cp_data[i].is_on_feature<<" "<<new_cp_data[i].is_bound
				<<" "<<new_cp_data[i].domain_midEdge.x()<<" "<<new_cp_data[i].domain_midEdge.y()<< "\n";
		}
		for (auto eit = cmInfo.edges.begin(); eit != cmInfo.edges.end(); eit++)
		{
			if (cmInfo.vertices[eit->first()].flag == 1 &&
				cmInfo.vertices[eit->second()].flag == 1)
			{
				out << "edge " << id_trans.at(eit->first()) << " " << id_trans.at(eit->second()) << "\n";
			}
		}		
		out << "End\n";

		//control points sequences
		out << "CPSequence\n";
		for (int i = 0;i<control_points_seqs.size();i++)
		{
			bool bout = false;
			for (int j = 0; j < control_points_seqs[i].size()-1; j++)
			{
				int v1 = control_points_seqs[i][j].flag;
				int v2 = control_points_seqs[i][j+1].flag;
				if (idpair_trans.find(v1) != idpair_trans.end() &&
					idpair_trans.find(v2) != idpair_trans.end()) {
					bout = true;
					break;
				}
			}
			if (bout) {
				out << "cps ";
				for (int j = 0; j < control_points_seqs[i].size(); j++)
				{
					if (idpair_trans.find(control_points_seqs[i][j].flag) != idpair_trans.end()) {
						out << control_points_seqs[i][j].vertex.x << " "
							<< control_points_seqs[i][j].vertex.y << " "
							<< idpair_trans.at(control_points_seqs[i][j].flag).first << " "
							<< idpair_trans.at(control_points_seqs[i][j].flag).second << " ";
					}
				}
				out << "\n";
			}
		}
		out << "End\n";

		//fitted mesh
		out << "FittedMesh\n";
		for (Vertex_iterator vit = fittedMesh->vertices_begin(); vit != fittedMesh->vertices_end(); vit++)
		{
			out << "v " << vit->point().x() << " " << vit->point().y() << " ";
			out << vit->vertex_pixel_color().x() << " " << vit->vertex_pixel_color().y() << " "
				<< vit->vertex_pixel_color().z() << " "<<vit->is_feature()<<"\n";
		}
		for (Facet_iterator fit = fittedMesh->facets_begin(); fit != fittedMesh->facets_end(); fit++)
		{
			out << "f ";
			Halfedge_around_facet_circulator pHalfedge = fit->facet_begin();
			do
			{
				out << pHalfedge->vertex()->vertex_index() << " ";
			} while (++pHalfedge != fit->facet_begin());
			out << "\n";
		}
		out << "End\n";

		file.close();

		return 1;
	}

	return 0;
}

//data IO
void Image_Approximation::set_initial_parameter(double gradient_thres, double anchor_thres,
	double anchor_wid, double min_length, int initial_knots, int num_ite)
{
	gradient_threshold = gradient_thres;
	anchor_threshold = anchor_thres;
	anchor_width = anchor_wid;
	min_legth_feature = min_length;
	nknotinsert_opti = initial_knots;
	Nite_now = num_ite;
}

vector<vector<PointTriple>> &Image_Approximation::get_feature_lines(bool is_original)
{
	return is_original ? image_feature_lines_set : feature_lines_modify;
}

vector<vector<PointTriple>>& Image_Approximation::get_knots_lines()
{
	return collinear_knots_lines;
}

bool &Image_Approximation::check_insert_feature_neighbor_knots()
{
	return do_insert_feature_neighbor_knots;
}

vector<vector<CMVertexInfo>> Image_Approximation::get_control_points_seqs()
{
	return control_points_seqs;
}

QImage * Image_Approximation::get_input_image()
{
	return input_image;
}
QString Image_Approximation::get_output_filename()
{
	return out_file_name;
}
QImage * Image_Approximation::get_approxi_image()
{
	return fitted_image;
}


Point_2 Image_Approximation::get_image_size()
{
	Point_2 p(imagemesh_dx,imagemesh_dy);
	return p;
}

Mesh *Image_Approximation::get_originalsurface_mesh()
{
	return originalMesh;
}
Mesh *Image_Approximation::get_fittedsurface_mesh()
{
	return fittedMesh;
}

void Image_Approximation::get_supplement_data(vector<Point_2>&d1, vector<RGBPoint>&dp1, vector<Point_2>&d2, vector<RGBPoint>&dp2,
	vector<Point_2>&d3, vector<RGBPoint>&dp3)
{
	d1 = supplementdomaindata;
	dp1 = supplement5Ddata;

	d2 = featureregion_sampleps;
	dp2 = featureregion_originalps;

	if (featurecurve_originalps.empty())
	{
		smooth_and_sample_feature_curve_points();
	}
	d3 = featurecurve_sampleps;
	dp3 = featurecurve_originalps;
}

CDT	*Image_Approximation::get_knots_config_ct()
{
	return knots_config;
}

Mesh *Image_Approximation::get_knots_config_mesh()
{
	return knot_mesh;
}

CDT * Image_Approximation::get_inserted_knots_config()
{
	return new_knot_config;
}

CMInfo *Image_Approximation::get_control_mesh()
{
	return &cmInfo;
}

BSplineBasis * Image_Approximation::get_basis_configs()
{
	return &bSplineBasis;
}

int & Image_Approximation::get_numberof_inserted_knots()
{
	return nknotinsert_ite;
}

OneStepInfo	*Image_Approximation::get_fitting_infor()
{
	return &(statisticsInfos.infos[0]);
}

//input image to cdt/mesh
void Image_Approximation::image2cdt(QImage *image_, CDT & cdt_image, vector<double> &values)
{
	if (image_ == NULL)
	{
		return;
	}

	//mesh size: x:-0.5-0.5
	int lengthw = image_->width();
	int lengthh = image_->height();

	imagemesh_dx = IMAGEWIDTHSIZE;
	double step_ = 2.0*IMAGEWIDTHSIZE / (lengthw - 1);
	imagemesh_dy= step_ * (lengthh - 1) / 2.0;

	map<int,pair<int, int>> vertex_dindex;
	int index_ = 0;
	for (int i = 0; i<lengthw; i++)
	{
		for (int j = 0; j<lengthh; j++)
		{
			double x = -imagemesh_dx + i*step_;
			double y = -imagemesh_dy + j*step_;
			CDT::Vertex_handle v_;
			v_ = cdt_image.insert(Point_2(x, y));
			v_->set_associated_index(index_);
			vertex_dindex[index_]=pair<int, int>(i, j);
			index_++;
		}
	}
	for (int k = 0; k < lengthw; k++)
	{
		Point_2 y_min(-imagemesh_dx + k*step_, -imagemesh_dy);
		Point_2 y_max(-imagemesh_dx + k*step_, imagemesh_dy);
		cdt_image.insert_constraint(y_min, y_max);
	}
	for (int j = 0; j < lengthh; j++)
	{
		Point_2 x_min(-imagemesh_dx, -imagemesh_dy + j*step_);
		Point_2 x_max(imagemesh_dx, -imagemesh_dy + j*step_);
		cdt_image.insert_constraint(x_min, x_max);
	}
	//gray color
	values.clear();
	for (CDT::Vertex_iterator vit = cdt_image.finite_vertices_begin(); vit != cdt_image.finite_vertices_end(); vit++)
	{
		int gray_v = qGray(input_image->pixel(vertex_dindex[vit->get_associated_index()].first, lengthh - 1 - vertex_dindex[vit->get_associated_index()].second));
		double value = IMAGEGRAYMIN + gray_v *(IMAGEGRAYMAX - IMAGEGRAYMIN) / double(255);
		values.push_back(value);
	}
}

void Image_Approximation::image2mesh(QImage *image_, Mesh **mesh_)
{
	if (image_ == NULL)
	{
		return;
	}
	
#if 0//using CDT to construct mesh
	CDT cdt_temp;
	vector <double> values;
	clock_t start_, makecdt_,make_mesh_;
	start_ = clock();
	image2cdt(image_, cdt_temp, values);
	makecdt_ = clock();
	from_triangulationdata_to_mesh(cdt_temp, mesh_, values);
	make_mesh_ = clock();
#endif

	//using grid
	//mesh size: x:-0.5-0.5
	int lengthw = image_->width();
	int lengthh = image_->height();

	vector<Point_3> vs(lengthw*lengthh);
	vector<vector<int>> faces;
	faces.reserve(2*(lengthw-1)*(lengthh-1));

	imagemesh_dx = IMAGEWIDTHSIZE;
	double step_ = 2.0*IMAGEWIDTHSIZE / (lengthw - 1);
	imagemesh_dy = step_ * (lengthh - 1) / 2.0;

	map<int, pair<int, int>> vertex_dindex;
//#pragma omp parallel for
	for (int i = 0; i < lengthw; i++)
	{
		for (int j = 0; j < lengthh; j++)
		{
			double x = -imagemesh_dx + i*step_;
			double y = -imagemesh_dy + j*step_;

			int gray_v = qGray(input_image->pixel(i, lengthh - 1 - j));
			double value = IMAGEGRAYMIN + gray_v *(IMAGEGRAYMAX - IMAGEGRAYMIN) / double(255);

			int index_ = i*lengthh + j;
			vertex_dindex[index_] = pair<int, int>(i, j);
			vs[index_] = Point_3(x, y, value);

			if (i != 0 && j != 0)
			{			
				if (i == 1 && j == lengthh-1 || i == lengthw - 1 && j == 1)
				{		
					vector<int> facet1 = { (i - 1)*lengthh + (j - 1),(i)*lengthh + (j - 1), (i-1)*lengthh + (j) };
					vector<int> facet2 = { (i - 1)*lengthh + j , (i )*lengthh + (j-1),(i)*lengthh + (j) };
					faces.push_back(facet1);
					faces.push_back(facet2);
				}
				else
				{
					vector<int> facet1 = { (i - 1)*lengthh + (j - 1),(i)*lengthh + (j - 1), (i)*lengthh + (j) };
					vector<int> facet2 = { (i - 1)*lengthh + (j - 1),(i)*lengthh + (j), (i - 1)*lengthh + (j) };
					faces.push_back(facet1);
					faces.push_back(facet2);
				}			
			}
		}
	}

	from_vfdata_to_mesh(mesh_, vs, faces);

	//compute double index
	for (int id = 0;id<vs.size();id++)
	{
		Mesh::Vertex_iterator v_m = (*mesh_)->get_vertex_iterator(id);
		v_m->vertex_coordinate() = vertex_dindex[id];
		QRgb color_ = image_->pixel(vertex_dindex[id].first, image_->height() - 1 - vertex_dindex[id].second);
		Point_3 pixelc(qRed(color_) / 255.0, qGreen(color_) / 255.0, qBlue(color_) / 255.0);
		v_m->vertex_pixel_color() = pixelc;
		v_m->vertex_input_color() = pixelc;
		v_m->vertex_color() = pixelc;
	}

	//compute initial domain 
	for (Mesh::Vertex_iterator vit = (*mesh_)->vertices_begin(); vit != (*mesh_)->vertices_end(); vit++)
	{
		Point_2 domain_p(vit->point().x() + imagemesh_dx, (vit->point().y() + imagemesh_dy) / (2 * imagemesh_dy));
		vit->get_old_domain() = domain_p;
		vit->get_domain() = domain_p;
		vit->is_feature() = false;
		vit->feature_type() = 0;
	}
	(*mesh_)->computeCornerVerticesType();
}

//output cdt/mesh to image
void Image_Approximation::mesh2image()
{
	if (fittedMesh == NULL)
	{
		return;
	}
	if (fitted_image != NULL)
	{
		delete fitted_image;
		fitted_image = NULL;
	}
	fitted_image = new QImage(input_image->width(),input_image->height(),QImage::Format_RGB32 );
	for (Mesh::Vertex_iterator vit = fittedMesh->vertices_begin();vit != fittedMesh->vertices_end();vit++ )
	{
		pair<int, int> coor_ = vit->vertex_coordinate();
		
		int r_ = int(vit->vertex_pixel_color().x()* 255.0 + 0.5);
		int g_ = int(vit->vertex_pixel_color().y()* 255.0 + 0.5);
		int b_ = int(vit->vertex_pixel_color().z()* 255.0 + 0.5);
		r_ = std::max(r_,0);
		r_ = std::min(r_,255);
		g_ = std::max(g_, 0);
		g_ = std::min(g_, 255);
		b_ = std::max(b_, 0);
		b_ = std::min(b_, 255);
		assert(r_ >= 0 && r_ <= 255);
		assert(g_ >= 0 && g_ <= 255);
		assert(b_ >= 0 && b_ <= 255);

		fitted_image->setPixel(coor_.first, input_image->height() - 1 - coor_.second, qRgb(r_, g_, b_));
	}
}

void Image_Approximation::resample_domain_image_on_mesh()
{
	int sz = 2;

	double dx = 1.0/sz / (input_image->width() - 1);
	double dy = 1.0/sz / (input_image->height() - 1);

	cv::Mat image_domain(sz * input_image->height() - sz + 1, sz * input_image->width() - sz + 1,CV_8UC3);

	double tracey = 1.0;
	for (int i = 0;i<sz * input_image->height() - sz + 1;i++)
	{
		double tracex = 0.0;
		for (int j = 0;j<sz * input_image->width() - sz + 1;j++)
		{
			if (tracex>1.0)
			{
				tracex = 1.0;
			}
			if (tracey < 0.0)
			{
				tracey = 0.0;
			}

			Point_2 temp(tracex, tracey);
			vector<int> triangle;
			RGBPoint pt;
			compute_originalcolor_at_domainpoint(temp, triangle, pt);
			int r_ = int(pt.r* 255.0 + 0.5);
			int g_ = int(pt.g* 255.0 + 0.5);
			int b_ = int(pt.b* 255.0 + 0.5);
			r_ = std::max(r_, 0);
			r_ = std::min(r_, 255);
			g_ = std::max(g_, 0);
			g_ = std::min(g_, 255);
			b_ = std::max(b_, 0);
			b_ = std::min(b_, 255);
			image_domain.at<cv::Vec3b>(i, j)[0] = b_;
			image_domain.at<cv::Vec3b>(i, j)[1] = g_;
			image_domain.at<cv::Vec3b>(i, j)[2] = r_;
			tracex += dx;
		}
		tracey += -dy;
	}

	//cv::imshow("domain",image_domain);

	QString gray_file_name = out_file_name;
	gray_file_name.append("/image_domain_gray.png");
	cv::Mat OutImg;
	cv::cvtColor(image_domain, OutImg, cv::COLOR_BGR2GRAY);
	cv::imwrite(gray_file_name.toStdString(), OutImg);

	QString file_name = out_file_name;
	file_name.append("/image_domainrgb.png");
	cv::imwrite(file_name.toStdString(), image_domain);
}

void Image_Approximation::compute_originalcolor_at_domainpoint(Point_2 pt, vector<int> &triangles_int, RGBPoint &pout)
{
	vector<Mesh::Vertex_iterator> triangles_vit;
	if (triangles_int.empty())
	{
		// build old domain KD-tree
		int				k = 3;									// number of nearest neighbors
		int				dim = 2;								// dimension
		double			eps = 0;								// error bound
		int				maxPts = originalMesh->size_of_vertices();// maximum number of data points

																  //search
		ANNpoint			queryPt_temp;				// query point
		ANNidxArray			nnIdx_temp;					// near neighbor indices
		ANNdistArray		dists_temp;					// near neighbor distances

		queryPt_temp = annAllocPt(dim);					// allocate query point
		nnIdx_temp = new ANNidx[k];						// allocate near neigh indices
		dists_temp = new ANNdist[k];					// allocate near neighbor dists

		queryPt_temp[0] = pt.x();
		queryPt_temp[1] = pt.y();

		kdTree_newdomain->annkSearch(						// search
			queryPt_temp,								// query point
			k,											// number of near neighbors
			nnIdx_temp,									// nearest neighbors (returned)
			dists_temp,									// distance (returned)
			eps);										// error bound

		for (int it = 0; it < k; it++)
		{
			set<Mesh::Facet_iterator> two_ring_faces;
			from_interior_to_adjacant_facets(nnIdx_temp[it],2, two_ring_faces, &originalMesh);

			for (auto it_ring = two_ring_faces.begin();it_ring!=two_ring_faces.end();it_ring++)
			{
				Mesh::Halfedge_around_facet_circulator hif = (*it_ring)->facet_begin();
				Mesh::Vertex_iterator v0 = hif->vertex(); hif++;
				Mesh::Vertex_iterator v1 = hif->vertex(); hif++;
				Mesh::Vertex_iterator v2 = hif->vertex(); 
				Point_2 p0 = v0->get_domain();
				Point_2 p1 = v1->get_domain();
				Point_2 p2 = v2->get_domain();
				RGBPoint p0new, p1new, p2new;
				p0new.x = v0->point().x(); p0new.y = v0->point().y();
				p0new.r = v0->vertex_pixel_color().x(); p0new.g = v0->vertex_pixel_color().y(); p0new.b = v0->vertex_pixel_color().z();

				p1new.x = v1->point().x(); p1new.y = v1->point().y();
				p1new.r = v1->vertex_pixel_color().x(); p1new.g = v1->vertex_pixel_color().y(); p1new.b = v1->vertex_pixel_color().z();

				p2new.x = v2->point().x(); p2new.y = v2->point().y();
				p2new.r = v2->vertex_pixel_color().x(); p2new.g = v2->vertex_pixel_color().y(); p2new.b = v2->vertex_pixel_color().z();

				//on points
				if (abs(pt.x() - p0.x()) < 1e-8&&abs(pt.y() - p0.y()) < 1e-8)
				{
					pout = p0new;
					delete[] nnIdx_temp;							// clean things up
					delete[] dists_temp;
					return;
				}
				if (abs(pt.x() - p1.x()) < 1e-8&&abs(pt.y() - p1.y()) < 1e-8)
				{
					pout = p1new;
					delete[] nnIdx_temp;							// clean things up
					delete[] dists_temp;
					return;
				}
				if (abs(pt.x() - p2.x()) < 1e-8&&abs(pt.y() - p2.y()) < 1e-8)
				{
					pout = p2new;
					delete[] nnIdx_temp;							// clean things up
					delete[] dists_temp;
					return;
				}
				//on lines
				if (abs(area(p0, p1, p2)) < 1e-16)
				{
					Point_2 minp, maxp;
					RGBPoint min_pt, max_pt;
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
					if (pt.x() > std::min(minp.x(), maxp.x()) && pt.x() < std::max(minp.x(), maxp.x()) && abs(area(pt, minp, maxp)) < 1e-16)
					{
						double dis_1 = sqrt((pt - minp).squared_length());
						double dis_mm = sqrt((maxp - minp).squared_length());
						double x_test = dis_1 / dis_mm * maxp.x() + (1.0 - dis_1 / dis_mm)*minp.x();
						assert(abs(x_test - pt.x()) < 1e-8);

						double x_ = dis_1 / dis_mm * max_pt.x + (1.0 - dis_1 / dis_mm)*min_pt.x;
						double y_ = dis_1 / dis_mm * max_pt.y + (1.0 - dis_1 / dis_mm)*min_pt.y;
						double r_ = dis_1 / dis_mm * max_pt.r + (1.0 - dis_1 / dis_mm)*min_pt.r;
						double g_ = dis_1 / dis_mm * max_pt.g + (1.0 - dis_1 / dis_mm)*min_pt.g;
						double b_ = dis_1 / dis_mm * max_pt.b + (1.0 - dis_1 / dis_mm)*min_pt.b;
						assert(r_ > -1e-12 && r_<1 + 1e-12 &&g_>-1e-12 && g_<1 + 1e-12 &&b_>-1e-12 && b_ < 1 + 1e-12);
						pout = RGBPoint{ x_, y_,r_,g_,b_ };
						delete[] nnIdx_temp;							// clean things up
						delete[] dists_temp;
						return;
					}
					continue;
				}

				//in triangles
				vector<Point_2> triangle_p = { p0,p1,p2 };
				int ret = CGAL::bounded_side_2(triangle_p.begin(), triangle_p.end(), pt);

				/*Pred::Sign sign;
				Pred gp;
				double x[2]; x[0] = pt.x(); x[1] = pt.y();
				double a[2]; a[0] = p0.x(); a[1] = p0.y();
				double b[2]; b[0] = p1.x(); b[1] = p1.y();
				double c[2]; c[0] = p2.x(); c[1] = p2.y();
				sign = gp.sideOfOpenTriangle(x, a, b, c);*/

				if (ret != CGAL::ON_UNBOUNDED_SIDE /*|| is_on_polygon_convex_bound(pt, triangle_p) */)
				{
					triangles_vit.push_back(v0);
					triangles_vit.push_back(v1);
					triangles_vit.push_back(v2);
					goto FindOut;
				}

				set<Mesh::Facet_iterator>::iterator itend = two_ring_faces.end();
				itend--;
				if (it == k - 1 && it_ring == itend)
				{
					pout.r = (p0new.r + p1new.r + p2new.r) / 3.0;                         
					pout.g = (p0new.g + p1new.g + p2new.g) / 3.0;
					pout.b = (p0new.b + p1new.b + p2new.b) / 3.0;

					delete[] nnIdx_temp;							// clean things up
					delete[] dists_temp;
					std::cout << __FUNCTION__ << ": " << "!!!!! wrong: has not find supplement point 5D£º" <<pt.x()<<","<<pt.y() <<std::endl;
					return;
				}

			}


		}


#if 0
for (int it = 0; it < k; it++)
		{
			Mesh::Vertex_iterator v = originalMesh->get_vertex_iterator(nnIdx_temp[it]);
			vector<int> inte = { nnIdx_temp[it] };
			vector<int> one_ring;
			from_interior_to_ccw_one_ring_cycle(inte, one_ring, &originalMesh);

			for (int it_ring = 0; it_ring < one_ring.size(); it_ring++)
			{
				Mesh::Vertex_iterator v1 = originalMesh->get_vertex_iterator(one_ring[it_ring]);
				Mesh::Vertex_iterator v2 = originalMesh->get_vertex_iterator(one_ring[(it_ring + 1) % one_ring.size()]);
				Point_2 p0 = v->get_domain();
				Point_2 p1 = v1->get_domain();
				Point_2 p2 = v2->get_domain();
				RGBPoint p0new, p1new, p2new;
				p0new.x = v->point().x(); p0new.y = v->point().y();
				p0new.r = v->vertex_pixel_color().x(); p0new.g = v->vertex_pixel_color().y(); p0new.b = v->vertex_pixel_color().z();

				p1new.x = v1->point().x(); p1new.y = v1->point().y();
				p1new.r = v1->vertex_pixel_color().x(); p1new.g = v1->vertex_pixel_color().y(); p1new.b = v1->vertex_pixel_color().z();

				p2new.x = v2->point().x(); p2new.y = v2->point().y();
				p2new.r = v2->vertex_pixel_color().x(); p2new.g = v2->vertex_pixel_color().y(); p2new.b = v2->vertex_pixel_color().z();

				//test
				/*if (pt.x() > min(p2.x(),min(p0.x(),p1.x())) && pt.x() < max(p2.x(), max(p0.x(), p1.x())))
				{
					if (pt.y() > min(p2.y(), min(p0.y(), p1.y())) && pt.y() < max(p2.y(), max(p0.y(), p1.y())))
					{
						int tr = 1;
					}
				}*/

				//on points
				if (abs(pt.x() - p0.x()) < 1e-8&&abs(pt.y() - p0.y()) < 1e-8)
				{
					pout = p0new;
					delete[] nnIdx_temp;							// clean things up
					delete[] dists_temp;
					return;
				}
				if (abs(pt.x() - p1.x()) < 1e-8&&abs(pt.y() - p1.y()) < 1e-8)
				{
					pout = p1new;
					delete[] nnIdx_temp;							// clean things up
					delete[] dists_temp;
					return;
				}
				if (abs(pt.x() - p2.x()) < 1e-8&&abs(pt.y() - p2.y()) < 1e-8)
				{
					pout = p2new;
					delete[] nnIdx_temp;							// clean things up
					delete[] dists_temp;
					return;
				}
				//on lines
				if (abs(area(p0, p1, p2)) < 1e-16)
				{
					Point_2 minp, maxp;
					RGBPoint min_pt, max_pt;
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
					if (pt.x() > std::min(minp.x(), maxp.x()) && pt.x() < std::max(minp.x(), maxp.x()) && abs(area(pt, minp, maxp)) < 1e-16)
					{
						double dis_1 = sqrt((pt - minp).squared_length());
						double dis_mm = sqrt((maxp - minp).squared_length());
						double x_test = dis_1 / dis_mm * maxp.x() + (1.0 - dis_1 / dis_mm)*minp.x();
						assert(abs(x_test - pt.x()) < 1e-8);

						double x_ = dis_1 / dis_mm * max_pt.x + (1.0 - dis_1 / dis_mm)*min_pt.x;
						double y_ = dis_1 / dis_mm * max_pt.y + (1.0 - dis_1 / dis_mm)*min_pt.y;
						double r_ = dis_1 / dis_mm * max_pt.r + (1.0 - dis_1 / dis_mm)*min_pt.r;
						double g_ = dis_1 / dis_mm * max_pt.g + (1.0 - dis_1 / dis_mm)*min_pt.g;
						double b_ = dis_1 / dis_mm * max_pt.b + (1.0 - dis_1 / dis_mm)*min_pt.b;
						assert(r_ > -1e-12 && r_<1 + 1e-12 &&g_>-1e-12 && g_<1 + 1e-12 &&b_>-1e-12 && b_ < 1 + 1e-12);
						pout = RGBPoint{ x_, y_,r_,g_,b_ };
						delete[] nnIdx_temp;							// clean things up
						delete[] dists_temp;
						return;
					}
					continue;
				}
					
				//in triangles
				vector<Point_2> triangle_p = { p0,p1,p2 };
				int ret = CGAL::bounded_side_2(triangle_p.begin(), triangle_p.end(), pt);

				Pred::Sign sign;
				Pred gp;
				double x[2]; x[0] = pt.x(); x[1] = pt.y();
				double a[2]; a[0] = p0.x(); a[1] = p0.y();
				double b[2]; b[0] = p1.x(); b[1] = p1.y();
				double c[2]; c[0] = p2.x(); c[1] = p2.y();
				sign = gp.sideOfOpenTriangle(x, a, b, c);

				if (ret != CGAL::ON_UNBOUNDED_SIDE || is_on_polygon_convex_bound(pt, triangle_p) || sign != -1)
				{
					triangles_vit.push_back(v);
					triangles_vit.push_back(v1);
					triangles_vit.push_back(v2);
					goto FindOut;
				}

				if (it == k - 1 && it_ring == one_ring.size() - 1)
				{
					pout.r = (p0new.r + p1new.r + p2new.r) / 3.0;
					pout.g = (p0new.g + p1new.g + p2new.g) / 3.0;
					pout.b = (p0new.b + p1new.b + p2new.b) / 3.0;

					delete[] nnIdx_temp;							// clean things up
					delete[] dists_temp;
					std::cout << __FUNCTION__ << ": " << "!!!!! wrong: has not find supplement point 5D" << std::endl;
					return;
				}
			}
		}
#endif


	FindOut:
		delete[] nnIdx_temp;							// clean things up
		delete[] dists_temp;
	}
	else
	{
		triangles_vit.push_back(originalMesh->get_vertex_iterator(triangles_int[0]));
		triangles_vit.push_back(originalMesh->get_vertex_iterator(triangles_int[1]));
		triangles_vit.push_back(originalMesh->get_vertex_iterator(triangles_int[2]));
	}

	Mesh::Vertex_iterator v = triangles_vit[0];
	Mesh::Vertex_iterator v1 = triangles_vit[1];
	Mesh::Vertex_iterator v2 = triangles_vit[2];
	Point_2 p0 = v->get_domain();
	Point_2 p1 = v1->get_domain();
	Point_2 p2 = v2->get_domain();
	RGBPoint p0new, p1new, p2new;
	p0new.x = v->point().x(); p0new.y = v->point().y();
	p0new.r = v->vertex_pixel_color().x(); p0new.g = v->vertex_pixel_color().y(); p0new.b = v->vertex_pixel_color().z();
	p1new.x = v1->point().x(); p1new.y = v1->point().y();
	p1new.r = v1->vertex_pixel_color().x(); p1new.g = v1->vertex_pixel_color().y(); p1new.b = v1->vertex_pixel_color().z();
	p2new.x = v2->point().x(); p2new.y = v2->point().y();
	p2new.r = v2->vertex_pixel_color().x(); p2new.g = v2->vertex_pixel_color().y(); p2new.b = v2->vertex_pixel_color().z();

	double a = area(p0, p1, p2);
	//assert(abs(area(p0, p1, p2)) > 1e-16);
	double centc0 = area(pt, p1, p2) / area(p0, p1, p2);
	double centc1 = area(p0, pt, p2) / area(p0, p1, p2);
	double centc2 = 1.0 - centc0 - centc1;
	//assert(abs(centc0*p0.x() + centc1*p1.x() + centc2*p2.x() - pt.x()) < 1e-8);
	//assert(abs(centc0*p0.y() + centc1*p1.y() + centc2*p2.y() - pt.y()) < 1e-8);

	//position
	double x_ = centc0*p0new.x + centc1*p1new.x + centc2*p2new.x;
	double y_ = centc0*p0new.y + centc1*p1new.y + centc2*p2new.y;

	//all of them lie on feature or not
	if (!v->is_feature() && !v1->is_feature() && !v2->is_feature() || v->is_feature() && v1->is_feature() && v2->is_feature())
	{
		double r_ = centc0*p0new.r + centc1*p1new.r + centc2*p2new.r;
		double g_ = centc0*p0new.g + centc1*p1new.g + centc2*p2new.g;
		double b_ = centc0*p0new.b + centc1*p1new.b + centc2*p2new.b;
		assert(r_ > -1e-12 && r_<1 + 1e-12 &&g_>-1e-12 && g_<1 + 1e-12 &&b_>-1e-12 && b_ < 1 + 1e-12);
		pout = RGBPoint{ x_, y_,r_,g_,b_ };
		return;
	}

	double weight_onering = 0.1, weight_tworing = 0.9;

	//only one of them lie on feature
	if (v->is_feature() && !v1->is_feature() && !v2->is_feature() ||
		!v->is_feature() && v1->is_feature() && !v2->is_feature() ||
		!v->is_feature() && !v1->is_feature() && v2->is_feature())
	{
		Mesh::Vertex_iterator vfea, vnor1, vnor2;
		if (v->is_feature() && !v1->is_feature() && !v2->is_feature())
		{
			vfea = v; vnor1 = v1; vnor2 = v2;
		}
		else if (!v->is_feature() && v1->is_feature() && !v2->is_feature())
		{
			vfea = v1; vnor1 = v; vnor2 = v2;
		}
		else if (!v->is_feature() && !v1->is_feature() && v2->is_feature())
		{
			vfea = v2; vnor1 = v; vnor2 = v1;
		}
		Point_2 pfea1, pfea2;
		pfea1 = vfea->get_domain();
		vector<int> inte0 = { vfea->vertex_index() };
		vector<int> one_ring_0;
		from_interior_to_ccw_one_ring_cycle(inte0, one_ring_0, &originalMesh);//carefully: using new feature mesh
		for (int k = 0; k < one_ring_0.size(); k++)
		{
			if (originalMesh->get_vertex_iterator(one_ring_0[k])->is_feature())
			{
				pfea2 = originalMesh->get_vertex_iterator(one_ring_0[k])->get_domain();
				break;
			}
			if (k == one_ring_0.size() - 1)
			{
				std::cout << __FUNCTION__ << ": " << "maybe wrong" << std::endl;
			}
		}

		double r_ = 0, g_ = 0, b_ = 0;
		r_ += (vnor1->vertex_pixel_color().x() + vnor2->vertex_pixel_color().x()) / 2.0;
		g_ += (vnor1->vertex_pixel_color().y() + vnor2->vertex_pixel_color().y()) / 2.0;
		b_ += (vnor1->vertex_pixel_color().z() + vnor2->vertex_pixel_color().z()) / 2.0;

		bool is_plusv1 = false;
		bool is_valid_onering = true;
		vector<int> inte1 = { vnor1->vertex_index() };
		vector<int> one_ring_1;
		from_interior_to_ccw_one_ring_cycle(inte1, one_ring_1, &originalMesh);
		for (int k = 0; k < one_ring_1.size(); k++)
		{
			if (originalMesh->get_vertex_iterator(one_ring_1[k])->is_feature())
			{
				if (abs(CGAL::area(pfea1, pfea2, originalMesh->get_vertex_iterator(one_ring_1[k])->get_domain())) > 1e-12)
				{
					is_valid_onering = false;
					break;
				}
			}
		}
		double r1 = 0, g1 = 0, b1 = 0;
		if (is_valid_onering)
		{
			double num_plus = 0.0;
			for (int k = 0; k < one_ring_1.size(); k++)
			{
				if (!originalMesh->get_vertex_iterator(one_ring_1[k])->is_feature())
				{
					if (CGAL::area(pfea1, pfea2, originalMesh->get_vertex_iterator(one_ring_1[k])->get_domain()) *
						CGAL::area(pfea1, pfea2, vnor1->get_domain()) > 1e-12)
					{
						r1 += originalMesh->get_vertex_iterator(one_ring_1[k])->vertex_pixel_color().x();
						g1 += originalMesh->get_vertex_iterator(one_ring_1[k])->vertex_pixel_color().y();
						b1 += originalMesh->get_vertex_iterator(one_ring_1[k])->vertex_pixel_color().z();
						num_plus += 1.0;
						is_plusv1 = true;
					}
				}
			}
			if (is_plusv1)
			{
				r1 = r1 / num_plus;
				g1 = g1 / num_plus;
				b1 = b1 / num_plus;
			}
		}
		bool is_plusv2 = false;
		is_valid_onering = true;
		vector<int> inte2 = { vnor2->vertex_index() };
		vector<int> one_ring_2;
		from_interior_to_ccw_one_ring_cycle(inte2, one_ring_2, &originalMesh);
		for (int k = 0; k < one_ring_2.size(); k++)
		{
			if (originalMesh->get_vertex_iterator(one_ring_2[k])->is_feature())
			{
				if (abs(CGAL::area(pfea1, pfea2, originalMesh->get_vertex_iterator(one_ring_2[k])->get_domain())) > 1e-12)
				{
					is_valid_onering = false;
					break;
				}
			}
		}
		double r2 = 0, g2 = 0, b2 = 0;
		if (is_valid_onering)
		{
			double num_plus = 0.0;
			for (int k = 0; k < one_ring_2.size(); k++)
			{
				if (!originalMesh->get_vertex_iterator(one_ring_2[k])->is_feature())
				{
					if (CGAL::area(pfea1, pfea2, originalMesh->get_vertex_iterator(one_ring_2[k])->get_domain()) *
						CGAL::area(pfea1, pfea2, vnor2->get_domain()) > 1e-12)
					{
						r2 += originalMesh->get_vertex_iterator(one_ring_2[k])->vertex_pixel_color().x();
						g2 += originalMesh->get_vertex_iterator(one_ring_2[k])->vertex_pixel_color().y();
						b2 += originalMesh->get_vertex_iterator(one_ring_2[k])->vertex_pixel_color().z();
						num_plus += 1.0;
						is_plusv2 = true;
					}
				}
			}
			if (is_plusv2)
			{
				r2 = r2 / num_plus;
				g2 = g2 / num_plus;
				b2 = b2 / num_plus;
			}
		}

		double plusv1 = 0.0, plusv2 = 0.0;
		if (is_plusv1)
		{
			plusv1 = 1.0;
		}
		if (is_plusv2)
		{
			plusv2 = 1.0;
		}
		if (is_plusv1 || is_plusv2)
		{
			r_ = weight_onering*r_ + weight_tworing * (plusv1*r1 + plusv2*r2) / (plusv1 + plusv2);
			g_ = weight_onering*g_ + weight_tworing * (plusv1*g1 + plusv2*g2) / (plusv1 + plusv2);
			b_ = weight_onering*b_ + weight_tworing * (plusv1*b1 + plusv2*b2) / (plusv1 + plusv2);
		}
		assert(r_ > -1e-12 && r_<1 + 1e-12 &&g_>-1e-12 && g_<1 + 1e-12 &&b_>-1e-12 && b_ < 1 + 1e-12);
		pout = RGBPoint{ x_, y_,r_,g_,b_ };
		return;
	}

	//two vertex lie on feature
	if (!v->is_feature() && v1->is_feature() && v2->is_feature() ||
		v->is_feature() && !v1->is_feature() && v2->is_feature() ||
		v->is_feature() && v1->is_feature() && !v2->is_feature())
	{
		Mesh::Vertex_iterator vfea1, vfea2, vnor;
		if (!v->is_feature())
		{
			vfea1 = v1; vfea2 = v2; vnor = v;
		}
		else if (!v1->is_feature())
		{
			vfea1 = v; vfea2 = v2; vnor = v1;
		}
		else if (!v2->is_feature())
		{
			vfea1 = v; vfea2 = v1; vnor = v2;
		}
		Point_2 pfea1, pfea2;
		pfea1 = vfea1->get_domain();
		pfea2 = vfea2->get_domain();

		double r_ = 0, g_ = 0, b_ = 0;
		r_ += vnor->vertex_pixel_color().x();
		g_ += vnor->vertex_pixel_color().y();
		b_ += vnor->vertex_pixel_color().z();

		bool is_plus = false;
		bool is_valid_onering = true;
		vector<int> inte1 = { vnor->vertex_index() };
		vector<int> one_ring_1;
		from_interior_to_ccw_one_ring_cycle(inte1, one_ring_1, &originalMesh);
		for (int k = 0; k < one_ring_1.size(); k++)
		{
			if (originalMesh->get_vertex_iterator(one_ring_1[k])->is_feature())
			{
				if (abs(CGAL::area(pfea1, pfea2, originalMesh->get_vertex_iterator(one_ring_1[k])->get_domain())) > 1e-12)
				{
					is_valid_onering = false;
					break;
				}
			}
		}
		double r1 = 0, g1 = 0, b1 = 0;
		if (is_valid_onering)
		{
			double num_plus = 0.0;
			for (int k = 0; k < one_ring_1.size(); k++)
			{
				if (!originalMesh->get_vertex_iterator(one_ring_1[k])->is_feature())
				{
					if (CGAL::area(pfea1, pfea2, originalMesh->get_vertex_iterator(one_ring_1[k])->get_domain()) *
						CGAL::area(pfea1, pfea2, vnor->get_domain()) > 1e-12)
					{
						r1 += originalMesh->get_vertex_iterator(one_ring_1[k])->vertex_pixel_color().x();
						g1 += originalMesh->get_vertex_iterator(one_ring_1[k])->vertex_pixel_color().y();
						b1 += originalMesh->get_vertex_iterator(one_ring_1[k])->vertex_pixel_color().z();
						num_plus += 1.0;
						is_plus = true;
					}
				}
			}
			if (is_plus)
			{
				r1 = r1 / num_plus;
				g1 = g1 / num_plus;
				b1 = b1 / num_plus;
			}
		}
		if (is_plus)
		{
			r_ = weight_onering*r_ + weight_tworing * r1;
			g_ = weight_onering*g_ + weight_tworing * g1;
			b_ = weight_onering*b_ + weight_tworing * b1;
		}
		assert(r_ > -1e-12 && r_<1 + 1e-12 &&g_>-1e-12 && g_<1 + 1e-12 &&b_>-1e-12 && b_ < 1 + 1e-12);
		pout = RGBPoint{ x_, y_,r_,g_,b_ };
		return;
	}
}

void Image_Approximation::compute_parametepoint_at_imagedpoint(Point_2 pimage, Point_2 &pparameter)
{
	//suppose has build KD-tree already
	int				k = 3;									// number of nearest neighbors
	int				dim = 2;								// dimension
	double			eps = 0;								// error bound
	int				maxPts = originalMesh->size_of_vertices();// maximum number of data points

	ANNpoint			queryPt_temp;						// query point
	ANNidxArray			nnIdx_temp;							// near neighbor indices
	ANNdistArray		dists_temp;							// near neighbor distances

	queryPt_temp = annAllocPt(dim);							// allocate query point
	nnIdx_temp = new ANNidx[k];								// allocate near neigh indices
	dists_temp = new ANNdist[k];							// allocate near neighbor dists

	queryPt_temp[0] = pimage.x();
	queryPt_temp[1] = pimage.y();

	kdTree_olddomain->annkSearch(										// search
		queryPt_temp,										// query point
		k,													// number of near neighbors
		nnIdx_temp,											// nearest neighbors (returned)
		dists_temp,											// distance (returned)
		eps);												// error bound

	for (int it = 0; it < k; it++)
	{
		Mesh::Vertex_iterator v = originalMesh->get_vertex_iterator(nnIdx_temp[it]);
		vector<int> inte = { nnIdx_temp[it] };
		vector<int> one_ring;
		from_interior_to_ccw_one_ring_cycle(inte, one_ring, &originalMesh);

		for (int it_ring = 0; it_ring < one_ring.size(); it_ring++)
		{
			Mesh::Vertex_iterator v1 = originalMesh->get_vertex_iterator(one_ring[it_ring]);
			Mesh::Vertex_iterator v2 = originalMesh->get_vertex_iterator(one_ring[(it_ring + 1) % one_ring.size()]);
			Point_2 p0 = v->get_old_domain();
			Point_2 p1 = v1->get_old_domain();
			Point_2 p2 = v2->get_old_domain();
			Point_2 p0new, p1new, p2new;
			p0new = Point_2(v->get_domain().x(), v->get_domain().y());
			p1new = Point_2(v1->get_domain().x(), v1->get_domain().y());
			p2new = Point_2(v2->get_domain().x(), v2->get_domain().y());

			//on points
			if (abs(pimage.x() - p0.x()) < 1e-8&&abs(pimage.y() - p0.y()) < 1e-8)
			{
				pparameter = p0new;
				goto FindOut;
			}
			if (abs(pimage.x() - p1.x()) < 1e-8&&abs(pimage.y() - p1.y()) < 1e-8)
			{
				pparameter = p1new;
				goto FindOut;
			}
			if (abs(pimage.x() - p2.x()) < 1e-8&&abs(pimage.y() - p2.y()) < 1e-8)
			{
				pparameter = p2new;
				goto FindOut;
			}
			//on lines
			if (abs(area(p0, p1, p2)) < 1e-16)
			{
				if (abs(area(pimage, p1, p2)) < 1e-16)
				{
					Point_2 minp, maxp;
					Point_2 min_ppara, max_ppara;
					if ((p0 - p2).squared_length() > (p0 - p1).squared_length() && (p0 - p2).squared_length() > (p1 - p2).squared_length())
					{
						minp = p0; maxp = p2; min_ppara = p0new; max_ppara = p2new;
					}
					else if ((p0 - p1).squared_length() > (p1 - p2).squared_length() && (p0 - p1).squared_length() > (p0 - p2).squared_length())
					{
						minp = p0; maxp = p1; min_ppara = p0new; max_ppara = p1new;
					}
					else if ((p1 - p2).squared_length() > (p0 - p1).squared_length() && (p1 - p2).squared_length() > (p0 - p2).squared_length())
					{
						minp = p1; maxp = p2; min_ppara = p1new; max_ppara = p2new;
					}
					if (pimage.x() > std::min(minp.x(), maxp.x()) && pimage.x() < std::max(minp.x(), maxp.x()))
					{
						double dis_1 = sqrt((pimage - minp).squared_length());
						double dis_mm = sqrt((maxp - minp).squared_length());
						double x_test = dis_1 / dis_mm * maxp.x() + (1.0 - dis_1 / dis_mm)*minp.x();
						assert(abs(x_test - pimage.x()) < 1e-8);

						double x_ = dis_1 / dis_mm * max_ppara.x() + (1.0 - dis_1 / dis_mm)*min_ppara.x();
						double y_ = dis_1 / dis_mm * max_ppara.y() + (1.0 - dis_1 / dis_mm)*min_ppara.y();
						pparameter = Point_2(x_, y_);
						goto FindOut;
					}
				}
			}
			else //in triangles
			{
				vector<Point_2> triangle_p = { p0,p1,p2 };
				int ret = CGAL::bounded_side_2(triangle_p.begin(), triangle_p.end(), pimage);
				if (ret != CGAL::ON_UNBOUNDED_SIDE)
				{
					double centc0 = area(pimage, p1, p2) / area(p0, p1, p2);
					double centc1 = area(p0, pimage, p2) / area(p0, p1, p2);
					double centc2 = 1.0 - centc0 - centc1;
					assert(abs(centc0*p0.x() + centc1*p1.x() + centc2*p2.x() - pimage.x()) < 1e-8);
					assert(abs(centc0*p0.y() + centc1*p1.y() + centc2*p2.y() - pimage.y()) < 1e-8);

					double x_ = centc0*p0new.x() + centc1*p1new.x() + centc2*p2new.x();
					double y_ = centc0*p0new.y() + centc1*p1new.y() + centc2*p2new.y();

					pparameter = Point_2(x_, y_);
					goto FindOut;
				}
			}
			if (it_ring == one_ring.size() - 1 && it == k - 1)
			{
				pparameter = pimage;
				std::cout << __FUNCTION__ << ": " << "!!!!! wrong: has not find corresponding parameter knot " << pimage.x() << "-" << pimage.y() << std::endl;
			}
		}
	}
FindOut:
	delete[] nnIdx_temp;							// clean things up
	delete[] dists_temp;

}


//fitting options
void Image_Approximation::set_triangulation_type(int t)
{
	if (t == 0)
	{
		triType = DELAUNAY;
	}
	else if (t == 1)
	{
		triType = DDT;
	}
}

void Image_Approximation::set_triangulation_optimization_alg(int toa)
{
	if (toa == 0)
	{
		optAlg = LOP;
	}
	else if (toa == 1)
	{
		optAlg = LOOKAHEAD;
	}
	else if (toa = 2)
	{
		optAlg = SIMULATEDANNEALING;
	}
}

void Image_Approximation::set_fitted_surface_degree(int d)
{
	nDeg = d+2;
	std::cout << __FUNCTION__ << ": " << "degree: " << nDeg << std::endl;
}

//Output mesh for simplification
void Image_Approximation::processForSimplification()
{
	//construct file
	out_file_name = "Results/";
	out_file_name.append(fileName.split("/").last().split(".").at(0));
	filename_foraddcollinear_knots = out_file_name;
	out_file_name.append(QString("/ite%1-d%2").arg(Nite_now).arg(nDeg));
	generate_output_directory(out_file_name);
	if (Nite_now > 0)
	{
		last_file_name = "Results/";
		last_file_name.append(fileName.split("/").last().split(".").at(0));
		last_file_name.append(QString("/ite%1-d%2").arg(Nite_now - 1).arg(nDeg));
		generate_output_directory(last_file_name);
	}

	std::cout << __FUNCTION__ << ": " << fileName.toStdString() << std::endl;
	std::cout << __FUNCTION__ << ": " << out_file_name.toStdString() << std::endl;

	//extract image feature
	compute_image_feature();

	//flip feature edge
	for (int i = 0; i < feature_lines_modify.size(); i++)
	{
		for (int j = 0; j < feature_lines_modify[i].size(); j++)
		{
			int star = feature_lines_modify[i][j].vit->vertex_index();
			int end_ = feature_lines_modify[i][(j + 1)% feature_lines_modify[i].size()].vit->vertex_index();

			Mesh::Vertex_iterator vnow = originalMesh->get_vertex_iterator(star);
			Mesh::Halfedge_around_vertex_circulator hc = vnow->vertex_begin(), hend = hc;
			bool is_flip = false;
			Mesh::Halfedge_iterator htemp;
			do
			{
				Mesh::Halfedge_iterator hside = hc->opposite()->next(), hother = hside->opposite()->next();
				if (hother->vertex()->vertex_index() == end_)
				{
					is_flip = true;
					htemp = hside;
					break;
				}
			} while (++hc != hend);
			if (is_flip)
			{
				Mesh::Halfedge_iterator hnew = originalMesh->flip_edge(htemp);
			}
		}
	}
	QString outfile("initialmesh.obj");
	originalMesh->write_obj_with_vtype(outfile);

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
						for (int n = 2;n<feature_lines_modify[j].size()-2;n++)
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
	
	//check
	for (int i = 0; i < feature_lines_modify.size(); i++)
	{
		vector<PointTriple> front_tail = { feature_lines_modify[i][0],
			feature_lines_modify[i][feature_lines_modify[i].size() - 1] };
		for (int k = 0; k < front_tail.size(); k++)
		{
			bool do_find = false;
			for (int j = 0; j < feature_lines_modify.size(); j++)
			{
				if (i == j)
				{
					continue;
				}
				for (auto it = feature_lines_modify[j].begin(); it != feature_lines_modify[j].end(); it++)
				{
					if (abs(it->index_x - front_tail[k].index_x) <= 1 &&
						abs(it->index_y - front_tail[k].index_y) <= 1)
					{
						do_find = true;
						break;
					}
				}
				if (do_find)
				{
					break;
				}
			}
			if (do_find)
			{
				std::cout << "still found intersection\n";			
			}
		}
	}
	std::cout << "process intersection end\n";

	ofstream out_;
	out_.open("feature.txt");
	if (out_.is_open())
	{
		for (int i = 0; i < feature_lines_modify.size(); i++)
		{
			out_ << feature_lines_modify[i].size() << " ";
			for (int j = 0; j < feature_lines_modify[i].size(); j++)
			{
				out_ << feature_lines_modify[i][j].vit->vertex_index() << " ";
			}
			out_ << "\n";
		}

		out_.close();
	}
	
}

///////////////////////////////////////////////////////////////
void Image_Approximation::mesh_fitting()
{
	//construct file
	out_file_name = "Results/";
	out_file_name.append(fileName.split("/").last().split(".").at(0));
	filename_foraddcollinear_knots = out_file_name;
	out_file_name.append(QString("/ite%1-d%2").arg(Nite_now).arg(nDeg));
	generate_output_directory(out_file_name);
	if (Nite_now >0)
	{
		last_file_name = "Results/";
		last_file_name.append(fileName.split("/").last().split(".").at(0));
		last_file_name.append(QString("/ite%1-d%2").arg(Nite_now-1).arg(nDeg));
		generate_output_directory(last_file_name);
	}

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
	//smooth position
	smooth_and_sample_feature_curve_points();
	project_featureps_onto_smoothps();
	finish = clock();
	statisticsInfos.infos[0].t_feature_process = (double(finish - start)) / CLOCKS_PER_SEC;
	
	//domain
	start = clock();
	compute_constrained_parameterization();
	create_kd_tree();
	finish = clock();
	statisticsInfos.infos[0].t_para = (double(finish - start)) / CLOCKS_PER_SEC;

	//remedy color
	start = clock();
	//remedy_color_of_one_neighbor();
	finish = clock();
	statisticsInfos.infos[0].t_feature_process += (double(finish - start)) / CLOCKS_PER_SEC;
	std::cout << __FUNCTION__ << ": " << "remedy_color_of_one_neighbor finished" << std::endl;
	
	start = clock();
	sample_feature_region(featureregion_sampleps, featureregion_originalps);
	std::cout << __FUNCTION__ << ": " << "sample feature region finished" << std::endl;
	sample_feature_curve_domainps();//after ploy-line, the number of circle feature points plus 1
	std::cout << __FUNCTION__ << ": " << "sample feature curve finished" << std::endl;
	resample_domain_image_on_mesh();
	std::cout << __FUNCTION__ << ": " << "resample domain image finished" << std::endl;
	finish = clock();
	statisticsInfos.infos[0].t_sample_data = (double(finish - start)) / CLOCKS_PER_SEC;

	start = clock();
	optimize_knots_triangulation();
	finish = clock();
	statisticsInfos.infos[0].t_knots_generation = (double(finish - start)) / CLOCKS_PER_SEC;

	knots_transform();
	std::cout << __FUNCTION__ << ": " << "knot generation finished" << std::endl;

	start = clock();
	generate_configs_ofsplitbasis();
	finish = clock();
	statisticsInfos.infos[0].t_basis_config = (double(finish - start)) / CLOCKS_PER_SEC;

	start = clock();
	link_triangulation_procedure();
	finish = clock();
	statisticsInfos.infos[0].t_LTP = (double(finish - start)) / CLOCKS_PER_SEC;
	std::cout << __FUNCTION__ << ": " << "link triangulation procedure finished -LTP config size:" << alltconfigs[nDeg].size() << std::endl;

	if (alltconfigs.empty())
		return;

	for (int i = 0; i < alltconfigs.size(); i++)
		restore_index(correspondence, alltconfigs[i]);
	release_basis(bSplineBasis);
	remove_degraded_simplex(alltconfigs[nDeg]);

	start = clock();
	merge_linearly_dependent_simplex(alltconfigs[nDeg]);
	std::cout << __FUNCTION__ << ": " << "merge linearly dependent simplex finished" << std::endl;

	//combine simplex into basis with same interior knots
	generate_basis_configuraitons(alltconfigs[nDeg], bSplineBasis);
	std::cout << __FUNCTION__ << ": " << "generate basis configurations finished" << std::endl;

	//split basis along feature edge
	splite_rupture_basis();
	std::cout << __FUNCTION__ << ": " << "splitting basis procedure finished" << std::endl;

	//merge linearly dependent basis
	map<int, int> controlpointindex;
	remove_singularities(bSplineBasis, cornerIds, controlpointindex);
	std::cout << __FUNCTION__ << ": " << "remove singularity finished" << std::endl;

	//convert the index for pairs of basis
	for (int k = 0; k < basis_point_pair.size(); k++)
	{
		int fir = controlpointindex.at(basis_point_pair[k].first);
		int sed = controlpointindex.at(basis_point_pair[k].second);
		basis_point_pair[k] = pair<int, int>(fir, sed);
	}
	finish = clock();
	statisticsInfos.infos[0].t_basis_config += (double(finish - start)) / CLOCKS_PER_SEC;

	std::cout << __FUNCTION__ << ": " << "basis point pair finished" << std::endl;

	//useless yet
	initial_control_points();

	//test
	//compute_test_basis();

	start = clock();
	compute_all_basis(originalMesh, bSplineBasis, dataMaps, alltconfig3s,
		supplementdomaindata, featureregion_sampleps, featurecurve_sampleps);
	finish = clock();
	statisticsInfos.infos[0].t_basis_value = (double(finish - start)) / CLOCKS_PER_SEC;
	std::cout << __FUNCTION__ << ": " << "compute all basis finished - all eligible simplex configs:" << alltconfig3s.size() << std::endl;

	//verify
	/*start = clock();
	verify_basis_normalizng();
	finish = clock();
	statisticsInfos.infos[ite_now - 1].t_verify = (double(finish - start)) / CLOCKS_PER_SEC;
	std::cout << __FUNCTION__ << ": " << "basis verify finished" << std::endl;*/

	//useless yet
	//generate_coplanar_condition(originalMesh, bSplineBasis, dataMaps);
	//std::cout << __FUNCTION__ << ": " << "generate coplanar condition finished" << std::endl;

	//for extra supplement data on null-basis
	start = clock();
	compute_supplement_5Ddata();
	finish = clock();
	statisticsInfos.infos[0].t_sample_data += (double(finish - start)) / CLOCKS_PER_SEC;
	std::cout << __FUNCTION__ << ": " << "supplement 5Ddata finished" << std::endl;

#ifdef CONSIDER_G1_CURVE
	generate_G1_pairs();
#endif

	if (!fittedMesh)
	{
		fittedMesh = new Mesh;
		*fittedMesh = *originalMesh;
		fittedMesh->map_vertex_index_to_iterator();
	}
	fitting = new Fitting(originalMesh, fittedMesh, out_file_name,image_bound_weight,
		feature_pos_weight,pos_fair_firder_wei,pos_fair_sedder_wei,feature_color_1band_weight);
	fitting->set_basis(&bSplineBasis);
	fitting->set_knots_data(dataMaps);
	fitting->set_allsimplex(&alltconfig3s);
	fitting->set_constrains(basis_point_pair,G1_constrains);
	fitting->set_supplement_data(supplement5Ddata, featureregion_originalps, featurecurve_originalps);
	fitting->fitting(statisticsInfos.infos[0]);
	std::cout << __FUNCTION__ << ": " << "fitting finished" << std::endl;

	update_tconfig3s(alltconfig3s);

	generate_control_mesh();

	mesh2image();
	std::cout << __FUNCTION__ << ": " << "mesh2image finished" << std::endl;

	//rasterize_image(4);
	//std::cout << __FUNCTION__ << ": " << "rasterize vector image finished" << std::endl;

	compute_mesh_error();
	std::cout << __FUNCTION__ << ": " << "compute error finished" << std::endl;

	clock_t compute_finishc = clock();
	std::cout << __FUNCTION__ << ": " << "time of implement: " << compute_finishc - compute_startc << "\n";
	
	archive_results();

	std::cout << "/////////////////////////////fitting finished,iterate: " << Nite_now << std::endl;
}

void Image_Approximation::gene_control_mesh(QString filename, CMInfo &realCm)
{
	CDT							domain_mesh;
	ACPs						CPs_new;

	std::set<Vpair> edge_constraints;
	map<int, CDT::Vertex_handle> cdt_vertex_handle;

	QFile file(filename);
	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QTextStream in(&file);
		QString line = in.readLine();
		if (line.isNull())
			return;

		//control points seq
		while (!line.isNull())
		{
			QStringList list = line.split(" ");
			if (!list.at(0).compare("End"))
			{
				line = in.readLine();
				break;
			}
			if (!list.at(0).compare("cps"))
			{
				vector<AuthoringP2> seq_;
				for (int j = 1; j < list.size() - 1; j = j + 14)
				{
					AuthoringP2 pnow;
					pnow.point2 = Point_2(list[j].toDouble(), list[j + 1].toDouble());
					pnow.is_cycle = list[j + 2].toInt();
					pnow.id = list[j + 3].toInt();
					pnow.v_positive.x = list[j + 4].toDouble();
					pnow.v_positive.y = list[j + 5].toDouble();
					pnow.v_positive.r = list[j + 6].toDouble();
					pnow.v_positive.g = list[j + 7].toDouble();
					pnow.v_positive.b = list[j + 8].toDouble();
					pnow.v_negative.x = list[j + 9].toDouble();
					pnow.v_negative.y = list[j + 10].toDouble();
					pnow.v_negative.r = list[j + 11].toDouble();
					pnow.v_negative.g = list[j + 12].toDouble();
					pnow.v_negative.b = list[j + 13].toDouble();
					seq_.push_back(pnow);
				}
				CPs_new.push_back(seq_);
			}
			line = in.readLine();
		}

		//domain mesh
		int nver = 0;
		while (!line.isNull())
		{
			QStringList list = line.split(" ");
			if (!list.at(0).compare("End"))
			{
				line = in.readLine();
				break;
			}
			if (!list.at(0).compare("v"))
			{
				int id = list[1].toInt();
				Vpair cps_id(list[2].toInt(), list[3].toInt());
				RGBPoint cp;
				cp.x = list[4].toDouble(); cp.y = list[5].toDouble(); cp.r = list[6].toDouble();
				cp.g = list[7].toDouble(); cp.b = list[8].toDouble();
				cp.r = std::max(cp.r, 0.0); cp.r = std::min(cp.r, 1.0);
				cp.g = std::max(cp.g, 0.0); cp.g = std::min(cp.g, 1.0);
				cp.b = std::max(cp.b, 0.0); cp.b = std::min(cp.b, 1.0);
				cp.is_on_bound = list[9].toInt();
				cp.is_on_feature = list[10].toInt();

				CDT::Vertex_handle vh;
				vh = domain_mesh.insert(Point_2(cp.x, cp.y));
				vh->set_associated_index(id);
				vh->set_cp(cp);
				vh->set_cseq_index(cps_id);
				cdt_vertex_handle[id] = vh;
				nver++;
			}
			if (!list.at(0).compare("e"))
			{
				Vpair e(list[1].toUInt(), list[2].toUInt());
				edge_constraints.insert(e);
			}
			line = in.readLine();
		}
		file.close();
	}
	else
		return;

	for (auto it = edge_constraints.begin(); it != edge_constraints.end(); it++)
	{
		domain_mesh.insert_constraint(cdt_vertex_handle.at(it->first), cdt_vertex_handle.at(it->second));
	}

	for (auto iv = domain_mesh.vertices_begin(); iv != domain_mesh.vertices_end(); iv++)
	{
		Vpair cpid = iv->get_cseq_index();
		CPs_new[cpid.first][cpid.second].id = iv->get_associated_index();
	}
	collinear_knots_lines.clear();
	for (int i = 0;i < CPs_new.size(); i++)
	{
		vector<PointTriple> cp;
		for (int j = 0; j < CPs_new[i].size(); j++)
		{
			PointTriple p;
			p.index = CPs_new[i][j].id;
			p.is_fixed = true;
			cp.push_back(p);			
		}
		if (CPs_new[i][0].is_cycle)
		{
			cp.push_back(*cp.begin());
		}
		collinear_knots_lines.push_back(cp);
	}

	nDeg = 2;
	nknotinsert_collinear = 0;
	dataMaps.clear();
	for (auto vit = domain_mesh.vertices_begin(); vit != domain_mesh.vertices_end(); vit++)
	{
		KnotData data;
		data.pt = Point_2(vit->point().x(), vit->point().y());
		data.index = vit->get_associated_index();
		data.flag.bBoundary = vit->get_cp().is_on_bound;
		bool on_corner = (abs(vit->point().x() +0.5) < 1e-4 && abs(vit->point().y() +0.5) < 1e-4) ||
			(abs(vit->point().x() +0.5) < 1e-4 && abs(vit->point().y() - 0.5) < 1e-4) ||
			(abs(vit->point().x() - 0.5) < 1e-4 && abs(vit->point().y() +0.5) < 1e-4) ||
			(abs(vit->point().x() - 0.5) < 1e-4 && abs(vit->point().y() - 0.5) < 1e-4);
		data.flag.bCorner = on_corner;
		dataMaps[vit->get_associated_index()] = data;
	} 

	rupture_edge_basis_config.clear();
	//inte set of basis are on edge
	for (int i = 0; i < collinear_knots_lines.size(); i++)
	{
		for (int j = 0; j < collinear_knots_lines[i].size();j++)
		{
			unsigned int vnow = collinear_knots_lines[i][j].index;
			unsigned int vnet = collinear_knots_lines[i][(j+1)%collinear_knots_lines[i].size()].index;
			unsigned int vpre = collinear_knots_lines[i][(j - 1+ collinear_knots_lines[i].size()) % collinear_knots_lines[i].size()].index;

			RuptureKnotConfig rcd20;
			rcd20.inte = { vnow,vnow };
			rcd20.constrain = { vpre,vnet };
			rupture_edge_basis_config.push_back(rcd20);

			RuptureKnotConfig rcd2;
			rcd2.inte = { vnow,vnet };
			rcd2.constrain = { vnow,vnet };
			rupture_edge_basis_config.push_back(rcd2);
		}
	}

	feature_bound_corner.clear();
	correspondence.clear();
	cornerIds.clear();
	CKnotsTriangulation *lt = new CKnotsTriangulation(&dataMaps, nDeg);
	lt->set_correspondence(&correspondence);
	lt->set_corner_indices(&cornerIds);
	lt->set_collinear_knots_config(collinear_knots_lines, feature_bound_corner,
		rupture_edge_basis_config, nknotinsert_collinear);
	vector<vector<TConfig2>> alltconfig;
	lt->link_triangulation_procedure(domain_mesh, alltconfig);//knots_config has corner multi-p
	//std::cout << __FUNCTION__ << ": " << "link triangulation procedure finished -LTP config size:" << alltconfig[nDeg].size() << std::endl;

	bSplineBasis.basisConfigs.clear();
	for (int i = 0; i < alltconfig.size(); i++)
		restore_index(correspondence, alltconfig[i]);
	generate_basis_configuraitons(alltconfig[nDeg], bSplineBasis);

	std::cout << "1done!!!\n";
	cmInfo.vertices.clear();
	cmInfo.edges.clear();
	cmInfo.faces.clear();
	vector<vector<TConfig2>> temp = alltconfig;
	compute_centroid_triangulation(temp, alltconfig, dataMaps, cmInfo);

	int realCpNum = 0;
	for (unsigned i = 0; i < bSplineBasis.basisConfigs.size(); i++)
	{
		vector<unsigned int> temp_ = bSplineBasis.basisConfigs[i].config;
		if (temp_[0] == 6 && temp_[1] == 6)
			int sss = 1;
		for (unsigned k = 0; k < cmInfo.vertices.size(); k++)
		{
			vector<unsigned int> scr_;
			scr_ = cmInfo.vertices[k].interior;
			sort(scr_.begin(), scr_.end());
			sort(temp_.begin(), temp_.end());
			if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
			{
				double x =( dataMaps.at(temp_[0]).pt.x() + dataMaps.at(temp_[1]).pt.x())/2;
				double y = (dataMaps.at(temp_[0]).pt.y() + dataMaps.at(temp_[1]).pt.y()) / 2;
				RGBPoint cp1=cdt_vertex_handle.at(temp_[0])->get_cp();
				RGBPoint cp2 = cdt_vertex_handle.at(temp_[1])->get_cp();

				cmInfo.vertices[k].do_display = true;
				cmInfo.vertices[k].flag = 1;
				cmInfo.vertices[k].vertex.x = x;
				cmInfo.vertices[k].vertex.y = y;
				cmInfo.vertices[k].vertex.r = 0.5 * (cp1.r + cp2.r);
				cmInfo.vertices[k].vertex.g = 0.5 * (cp1.g + cp2.g);
				cmInfo.vertices[k].vertex.b = 0.5 * (cp1.b + cp2.b);
				cmInfo.vertices[k].is_on_feature = false;
				realCpNum++;
				break;
			}
		}
	}
	for (unsigned k = 0; k < cmInfo.vertices.size(); k++) {
		vector<unsigned int> scr_;
		scr_ = cmInfo.vertices[k].interior;
		/*if (scr_.size() == 1) {
			if (scr_[0] == 0 || scr_[0] == 3 || scr_[0] == 6 || scr_[0] == 9) {
				cmInfo.vertices[k].do_display = true;
				cmInfo.vertices[k].flag = 1;
				cmInfo.vertices[k].vertex.x = dataMaps.at(scr_[0]).pt.x();
				cmInfo.vertices[k].vertex.y = dataMaps.at(scr_[0]).pt.y();
				cmInfo.vertices[k].is_on_feature = false;
			}
		}
		else */if (scr_.size() == 2) {
			double x = (dataMaps.at(scr_[0]).pt.x() + dataMaps.at(scr_[1]).pt.x()) / 2;
			double y = (dataMaps.at(scr_[0]).pt.y() + dataMaps.at(scr_[1]).pt.y()) / 2;
			if (scr_[0] == 0 || scr_[0] == 3 || scr_[0] == 6 || scr_[0] == 9 ||
				scr_[1] == 0 || scr_[1] == 3 || scr_[1] == 6 || scr_[1] == 9) {
				cmInfo.vertices[k].do_display = true;
				cmInfo.vertices[k].flag = 1;
				cmInfo.vertices[k].vertex.x = x;
				cmInfo.vertices[k].vertex.y = y;
				cmInfo.vertices[k].is_on_feature = false;
			}
		}
	}

	std::cout << "2done!!!\n";
	extract_real_control_mesh(cmInfo, realCm);
	::control_mesh_output(realCm, "real.obj");
	std::cout << "3done!!!\n";
}

//test initial value
void Image_Approximation::initial_control_points()
{
	// build old domain KD-tree
	int				k = 1;									// number of nearest neighbors
	int				dim = 3;								// dimension
	double			eps = 0;								// error bound
	int				maxPts = originalMesh->size_of_vertices();// maximum number of data points
															  //compute domain
	for (int i = 0; i < bSplineBasis.basisConfigs.size(); i++)
	{
		double cenx = 0.0, ceny = 0.0;
		for (int j = 0; j < bSplineBasis.basisConfigs[i].config.size(); j++)
		{
			cenx += dataMaps[bSplineBasis.basisConfigs[i].config[j]].pt.x();
			ceny += dataMaps[bSplineBasis.basisConfigs[i].config[j]].pt.y();
		}

		cenx = cenx / double(bSplineBasis.basisConfigs[i].config.size());
		ceny = ceny / double(bSplineBasis.basisConfigs[i].config.size());

		//search
		ANNpoint			queryPt_temp;				// query point
		ANNidxArray			nnIdx_temp;					// near neighbor indices
		ANNdistArray		dists_temp;					// near neighbor distances

		queryPt_temp = annAllocPt(dim);					// allocate query point
		nnIdx_temp = new ANNidx[k];						// allocate near neigh indices
		dists_temp = new ANNdist[k];					// allocate near neighbor dists

		queryPt_temp[0] = cenx;
		queryPt_temp[1] = ceny;

		nnIdx_temp[0] = 0;
		kdTree_newdomain->annkSearch(						// search
			queryPt_temp,								// query point
			k,											// number of near neighbors
			nnIdx_temp,									// nearest neighbors (returned)
			dists_temp,									// distance (returned)
			eps);										// error bound

		Mesh::Vertex_iterator v = originalMesh->get_vertex_iterator(nnIdx_temp[0]);
		Point_3 color_value = v->vertex_pixel_color();

		delete[] nnIdx_temp;							// clean things up
		delete[] dists_temp;

		double x = cenx * 2.0*imagemesh_dx - imagemesh_dx;
		double y = ceny * 2.0*imagemesh_dy - imagemesh_dy;
		bSplineBasis.basisConfigs[i].initial_points.push_back(x);
		bSplineBasis.basisConfigs[i].initial_points.push_back(y);
		bSplineBasis.basisConfigs[i].initial_points.push_back(color_value.x());
		bSplineBasis.basisConfigs[i].initial_points.push_back(color_value.y());
		bSplineBasis.basisConfigs[i].initial_points.push_back(color_value.z());
	}

}

//fitting result out
void Image_Approximation::archive_results()
{
	std::cout << "time of implement including several steps as follow: \n";
	std::cout << "time of feature process   : " << statisticsInfos.infos[0].t_feature_process << "\n";
	std::cout << "time of parameterization  : " << statisticsInfos.infos[0].t_para << "\n";
	std::cout << "time of knots generation  : " << statisticsInfos.infos[0].t_knots_generation << "\n";
	std::cout << "time of LTP process       : " << statisticsInfos.infos[0].t_LTP << "\n";
	std::cout << "time of basis config      : " << statisticsInfos.infos[0].t_basis_config << "\n";
	std::cout << "time of basis value       : " << statisticsInfos.infos[0].t_basis_value << "\n";
	std::cout << "time of data sampling     : " << statisticsInfos.infos[0].t_sample_data << "\n";
	std::cout << "time of optimization      : " << statisticsInfos.infos[0].t_solve_equation << "\n";
	statisticsInfos.infos[0].t_timeall =
		statisticsInfos.infos[0].t_feature_process +
		statisticsInfos.infos[0].t_para +
		statisticsInfos.infos[0].t_knots_generation +
		statisticsInfos.infos[0].t_LTP +
		statisticsInfos.infos[0].t_basis_config +
		statisticsInfos.infos[0].t_basis_value +
		statisticsInfos.infos[0].t_sample_data +
		statisticsInfos.infos[0].t_solve_equation;
	std::cout << "total time		 		: " << statisticsInfos.infos[0].t_timeall << "\n";

	statisticsInfos.infos[0].num_knots = knot_mesh->size_of_vertices();
	statisticsInfos.infos[0].num_knots_face = knot_mesh->size_of_facets();
	statisticsInfos.infos[0].size_representation =
		(statisticsInfos.infos[0].num_controlp*20.0 +
		knot_mesh->size_of_vertices()*8.0 + knot_mesh->size_of_facets()*6.0)/1000.0;
	statisticsInfos.infos[0].compression_ratio =
		double(input_image->width()*input_image->height()) / double(knot_mesh->size_of_vertices());

	std::cout << "other information as follow: \n";
	std::cout << "number of control points  : " << statisticsInfos.infos[0].num_controlp << "\n";
	std::cout << "number of knots           : " << statisticsInfos.infos[0].num_knots << "\n";
	std::cout << "number of knot-face       : " << statisticsInfos.infos[0].num_knots_face << "\n";
	std::cout << "size of representation    : " << statisticsInfos.infos[0].size_representation << "\n";
	std::cout << "compression ratio         : " << statisticsInfos.infos[0].compression_ratio << "\n";
	std::cout << "PSNR                      : " << statisticsInfos.infos[0].psnr << "\n";
	std::cout << "RMSE						: " << statisticsInfos.infos[0].RMSE << "\n";
	std::cout << "SNR						: " << statisticsInfos.infos[0].SNR << "\n";
	std::cout << "mean errors               : " << statisticsInfos.infos[0].mean_error << "\n";
	std::cout << "mean errors without feature: " << statisticsInfos.infos[0].mean_err_nofeature << "\n";

	//write to file
	QString name = out_file_name;
	name.append("/statisticsInfos.txt");
	reconstruction_info_output(name);

	//knots
	name = out_file_name;
	name.append("/knots_mesh.obj");
	mesh_output(knot_mesh, name, false,false);

	//fitted mesh -original topology
	name = out_file_name;
	name.append("/fitted_mesh_3d.obj");
	mesh_output(fittedMesh, name, true, true);
	name = out_file_name;
	name.append("/fitted_mesh_2d.obj");
	mesh_output(fittedMesh, name, true, false);
	name = out_file_name;
	name.append("/fitted_mesh_flag.txt");
	mesh_output(fittedMesh, name);

	//fitted image
	name = out_file_name;
	name.append("/fitted_image.jpg");
	fitted_image->save(name);

	//control mesh
	name = out_file_name;
	name.append("/control_mesh");
	control_mesh_output(name);

	//for edit
	char rootPath[MAX_PATH];
	GetModuleFileName(NULL, rootPath, MAX_PATH);
	QString saveEditPath(rootPath);
	QStringList dataColumns = saveEditPath.split("\\");
	dataColumns.pop_back();
	dataColumns.pop_back();
	saveEditPath = dataColumns.join('/');
	name = out_file_name;
	generate_output_directory(name, saveEditPath,1);
	name.append("-out.edit");
	saveEditPath = saveEditPath+"/" + name;
	std::cout << "path: " << saveEditPath.toStdString() << std::endl;
	write_editdata(saveEditPath);

	name = out_file_name;
	name.append("/midprocess_coplanar.txt");
	coplanar_info_output(bSplineBasis, dataMaps, name);

	name = out_file_name;
	name.append("/midprocess_correspodence.txt");
	QFile file1(name);
	if (file1.open(QIODevice::WriteOnly))
	{
		QTextStream out(&file1);
		for (auto it = correspondence.begin();it != correspondence.end();it++)
			out << it->first << " " <<it->second <<";\r\n";

	}

	name = out_file_name;
	name.append("/midprocess_alltconfigs_deg.txt");
	QFile file2(name);
	if (file2.open(QIODevice::WriteOnly))
	{
		QTextStream out(&file2);
		for (auto it = alltconfigs[nDeg].begin(); it != alltconfigs[nDeg].end(); it++)
		{
			for (int i = 0; i < it->tconfig.size(); i++)
			{
				out << (*it).tconfig[i] << " ";
			}
			out << ";\r\n";
		}
	}

	name = out_file_name;
	name.append("/midprocess_bSplineBasis.txt");
	QFile file3(name);
	if (file3.open(QIODevice::WriteOnly))
	{
		QTextStream out(&file3);
		for (auto it = bSplineBasis.basisConfigs.begin(); it != bSplineBasis.basisConfigs.end(); it++)
		{
			out << "config center: ";
			for (int i = 0; i < it->config.size(); i++)
			{
				out << (it->config)[i] << " ";
			}
			out << "--support size:" << it->supports.size();
			out << ";\r\n";
			out << "config all: ";
			for (int i = 0; i < it->tconfigs.size(); i++)
			{
				out << "num"<<i<<": ";
				for (int j = 0;j<(it->tconfigs)[i].tconfig.size();j++)
				{
					out << (it->tconfigs)[i].tconfig[j] << " ";
				}
				out << "; ";
			}
			out << ";\r\n";
		}
	}

}

QString Image_Approximation::generate_output_dir()
{
	QString dir = out_file_name;

	if (triType == DELAUNAY)
	{
		dir.append("_DEL");
	}
	else
	{
		if (optAlg == LOP)
			dir.append("Lop");
		else if (optAlg == LOOKAHEAD)
			dir.append("LA");
		else
			dir.append("SA");

		dir.append("_DDT");
	}

	dir.append("_");

	QDate date = QDate::currentDate();
	dir.append(QString::number(date.year()));
	dir.append("-");
	dir.append(QString::number(date.month()));
	dir.append("-");
	dir.append(QString::number(date.day()));

	dir.append("_");

	QTime time = QTime::currentTime();
	dir.append(QString::number(time.hour()));
	dir.append("-");
	dir.append(QString::number(time.minute()));
	dir.append("-");
	dir.append(QString::number(time.second()));

	return dir;
}

void Image_Approximation::control_mesh_output(QString fileName)
{
	QString name = fileName;
	name.append(".obj");
	::control_mesh_output(cmInfo, name);
	name = fileName;

	CMInfo realCm;
	name = fileName;
	name.append("_real.obj");
	extract_real_control_mesh(cmInfo, realCm);
	::control_mesh_output(realCm, name);
	name = fileName;
	name.append("_ct.obj");
	centroid_triangulation_output(realCm, fileName);
}

void Image_Approximation::reconstruction_info_output(QString fileName)
{
	QFile file(fileName);
	if (file.open(QIODevice::Append))
	{
		QTextStream out(&file);

		out << "/////////////////////////////option information \r\n";
		out << "Iterate number currently: " << Nite_now << "\r\n";
		out << "Triangulation type: " << triType << "\r\n";
		out << "Triangulation optimize algorithm: " << optAlg << "\r\n";
		out << "Fitting degree: " << nDeg << "\r\n";
		out << "Error of feature polyline: " << simply_error << "\r\n";
		out << "Number of inserted knots on feature: " << 3 << "\r\n";
		out << "Number of knots: " << nknot<< "\r\n";
		
		out << "Fairing weights: " <<pos_fair_firder_wei<<"-"<< pos_fair_sedder_wei << "\r\n";
		out << "Number of Real control points: " << statisticsInfos.infos[0].num_controlp << "\r\n";
		out << "Number of Aux control points:: " << cmInfo.vertices.size() - statisticsInfos.infos[0].num_controlp << "\r\n";
		out << "Number of all control points:: " << cmInfo.vertices.size() << "\r\n";

		out << "/////////////////////////////time information \r\n";
		out << "ParameterTime: " << statisticsInfos.infos[0].t_para << "\r\n";
		out << "MakingKnotTime: " << statisticsInfos.infos[0].t_knots_generation << "\r\n";
		out << "LinkTriangulationTime: " << statisticsInfos.infos[0].t_LTP << "\r\n";
		out << "BasisTime: " << statisticsInfos.infos[0].t_basis_value << "\r\n";
		out << "VerifyTime: " << statisticsInfos.infos[0].t_verify << "\r\n";
		out << "FittingTime: " << statisticsInfos.infos[0].t_solve_equation << "\r\n";
		out << "FittingfairingTime: " << statisticsInfos.infos[0].t_fairing << "\r\n";
		out << "AllTime: " << statisticsInfos.infos[0].t_timeall<< "\r\n";

		file.close();
	}
}

//////////////////////////////////////////////////////////////////////////
