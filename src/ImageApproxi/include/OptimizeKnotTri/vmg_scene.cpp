
#include "OptimizeKnotTri/vmg_scene.h"

void OptiKnotTri::clear()
{
	if (_image)
	{
		delete _image;
		_image = NULL;
	}

	delete_mesh();

	if (_approxImage)
	{
		delete _approxImage;
		_approxImage = NULL;
	}
}

// io
void OptiKnotTri::set_image(const unsigned char* data, const int& w, const int& h, const int& c)
{
	clear();

	if (!_image)
		_image = new ImageDomain;

	_image->set_image(data, w, h, c);
}

void OptiKnotTri::load_mesh(const char* fileName,int nfixedp)
{
	if (_mesh == NULL)
		new_mesh();
	printf("%s: new mesh\n", __FUNCTION__);
	if (!OpenMesh::IO::read_mesh(*_mesh, std::string(fileName)))
	{
		printf("%s: failed\n", __FUNCTION__);
		delete_mesh();
	}

	_fixedVerticesNb = nfixedp;

}

void OptiKnotTri::load_fixed_edge(const char* fileName)
{
	if (_mesh == NULL)
		return;

	std::ifstream file(fileName);
	if (!file)
		return;

	_fixedEdges.clear();
	int number_seg;
	while (file >> number_seg)
	{
		for (int i = 0; i < number_seg; i++)
		{
			int seg_s, seg_e;
			file >> seg_s >> seg_e;
			_fixedEdges.insert(std::multimap<int, int>::value_type(seg_s, seg_e));
		}
	}
	file.close();
}

bool OptiKnotTri::is_fixed_edge(const TriMesh::EdgeHandle& eh)
{
	TriMesh::HalfedgeHandle ehe = _mesh->halfedge_handle(eh, 0);

	int fromv = _mesh->from_vertex_handle(ehe).idx();
	int tov = _mesh->to_vertex_handle(ehe).idx();

	std::multimap<int, int>::iterator it = _fixedEdges.lower_bound(fromv), end = _fixedEdges.upper_bound(fromv);
	for (; it != end; it++)
	{
		if (it->second == tov)
			return true;
	}

	it = _fixedEdges.lower_bound(tov);
	end = _fixedEdges.upper_bound(tov);
	for (; it != end; it++)
	{
		if (it->second == fromv)
			return true;
	}

	return false;
}

// assign
void OptiKnotTri::assign_pixels()
{
	if (_mesh == NULL || _image == NULL)
	{
		printf("%s: null\n", __FUNCTION__);
		return;
	}

	int fnb = (int)_mesh->n_faces();

#pragma omp parallel for
	for (int f = 0; f < fnb; f++)
	{
		assign_pixel(f);
	}
}

void OptiKnotTri::assign_pixel(const int& f)
{
	double *triangle = mesh_face_triangle(_mesh, f);

	Rasterizer ras(_image);
	ras.rasterize(triangle, 3, _mesh->property(_facePixelSet, _mesh->face_handle(f)));

	delete[] triangle;
}

// polynomials
void OptiKnotTri::compute_polynomials()
{
	if (_mesh == NULL || _image == NULL)
	{
		printf("%s: null\n", __FUNCTION__);
		return;
	}

	int fnb = (int)_mesh->n_faces();
	
#pragma omp parallel for
	for (int f = 0; f < fnb; f++)
	{
		TriMesh::FaceHandle fh = _mesh->face_handle(f);
		_mesh->property(_facePolynomial, fh) = Polynomial(_image, _degree);
		compute_polynomial(f);
	}
}

void OptiKnotTri::compute_polynomial(const int& f)
{
	TriMesh::FaceHandle fh = _mesh->face_handle(f);
	_mesh->property(_facePolynomial, fh).compute_factors(&_mesh->property(_facePixelSet, fh));
}

// energy
double OptiKnotTri::compute_energy()
{
	if (_mesh == NULL || _image == NULL)
	{
		printf("%s: null\n", __FUNCTION__);
		return 0.0;
	}

	int fnb = (int)_mesh->n_faces();

	double total = 0.0;
#pragma omp parallel for reduction(+:total)
	for (int f = 0; f < fnb; f++)
	{
		double e = compute_energy(f);
		total += e;
	}

	return total; 
}

double OptiKnotTri::compute_energy(const int& f)
{
	TriMesh::FaceHandle fh = _mesh->face_handle(f);
	double e = _mesh->property(_facePolynomial, fh).compute_energy(&_mesh->property(_facePixelSet, fh));
	_mesh->property(_faceEnergy, fh) = e;
	return e;
}

// init
bool OptiKnotTri::init_greedy_mesh(const int& vnb,const char*file)
{
	if (_mesh == NULL)
		return false;

	printf("%s: begin\n", __FUNCTION__);
	Timer tempTimer;
	tempTimer.start();

	insert_vertices(vnb - _fixedVerticesNb);

	//loose mesh
	/*int iteloose = 1;
	for (int k = 0;k<iteloose;k++)
	{
		for (auto vit = _mesh->vertices_begin(); vit != _mesh->vertices_end(); vit++)
		{
			if (vit->idx() < _fixedVerticesNb)
			{
				continue;
			}
			TriMesh::Point cog;
			double valence;
			cog[0] = cog[1] = cog[2] = valence = 0.0;
			for (auto vv_it = _mesh->vv_iter(*vit); vv_it.is_valid(); ++vv_it)
			{
				cog += _mesh->point(*vv_it);
				++valence;
			}
			_mesh->point(vit) = cog / valence;
		}
	}*/
	
	tempTimer.stop();
	printf("%s: takes time = %f seconds\n", __FUNCTION__, tempTimer.get_elapsed_time());

	printf("%s: end\n", __FUNCTION__);

	//write to file
	if (file == NULL)
	{
		return 1;
	}
	std::ofstream fout_du;
	fout_du.open(file);
	if (fout_du.is_open())
	{
		for (int i = 0; i < _mesh->n_vertices(); i++)
		{
			for (auto vit = _mesh->vertices_begin(); vit != _mesh->vertices_end(); vit++)
			{
				if (vit->idx() == i)
				{
					if (1)
					{
						double x = _mesh->point(*vit).data()[0];
						double y = _mesh->point(*vit).data()[1];
						double z = _mesh->point(*vit).data()[2];
						fout_du << "v" << " " << x << " " << y << " " << z << "\n";
					}
					else
					{
						double x = _mesh->point(*vit).data()[0] + _image->dx();
						double y = (_mesh->point(*vit).data()[1] + _image->dy()) / (2 * _image->dy());
						double z = _mesh->point(*vit).data()[2];
						fout_du << "v" << " " << x << " " << y << " " << z << "\n";
					}								
					break;
				}
			}
		}
		for (auto fit = _mesh->faces_begin(); fit != _mesh->faces_end(); fit++)
		{
			fout_du << "f";
			for (auto fv_it = _mesh->fv_iter(*fit); fv_it.is_valid(); ++fv_it)
			{
				//obj index from 1
				int index1 = fv_it->idx() + 1;
				fout_du << " " << index1;
			}
			fout_du << "\n";
		}
	}
	fout_du.close();

	return true;
}

void OptiKnotTri::insert_vertices(const int& vnb)
{
	if (_mesh == NULL)
		return;

	assign_pixels();
	compute_polynomials();
	compute_energy();

	std::multimap<double, int, std::greater<double>> maxMap;
	int fnb = (int)_mesh->n_faces();
	for (int f = 0; f < fnb; f++)
	{
		TriMesh::FaceHandle fh = _mesh->face_handle(f);
		TriMesh::HalfedgeHandle heh = _mesh->halfedge_handle(fh);
		double farea = _mesh->calc_sector_area(heh);
		//decrease the influence of area
		double ln_area = log10(1.0/farea);
		maxMap.insert(std::multimap<double, int>::value_type(_mesh->property(_faceEnergy, _mesh->face_handle(f))*ln_area, f));
	}

	int prevvnb = (int)_mesh->n_vertices();
	while ((int)_mesh->n_vertices() - prevvnb < vnb && !maxMap.empty())
	{
		int prevfnb = (int)_mesh->n_faces();

		int fid = (*maxMap.begin()).second;
		/*if ((int)_mesh->n_vertices() % 100 == 0)
		{
			printf("%s: vnb = %d, max error face index = %d\n", __FUNCTION__, (int)_mesh->n_vertices(), fid);
		}*/

		TriMesh::FaceHandle fh = _mesh->face_handle(fid);
		TriMesh::HalfedgeHandle heh = _mesh->halfedge_handle(fh);
		TriMesh::VertexHandle nvh;
#if 0
		bool coline = false;
		TriMesh::HalfedgeHandle temphe = heh;
		do
		{
			double angle = _mesh->calc_sector_angle(temphe);
			if (angle > MAX_ANGLE)
			{
				coline = true;
				break;
			}
			temphe = _mesh->next_halfedge_handle(temphe);
		} while (temphe != heh);

		if (coline && !is_fixed_edge(_mesh->edge_handle(_mesh->prev_halfedge_handle(temphe))))
		{
			/*TriMesh::Point pb1, pb2;
			if (_mesh->is_boundary(_mesh->opposite_halfedge_handle(heh)))
			{
				pb1 = _mesh->point(_mesh->to_vertex_handle(heh));
				pb2 = _mesh->point(_mesh->from_vertex_handle(heh));
				nvh = _mesh->split(_mesh->edge_handle(heh), (pb1 + pb2) * 0.5);
			}
			else if (_mesh->is_boundary(_mesh->opposite_halfedge_handle(_mesh->next_halfedge_handle(heh))))
			{
				pb1 = _mesh->point(_mesh->to_vertex_handle(_mesh->next_halfedge_handle(heh)));
				pb2 = _mesh->point(_mesh->from_vertex_handle(_mesh->next_halfedge_handle(heh)));
				nvh = _mesh->split(_mesh->edge_handle(_mesh->next_halfedge_handle(heh)), (pb1 + pb2) * 0.5);

			}
			else if (_mesh->is_boundary(_mesh->opposite_halfedge_handle(_mesh->prev_halfedge_handle(heh))))
			{
				pb1 = _mesh->point(_mesh->to_vertex_handle(_mesh->prev_halfedge_handle(heh)));
				pb2 = _mesh->point(_mesh->from_vertex_handle(_mesh->prev_halfedge_handle(heh)));
				nvh = _mesh->split(_mesh->edge_handle(_mesh->prev_halfedge_handle(heh)), (pb1 + pb2) * 0.5);
			}*/
			/*TriMesh::Point p1 = _mesh->point(_mesh->to_vertex_handle(heh));
			TriMesh::Point p2 = _mesh->point(_mesh->to_vertex_handle(_mesh->next_halfedge_handle(heh)));
			TriMesh::Point p3 = _mesh->point(_mesh->from_vertex_handle(heh));
			TriMesh::Point pt;
			bool is_bound1 = false, is_bound2 = false, is_bound3 = false;
			if (abs(p1.data()[0] - 0.0) < 1e-8 || abs(p1.data()[0] - 1.0) < 1e-8 || abs(p1.data()[1] - 0.0) < 1e-8 || abs(p1.data()[1] - 1.0) < 1e-8)
			{
				is_bound1 = true;
			}
			if (abs(p2.data()[0] - 0.0) < 1e-8 || abs(p2.data()[0] - 1.0) < 1e-8 || abs(p2.data()[1] - 0.0) < 1e-8 || abs(p2.data()[1] - 1.0) < 1e-8)
			{
				is_bound2 = true;
			}
			if (abs(p3.data()[0] - 0.0) < 1e-8 || abs(p3.data()[0] - 1.0) < 1e-8 || abs(p3.data()[1] - 0.0) < 1e-8 || abs(p3.data()[1] - 1.0) < 1e-8)
			{
				is_bound3 = true;
			}
			if (is_bound1 && is_bound2)
			{
				nvh = _mesh->split(_mesh->edge_handle(_mesh->next_halfedge_handle(heh)), (p1 + p2) * 0.5);
			}
			else if (is_bound2 && is_bound3)
			{
				nvh = _mesh->split(_mesh->edge_handle(_mesh->prev_halfedge_handle(heh)), (p2 + p3) * 0.5);
			}
			else if (is_bound3 && is_bound1)
			{
				nvh = _mesh->split(_mesh->edge_handle(heh), (p1 + p3) * 0.5);
			}
			else*/
			{
				temphe = _mesh->prev_halfedge_handle(temphe);
				TriMesh::Point p1 = _mesh->point(_mesh->from_vertex_handle(temphe));
				TriMesh::Point p2 = _mesh->point(_mesh->to_vertex_handle(temphe));
				nvh = _mesh->split(_mesh->edge_handle(temphe), (p1 + p2) * 0.5);
			}	
		}
		else
#endif
		{
			TriMesh::Point p1 = _mesh->point(_mesh->to_vertex_handle(heh));
			TriMesh::Point p2 = _mesh->point(_mesh->to_vertex_handle(_mesh->next_halfedge_handle(heh)));
			TriMesh::Point p3 = _mesh->point(_mesh->from_vertex_handle(heh));

			double farea = _mesh->calc_sector_area(heh);
			
			double lamda1 = double(rand()) / double(RAND_MAX);
			double lamda2 = double(rand()) / double(RAND_MAX) * (1.0 - lamda1);
			double lamda3 = (1.0 - lamda1 - lamda2);
			//TriMesh::Point p = (p1 + 2 * p2 + p3) * 0.25 * lamda1 + (p2 + 2 * p3 + p1) * 0.25 * lamda2 + (p3 + 2 * p1 + p2) * 0.25 * lamda3;

			TriMesh::Point centroid = (p1 + p2 + p3) / 3.0;
			nvh = _mesh->split(fh, centroid);
		}
		
		for (int f = prevfnb; f < (int)_mesh->n_faces(); f++)
		{
			_mesh->property(_facePolynomial, _mesh->face_handle(f)) = Polynomial(_image, _degree);
		}

		for (TriMesh::VertexFaceIter vfit = _mesh->vf_begin(nvh); vfit != _mesh->vf_end(nvh); vfit++)
		{
			int f = (*vfit).idx();
			assign_pixel(f);
			compute_polynomial(f);
			compute_energy(f);
		}
		
		optimize_connectivity();

		maxMap.clear();
		for (int f = 0; f < (int)_mesh->n_faces(); f++)
		{			
			TriMesh::FaceHandle fh = _mesh->face_handle(f);
			TriMesh::HalfedgeHandle heh = _mesh->halfedge_handle(fh);
			double farea = _mesh->calc_sector_area(heh);
			double ln_area = (log10(1.0/farea)) / 2.0 ;
			if (farea < 1e-4)
			{
				ln_area = 0.1;
			}
			double e = _mesh->property(_faceEnergy, _mesh->face_handle(f));
			maxMap.insert(std::multimap<double, int>::value_type(e*ln_area, f));
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////

bool OptiKnotTri::optimize_mesh(const int& iterations,const char*file, bool is_update_position)
{
	if (_mesh == NULL || _image == NULL)
	{
		printf("%s: null\n", __FUNCTION__);
		return false;
	}

	if((int)_mesh->n_vertices() < 5)
	{
		printf("%s: vnb less than 5\n", __FUNCTION__);
		return false;
	}

	printf("%s: begin\n", __FUNCTION__);

	Timer tempTimer;
	tempTimer.start();

	//test
	//std::ofstream fout_out;
	//fout_out.open("Results/err.txt");
	//if (fout_out.is_open())
	{
		for (int i = 0; i < iterations; i++)
		{
			//printf("%s: -------------------------- %d ---------------------------\n", __FUNCTION__, i);
			double temp = std::pow(global_base, double(i) / double(iterations - i));
			global_alpha = global_beta * temp;

			assign_pixels();
			compute_polynomials();
			double error_ = compute_energy();

			//fout_out <<  error_ << "\n";

			if (is_update_position)
			{
				optimize_vertices();
			}
			process_boundary();

			optimize_connectivity();
		}
	}
	//fout_out.close();

	tempTimer.stop();
	printf("%s: takes time = %f seconds\n", __FUNCTION__, tempTimer.get_elapsed_time());

	//write to file
	std::ofstream fout_du;
	fout_du.open(file);
	if (fout_du.is_open())
	{
		for (int i = 0; i < _mesh->n_vertices(); i++)
		{
			for (auto vit = _mesh->vertices_begin(); vit != _mesh->vertices_end(); vit++)
			{
				if (vit->idx() == i)
				{
					double x = _mesh->point(*vit).data()[0];
					double y = _mesh->point(*vit).data()[1];
					double z = _mesh->point(*vit).data()[2];
					fout_du << "v" << " " << x << " " << y << " " << z << "\n";
					break;
				}
			}
		}
		for (auto fit = _mesh->faces_begin(); fit != _mesh->faces_end(); fit++)
		{
			fout_du << "f";
			for (auto fv_it = _mesh->fv_iter(*fit); fv_it.is_valid(); ++fv_it)
			{
				//obj index from 1
				int index1 = fv_it->idx() + 1;
				fout_du << " " << index1;
			}
			fout_du << "\n";
		}
	}
	else
		std::cout << "open failed" << std::endl;
	fout_du.close();

	printf("%s: end\n", __FUNCTION__);
	return true;
}

void OptiKnotTri::optimize_vertices()
{
	compute_vertices_batch();
	update_positions();
}

void OptiKnotTri::compute_vertices_gradient()
{
	printf("%s: begin\n", __FUNCTION__);
	int vnb = (int)_mesh->n_vertices();
	for (int v = 0; v < vnb; v++)
	{
		_mesh->property(_vertexGradient, _mesh->vertex_handle(v)) = TriMesh::Normal(0.0, 0.0, 0.0);
	}

	int enb = (int)_mesh->n_edges();

#pragma omp parallel for
	for (int e = 0; e < enb; e++)
	{
		TriMesh::EdgeHandle eh = _mesh->edge_handle(e);
		TriMesh::HalfedgeHandle ehe = _mesh->halfedge_handle(eh, 0);
		int f1 = _mesh->face_handle(ehe).idx(); 
		int f2 = _mesh->opposite_face_handle(ehe).idx();

		TriMesh::VertexHandle v1 = _mesh->from_vertex_handle(ehe);
		TriMesh::VertexHandle v2 = _mesh->to_vertex_handle(ehe);
		TriMesh::Point p1 = _mesh->point(v1);
		TriMesh::Point p2 = _mesh->point(v2);
		int id1 = v1.idx();
		int id2 = v2.idx();

		Polynomial *poly1 = (f1 == -1 ? NULL : &_mesh->property(_facePolynomial, _mesh->face_handle(f1)));
		Polynomial *poly2 = (f2 == -1 ? NULL : &_mesh->property(_facePolynomial, _mesh->face_handle(f2)));

		double result1[2] = { 0.0, 0.0 };
		double result2[2] = { 0.0, 0.0 };

		compute_vertices_gradient(&p1[0], &p2[0], poly1, poly2, result1, result2);

#pragma omp critical
		{
			_mesh->property(_vertexGradient, _mesh->vertex_handle(id1)) += TriMesh::Normal(result1[0], result1[1], 0.0);
			_mesh->property(_vertexGradient, _mesh->vertex_handle(id2)) += TriMesh::Normal(result2[0], result2[1], 0.0);
		}
	}

#pragma omp parallel for
	for (int v = 4; v < vnb; v++)
	{
		process_vertex_gradient(v);
	}
	printf("%s: end\n", __FUNCTION__);
}

void OptiKnotTri::compute_vertices_gradient(const double *p1, const double *p2, 
	const Polynomial *poly1, const Polynomial *poly2, double *result1, double *result2) const
{
	double pixWidth = _image->pixel_width();
	double dx = p1[0] - p2[0];
	double dy = p1[1] - p2[1];
	double edgeLength = std::sqrt(dx * dx + dy * dy);
	int segmentnb = int(edgeLength / pixWidth + 0.5);

	double step = edgeLength / segmentnb;
	for (int s = 1; s < segmentnb * 2; s += 2)
	{
		const double lamda = double(s) / double(segmentnb * 2);
		double p[2] = { p1[0] * (1.0 - lamda) + p2[0] * lamda, p1[1] * (1.0 - lamda) + p2[1] * lamda };
		int i = -1, j = -1;
		_image->pixel_location(p, i, j);

		double energy1 = 0.0, energy2 = 0.0;

		const unsigned char* pixColor = _image->pixel_color_pointer(i, j);
		for (int c = 0; c < _image->channel(); c++)
		{
			double tempEnergy1 = (poly1 == NULL ? 0 : pixColor[c] - poly1->evaluate(c, p));
			energy1 += tempEnergy1 * tempEnergy1;

			double tempEnergy2 = (poly2 == NULL ? 0 : pixColor[c] - poly2->evaluate(c, p));
			energy2 += tempEnergy2 * tempEnergy2;
		}

		double energyDiff = energy1 - energy2;

		double vec1[2] = { p2[1] - p[1], p[0] - p2[0] };
		double vec2[2] = { p1[1] - p[1], p[0] - p1[0] };

		result1[0] += energyDiff * vec1[0] * step;
		result1[1] += energyDiff * vec1[1] * step;
		result2[0] += -energyDiff * vec2[0] * step;
		result2[1] += -energyDiff * vec2[1] * step;
	}

	result1[0] /= edgeLength;
	result1[1] /= edgeLength;
	result2[0] /= edgeLength;
	result2[1] /= edgeLength;
}

void OptiKnotTri::compute_vertices_batch()
{
	//printf("%s: begin\n", __FUNCTION__);
	
	_verticesBatch.clear();

	int vnb = (int)_mesh->n_vertices();
	std::vector<int> verticesFlag;
	verticesFlag.resize(vnb, -1);
	const int batchMaxnb = 32;

	for (int v = 0; v < vnb; v++)
	{
		int batch[batchMaxnb] = { 0 };
		memset(batch, 0, batchMaxnb * sizeof(int));
		TriMesh::VertexHandle vh = _mesh->vertex_handle(v);
		for (TriMesh::VertexVertexIter vvit = _mesh->vv_begin(vh); vvit != _mesh->vv_end(vh); vvit++)
		{
			int vvid = (*vvit).idx();
			if (verticesFlag[vvid] > -1)
			{
				batch[verticesFlag[vvid]]++;
			}
		}

		int bid = 0;
		for (int b = 0; b < batchMaxnb; b++)
		{
			if (batch[b] == 0)
			{
				verticesFlag[v] = b;
				bid = b;
				break;
			}
		}

		while(bid >= (int)_verticesBatch.size())
		{
			_verticesBatch.push_back(std::vector<int>());
		}
		_verticesBatch[bid].push_back(v);
	}
	//printf("%s: end\n", __FUNCTION__);
}

void OptiKnotTri::update_positions()
{
	//printf("%s: begin\n", __FUNCTION__);

	int batchnb = (int)_verticesBatch.size();
	for (int b = 0; b < batchnb; b++)
	{
		std::vector<int>& batch = _verticesBatch[b];
		int batchvnb = (int)batch.size();
#pragma omp parallel for
		for (int i = 0; i < batchvnb; i++)
		{
			int v = batch[i];
			if (v < 4) // corner
				continue;

			update_position(v);
		}
	}
	//printf("%s: end\n", __FUNCTION__);
}

void OptiKnotTri::update_position(const int& v)
{
	if (v < _fixedVerticesNb)
		return;

	compute_vertex_gradient(v);

	TriMesh::VertexHandle vh = _mesh->vertex_handle(v);
	TriMesh::Normal direction = -_mesh->property(_vertexGradient, vh);
	double maxStep = compute_maximum_step(v, &direction[0]);
	if (maxStep < 1e-6)
	{
		return;
	}
	
	TriMesh::Point vp = _mesh->point(vh);

	std::map<TriMesh::FaceHandle, PixelSet>   newPixelSetMap;
	std::map<TriMesh::FaceHandle, Polynomial> newPolynomialMap;
	std::map<TriMesh::FaceHandle, double>     newEnergyMap;
	std::map<TriMesh::FaceHandle, double*>    newTriangleMap;
	double prevEnergy = 0.0;
	for (TriMesh::ConstVertexFaceIter cvfit = _mesh->cvf_begin(vh); cvfit != _mesh->cvf_end(vh); cvfit++)
	{
		prevEnergy += _mesh->property(_faceEnergy, *cvfit);

		newPixelSetMap[*cvfit] = PixelSet();
		newPolynomialMap[*cvfit] = Polynomial(_image, _degree);
		
		TriMesh::HalfedgeHandle fhe = _mesh->halfedge_handle(*cvfit);
		while (_mesh->to_vertex_handle(_mesh->next_halfedge_handle(fhe)) != vh)
			fhe = _mesh->next_halfedge_handle(fhe);

		double* triangle = new double[6];
		triangle[0] = vp[0];
		triangle[1] = vp[1];

		TriMesh::Point p = _mesh->point(_mesh->from_vertex_handle(fhe));
		triangle[2] = p[0];
		triangle[3] = p[1];
		p = _mesh->point(_mesh->to_vertex_handle(fhe));
		triangle[4] = p[0];
		triangle[5] = p[1];

		newTriangleMap[*cvfit] = triangle;
	}

	double pixWidth = _image->pixel_width();	
	double tempAlpha = global_alpha;
	Rasterizer ras(_image);
	int count = global_max_search_count;
	TriMesh::Point newPosition;
	while (count--)
	{
		newPosition = vp + maxStep * tempAlpha * direction;

		double newEnergy = 0.0;
		for (TriMesh::ConstVertexFaceIter cvfit = _mesh->cvf_begin(vh); cvfit != _mesh->cvf_end(vh); cvfit++)
		{
			double *triangle = newTriangleMap[*cvfit];
			triangle[0] = newPosition[0];
			triangle[1] = newPosition[1];

			ras.rasterize(triangle, 3, newPixelSetMap[*cvfit]);
			newPolynomialMap[*cvfit].compute_factors(&newPixelSetMap[*cvfit]);
			newEnergyMap[*cvfit] = newPolynomialMap[*cvfit].compute_energy(&newPixelSetMap[*cvfit]);
			newEnergy += newEnergyMap[*cvfit];
		}

		bool acceptOK = ((newPosition - vp).length() < pixWidth * 0.1 ? newEnergy <= prevEnergy : newEnergy < prevEnergy);

		if (acceptOK)
		{
			_mesh->set_point(vh, newPosition);
			
			for (TriMesh::ConstVertexFaceIter cvfit = _mesh->cvf_begin(vh); cvfit != _mesh->cvf_end(vh); cvfit++)
			{
				_mesh->property(_facePixelSet, *cvfit) = newPixelSetMap[*cvfit];
				_mesh->property(_facePolynomial, *cvfit) = newPolynomialMap[*cvfit];
				_mesh->property(_faceEnergy, *cvfit) = newEnergyMap[*cvfit];
			}

			break;
		}

		tempAlpha *= 0.2;
	}

	newPixelSetMap.clear();
	newPolynomialMap.clear();
	newEnergyMap.clear();
	for (std::map<TriMesh::FaceHandle, double*>::iterator mit = newTriangleMap.begin(); mit != newTriangleMap.end(); mit++)
	{
		double *triangle = mit->second;
		delete[] triangle;
		triangle = NULL;
		newTriangleMap[mit->first] = NULL;
	}
	newTriangleMap.clear();
}

void OptiKnotTri::compute_vertex_gradient(const int& v)
{
	TriMesh::VertexHandle vh = _mesh->vertex_handle(v);
	TriMesh::Point p1 = _mesh->point(vh);

	_mesh->property(_vertexGradient, vh) = TriMesh::Normal(0.0, 0.0, 0.0);

	for (TriMesh::VertexOHalfedgeIter vohit = _mesh->voh_begin(vh); vohit != _mesh->voh_end(vh); vohit++)
	{
		int f1 = _mesh->face_handle(*vohit).idx();
		int f2 = _mesh->opposite_face_handle(*vohit).idx();

		TriMesh::VertexHandle v2 = _mesh->to_vertex_handle(*vohit);
		TriMesh::Point p2 = _mesh->point(v2);

		Polynomial *poly1 = (f1 == -1 ? NULL : &_mesh->property(_facePolynomial, _mesh->face_handle(f1)));
		Polynomial *poly2 = (f2 == -1 ? NULL : &_mesh->property(_facePolynomial, _mesh->face_handle(f2)));

		double result1[2] = { 0.0, 0.0 };
		double result2[2] = { 0.0, 0.0 };

		compute_vertices_gradient(&p1[0], &p2[0], poly1, poly2, result1, result2);

		_mesh->property(_vertexGradient, vh) += TriMesh::Normal(result1[0], result1[1], 0.0);
	}

	process_vertex_gradient(v);
}

void OptiKnotTri::process_vertex_gradient(const int& v)
{
	TriMesh::VertexHandle vh = _mesh->vertex_handle(v);
	if (_mesh->is_boundary(vh))
	{
		for (TriMesh::VertexOHalfedgeIter vohit = _mesh->voh_begin(vh); vohit != _mesh->voh_end(vh); vohit++)
		{
			if (_mesh->is_boundary(_mesh->edge_handle(*vohit)))
			{
				TriMesh::Normal vohVec = _mesh->calc_edge_vector(*vohit);
				vohVec.normalize();
				double dotVal = dot(vohVec, _mesh->property(_vertexGradient, vh));
				if (dotVal > 0.0)
				{
					_mesh->property(_vertexGradient, vh) = dotVal * vohVec;
					return;
				}
			}
		}
	}
}

double OptiKnotTri::compute_maximum_step(const int &v, const double *direction) const
{
	// need to consider the concave situation of 1-ring neighborhood
	double norm = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
	if (norm < 1e-20)
	{
		printf("%s: v = %d, gradient norm is too small\n", __FUNCTION__, v);
		return 0.0;
	}

	TriMesh::VertexHandle vh = _mesh->vertex_handle(v);
	TriMesh::Point vp = _mesh->point(vh);

	std::vector<TriMesh::Point> polygon;
	for (TriMesh::VertexVertexCCWIter vvccwit = _mesh->vv_ccwbegin(vh); vvccwit != _mesh->vv_ccwend(vh); vvccwit++)
	{
		polygon.push_back(_mesh->point(*vvccwit));
	}

	double result = 1e10;
	int vvsize = (int)polygon.size();
	for (int i = 0; i < vvsize; i++)
	{
		int j = (i + 1) % vvsize;
		double temp = compute_safe_step(&vp[0], &polygon[i][0], &polygon[j][0], direction);
		if (temp < result)
			result = temp;
	}

	return result;
}

double OptiKnotTri::compute_safe_step(const double *p1, const double *p2, const double *p3, const double *direction) const
{
	/*
	double m[2] = { p2[0] - p1[0], p2[1] - p1[1] };
	double n[2] = { p3[0] - p1[0], p3[1] - p1[1] };

	double b = m[0] * direction[1] + n[1] * direction[0] - (n[0] * direction[1] + m[1] * direction[0]);
	double c = m[0] * n[1] - n[0] * m[1];

	double result = 1.0;
	if (fabs(b) > 1e-20)  // b != 0
	{
		double temp = c / b;
		if (temp > 0)
			result = temp;
	}

	return result;
	*/

	
	double c = p3[0] - p2[0];
	double d = p3[1] - p2[1];

	double m = c * direction[1] - d * direction[0];
	if (fabs(m) > 1e-20)
	{
		double a = p1[0] - p2[0];
		double b = p1[1] - p2[1];
		double n = a * d - b * c;
		double result = n / m;
		return (result >= 0.0 ? result : 1e10);
	}

	// coline
	return 1e10;
	
}

void OptiKnotTri::process_boundary()
{
	// make an inner vertex be boundary if it is close to boundary
	//printf("%s: begin\n", __FUNCTION__);
	int bv = -1;
	for (int v = 0; v < (int)_mesh->n_vertices(); v++)
	{
		if (v < _fixedVerticesNb)
			continue;
		if (_mesh->is_boundary(_mesh->vertex_handle(v)))
		{
			bv = v;
			break;
		}
	}
	if (bv == -1)
		return;

	TriMesh::VertexHandle vh = _mesh->vertex_handle(bv);	
	TriMesh::HalfedgeHandle headHe;
	for (TriMesh::VertexOHalfedgeIter vohit = _mesh->voh_begin(vh); vohit != _mesh->voh_end(vh); vohit++)
	{
		if (_mesh->is_boundary(*vohit))
		{
			headHe = *vohit;
			break;
		}
	}

	while (1)
	{
		if (!process_boundary_face(headHe))
			break;
	}

	TriMesh::HalfedgeHandle temphe = _mesh->next_halfedge_handle(headHe);
	while (temphe != headHe)
	{
		if (!process_boundary_face(temphe))
		{
			temphe = _mesh->next_halfedge_handle(temphe);
		}
	}

	//printf("%s: end\n", __FUNCTION__);
}

bool OptiKnotTri::process_boundary_face(TriMesh::HalfedgeHandle& borderHe)
{
	double tempAngle = _mesh->calc_sector_angle(_mesh->next_halfedge_handle(_mesh->opposite_halfedge_handle(borderHe)));
	if (tempAngle <= MAX_ANGLE)
	{
		return false;
	}

	TriMesh::Normal heVec = _mesh->calc_edge_vector(borderHe);
	double heLength = _mesh->calc_edge_length(borderHe);
	double triArea = _mesh->calc_sector_area(_mesh->next_halfedge_handle(_mesh->opposite_halfedge_handle(borderHe)));
	double height = triArea * 2.0 / heLength;

	TriMesh::Normal perp(-heVec[1], heVec[0], 0.0);
	perp.normalize();

	TriMesh::HalfedgeHandle* prevhe = &(_mesh->prev_halfedge_handle(borderHe));
	TriMesh::VertexHandle vh = _mesh->to_vertex_handle(_mesh->next_halfedge_handle(_mesh->opposite_halfedge_handle(borderHe)));
	TriMesh::Point vp = _mesh->point(vh);

	TriMesh::FaceHandle fh = _mesh->opposite_face_handle(borderHe);
	int f = fh.idx();
	int fnb = (int)_mesh->n_faces();

	_mesh->delete_face(fh);
	_mesh->garbage_collection();
	_mesh->set_point(vh, vp + height * perp);

	for (TriMesh::ConstVertexFaceIter cvfit = _mesh->cvf_begin(vh); cvfit != _mesh->cvf_end(vh); cvfit++)
	{
		int f = (*cvfit).idx();
		assign_pixel(f);
		compute_polynomial(f);
		compute_energy(f);
	}

	borderHe = _mesh->next_halfedge_handle(*prevhe);
	return true;
}

void OptiKnotTri::optimize_connectivity()
{
	compute_edges_batch();
	update_connectivity();
}

void OptiKnotTri::compute_edges_batch()
{
	//printf("%s: begin\n", __FUNCTION__);
	_edgesBatch.clear();

	int enb = (int)_mesh->n_edges();
	std::vector<int> edgesFlag;
	edgesFlag.resize(enb, -1);
	const int batchMaxnb = 64;

	for (int e = 0; e < enb; e++)
	{
		int batch[batchMaxnb] = { 0 };
		memset(batch, 0, batchMaxnb * sizeof(int));
		TriMesh::EdgeHandle eh = _mesh->edge_handle(e);
		TriMesh::HalfedgeHandle he = _mesh->halfedge_handle(eh, 0);

		TriMesh::VertexHandle vh[4];
		vh[0] = _mesh->to_vertex_handle(he);
		vh[1] = _mesh->to_vertex_handle(_mesh->next_halfedge_handle(he));
		vh[2] = _mesh->from_vertex_handle(he);
		vh[3] = _mesh->to_vertex_handle(_mesh->next_halfedge_handle(_mesh->opposite_halfedge_handle(he)));

		for (int ev = 0; ev < 4; ev++)
		{
			for (TriMesh::VertexEdgeIter veit = _mesh->ve_begin(vh[ev]); veit != _mesh->ve_end(vh[ev]); veit++)
			{
				int veid = (*veit).idx();
				if (edgesFlag[veid] > -1)
				{
					batch[edgesFlag[veid]]++;
				}
			}
		}

		int bid = 0;
		for (int b = 0; b < batchMaxnb; b++)
		{
			if (batch[b] == 0)
			{
				edgesFlag[e] = b;
				bid = b;
				break;
			}
		}

		while (bid >= (int)_edgesBatch.size())
		{
			_edgesBatch.push_back(std::vector<int>());
		}
		_edgesBatch[bid].push_back(e);
	}

	//printf("%s: end\n", __FUNCTION__);
}

void OptiKnotTri::update_connectivity()
{
	//printf("%s: begin\n", __FUNCTION__);

	int batchnb = (int)_edgesBatch.size();
	for (int b = 0; b < batchnb; b++)
	{
		std::vector<int>& batch = _edgesBatch[b];
		int enb = (int)batch.size();
#pragma omp parallel for
		for (int i = 0; i < enb; i++)
		{
			int e = batch[i];
			if (!is_flippable(_mesh, e)) // boundary edge
				continue;

			update_connection(e);
		}
	}
	//printf("%s: end\n", __FUNCTION__);
}

bool OptiKnotTri::update_connection(const int& e)
{
	TriMesh::EdgeHandle eh = _mesh->edge_handle(e);
	if (is_fixed_edge(eh))
		return false;

	TriMesh::HalfedgeHandle he = _mesh->halfedge_handle(eh, 0);

	double angle1 = _mesh->calc_sector_angle(he) + 
		_mesh->calc_sector_angle(_mesh->prev_halfedge_handle(_mesh->opposite_halfedge_handle(he)));
	double angle2 = _mesh->calc_sector_angle(_mesh->prev_halfedge_handle(he)) + 
		_mesh->calc_sector_angle(_mesh->opposite_halfedge_handle(he));
	if (GEOMETRY_CONTROL)
	{
		// geometry control
		if (angle1 > MAX_ANGLE)
			return false;

		if (angle2 > MAX_ANGLE)
			return false;
	}

	//////////////////////////////////////
	TriMesh::FaceHandle fh0 = _mesh->face_handle(he);
	TriMesh::FaceHandle fh1 = _mesh->opposite_face_handle(he);

	double angle3 = _mesh->calc_sector_angle(_mesh->next_halfedge_handle(he));
	double angle4 = _mesh->calc_sector_angle(_mesh->next_halfedge_handle(_mesh->opposite_halfedge_handle(he)));

	double prevEnergy = _mesh->property(_faceEnergy, fh0) + _mesh->property(_faceEnergy, fh1);

	PixelSet newPixelSet[2];
	Polynomial newPolynomial[2];
	newPolynomial[0] = Polynomial(_image, _degree);
	newPolynomial[1] = Polynomial(_image, _degree);
	double newEnergy[2];

	double triangle1[6];
	double triangle2[6];
	TriMesh::Point p1 = _mesh->point(_mesh->from_vertex_handle(he));
	TriMesh::Point p2 = _mesh->point(_mesh->to_vertex_handle(he));
	TriMesh::Point p3 = _mesh->point(_mesh->to_vertex_handle(_mesh->next_halfedge_handle(he)));
	TriMesh::Point p4 = _mesh->point(_mesh->to_vertex_handle(_mesh->next_halfedge_handle(_mesh->opposite_halfedge_handle(he))));

	triangle2[0] = p2[0];
	triangle2[1] = p2[1];
	triangle2[2] = p3[0];
	triangle2[3] = p3[1];
	triangle2[4] = p4[0];
	triangle2[5] = p4[1];

	triangle1[0] = p1[0];
	triangle1[1] = p1[1];
	triangle1[2] = p4[0];
	triangle1[3] = p4[1];
	triangle1[4] = p3[0];
	triangle1[5] = p3[1];

	Rasterizer ras(_image);
	ras.rasterize(triangle1, 3, newPixelSet[0]);
	ras.rasterize(triangle2, 3, newPixelSet[1]);
	newPolynomial[0].compute_factors(&newPixelSet[0]);
	newPolynomial[1].compute_factors(&newPixelSet[1]);
	newEnergy[0] = newPolynomial[0].compute_energy(&newPixelSet[0]);
	newEnergy[1] = newPolynomial[1].compute_energy(&newPixelSet[1]);

	double currentEnergy = newEnergy[0] + newEnergy[1];
	bool energyok = currentEnergy < prevEnergy;
	bool geometryok = (GEOMETRY_CONTROL && (angle3 > MAX_ANGLE || angle4 > MAX_ANGLE));
	bool delaunayok = (currentEnergy <= prevEnergy && angle1 + angle2 < M_PI);
	if (energyok || geometryok || delaunayok) // energy || geometry control || delaunay
	{
		_mesh->flip(eh);

		_mesh->property(_facePixelSet, fh0) = newPixelSet[0];
		_mesh->property(_facePixelSet, fh1) = newPixelSet[1];
		_mesh->property(_facePolynomial, fh0) = newPolynomial[0];
		_mesh->property(_facePolynomial, fh1) = newPolynomial[1];
		_mesh->property(_faceEnergy, fh0) = newEnergy[0];
		_mesh->property(_faceEnergy, fh1) = newEnergy[1];

		return true;
	}

	return false;
}

double OptiKnotTri::compute_psnr()
{
	if (_image == NULL || _approxImage == NULL)
		return 0.0;

	if (_image->width() != _approxImage->width() || _image->height() != _approxImage->height() ||
		_image->channel() != _approxImage->channel())
		return 0.0;

	int w = _image->width();
	int h = _image->height();

	double mse = 0.0;
	for (int j = 0; j < h; j++)
	{
		for (int i = 0; i < w; i++)
		{
			const unsigned char* color1 = _image->pixel_color_pointer(i, j);
			const unsigned char* color2 = _approxImage->pixel_color_pointer(i, j);

			for (int c = 0; c < _image->channel(); c++)
			{
				double diff = double(color1[c] - color2[c]);
				mse += diff * diff;
			}
		}
	}

	mse /= (w * h * _image->channel());
	if (mse < 1e-10)
		return 50.0;

	double psnr = 10 * std::log10(255.0 * 255.0 / mse);
	printf("%s: PSNR = %.3f dB\n", __FUNCTION__, psnr);

	return psnr;
}
