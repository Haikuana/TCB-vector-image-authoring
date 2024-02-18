#include "Corefuncs/fitting.h"

Fitting::Fitting(void)
{
	originalMesh = NULL;
	fittedMesh = NULL;
	bSplineBasis = NULL;
	basisNum = 0;
	mclInitializeApplication(NULL,0);

	sparse_quad_progInitialize();
}

Fitting::Fitting(Mesh *origin_mesh, Mesh *fitted_mesh, QString filename,double image_bound_weight,
	double feature_pos_weight, double pos_fair_firder_wei,double pos_fair_sedder_wei,
	double feature_color_weight)
{
	ImageFileName = filename;

	//position
	pos_adjacent_weight_origin = 0.0;
	pos_feature_weight_origin = 0.0;
	pos_boundary_weight = image_bound_weight;
	pos_supple_featuredata_weight = 0.0;
	pos_featurecurve_weight = feature_pos_weight;

	//position fairing
	firorder_weight_ = pos_fair_firder_wei;
	secorder_weight_ = pos_fair_sedder_wei;

	//color
	color_feature_weight = 0.0;
	color_featurecurve_weight = 0.0;
	color_adjacent_weight = feature_color_weight;
	color_supple_featuredata_weight = 0.0;

	color_featurecurve_weight += -1;
	color_feature_weight += -1;
	color_adjacent_weight += -1;

	originalMesh = origin_mesh;
	fittedMesh = fitted_mesh;
	bSplineBasis = NULL;
	cfair = NULL;
	basisNum = 0;
	if (originalMesh)
	{
		compute_range_box();
	}
	mclInitializeApplication(NULL, 0);

	//has some bugs
	sparse_quad_progInitialize();

}


Fitting::Fitting(Mesh *origin_mesh, Mesh *fitted_mesh,QString filename)
{
	ImageFileName = filename;

	//position
	pos_adjacent_weight_origin = 0.0;
	pos_feature_weight_origin = 0.0;
	pos_boundary_weight = 100.0;
	pos_supple_featuredata_weight = 0.0;
	pos_featurecurve_weight = 20.0;
	//color
	color_feature_weight = 0.0;
	color_featurecurve_weight = 0.0;
	color_adjacent_weight = 0.5;
	color_supple_featuredata_weight = 0.0;

	color_featurecurve_weight += -1;
	color_feature_weight += -1;
	color_adjacent_weight += -1;

	originalMesh = origin_mesh;
	fittedMesh = fitted_mesh;
	bSplineBasis = NULL;
	cfair = NULL;
	basisNum = 0;
	if (originalMesh)
	{
		compute_range_box();
	}
	mclInitializeApplication(NULL,0);

	//has some bugs
	sparse_quad_progInitialize();

}

Fitting::~Fitting(void)
{
	if (cfair != NULL)
	{
		delete cfair;
		cfair = NULL;
	}
	sparse_quad_progTerminate();

	mclTerminateApplication();
}

void Fitting::set_original_mesh(Mesh *mesh)
{
	originalMesh = mesh;
	compute_range_box();
}

void Fitting::set_fairing_weight(double alpha, double belta)
{
	firorder_weight_ = alpha;
	secorder_weight_ = belta;
}

void Fitting::set_fitted_mesh(Mesh *mesh)
{
	fittedMesh = mesh;
}

void Fitting::set_constrains(vector<pair<int, int>> cpc, vector<G1Corner> G1_constrain)
{
	controlpointpair = cpc;
	G1_constrains = G1_constrain;
}

void Fitting::set_supplement_data(vector<RGBPoint> &basisdata, vector<RGBPoint> &featuredata, vector<RGBPoint> &featurecurvedata)
{
	supplement_basisdata = basisdata;
	supplement_featuredata = featuredata;
	supplement_featurecurvedata = featurecurvedata;
}

void Fitting::set_basis(BSplineBasis *basis)
{
	bSplineBasis = basis;
}

void Fitting::set_allsimplex(vector<TConfig3>* allsimplex)
{
	alltconfig3s = allsimplex;
}

void Fitting::set_knots_data(std::map<unsigned, KnotData> knotsdata)
{
	dataMaps = knotsdata;
}


void Fitting::compute_range_box()
{
	box = originalMesh->bbox();
	const double ratio = 0.01;
	double xlength = box.xmax()-box.xmin();
	double ylength = box.ymax()-box.ymin();
	double xlowerbound = box.xmin()-ratio*xlength;
	double ylowerbound = box.ymin()-ratio*ylength;
	double xupperbound = box.xmax()+ratio*xlength;
	double yupperbound = box.ymax()+ratio*ylength;

	box = Iso_cuboid(xlowerbound, ylowerbound, 0, xupperbound, yupperbound, 0);

	std::cout << __FUNCTION__ << ": " << "mesh box: " << xlowerbound << "," << xupperbound << "-" << ylowerbound << "," << yupperbound << "\n";
}

void Fitting::assemble_constraint_matrix(mwArray &Aeq, mwArray &beq)
{
	if (bSplineBasis->cpInfos.empty())
		return;
	
	vector<mwSize> rowIndices, colIndices;
	vector<mxDouble> aData;
	int row = 1;
	vector<mxDouble> beqData;

	//corner control points - normal constrain
	int loc1, loc2;
	//feature basis - control points - position constrains
	for (int i = 0; i < controlpointpair.size(); i++)
	{
		loc1 = bSplineBasis->basisMergeInfos[controlpointpair[i].first].index;
		loc2 = bSplineBasis->basisMergeInfos[controlpointpair[i].second].index;

		if (loc1 != -1 && loc2 != -1)
		{
			rowIndices.push_back(row);
			rowIndices.push_back(row);
			colIndices.push_back(loc1);
			colIndices.push_back(loc2);
			aData.push_back(1);
			aData.push_back(-1);
			row++;

			rowIndices.push_back(row);
			rowIndices.push_back(row);
			colIndices.push_back(loc1 + (basisNum - 1));
			colIndices.push_back(loc2 + (basisNum - 1));
			aData.push_back(1);
			aData.push_back(-1);
			row++;
		}
	}

	row--;

	for (int i = 0; i < row; i++)
		beqData.push_back(0.0);

	Aeq = mwArray::NewSparse(rowIndices.size(), rowIndices.data(), colIndices.size(), colIndices.data(),
		aData.size(), aData.data(), aData.size());

	beq = mwArray(row, 1, mxDOUBLE_CLASS);
	beq.SetData(beqData.data(), beqData.size());

	/*QString dirAeq = ImageFileName;
	dirAeq.append("Aeq.txt");
	std::ofstream fout_Aeq;
	fout_Aeq.open(dirAeq.toStdString());
	if (fout_Aeq.is_open())
	{
		for (int i = 0; i < aData.size(); i++)
		{		
				fout_Aeq << rowIndices[i] << " " << colIndices[i] << " " << aData[i] << "\n";	
		}
	}
	fout_Aeq.close();*/
}

void Fitting::assemble_range_matrix(mwArray &lb, mwArray &ub)
{
	vector<mxDouble> lbData, ubData;
	for (int i=1; i<basisNum; i++)
	{
		lbData.push_back(box.xmin());
		ubData.push_back(box.xmax());
	}

	for (int i=1; i<basisNum; i++)
	{
		lbData.push_back(box.ymin());
		ubData.push_back(box.ymax());
	}
	

	for (int i=1; i<basisNum; i++)
	{
		lbData.push_back(-0.05);
		ubData.push_back(1.05);
	}
	for (int i = 1; i < basisNum; i++)
	{
		lbData.push_back(-0.05);
		ubData.push_back(1.05);
	}
	for (int i = 1; i < basisNum; i++)
	{
		lbData.push_back(-0.05);
		ubData.push_back(1.05);
	}

	lb = mwArray(lbData.size(), 1, mxDOUBLE_CLASS);
	ub = mwArray(ubData.size(), 1, mxDOUBLE_CLASS);
	lb.SetData(lbData.data(), lbData.size());
	ub.SetData(ubData.data(), ubData.size());

	//QString dirlb = ImageFileName;
	//dirlb.append("lb.txt");
	//std::ofstream fout_lb;
	//fout_lb.open(dirlb.toStdString());
	//if (fout_lb.is_open())
	//{
	//	for (int i = 0; i < lbData.size(); i++)
	//	{
	//		fout_lb << lbData[i] << " " << "\n";
	//	}
	//}
	//fout_lb.close();

	//QString dirub = ImageFileName;
	//dirub.append("ub.txt");
	//std::ofstream fout_ub;
	//fout_ub.open(dirub.toStdString());
	//if (fout_ub.is_open())
	//{
	//	for (int i = 0; i < ubData.size(); i++)
	//	{
	//		fout_ub << ubData[i] << " " << "\n";
	//	}
	//}
	//fout_ub.close();
}

void Fitting::assemble_coefficient_matrix(mwArray &A, mwArray &Afeature, mwArray &Aboundary, mwArray &AfeaColor)
{
	vector<mwSize> rowIndices;
	vector<mwSize> columnIndices;
	vector<mxDouble> aData;

	//position
	vector<mwSize> FeaPosrowIndices;
	vector<mwSize> FeaPoscolumnIndices;
	vector<mxDouble> FeaPosData;

	vector<mwSize> boundrowIndices;
	vector<mwSize> boundcolumnIndices;
	vector<mxDouble> boundData;

	//color
	vector<mwSize> FeaCoLrowIndices;
	vector<mwSize> FeaColcolumnIndices;
	vector<mxDouble> FeaColData;

	int index;
	basisNum = 1;
	max_A = 0.0;
	for (int i=0; i<bSplineBasis->basisMergeInfos.size(); i++)
	{
		index = bSplineBasis->basisMergeInfos[i].basisMerge[0];
		if (bSplineBasis->basisMergeInfos[i].basisMerge.size()>1)
		{
			map<unsigned, double> combValues;
			for (int k=0; k<bSplineBasis->basisMergeInfos[i].basisMerge.size(); k++)
			{
				int id = bSplineBasis->basisMergeInfos[i].basisMerge[k];
				if (!bSplineBasis->basisConfigs[id].supports.empty())
				{
					map<unsigned, DomainValue>::iterator it = bSplineBasis->basisConfigs[id].supports.begin();
					map<unsigned, DomainValue>::iterator end = bSplineBasis->basisConfigs[id].supports.end();
					for (; it!=end; it++)
						combValues[it->first] = 0;
				}
			}
			for (int k=0; k<bSplineBasis->basisMergeInfos[i].basisMerge.size(); k++)
			{
				int id = bSplineBasis->basisMergeInfos[i].basisMerge[k];
				if (!bSplineBasis->basisConfigs[id].supports.empty())
				{
					map<unsigned, DomainValue>::iterator it = bSplineBasis->basisConfigs[id].supports.begin();
					map<unsigned, DomainValue>::iterator end = bSplineBasis->basisConfigs[id].supports.end();
					for (; it!=end; it++)
						combValues[it->first] += it->second.value;
				}
			}
			if (combValues.empty())
			{
				std::cout << __FUNCTION__ << ": " << "!! warning: basis haven't support points" << std::endl;
				bSplineBasis->basisMergeInfos[i].index = -1;
			}
			else
			{
				int has_nonzero = false;
				for (map<unsigned, double>::iterator it = combValues.begin(); it != combValues.end(); it++)
				{
					if (it->second > 1e-8)
					{
						has_nonzero = true;
						break;
					}
				}
				if (!has_nonzero)
				{
					std::cout << __FUNCTION__ << ": " << "!!!!!!!!!!wrong basis has all-zero elements-combine" << std::endl;
				}
				for(map<unsigned, double>::iterator it=combValues.begin(); it!=combValues.end(); it++)
				{
					double weight_feapos = 0.0, weight_bound = 0.0, weight_feacolor = 0.0;
					//position
					if (it->first < originalMesh->size_of_vertices() && originalMesh->get_vertex_iterator(it->first)->is_border())
					{
						weight_bound = pos_boundary_weight;
					}
					else  if (it->first < originalMesh->size_of_vertices() && originalMesh->get_vertex_iterator(it->first)->tag() == 1)
					{
						weight_feapos = pos_feature_weight_origin;
					}
					else if (it->first < originalMesh->size_of_vertices() && originalMesh->get_vertex_iterator(it->first)->tag() == 2)
					{
						weight_feapos = pos_adjacent_weight_origin;
					}
					else if (it->first >= originalMesh->size_of_vertices() && it->first < originalMesh->size_of_vertices() + supplement_featurecurvedata.size())
					{
						weight_feapos = pos_featurecurve_weight;
					}
					else if (it->first >= originalMesh->size_of_vertices() + supplement_featurecurvedata.size()
						&& it->first < originalMesh->size_of_vertices() + supplement_featurecurvedata.size() + supplement_featuredata.size())
					{
						weight_feapos = pos_supple_featuredata_weight;
					}

					//color
					if (it->first < originalMesh->size_of_vertices() && originalMesh->get_vertex_iterator(it->first)->tag() == 1)
					{
						weight_feacolor = color_feature_weight;
					}
					else if (it->first < originalMesh->size_of_vertices() && originalMesh->get_vertex_iterator(it->first)->tag() == 2)
					{
						weight_feacolor = color_adjacent_weight;
					}
					else if (it->first >= originalMesh->size_of_vertices() && it->first < originalMesh->size_of_vertices() + supplement_featurecurvedata.size())
					{
						weight_feacolor = color_featurecurve_weight;
					}
					else if (it->first >= originalMesh->size_of_vertices() + supplement_featurecurvedata.size()
						&& it->first < originalMesh->size_of_vertices() + supplement_featurecurvedata.size() + supplement_featuredata.size())
					{
						weight_feacolor = color_supple_featuredata_weight;
					}

					FeaPosrowIndices.push_back(it->first + 1);
					FeaPoscolumnIndices.push_back(basisNum);
					FeaPosData.push_back((it->second)*weight_feapos);

					boundrowIndices.push_back(it->first + 1);
					boundcolumnIndices.push_back(basisNum);
					boundData.push_back((it->second)*weight_bound);

					FeaCoLrowIndices.push_back(it->first + 1);
					FeaColcolumnIndices.push_back(basisNum);
					FeaColData.push_back((it->second)*weight_feacolor);

					rowIndices.push_back(it->first+1);
					columnIndices.push_back(basisNum);
					aData.push_back(it->second);
					if (it->second > max_A)
					{
						max_A = it->second;
					}
				}
				bSplineBasis->basisMergeInfos[i].index = basisNum;
				basisNum++;
			}
		}
		else
		{
			if (bSplineBasis->basisConfigs[index].supports.empty())
			{
				std::cout << __FUNCTION__ << ": " << "!! warning: basis haven't support points:-" << bSplineBasis->basisConfigs[index].config[0]<<","
					<< bSplineBasis->basisConfigs[index].config[1] << std::endl;
				for (int m = 0;m<bSplineBasis->basisConfigs[index].tconfigs.size();m++)
				{
					std::cout << __FUNCTION__ << ": " << "!! warning:config: ";
					for (int n = 0;n<bSplineBasis->basisConfigs[index].tconfigs[m].tconfig.size();n++)
					{
						std::cout << bSplineBasis->basisConfigs[index].tconfigs[m].tconfig[n]<< "-" ;
					}
					std::cout << "\n";
				}
				bSplineBasis->basisMergeInfos[i].index = -1;
			}
			else
			{
				map<unsigned, DomainValue>::iterator it = bSplineBasis->basisConfigs[index].supports.begin();
				map<unsigned, DomainValue>::iterator end = bSplineBasis->basisConfigs[index].supports.end();
				int has_nonzero = false;
				for (; it != end; it++)
				{
					if (it->second.value >1e-8 )
					{
						has_nonzero = true;
						break;
					}
				}
				if (!has_nonzero)
				{
					std::cout << __FUNCTION__ << ": " << "!!!!!!!!!!wrong basis has all-zero elements-single" << bSplineBasis->basisConfigs[index].config[0] << ","
						<< bSplineBasis->basisConfigs[index].config[1] <<std::endl;
				}
				for (; it!=end; it++)
				{
					double weight_feapos = 0.0, weight_bound = 0.0, weight_feacolor = 0.0;
					//position
					if (it->first < originalMesh->size_of_vertices() && originalMesh->get_vertex_iterator(it->first)->is_border())
					{
						weight_bound = pos_boundary_weight;
					}
					else  if (it->first < originalMesh->size_of_vertices() && originalMesh->get_vertex_iterator(it->first)->tag() == 1)
					{
						weight_feapos = pos_feature_weight_origin;
					}
					else if (it->first < originalMesh->size_of_vertices() && originalMesh->get_vertex_iterator(it->first)->tag() == 2)
					{
						weight_feapos = pos_adjacent_weight_origin;
					}
					else if (it->first >= originalMesh->size_of_vertices() && it->first < originalMesh->size_of_vertices() + supplement_featurecurvedata.size())
					{
						weight_feapos = pos_featurecurve_weight;
					}
					else if (it->first >= originalMesh->size_of_vertices() + supplement_featurecurvedata.size()
						&& it->first < originalMesh->size_of_vertices() + supplement_featurecurvedata.size() + supplement_featuredata.size())
					{
						weight_feapos = pos_supple_featuredata_weight;
					}

					//color
					if (it->first < originalMesh->size_of_vertices() && originalMesh->get_vertex_iterator(it->first)->tag() == 1)
					{
						weight_feacolor = color_feature_weight;
					}
					else if (it->first < originalMesh->size_of_vertices() && originalMesh->get_vertex_iterator(it->first)->tag() == 2)
					{
						weight_feacolor = color_adjacent_weight;
					}
					else if (it->first >= originalMesh->size_of_vertices() && it->first < originalMesh->size_of_vertices() + supplement_featurecurvedata.size())
					{
						weight_feacolor = color_featurecurve_weight;
					}
					else if (it->first >= originalMesh->size_of_vertices() + supplement_featurecurvedata.size()
						&& it->first < originalMesh->size_of_vertices() + supplement_featurecurvedata.size() + supplement_featuredata.size())
					{
						weight_feacolor = color_supple_featuredata_weight;
					}

					FeaPosrowIndices.push_back(it->first + 1);
					FeaPoscolumnIndices.push_back(basisNum);
					FeaPosData.push_back((it->second.value)*weight_feapos);

					boundrowIndices.push_back(it->first + 1);
					boundcolumnIndices.push_back(basisNum);
					boundData.push_back((it->second.value)*weight_bound);

					FeaCoLrowIndices.push_back(it->first + 1);
					FeaColcolumnIndices.push_back(basisNum);
					FeaColData.push_back((it->second.value)*weight_feacolor);

					rowIndices.push_back(it->first+1);
					columnIndices.push_back(basisNum);
					aData.push_back(it->second.value);
				}
				bSplineBasis->basisMergeInfos[i].index = basisNum;
				basisNum++;
			}
		}
	}

	A = mwArray::NewSparse(rowIndices.size(), rowIndices.data(), columnIndices.size(), columnIndices.data(),
		aData.size(), aData.data(), aData.size());

	Afeature = mwArray::NewSparse(FeaPosrowIndices.size(), FeaPosrowIndices.data(), FeaPoscolumnIndices.size(), FeaPoscolumnIndices.data(),
		FeaPosData.size(), FeaPosData.data(), FeaPosData.size());

	Aboundary = mwArray::NewSparse(boundrowIndices.size(), boundrowIndices.data(), boundcolumnIndices.size(), boundcolumnIndices.data(),
		boundData.size(), boundData.data(), boundData.size());

	AfeaColor = mwArray::NewSparse(FeaCoLrowIndices.size(), FeaCoLrowIndices.data(), FeaColcolumnIndices.size(), FeaColcolumnIndices.data(),
		FeaColData.size(), FeaColData.data(), FeaColData.size());

	//QString dirA = ImageFileName;
	//dirA.append("A.txt");
	//std::ofstream fout_A;
	//fout_A.open(dirA.toStdString());
	//if (fout_A.is_open())
	//{
	//	for (int i = 0; i < aData.size(); i++)
	//	{
	//		//if (columnIndices[i] == 2298 || columnIndices[i] == 2304)
	//		{
	//			fout_A << rowIndices[i] << " " << columnIndices[i] << " " << aData[i] << "\n";
	//		}
	//	}
	//}
	//fout_A.close();

	//QString dirfA = ImageFileName;
	//dirfA.append("fA.txt");
	//std::ofstream fout_fA;
	//fout_fA.open(dirfA.toStdString());
	//if (fout_fA.is_open())
	//{
	//	for (int i = 0; i < FeaPosData.size(); i++)
	//	{
	//		//if (FeaPoscolumnIndices[i] == 2298 || FeaPoscolumnIndices[i] == 2304)
	//		fout_fA << FeaPosrowIndices[i] << " " << FeaPoscolumnIndices[i] << " " << FeaPosData[i] << "\n";
	//	}
	//}
	//fout_fA.close();

	//QString dirbA = ImageFileName;
	//dirbA.append("bA.txt");
	//std::ofstream fout_bA;
	//fout_bA.open(dirbA.toStdString());
	//if (fout_bA.is_open())
	//{
	//	for (int i = 0; i < boundData.size(); i++)
	//	{
	//		//if (boundcolumnIndices[i] == 2298 || boundcolumnIndices[i] == 2304)
	//		fout_bA << boundrowIndices[i] << " " << boundcolumnIndices[i] << " " << boundData[i] << "\n";
	//	}
	//}
	//fout_bA.close();
}

void Fitting::assemble_fairing_matrix(mwArray &Ader)
{
#if 0
	vector<mwSize> rowIndices;
	vector<mwSize> columnIndices;
	vector<mxDouble> aData;

	clock_t start = clock(),end_;
	if (cfair == NULL)
	{
		cfair = new CFairing(dataMaps, bSplineBasis, alltconfig3s,ImageFileName);
	}
	if (!cfair->has_compute())
	{
		cfair->compute_fairing_matrix();
		cfair->has_compute() = true;
	}
	end_ = clock();
	std::cout << __FUNCTION__ << ": " << "compute fairing matrix finished, cost time: "<<end_-start << std::endl;

	SpMat Ax = cfair->get_Bx();
	SpMat Ay = cfair->get_By();
	SpMat Axx = cfair->get_Bxx();
	SpMat Axy = cfair->get_Bxy();
	SpMat Ayy = cfair->get_Byy();
	//normalize
	SpMat Ad1 = Ax + Ay;
	vector <double> d1_values;
	for (int k = 0; k < Ad1.outerSize(); ++k)
	{
		for (SpMat::InnerIterator it(Ad1, k); it; ++it)
		{
			d1_values.push_back(abs(it.value()));
		}
	}
	sort(d1_values.begin(), d1_values.end());

	int range_num = int(double(d1_values.size()) * 0.01);
	double range_value = d1_values[d1_values.size() - 1];
	std::cout << __FUNCTION__ << ": " << "max value of first derivate optimization: " << range_value << std::endl;
	Ad1 = Ad1 / range_value;

	SpMat Ad2 = Axx + 2.0*Axy + Ayy;
	vector <double> d2_values;
	for (int k = 0; k < Ad2.outerSize(); ++k)
	{
		for (SpMat::InnerIterator it(Ad2, k); it; ++it)
		{
			d2_values.push_back(abs(it.value()));
		}
	}
	sort(d2_values.begin(), d2_values.end());
	//int range_num2 = int(double(d2_values.size()) * 0.01);
	double range_value2 = d2_values[d2_values.size() - 1];
	Ad2 = Ad2 / range_value2;
	std::cout << __FUNCTION__ << ": " << "max value of second derivate optimization: " << range_value2 << std::endl;

	//SpMat Aderivate = firorder_weight_ *Ad1 + secorder_weight_ *Ad2;
	SpMat Aderivate = firorder_weight_ *Ad1;

	for (int k = 0; k < Aderivate.outerSize(); ++k)
	{
		for (SpMat::InnerIterator it(Aderivate, k); it; ++it)
		{
			rowIndices.push_back(it.row() + 1);
			columnIndices.push_back(it.col() + 1);
			aData.push_back(it.value());
		}
	}
	Ader = mwArray::NewSparse(rowIndices.size(), rowIndices.data(), columnIndices.size(), columnIndices.data(),
		aData.size(), aData.data(), aData.size());
#else

	Ader = mwArray::NewSparse(basisNum - 1, basisNum - 1, 0, mxDOUBLE_CLASS);
#endif

	/*std::ofstream fout_;
	QString dir = ImageFileName;
	dir.append("bsplinebasis_cor.txt");
	fout_.open(dir.toStdString());
	if (fout_.is_open())
	{
		for (int i = 0; i < bSplineBasis->basisMergeInfos.size(); i++)
		{
			fout_ << bSplineBasis->basisMergeInfos[i].index << " - ";
			assert(bSplineBasis->basisMergeInfos[i].basisMerge.size() < 2);
			fout_ << bSplineBasis->basisMergeInfos[i].basisMerge[0] << " - ";
			int index_ = bSplineBasis->basisMergeInfos[i].basisMerge[0];
			for (int j = 0; j < bSplineBasis->basisConfigs[index_].tconfigs.size(); j++)
			{
				for (int k = 0;k<bSplineBasis->basisConfigs[index_].tconfigs[j].size();k++)
				{
					fout_ << bSplineBasis->basisConfigs[index_].tconfigs[j][k] << " ";
				}
			}
			fout_ << "\n";
		}
	}
	fout_.close();*/
	
}

void Fitting::assemble_known_matrix()
{
	//position
	//x
	for (Vertex_iterator vit = originalMesh->vertices_begin(); vit != originalMesh->vertices_end(); vit++)
	{
		double weight_ = 0.0;
		if (vit->is_border())
		{
			weight_ = pos_boundary_weight;
		}
		else if (vit->tag() == 1)
		{
			weight_ = pos_feature_weight_origin;
		}
		else if (vit->tag() == 2)
		{
			weight_ = pos_adjacent_weight_origin;
		}
		bData.push_back((-vit->point().x())*(weight_ + 1.0));
	}
	for (int i = 0; i < supplement_featurecurvedata.size(); i++)
		bData.push_back(-supplement_featurecurvedata[i].x *(pos_featurecurve_weight + 1.0));
	for (int i = 0; i < supplement_featuredata.size(); i++)
		bData.push_back(-supplement_featuredata[i].x *(pos_supple_featuredata_weight + 1.0));
	for (int i = 0; i < supplement_basisdata.size(); i++)
		bData.push_back(-supplement_basisdata[i].x);
	//y
	for (Vertex_iterator vit = originalMesh->vertices_begin(); vit != originalMesh->vertices_end(); vit++)
	{
		double weight_ = 0.0;
		if (vit->is_border())
		{
			weight_ = pos_boundary_weight;
		}
		else if (vit->tag() == 1)
		{
			weight_ = pos_feature_weight_origin;
		}
		else if (vit->tag() == 2)
		{
			weight_ = pos_adjacent_weight_origin;
		}
		bData.push_back((-vit->point().y()) *(weight_ + 1.0));
	}
	for (int i = 0; i < supplement_featurecurvedata.size(); i++)
		bData.push_back(-supplement_featurecurvedata[i].y *(pos_featurecurve_weight + 1.0));
	for (int i = 0; i < supplement_featuredata.size(); i++)
		bData.push_back(-supplement_featuredata[i].y*(pos_supple_featuredata_weight + 1.0));
	for (int i = 0; i < supplement_basisdata.size(); i++)
		bData.push_back(-supplement_basisdata[i].y);
	
	//color
	//r
	for (Vertex_iterator vit = originalMesh->vertices_begin(); vit != originalMesh->vertices_end(); vit++)
	{
		double weight_ = 0.0;
		if (vit->tag() == 1)
		{
			weight_ = color_feature_weight;
		}
		else if (vit->tag() == 2)
		{
			weight_ = color_adjacent_weight;
		}
		bData.push_back(-vit->vertex_pixel_color().x() *(weight_ + 1.0));
		//bData.push_back(-vit->vertex_pixel_color().x());
	}
	for (int i = 0; i < supplement_featurecurvedata.size(); i++)
		bData.push_back(0);
	for (int i = 0; i < supplement_featuredata.size(); i++)
		bData.push_back(-supplement_featuredata[i].r *(color_supple_featuredata_weight+1.0));	
	for (int i = 0; i < supplement_basisdata.size(); i++)
		bData.push_back(-supplement_basisdata[i].r);
	//g
	for (Vertex_iterator vit = originalMesh->vertices_begin(); vit != originalMesh->vertices_end(); vit++)
	{
		double weight_ = 0.0;
		if (vit->tag() == 1)
		{
			weight_ = color_feature_weight;
		}
		else if (vit->tag() == 2)
		{
			weight_ = color_adjacent_weight;
		}
		bData.push_back(-vit->vertex_pixel_color().y() *(weight_ + 1.0));
		//bData.push_back(-vit->vertex_pixel_color().y());
	}
	for (int i = 0; i < supplement_featurecurvedata.size(); i++)
		bData.push_back(0);
	for (int i = 0; i < supplement_featuredata.size(); i++)
		bData.push_back(-supplement_featuredata[i].g *(color_supple_featuredata_weight + 1.0));
	for (int i = 0; i < supplement_basisdata.size(); i++)
		bData.push_back(-supplement_basisdata[i].g);
	//b
	for (Vertex_iterator vit = originalMesh->vertices_begin(); vit != originalMesh->vertices_end(); vit++)
	{
		double weight_ = 0.0;
		if (vit->tag() == 1)
		{
			weight_ = color_feature_weight;
		}
		else if (vit->tag() == 2)
		{
			weight_ = color_adjacent_weight;
		}
		bData.push_back(-vit->vertex_pixel_color().z() *(weight_ + 1.0));
		//bData.push_back(-vit->vertex_pixel_color().z());
	}
	for (int i = 0; i < supplement_featurecurvedata.size(); i++)
		bData.push_back(0);
	for (int i = 0; i < supplement_featuredata.size(); i++)
		bData.push_back(-supplement_featuredata[i].b *(color_supple_featuredata_weight + 1.0));
	for (int i = 0; i < supplement_basisdata.size(); i++)
		bData.push_back(-supplement_basisdata[i].b);

	//test
	/*QString dirb = ImageFileName;
	dirb.append("b.txt");
	std::ofstream fout_b;
	fout_b.open(dirb.toStdString());
	if (fout_b.is_open())
	{
		for (int i = 0; i < bData.size(); i++)
		{
			fout_b << bData[i] << " " << "\n";
		}
	}
	fout_b.close();*/
}

void Fitting::set_initial_value(mwArray & X0)
{
	vector<mxDouble> x0;
	for (int k = 0;k<NUMFITTINGELEMENT;k++)
	{
		int num = 0;
		for (int i = 0; i < bSplineBasis->basisMergeInfos.size(); i++)
		{
			if (bSplineBasis->basisMergeInfos[i].index != -1)
			{
				int loc = bSplineBasis->basisMergeInfos[i].basisMerge[0];
				vector<double> va_ = bSplineBasis->basisConfigs[loc].initial_points;
				if (va_.size() >= NUMFITTINGELEMENT)
				{
					x0.push_back(va_[k]);
					num++;
				}
			}
		}
		assert(num == basisNum-1);
	}

	X0 = mwArray((NUMFITTINGELEMENT)*(basisNum-1), 1, mxDOUBLE_CLASS);
	X0.SetData(x0.data(), x0.size());

	//QString dirx0 = ImageFileName;
	//dirx0.append("x0.txt");
	//std::ofstream fout_x0;
	//fout_x0.open(dirx0.toStdString());
	//if (fout_x0.is_open())
	//{
	//	for (int i = 0; i < x0.size(); i++)
	//	{
	//		fout_x0 << x0[i] << " " << "\n";
	//	}
	//}
	//fout_x0.close();
}

void Fitting::extract_control_points(mxDouble *cps, int nNum)
{
	int index, loc;
	int basisSz = nNum/ NUMFITTINGELEMENT;
	for (int i=0; i<bSplineBasis->basisMergeInfos.size(); i++)
	{
		index = bSplineBasis->basisMergeInfos[i].index;
		if (index!=-1)
		{
			for (int j=0; j<bSplineBasis->basisMergeInfos[i].basisMerge.size(); j++)
			{
				loc = bSplineBasis->basisMergeInfos[i].basisMerge[j];
				RGBPoint p;
				p.basis_index = loc;
				p.x = cps[index-1];p.y = cps[index - 1 + basisSz];
				p.r = cps[index - 1 + basisSz*2];p.g = cps[index - 1 + basisSz*3];p.b = cps[index - 1 + basisSz*4];
#ifdef OPTIMIZE_GEO
				bSplineBasis->basisConfigs[loc].controlPt = p;
#else
				bSplineBasis->basisConfigs[loc].controlPt = p;
				bSplineBasis->basisConfigs[loc].controlPt.x = bSplineBasis->basisConfigs[loc].initial_points[0];
				bSplineBasis->basisConfigs[loc].controlPt.y = bSplineBasis->basisConfigs[loc].initial_points[1];
#endif
							
			}
		}
		else
		{
			for (int j = 0; j < bSplineBasis->basisMergeInfos[i].basisMerge.size(); j++)
			{
				loc = bSplineBasis->basisMergeInfos[i].basisMerge[j];
				RGBPoint p;
				p.basis_index = -1;
				p.x = 0; p.y = 0;
				p.r = 0; p.g = 0; p.b = 0;
#ifdef OPTIMIZE_GEO
				bSplineBasis->basisConfigs[loc].controlPt = p;
#else
				bSplineBasis->basisConfigs[loc].controlPt = p;
				bSplineBasis->basisConfigs[loc].controlPt.x = bSplineBasis->basisConfigs[loc].initial_points[0];
				bSplineBasis->basisConfigs[loc].controlPt.y = bSplineBasis->basisConfigs[loc].initial_points[1];
#endif
			}
		}
	}

	//std::cout << __FUNCTION__ << ": " << "then: add G1 constraints" << std::endl;
#ifdef CONSIDER_G1_CURVE
	for (int i = 0; i < G1_constrains.size(); i++)
	{
		if (G1_constrains[i].index_cen_pair1 == -1 || G1_constrains[i].index_cen_pair2 == -1)
		{
			std::cout << __FUNCTION__ << ": " << "something wrong" << std::endl;
		}

		int loccor1 = bSplineBasis->basisMergeInfos[G1_constrains[i].index_cen_pair1].basisMerge[0];
		int loccor2 = bSplineBasis->basisMergeInfos[G1_constrains[i].index_cen_pair2].basisMerge[0];
		RGBPoint pcorner = bSplineBasis->basisConfigs[loccor1].controlPt;
		RGBPoint pre = bSplineBasis->basisConfigs[bSplineBasis->
			basisMergeInfos[G1_constrains[i].index_side1].basisMerge[0]].controlPt;
		RGBPoint pnext = bSplineBasis->basisConfigs[bSplineBasis->
			basisMergeInfos[G1_constrains[i].index_side2].basisMerge[0]].controlPt;

		Point_2 c(pcorner.x,pcorner.y);
		Point_2 a(pre.x, pre.y);
		Point_2 b(pnext.x, pnext.y);
		Vector_2 ptemp = ((c - a)*(b - a)) / (b - a).squared_length()*(b - a);
		bSplineBasis->basisConfigs[loccor1].controlPt.x = ptemp.x() + a.x();
		bSplineBasis->basisConfigs[loccor1].controlPt.y = ptemp.y() + a.y();
		bSplineBasis->basisConfigs[loccor2].controlPt.x = ptemp.x() + a.x();
		bSplineBasis->basisConfigs[loccor2].controlPt.y = ptemp.y() + a.y();

	}
#endif
	//std::cout << __FUNCTION__ << ": " << "finished: add G1 constraints" << std::endl;
}

void Fitting::update_fitted_mesh(mwArray &Ax)
{
	int cpNum = Ax.NumberOfElements();
	if (!cpNum)
		return;

	int vSz = cpNum/NUMFITTINGELEMENT;
	
	double *pts = new mxDouble[cpNum];
	Ax.GetData(pts, cpNum);
	unsigned index;
	for (Vertex_iterator vit=fittedMesh->vertices_begin(); vit!= fittedMesh->vertices_end(); vit++)
	{
		index = vit->vertex_index();
		vit->vertex_pixel_color() = Point_3(pts[index + vSz * 2], pts[index + vSz * 3], pts[index + vSz * 4]);
		double gray_ = 0.2989*pts[index + vSz * 2] + 0.5870*pts[index + vSz * 3] + 0.1140*pts[index + vSz * 4];//for surface display
		vit->point() = Point(pts[index], pts[index + vSz], gray_);
	}
	//fittedMesh->compute_normals();//surface normal under lighting
	delete []pts;

	std::cout << __FUNCTION__ << ": " << "update mesh with new position" << std::endl;
	//update mesh with new position
#pragma omp parallel for
	for (int it = 0;it<fittedMesh->size_of_vertices();it++)
	{
		int index = it;
		Vector_2 position_now(0, 0);
		for (int i = 0; i < bSplineBasis->basisMergeInfos.size(); i++)
		{
			for (int j = 0; j < bSplineBasis->basisMergeInfos[i].basisMerge.size(); j++)
			{
				int loc = bSplineBasis->basisMergeInfos[i].basisMerge[j];
				if (bSplineBasis->basisConfigs[loc].supports.size() != NULL)
				{
					map<unsigned int, DomainValue>::iterator res = bSplineBasis->basisConfigs[loc].supports.find(index);
					if (res != bSplineBasis->basisConfigs[loc].supports.end())
					{
						position_now += Vector_2(res->second.value*bSplineBasis->basisConfigs[loc].controlPt.x,
							res->second.value*bSplineBasis->basisConfigs[loc].controlPt.y);
					}
				}
			}
		}
		Mesh::Vertex_iterator vit = fittedMesh->get_vertex_iterator(it);
		vit->point() = Point(position_now.x(), position_now.y(), vit->point().z());
	}
	fittedMesh->compute_normals();
}

void Fitting::fitting(OneStepInfo &timeInfo)
{
	std::cout << __FUNCTION__ << ": " << "fitting started" << std::endl;

	clock_t start_,finish_;

	//A-b
	mwArray A,Afeapos,Aboundary,Afeacolor;
	assemble_coefficient_matrix(A, Afeapos, Aboundary,Afeacolor);
	std::cout << __FUNCTION__ << ": " << "assemble A finished" << std::endl;


	if (bData.empty())
		assemble_known_matrix();
	mwArray b = mwArray(bData.size(), 1, mxDOUBLE_CLASS);
	b.SetData(bData.data(), bData.size());
	std::cout << __FUNCTION__ << ": " << "assemble b finished,"<< std::endl;

	//fairing derivate
	mwArray Aderivate;
	start_ = clock();
	assemble_fairing_matrix(Aderivate);
	finish_ = clock();
	timeInfo.t_fairing += (double(finish_ - start_)) / CLOCKS_PER_SEC;
	std::cout << __FUNCTION__ << ": " << "assemble fairing matrix finished" << std::endl;

	//constrain
	mwArray Aeq;
	mwArray beq;
	assemble_constraint_matrix(Aeq, beq);
	std::cout << __FUNCTION__ << ": " << "assemble constrain matrix finished" << std::endl;

	//range
	mwArray lb;
	mwArray ub;
	assemble_range_matrix(lb, ub);

	//initial point
	//mwArray X0 = mwArray(bData.size(), 1, mxDOUBLE_CLASS);
	//set_initial_value(X0);
	//std::cout << __FUNCTION__ << ": " << "assemble initial p finished" << std::endl;

	start_ = clock();
	mwArray cp, Asample;
	mwArray conditons;
	mwArray rank, Nullindex;
	mwArray Jv;//index of maximum linearly independent vector group
	//sparse_quad_prog(4, cp, Asample, rank,Jv, A, Afeapos,Aboundary,Afeacolor,Aderivate, b, Aeq, beq, lb, ub);
	//sparse_quad_prog(3, cp, Asample, rank, A, Afeapos, Aboundary, Afeacolor, Aderivate, b, Aeq, beq, lb, ub);
	//sparse_quad_prog(2, cp, Asample, A, Afeapos, Aboundary, Afeacolor, Aderivate, b, Aeq, beq, lb, ub);
	sparse_quad_prog(4, cp, Asample, rank, Nullindex, A, Afeapos, Aboundary, Afeacolor, Aderivate, b, Aeq, beq, lb, ub);

	finish_ = clock();
	timeInfo.t_solve_equation = (double(finish_ - start_)) / CLOCKS_PER_SEC;

	int cpNum = cp.NumberOfElements();
	double *cps = new mxDouble[cpNum];
	cp.GetData(cps, cpNum);

	extract_control_points(cps, cpNum);
	update_fitted_mesh(Asample);

#if 0
	//test
	//find linearly dependent - through reff()
	std::cout << __FUNCTION__ << ": " << "end fitting\n";
	int cvNum = Jv.NumberOfElements();
	vector<int> index_independ;
	for (int i = 0;i<cvNum;i++)
	{
		int id = Jv.Get(1, i+1);
		index_independ.push_back(id);
	}
	//find linearly dependent
	vector<int> index_depend;
	int numbasis = rank.Get(1, 3);
	std::cout << __FUNCTION__ << ": " << "index of linearly dependent vector group: ";
	for (int i = 0;i<numbasis;i++)
	{
		vector<int>::iterator res = find(index_independ.begin(),index_independ.end(),i+1);
		if (res == index_independ.end())
		{
			index_depend.push_back(i+1);
			std::cout <<i+1<< "-";
		}
	}
	std::cout << std::endl;
	for (int i = 0; i < bSplineBasis->basisMergeInfos.size(); i++)
	{
		int inde = bSplineBasis->basisMergeInfos[i].index;
		vector<int>::iterator res = find(index_depend.begin(), index_depend.end(), inde);
		if (res != index_depend.end())
		{
			for (int j = 0;j<bSplineBasis->basisMergeInfos[i].basisMerge.size();j++)
			{
				std::cout << "merge num: "<<j<<"\n";
				int nowdex = bSplineBasis->basisMergeInfos[i].basisMerge[j];
				for (int k = 0;k<bSplineBasis->basisConfigs[nowdex].tconfigs.size();k++)
				{	
					for (int m = 0;m<bSplineBasis->basisConfigs[nowdex].tconfigs[k].tconfig.size();m++)
					{
						std::cout << bSplineBasis->basisConfigs[nowdex].tconfigs[k].tconfig[m] << "-";
					}
					std::cout << "\n";
				}
			}
		}
	}
#endif

#if 1
	//test
	//find linearly dependent - through null()
	std::cout << __FUNCTION__ << ": " << "end fitting\n";
	std::cout << __FUNCTION__ << ": " << "index of linearly dependent vector group: ";
	int cvNum = Nullindex.NumberOfElements();
	vector<int> index_depend;
	for (int i = 1; i < cvNum+1; i++)
	{
		int id = Nullindex.Get(1, i);
		if (id >0)
		{
			index_depend.push_back(id);
			std::cout << id << "-";
		}	
	}
	std::cout << std::endl;

	for (int i = 0; i < bSplineBasis->basisMergeInfos.size(); i++)
	{
		int inde = bSplineBasis->basisMergeInfos[i].index;
		vector<int>::iterator res = find(index_depend.begin(), index_depend.end(), inde);
		if (res != index_depend.end())
		{
			std::cout << "basis:" << "\n";
			for (int j = 0; j < bSplineBasis->basisMergeInfos[i].basisMerge.size(); j++)
			{
				std::cout << "merge num: " << j << "\n";
				int nowdex = bSplineBasis->basisMergeInfos[i].basisMerge[j];
				for (int k = 0; k < bSplineBasis->basisConfigs[nowdex].tconfigs.size(); k++)
				{
					for (int m = 0; m < bSplineBasis->basisConfigs[nowdex].tconfigs[k].tconfig.size(); m++)
					{
						std::cout << bSplineBasis->basisConfigs[nowdex].tconfigs[k].tconfig[m] << "-";
					}
					std::cout << "\n";
				}
			}
		}
	}
#endif
	timeInfo.num_controlp = basisNum - 1 - 2*controlpointpair.size();
	std::cout << __FUNCTION__ << ": " << "quad program finished" << std::endl;
	if (0) {
		std::cout << __FUNCTION__ << ": " << "matrix size-valid basis size: " << rank.Get(1, 3) << "-" << basisNum - 1 << std::endl;
		std::cout << __FUNCTION__ << ": " << "matrix-color det and condition number: " << rank.Get(1, 1) << " ; " << /*conditons.Get(1, 1) << */std::endl;
		std::cout << __FUNCTION__ << ": " << "matrix-position det and condition number: " << rank.Get(1, 2) << " ; " << /*conditons.Get(1, 2) << */std::endl;
	}
	else {
		std::cout << __FUNCTION__ << ": " << "matrix size-valid basis size: " << rank.Get(1, 5) << "-" << basisNum - 1 << std::endl;
		std::cout << __FUNCTION__ << ": " << "matrix-color det and condition number: " << rank.Get(1, 1) << " ; " << rank.Get(1, 2) << std::endl;
		std::cout << __FUNCTION__ << ": " << "matrix-position det and condition number: " << rank.Get(1, 3) << " ; " << rank.Get(1, 4) << std::endl;
	}

	//std::cout << __FUNCTION__ << ": " << "extract_control_points and update finished" << std::endl;
	delete []cps;
}