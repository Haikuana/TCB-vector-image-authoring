#include "Corefuncs/basis_computation.h"
#include "Corefuncs/simplex_spline.h"
#include "Corefuncs/auxfunc.h"
#include "Corefuncs/sample_drawfunc.h"

void generate_basis_configuraitons(vector<TConfig2> &tconfigs, BSplineBasis &bSplineBasis)
{
	bSplineBasis.deg = tconfigs[0].tconfig.size() - BOUNDARYSIZE;
	for (int i = 0; i < tconfigs.size(); i++)
	{
		vector<unsigned> test;
		for (int j = BOUNDARYSIZE; j < tconfigs[i].tconfig.size(); j++)
			test.push_back(tconfigs[i].tconfig[j]);
		bool bExist = false;
		int j = 0;
		for (; j < bSplineBasis.basisConfigs.size(); j++)
		{
			vector<unsigned int> scr_, temp_;
			scr_ = test; temp_ = bSplineBasis.basisConfigs[j].config;
			sort(scr_.begin(), scr_.end());
			sort(temp_.begin(), temp_.end());
			if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
			{
				bExist = true;
				break;
			}
		}
		if (bExist)
		{
			bSplineBasis.basisConfigs[j].tconfigs.push_back(tconfigs[i]);
		}
		else
		{
			SplineBasisConfigs basisConfig;
			basisConfig.config = test;
			basisConfig.tconfigs.push_back(tconfigs[i]);
			bSplineBasis.basisConfigs.push_back(basisConfig);
		}
	}
	std::cout << __FUNCTION__ << ": " << "--merge config - number of basis configs: " << bSplineBasis.basisConfigs.size() << std::endl;
}

void remove_singularities(BSplineBasis &bSplineBasis, vector<unsigned> &cornerIds, map<int, int> &controlpointindex)
{
	bSplineBasis.basisMergeInfos.reserve(cornerIds.size());
	std::vector<bool> bProcessed(bSplineBasis.basisConfigs.size());
	for (int i=0; i<bSplineBasis.basisConfigs.size(); i++)
		bProcessed[i] = false;

	std::vector<std::vector<TConfig2>> templates(bSplineBasis.basisConfigs.size());
	for (int i = 0; i < bSplineBasis.basisConfigs.size(); i++)
	{
		templates[i] = bSplineBasis.basisConfigs[i].tconfigs;
		for (int j = 0; j < templates[i].size(); j++)
			std::sort(templates[i][j].tconfig.begin(), templates[i][j].tconfig.end());
	}

	//boundary corner
	for (int i = 0; i < cornerIds.size(); i++)
	{
		BasisMergeInfo mergeInfo;
		for (int j = 0; j < bSplineBasis.basisConfigs.size(); j++)
		{
			if (bProcessed[j])
				continue;

			for (int k = 0; k < bSplineBasis.basisConfigs[j].tconfigs.size(); k++)
			{
				/*if (bSplineBasis.basisConfigs[j].tconfigs[k].tconfig[0] == bSplineBasis.basisConfigs[j].tconfigs[k].tconfig[1]
					|| bSplineBasis.basisConfigs[j].tconfigs[k].tconfig[1] == bSplineBasis.basisConfigs[j].tconfigs[k].tconfig[2]
					|| bSplineBasis.basisConfigs[j].tconfigs[k].tconfig[2] == bSplineBasis.basisConfigs[j].tconfigs[k].tconfig[0])
				{
					continue;
				}*/
				int nMulti = 0;
				for (int l = 0; l < bSplineBasis.basisConfigs[j].tconfigs[k].tconfig.size(); l++)
				{
					if (cornerIds[i] == bSplineBasis.basisConfigs[j].tconfigs[k].tconfig[l])
					{
						nMulti++;
					}
				}
				if (nMulti == bSplineBasis.deg + 1)
				{
					mergeInfo.basisMerge.push_back(j);
					bProcessed[j] = true;
					break;
				}
			}
		}

		if (!mergeInfo.basisMerge.empty())
		{
			mergeInfo.type = 2;
			int index_now = bSplineBasis.basisMergeInfos.size();
			for (int it = 0; it < mergeInfo.basisMerge.size(); it++)
			{
				controlpointindex[mergeInfo.basisMerge[it]] = index_now;
			}
			bSplineBasis.basisMergeInfos.push_back(mergeInfo);
			CoplanarInfo coplanarInfo;
			coplanarInfo.cornerBasisId = bSplineBasis.basisMergeInfos.size() - 1;
			coplanarInfo.cornerId = cornerIds[i];
			bSplineBasis.cpInfos.push_back(coplanarInfo);
		}
	}

	//merge linearly dependent basis
	for (int i = 0; i < templates.size(); i++)
	{
		if (bProcessed[i])
			continue;

		BasisMergeInfo mergeInfo;
		mergeInfo.type = -1;
		mergeInfo.basisMerge.push_back(i);
		vector<SubSet> allSameIds;
		//for size smaller
		for (int j = 0; j < templates.size(); j++)
		{
			if (i == j || bProcessed[j] || templates[j].size() > templates[i].size())
				continue;

			SubSet subset;
			subset.loc = j;
			for (int l = 0; l < templates[j].size(); l++)
			{
				for (int m = 0; m < templates[i].size(); m++)
				{
					vector<unsigned int> scr_, temp_;
					scr_ = templates[j][l].tconfig; temp_ = templates[i][m].tconfig;
					sort(scr_.begin(), scr_.end());
					sort(temp_.begin(), temp_.end());
					if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
					{
						subset.sameIds.push_back(m);
						break;
					}
				}
			}
			if (subset.sameIds.size() == templates[j].size())
				allSameIds.push_back(subset);
		}


		if (!allSameIds.empty())
		{
			if (templates[i].size() == 1)
			{
				for (int j = 0; j < allSameIds.size(); j++)
				{
					mergeInfo.basisMerge.push_back(allSameIds[j].loc);
					bProcessed[allSameIds[j].loc] = true;
				}
				mergeInfo.type = 0;
			}
			else
			{
				int num = 0;
				for (int j = 0; j < allSameIds.size(); j++)
					num += allSameIds[j].sameIds.size();

				if (num > templates[i].size())
				{
					std::cout << __FUNCTION__ << ": " << "merge linearly dependent process- num>templates[i].size()" << std::endl;
					vector<unsigned> a;
					for (unsigned j = 0; j < allSameIds.size(); j++)
						a.push_back(j);

					vector<unsigned> b;
					for (unsigned j = 0; j < templates[i].size(); j++)
						b.push_back(j);

					vector<vector<unsigned>> combs;
					combine_near_all(a, combs);
					for (unsigned j = 0; j < combs.size(); j++)
					{
						vector<unsigned> comb = allSameIds[combs[j][0]].sameIds;
						for (unsigned l = 1; l < combs[j].size(); l++)
							comb.insert(comb.end(), allSameIds[combs[j][l]].sameIds.begin(), allSameIds[combs[j][l]].sameIds.end());

						vector<unsigned int> scr_, temp_;
						scr_ = comb; temp_ = b;
						sort(scr_.begin(), scr_.end());
						sort(temp_.begin(), temp_.end());
						if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
						{
							for (unsigned l = 0; l < combs[j].size(); l++)
							{
								mergeInfo.basisMerge.push_back(allSameIds[combs[j][l]].loc);
								bProcessed[allSameIds[combs[j][l]].loc] = true;
							}
							mergeInfo.type = 1;
							break;
						}
					}
				}
				else if (num == templates[i].size())
				{
					std::cout << __FUNCTION__ << ": " << "merge linearly dependent process- num=templates[i].size()" << std::endl;
					vector<unsigned> a = allSameIds[0].sameIds;
					vector<unsigned> b;
					for (unsigned j = 0; j < templates[i].size(); j++)
						b.push_back(j);
					for (unsigned j = 1; j < allSameIds.size(); j++)
						a.insert(a.end(), allSameIds[j].sameIds.begin(), allSameIds[j].sameIds.end());
					vector<unsigned int> scr_, temp_;
					scr_ = a; temp_ = b;
					sort(scr_.begin(), scr_.end());
					sort(temp_.begin(), temp_.end());
					if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
					{
						//std::cout << __FUNCTION__ << ": " << "num=templates[i].size() and equal" << std::endl;
						for (unsigned j = 0; j < allSameIds.size(); j++)
						{
							mergeInfo.basisMerge.push_back(allSameIds[j].loc);
							bProcessed[allSameIds[j].loc] = true;
						}
						mergeInfo.type = 1;
					}
				}
				else
				{
					/*for (int j = 0; j < allSameIds.size(); j++)
					{
						mergeInfo.basisMerge.push_back(allSameIds[j].loc);
						bProcessed[allSameIds[j].loc] = true;
					}
					mergeInfo.type = 3;*/
				}
			}
		}
		if (mergeInfo.type != -1)
		{
			bProcessed[i] = true;
			int index_now = bSplineBasis.basisMergeInfos.size();
			for (int it = 0; it < mergeInfo.basisMerge.size(); it++)
			{
				controlpointindex[mergeInfo.basisMerge[it]] = index_now;
			}
			bSplineBasis.basisMergeInfos.push_back(mergeInfo);
		}
	}
	for (int i = 0; i < templates.size(); i++)
	{
		if (bProcessed[i])
			continue;

		BasisMergeInfo mergeInfo;
		mergeInfo.type = 0;
		mergeInfo.basisMerge.push_back(i);
		bProcessed[i] = true;
		int index_now = bSplineBasis.basisMergeInfos.size();
		for (int it = 0; it < mergeInfo.basisMerge.size(); it++)
		{
			controlpointindex[mergeInfo.basisMerge[it]] = index_now;
		}
		bSplineBasis.basisMergeInfos.push_back(mergeInfo);
	}
}


void compute_all_basis(Mesh *mesh, BSplineBasis &bSplineBasis, map<unsigned, KnotData> &dataMaps, 
	vector<TConfig3> &alltconfig3s,vector<Point_2> &supplebasisdata, vector<Point_2> &supplefeaturedata, vector<Point_2> &supplefeaturecurvedata)
{
	update_basic_tconfigurations(bSplineBasis, alltconfig3s, mesh, dataMaps);
	update_basis_support(bSplineBasis, alltconfig3s);
	update_basis_data(bSplineBasis, alltconfig3s, dataMaps);

	replenish_feature_data(bSplineBasis, alltconfig3s, dataMaps, mesh->size_of_vertices(), supplefeaturecurvedata);
	update_basis_data(bSplineBasis, alltconfig3s, dataMaps);

	replenish_feature_data(bSplineBasis, alltconfig3s, dataMaps, mesh->size_of_vertices()+ supplefeaturecurvedata.size(), supplefeaturedata);
	update_basis_data(bSplineBasis, alltconfig3s, dataMaps);

	bool need_supplement = true;
	while (need_supplement)
	{
		if (replenish_basis_data(bSplineBasis, alltconfig3s, dataMaps, mesh->size_of_vertices() + supplefeaturedata.size() + supplefeaturecurvedata.size(), supplebasisdata))
		{
			need_supplement = !(update_basis_data(bSplineBasis, alltconfig3s, dataMaps));
		}
		else
		{
			need_supplement = false;
		}
		std::cout << __FUNCTION__ << ": " << "basis analysis result if it need supplement data: " << need_supplement << std::endl;
	}
}

void update_basic_tconfigurations(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, Mesh *mesh, map<unsigned, KnotData> &dataMaps)
{
	int startSz = tconfigs.size();
	for (int i=0; i<bSplineBasis.basisConfigs.size(); i++)
	{
		for (int j=0; j<bSplineBasis.basisConfigs[i].tconfigs.size(); j++)
		{
			vector<unsigned> vec = bSplineBasis.basisConfigs[i].tconfigs[j].tconfig;
			bool bExist = false;
			
			int it = 0;
			for (; it<tconfigs.size(); it++)
			{
				vector<unsigned int> scr_, temp_;
				scr_ = vec; temp_ = tconfigs[it].tconfig;
				sort(scr_.begin(), scr_.end());
				sort(temp_.begin(), temp_.end());
				if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
				{
					bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index = it;
					bExist = true;
					break;
				}
			}
			if (!bExist)
			{
				TConfig3 config;
				config.tconfig = vec;
				tconfigs.push_back(config);

				it = tconfigs.size() - 1;
				bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index = it;
			}
		}
	}

	#pragma omp parallel for
	for (int it = 0; it < tconfigs.size(); it++)
	{
		compute_tconfig_basis_support(tconfigs[it], mesh, dataMaps);
		compute_tconfig_basis_data(tconfigs[it], dataMaps);
	}
}

void compute_tconfig_basis_data(TConfig3 &tconfig, map<unsigned, KnotData> &dataMaps)
{
	if (!tconfig.supports.empty())
	{
		vector<Point_2> pts;
		for (unsigned i = 0; i<tconfig.tconfig.size(); i++)
			pts.push_back(dataMaps[tconfig.tconfig[i]].pt);

		//test
		/*vector<unsigned> vec1, vec2 = { 0,1,2,203,300 };
		vec1 = tconfig.tconfig;
		sort(vec1.begin(), vec1.end());
		if (equal(vec1.begin(), vec1.end(), vec2.begin()))
		{
			int stop = 1;
		}*/

		double** dataPts = new double*[pts.size()];//[2];
		for (int i = 0; i<pts.size(); i++)
			dataPts[i] = new double[2];

		for (int i = 0; i<pts.size(); i++)
		{
			dataPts[i][0] = pts[i].x();
			dataPts[i][1] = pts[i].y();
		}

		Simplex_Spline_n spline(dataPts, pts.size() - 3, 1, 1);


		map<unsigned, DomainValue>::iterator it = tconfig.supports.begin();
		for (; it != tconfig.supports.end(); it++)
		{
			//test
			/*if (abs(it->second.pt.x() - 1) < 1e-8 && it->second.pt.y() > 0.01 && it->second.pt.y() < 0.99)
			{
			int stop = 1;
			std::cout << __FUNCTION__ << ": " << "a wrong point:" << it->second.pt.x() << "-" << it->second.pt.y() << std::endl;
			}*/
			double pt[2] = { it->second.pt.x(), it->second.pt.y() };
			it->second.value = spline.computeSimplexSpline(pt);


			//test
			if (it->first == 261874)
			{
				int stop1 = 1;
				//std::cout << __FUNCTION__ << ": " << "a wrong point:" << it->second.pt.x() << "-" << it->second.pt.y() << std::endl;
			}

			double x_dir[2] = {1.0,0.0};
			double y_dir[2] = {0.0,1.0};
			it->second.dx = spline.firstdirectionalderivate(pt, x_dir);
			it->second.dy = spline.firstdirectionalderivate(pt, y_dir);
			it->second.dxx = spline.seconddirectionalderivate(pt, x_dir, x_dir);
			it->second.dxy = spline.seconddirectionalderivate(pt, x_dir, y_dir);
			it->second.dyy = spline.seconddirectionalderivate(pt, y_dir, y_dir);
		}

		for (int i = 0; i < pts.size(); i++)
			delete[]dataPts[i];
		delete[]dataPts;
	}
}

void compute_tconfig_basis_support(TConfig3 &tconfig, Mesh *mesh, map<unsigned, KnotData> &dataMaps)
{
	vector<Point_2> pts, convex_hull;

	std::vector<unsigned> indices = tconfig.tconfig;
	std::sort(indices.begin(), indices.end());
	std::vector<unsigned>::iterator it = std::unique(indices.begin(), indices.end());
	indices.erase(it, indices.end());

	for (it = indices.begin(); it != indices.end(); it++)
		pts.push_back(dataMaps[*it].pt);

	CGAL::convex_hull_2(pts.begin(), pts.end(), back_inserter(convex_hull));
	points_on_polygon(mesh, convex_hull, tconfig.supports);
}

void update_basis_support(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs)
{
	for (vector<TConfig3>::iterator it=tconfigs.begin(); it!=tconfigs.end(); it++)
		it->bUsed = false;

#pragma omp parallel for
	for (int i=0; i<bSplineBasis.basisConfigs.size(); i++)
	{
		for (int j=0; j<bSplineBasis.basisConfigs[i].tconfigs.size(); j++)
			tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].bUsed = true;
	}
	
#pragma omp parallel for
	for (int i=0; i<bSplineBasis.basisConfigs.size(); i++)
	{
		bSplineBasis.basisConfigs[i].supports.clear();
		for (int j=0; j<bSplineBasis.basisConfigs[i].tconfigs.size(); j++)
		{
			map<unsigned, DomainValue>::iterator it = tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].supports.begin();
			map<unsigned, DomainValue>::iterator end = tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].supports.end();
			for (; it!=end; it++)
				bSplineBasis.basisConfigs[i].supports.insert(*it);
		}
	}
}

void replenish_feature_data(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, map<unsigned, KnotData> &dataMaps, int current_datasize, vector<Point_2> &suppledata)
{
	int datasize = current_datasize;

	//compute basis value
	BSplineBasis new_bsplinebasis = bSplineBasis;
	vector<TConfig3> new_alltconfig3s = tconfigs;
	new_alltconfig3s.clear();
	update_sample_basic_tconfigurations(new_bsplinebasis, new_alltconfig3s, suppledata, dataMaps);

	//insert into simplex support
#pragma omp parallel for
	for (int k = 0; k < new_alltconfig3s.size(); k++)
	{
		vector<unsigned int> scr_, temp_;
		scr_ = new_alltconfig3s[k].tconfig; temp_ = tconfigs[k].tconfig;
		sort(scr_.begin(), scr_.end());
		sort(temp_.begin(), temp_.end());
		assert(std::equal(scr_.begin(), scr_.end(), temp_.begin()));

		for (auto it = new_alltconfig3s[k].supports.begin(); it != new_alltconfig3s[k].supports.end(); it++)
		{
			int index_now = it->first;
			tconfigs[k].supports[datasize + index_now] = it->second;
		}
	}
	std::cout << __FUNCTION__ << ": " << "supplement feature data finished" << std::endl;

#pragma omp parallel for
	for (int i = 0; i < bSplineBasis.basisConfigs.size(); i++)
	{
		bSplineBasis.basisConfigs[i].supports.clear();
		for (int j = 0; j < bSplineBasis.basisConfigs[i].tconfigs.size(); j++)
		{
			map<unsigned, DomainValue>::iterator it = tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].supports.begin();
			map<unsigned, DomainValue>::iterator end = tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].supports.end();
			for (; it != end; it++)
				bSplineBasis.basisConfigs[i].supports.insert(*it);
		}
	}
}

bool replenish_basis_data(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, map<unsigned, KnotData> &dataMaps,int current_datasize, vector<Point_2> &suppledata)
{
	int datasize = current_datasize;
	vector<Point_2> new_points;
	for (int i = 0; i < bSplineBasis.basisConfigs.size(); i++)
	{
		int numvalid_value = 0;
		for (auto it = bSplineBasis.basisConfigs[i].supports.begin();it != bSplineBasis.basisConfigs[i].supports.end();it++)
		{
			if (it->second.value > 1e-5)
			{
				numvalid_value++;
			}
		}

		if (bSplineBasis.basisConfigs[i].supports.empty() || numvalid_value < 6)
		{
			//compute sample points
			vector<Point_2> basis_poly, convex_;
			for (int j = 0; j < bSplineBasis.basisConfigs[i].tconfigs.size(); j++)
			{
				for (int k = 0; k < bSplineBasis.basisConfigs[i].tconfigs[j].tconfig.size(); k++)
				{
					basis_poly.push_back(dataMaps.at(bSplineBasis.basisConfigs[i].tconfigs[j].tconfig[k]).pt);
				}
			}
			CGAL::convex_hull_2(basis_poly.begin(), basis_poly.end(), back_inserter(convex_));

			double sample_sate = 10.0;
			vector<Point_2> sampleps;
			SamplingPoint2dInPolygonUniform(convex_, sample_sate, sampleps);

			//remove bound points
			for (int k = 0; k < sampleps.size(); k++)
			{
				int ret = CGAL::bounded_side_2(convex_.begin(), convex_.end(), sampleps[k], K());
				if (ret == CGAL::ON_BOUNDED_SIDE && !is_on_polygon_convex_bound(sampleps[k], convex_))
				{
					new_points.push_back(sampleps[k]);
				}
			}
		}
	}
	if (new_points.empty())
	{
		return 0;
	}
	std::cout << __FUNCTION__ << ": " << "generating new data in null-support basis" << std::endl;

	//compute basis value
	BSplineBasis new_bsplinebasis = bSplineBasis;
	vector<TConfig3> new_alltconfig3s = tconfigs;
	new_alltconfig3s.clear();
	update_sample_basic_tconfigurations(new_bsplinebasis, new_alltconfig3s, new_points, dataMaps);
	suppledata.insert(suppledata.end(), new_points.begin(), new_points.end());
	std::cout << __FUNCTION__ << ": " << "degenerated basis sampling" << std::endl;

	//insert into simplex support
#pragma omp parallel for
	for (int k = 0; k < new_alltconfig3s.size(); k++)
	{
		vector<unsigned int> scr_, temp_;
		scr_ = new_alltconfig3s[k].tconfig; temp_ = tconfigs[k].tconfig;
		sort(scr_.begin(), scr_.end());
		sort(temp_.begin(), temp_.end());
		assert(std::equal(scr_.begin(), scr_.end(), temp_.begin()));

		for (auto it = new_alltconfig3s[k].supports.begin(); it != new_alltconfig3s[k].supports.end(); it++)
		{
			int index_now = it->first;
			tconfigs[k].supports[datasize + index_now] = it->second;
		}
	}
	datasize += new_points.size();
	std::cout << __FUNCTION__ << ": " << "degenerated basis get supplement data" << std::endl;

#pragma omp parallel for
	for (int i = 0; i < bSplineBasis.basisConfigs.size(); i++)
	{
		bSplineBasis.basisConfigs[i].supports.clear();
		for (int j = 0; j < bSplineBasis.basisConfigs[i].tconfigs.size(); j++)
		{
			map<unsigned, DomainValue>::iterator it = tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].supports.begin();
			map<unsigned, DomainValue>::iterator end = tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].supports.end();
			for (; it != end; it++)
				bSplineBasis.basisConfigs[i].supports.insert(*it);
		}
	}

	return 1;
}

bool update_basis_data(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, map<unsigned, KnotData> &dataMaps)
{
	//double dataPts[3][2];
	double remove_criterion = 1e-9;
	bool has_degenerated_basis = false;
#pragma omp parallel for
	for (int i = 0; i < bSplineBasis.basisConfigs.size(); i++)
	{
		if (bSplineBasis.basisConfigs[i].supports.empty())
		{
			//std::cout << __FUNCTION__ << ": " << "has null support again after supplement" << std::endl;
			continue;
		}		

		map<unsigned, DomainValue>::iterator it = bSplineBasis.basisConfigs[i].supports.begin();
		map<unsigned, DomainValue>::iterator end = bSplineBasis.basisConfigs[i].supports.end();
		for (; it != end; it++)
		{
			it->second.value = 0;
			it->second.dx = 0;
			it->second.dy = 0;
			it->second.dxx = 0;
			it->second.dxy = 0;
			it->second.dyy = 0;
		}

		vector<bool> has_simplex_witharea0;
		for (int j = 0; j < bSplineBasis.basisConfigs[i].tconfigs.size(); j++)
		{
			/*if (bSplineBasis.basisConfigs[i].tconfigs[j].tconfig[0] == bSplineBasis.basisConfigs[i].tconfigs[j].tconfig[1]
				|| bSplineBasis.basisConfigs[i].tconfigs[j].tconfig[1] == bSplineBasis.basisConfigs[i].tconfigs[j].tconfig[2]
				|| bSplineBasis.basisConfigs[i].tconfigs[j].tconfig[2] == bSplineBasis.basisConfigs[i].tconfigs[j].tconfig[0])
			{
				has_simplex_witharea0.push_back(true);
				continue;
			}
			for (unsigned k = 0; k < BOUNDARYSIZE; k++)
			{
				dataPts[k][0] = dataMaps[bSplineBasis.basisConfigs[i].tconfigs[j].tconfig[k]].pt.x();
				dataPts[k][1] = dataMaps[bSplineBasis.basisConfigs[i].tconfigs[j].tconfig[k]].pt.y();
			}
			double area_ = std::abs(orient2d(dataPts[0], dataPts[1], dataPts[2])) / 2.0;
			if (area_ < std::numeric_limits<double>::epsilon())
			{
				has_simplex_witharea0.push_back(true);
				continue;
			}*/
			double area_ = bSplineBasis.basisConfigs[i].tconfigs[j].tri_area;
			if (area_ < std::numeric_limits<double>::epsilon())
			{
				has_simplex_witharea0.push_back(true);
				continue;
			}
			has_simplex_witharea0.push_back(false);
			map<unsigned, DomainValue>::iterator itr = tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].supports.begin();
			map<unsigned, DomainValue>::iterator endr = tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].supports.end();
			for (; itr != endr; itr++)
			{
				bSplineBasis.basisConfigs[i].supports[itr->first].value += area_*itr->second.value;
				bSplineBasis.basisConfigs[i].supports[itr->first].dx += area_*itr->second.dx;
				bSplineBasis.basisConfigs[i].supports[itr->first].dy += area_*itr->second.dy;
				bSplineBasis.basisConfigs[i].supports[itr->first].dxx += area_*itr->second.dxx;
				bSplineBasis.basisConfigs[i].supports[itr->first].dxy += area_*itr->second.dxy;
				bSplineBasis.basisConfigs[i].supports[itr->first].dyy += area_*itr->second.dyy;
			}
		}

		bool is_normal_degenerated = true;
		for (int k = 0;k<has_simplex_witharea0.size();k++)
		{
			if (has_simplex_witharea0[k] == false)
			{
				is_normal_degenerated = false;
				break;
			}
		}
	
		bool isallzero = true;
		map<unsigned, DomainValue>::iterator ifrom = bSplineBasis.basisConfigs[i].supports.begin();
		map<unsigned, DomainValue>::iterator iend = bSplineBasis.basisConfigs[i].supports.end();
		for (; ifrom != iend; ifrom++)
		{
			if (ifrom->second.value > remove_criterion /*||
				ifrom->second.dx > remove_criterion ||
				ifrom->second.dy > remove_criterion ||
				ifrom->second.dxx > remove_criterion ||
				ifrom->second.dxy > remove_criterion ||
				ifrom->second.dyy > remove_criterion*/)
			{
				isallzero = false;
				break;
			}
		}

		if (isallzero)
		{
			if (!is_normal_degenerated)
			{
				has_degenerated_basis = true;
			}
			bSplineBasis.basisConfigs[i].supports.clear();
		}
	}

	std::cout << __FUNCTION__ << ": " << "update basis data" << std::endl;
	return !has_degenerated_basis;
}


void compute_sample_basis(vector<Point_2> &sample_points, BSplineBasis &bSplineBasis, map<unsigned, KnotData> &dataMaps, vector<TConfig3> &alltconfig3s)
{
	update_sample_basic_tconfigurations(bSplineBasis, alltconfig3s, sample_points, dataMaps);
	update_basis_support(bSplineBasis, alltconfig3s);
	update_basis_data(bSplineBasis, alltconfig3s, dataMaps);
}

void update_sample_basic_tconfigurations(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, vector<Point_2> &sample_point, map<unsigned, KnotData> &dataMaps)
{
	for (int i = 0; i < bSplineBasis.basisConfigs.size(); i++)
	{
		for (int j = 0; j < bSplineBasis.basisConfigs[i].tconfigs.size(); j++)
		{
			vector<unsigned> vec = bSplineBasis.basisConfigs[i].tconfigs[j].tconfig;
			bool bExist = false;

			int it = 0;
			for (; it < tconfigs.size(); it++)
			{
				vector<unsigned int> scr_, temp_;
				scr_ = vec; temp_ = tconfigs[it].tconfig;
				sort(scr_.begin(), scr_.end());
				sort(temp_.begin(), temp_.end());
				if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
				{
					bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index = it;
					bExist = true;
					break;
				}
			}
			if (!bExist)
			{
				TConfig3 config;
				config.tconfig = vec;
				tconfigs.push_back(config);

				it = tconfigs.size() - 1;
				bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index = it;
			}
		}
	}

	//std::cout << __FUNCTION__ << ": " << "tconfig f" << std::endl;
	#pragma omp parallel for
	for (int it = 0; it < tconfigs.size(); it++)
	{
		//std::cout << __FUNCTION__ << ": " << "tconfig for:"<<it << std::endl;
		compute_tconfig_sample_basis_support(tconfigs[it], sample_point, dataMaps);
		compute_tconfig_basis_data(tconfigs[it], dataMaps);
	}
}

void compute_tconfig_sample_basis_support(TConfig3 &tconfig, vector<Point_2> &sample_point, map<unsigned, KnotData> &dataMaps)
{
	vector<Point_2> pts, convex_hull;

	std::vector<unsigned> indices = tconfig.tconfig;
	std::sort(indices.begin(), indices.end());
	std::vector<unsigned>::iterator it = std::unique(indices.begin(), indices.end());
	indices.erase(it, indices.end());

	for (it = indices.begin(); it != indices.end(); it++)
		pts.push_back(dataMaps[*it].pt);

	CGAL::convex_hull_2(pts.begin(), pts.end(), back_inserter(convex_hull));
	points_on_polygon(sample_point, convex_hull, tconfig.supports);
}


void compute_basis_derivate_forG1(vector<SurfaceTangentForG1> &inputdata, BSplineBasis &bSplineBasis, map<unsigned, KnotData> &dataMaps, vector<TConfig3> &alltconfig3s)
{
	BSplineBasis new_bsplinebasis = bSplineBasis;
	vector<TConfig3> new_alltconfig3s = alltconfig3s;
	new_alltconfig3s.clear();

	update_basis_tconfigurations_forG1(new_bsplinebasis, new_alltconfig3s, inputdata, dataMaps);
	update_basis_supportanddata_forG1(new_bsplinebasis, new_alltconfig3s, dataMaps);

	//need something
}

void update_basis_tconfigurations_forG1(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, vector<SurfaceTangentForG1> &inputdata, map<unsigned, KnotData> &dataMaps)
{
	for (int i = 0; i < bSplineBasis.basisConfigs.size(); i++)
	{
		for (int j = 0; j < bSplineBasis.basisConfigs[i].tconfigs.size(); j++)
		{
			vector<unsigned> vec = bSplineBasis.basisConfigs[i].tconfigs[j].tconfig;
			bool bExist = false;

			int it = 0;
			for (; it < tconfigs.size(); it++)
			{
				vector<unsigned int> scr_, temp_;
				scr_ = vec; temp_ = tconfigs[it].tconfig;
				sort(scr_.begin(), scr_.end());
				sort(temp_.begin(), temp_.end());
				if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
				{
					bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index = it;
					bExist = true;
					break;
				}
			}
			if (!bExist)
			{
				TConfig3 config;
				config.tconfig = vec;
				tconfigs.push_back(config);

				it = tconfigs.size() - 1;
				bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index = it;
			}
		}
	}

#pragma omp parallel for
	for (int it = 0; it < tconfigs.size(); it++)
	{
		vector<Point_2> pts, convex_hull;

		std::vector<unsigned> indices = tconfigs[it].tconfig;
		std::sort(indices.begin(), indices.end());
		std::vector<unsigned>::iterator ik = std::unique(indices.begin(), indices.end());
		indices.erase(ik, indices.end());

		for (ik = indices.begin(); ik != indices.end(); ik++)
			pts.push_back(dataMaps[*ik].pt);

		CGAL::convex_hull_2(pts.begin(), pts.end(), back_inserter(convex_hull));
		points_on_polygon(inputdata, convex_hull, tconfigs[it].supports);

		compute_tconfig_basis_data_forG1(tconfigs[it], dataMaps);
	}
}

void compute_tconfig_basis_data_forG1(TConfig3 &tconfig, map<unsigned, KnotData> &dataMaps)
{
	if (!tconfig.supports.empty())
	{
		vector<Point_2> pts;
		for (unsigned i = 0; i<tconfig.tconfig.size(); i++)
			pts.push_back(dataMaps[tconfig.tconfig[i]].pt);
		double** dataPts = new double*[pts.size()];//[2];
		for (int i = 0; i<pts.size(); i++)
			dataPts[i] = new double[2];

		for (int i = 0; i<pts.size(); i++)
		{
			dataPts[i][0] = pts[i].x();
			dataPts[i][1] = pts[i].y();
		}

		Simplex_Spline_n spline(dataPts, pts.size() - 3, 1, 1);

		map<unsigned, DomainValue>::iterator it = tconfig.supports.begin();
		for (; it != tconfig.supports.end(); it++)
		{
			double pt[2] = { it->second.pt.x(), it->second.pt.y() };
			double dir1[2] = {it->second.dir1.x(),it->second.dir1.y() };
			double dir2[2] = { it->second.dir2.x(),it->second.dir2.y() };
			it->second.value_dir1 = spline.firstdirectionalderivate(pt, dir1);
			it->second.value_dir2 = spline.firstdirectionalderivate(pt, dir2);
		}

		for (int i = 0; i < pts.size(); i++)
			delete[]dataPts[i];
		delete[]dataPts;
	}
}

void update_basis_supportanddata_forG1(BSplineBasis &bSplineBasis, vector<TConfig3> &tconfigs, map<unsigned, KnotData> &dataMaps)
{
	for (vector<TConfig3>::iterator it = tconfigs.begin(); it != tconfigs.end(); it++)
		it->bUsed = false;

	for (int i = 0; i < bSplineBasis.basisConfigs.size(); i++)
	{
		for (int j = 0; j < bSplineBasis.basisConfigs[i].tconfigs.size(); j++)
			tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].bUsed = true;
	}

	for (int i = 0; i < bSplineBasis.basisConfigs.size(); i++)
	{
		bSplineBasis.basisConfigs[i].supports.clear();
		for (int j = 0; j < bSplineBasis.basisConfigs[i].tconfigs.size(); j++)
		{
			map<unsigned, DomainValue>::iterator it = tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].supports.begin();
			map<unsigned, DomainValue>::iterator end = tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].supports.end();
			for (; it != end; it++)
				bSplineBasis.basisConfigs[i].supports.insert(*it);
		}
	}

	double dataPts[3][2];
	double dt;
	for (int i = 0; i < bSplineBasis.basisConfigs.size(); i++)
	{
		if (bSplineBasis.basisConfigs[i].supports.empty())
		{
			continue;
		}

		map<unsigned, DomainValue>::iterator it = bSplineBasis.basisConfigs[i].supports.begin();
		map<unsigned, DomainValue>::iterator end = bSplineBasis.basisConfigs[i].supports.end();
		for (; it != end; it++)
		{
			it->second.value_dir1 = 0;
			it->second.value_dir2 = 0;
		}

		vector<bool> has_simplex_witharea0;
		for (int j = 0; j < bSplineBasis.basisConfigs[i].tconfigs.size(); j++)
		{
			dt = bSplineBasis.basisConfigs[i].tconfigs[j].tri_area;
			if (dt < std::numeric_limits<double>::epsilon())
			{
				continue;
			}
			map<unsigned, DomainValue>::iterator itr = tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].supports.begin();
			map<unsigned, DomainValue>::iterator endr = tconfigs[bSplineBasis.basisConfigs[i].tconfigs[j].allconfigs_index].supports.end();
			for (; itr != endr; itr++)
			{
				bSplineBasis.basisConfigs[i].supports[itr->first].value_dir1 += dt*itr->second.value_dir1;
				bSplineBasis.basisConfigs[i].supports[itr->first].value_dir2 += dt*itr->second.value_dir2;
			}
		}
	}
}


void compute_simplex_spline(vector<Point_2> simplexps, vector<Point_2> sampleps, vector<DomainValue> &values)
{
	vector<Point_2> convex_;
	CGAL::convex_hull_2(simplexps.begin(), simplexps.end(), back_inserter(convex_));

	//simplex spline
	double** dataPts = new double*[simplexps.size()];//[2];
	for (int i = 0; i < simplexps.size(); i++)
		dataPts[i] = new double[2];
	for (int i = 0; i < simplexps.size(); i++)
	{
		dataPts[i][0] = simplexps[i].x();
		dataPts[i][1] = simplexps[i].y();
	}
	Simplex_Spline_n spline(dataPts, simplexps.size() - 3, 1, 1);

	//box predicate
	vector<int> vIndices;
	vector<double> box_ = return_box2d_from_vector_point(simplexps);
	for (auto k = 0;k<sampleps.size();k++)
	{
		if (sampleps[k].x() >= (box_[0] - 1e-8) && sampleps[k].x() <= (box_[1] + 1e-8)
			&& sampleps[k].y() >= (box_[2] - 1e-8) && sampleps[k].y() <= (box_[3] + 1e-8))
		{
			vIndices.push_back(k);
		}
	}
	//convex predicate and compute simplex
	map<int, DomainValue> allsupport;
	for (int i = 0; i < vIndices.size(); i++)
	{
		int ret = CGAL::bounded_side_2(convex_.begin(), convex_.end(), sampleps[vIndices[i]], K());
		if (ret != CGAL::ON_UNBOUNDED_SIDE || is_on_polygon_convex_bound(sampleps[vIndices[i]], convex_))
		{
			DomainValue dv;
			dv.pt = sampleps[vIndices[i]];
			double pt[2] = { dv.pt.x(),dv.pt.y() };
			double value_ = spline.computeSimplexSpline(pt);
			double xdirm[2] = { 1.0,0.0 };
			double ydirm[2] = { 0.0,1.0 };
			double dx = spline.firstdirectionalderivate(pt, xdirm);
			double dy = spline.firstdirectionalderivate(pt, ydirm);
			double dxx = spline.seconddirectionalderivate(pt, xdirm, xdirm);
			double dxy = spline.seconddirectionalderivate(pt, xdirm, ydirm);
			double dyy = spline.seconddirectionalderivate(pt, ydirm, ydirm);
			dv.value = value_; dv.dx = dx; dv.dy = dy; dv.dxx = dxx; dv.dxy = dxy; dv.dyy = dyy;
			allsupport[vIndices[i]] = dv;
		}
	}

	for (int i = 0; i < simplexps.size(); i++)
		delete[]dataPts[i];
	delete[]dataPts;

	values.clear();
	values.resize(sampleps.size());
	for (int i = 0; i < values.size(); i++)
	{
		DomainValue dv;
		dv.value = 0.0; dv.dx = 0.0; dv.dy = 0.0; dv.dxx = 0.0; dv.dxy = 0.0; dv.dyy = 0.0;
		map<int, DomainValue>::iterator res = allsupport.find(i);
		if (res != allsupport.end())
		{
			dv = res->second;
		}
		values[i] = dv;
	}
}

bool generate_coplanar_condition(Mesh *mesh, BSplineBasis &bSplineBasis, map<unsigned, KnotData> &dataMaps)
{
	int deg = bSplineBasis.deg;
	for (unsigned i = 0; i < bSplineBasis.cpInfos.size(); i++)
	{
		unsigned id = bSplineBasis.cpInfos[i].cornerId;
		for (unsigned j = 0; j < bSplineBasis.basisMergeInfos.size(); j++)
		{
			if (bSplineBasis.basisMergeInfos[j].type != 2)
			{
				bool bInsert = false;
				for (unsigned k = 0; k < bSplineBasis.basisMergeInfos[j].basisMerge.size(); k++)
				{
					int loc = bSplineBasis.basisMergeInfos[j].basisMerge[k];
					if (!bSplineBasis.basisConfigs[loc].supports.empty())
					{
						for (unsigned l = 0; l < bSplineBasis.basisConfigs[loc].tconfigs.size(); l++)
						{
							if (bSplineBasis.basisConfigs[loc].tconfigs[l].tri_area<1e-15)
							{
								continue;
							}
							if (deg == number_multiple_knot(bSplineBasis.basisConfigs[loc].tconfigs[l].tconfig, id))
							{
								bSplineBasis.cpInfos[i].c0Ids.push_back(j);
								bInsert = true;
								break;
							}
						}

						if (bInsert)
							break;
					}
				}
			}
		}
	}

	vector<CoplanarInfo> cpInfos;
	for (unsigned i=0; i<bSplineBasis.cpInfos.size(); i++)
	{
		if (bSplineBasis.cpInfos[i].c0Ids.size()>2)
			cpInfos.push_back(bSplineBasis.cpInfos[i]);
	}
	
	bSplineBasis.cpInfos = cpInfos;

	for (unsigned i = 0; i < bSplineBasis.cpInfos.size(); i++)
	{
		for (Vertex_iterator vit = mesh->vertices_begin(); vit != mesh->vertices_end(); vit++)
		{
			if ((vit->get_domain() - dataMaps[bSplineBasis.cpInfos[i].cornerId].pt).squared_length()
				< std::numeric_limits<double>::epsilon())
			{
				bSplineBasis.cpInfos[i].normal = vit->normal();
				break;
			}
		}
	}
	return true;
}
