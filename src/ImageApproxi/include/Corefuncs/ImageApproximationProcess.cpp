#include "Corefuncs/ImageApproximation.h"

//fitting process

void Image_Approximation::compute_image_feature()
{
	if (has_compute_feature)
	{
		return;
	}

#if 1
	//***************************** ED Edge Segment Detection *****************************
	//Detection of edge segments from an input image	
	cv::Mat testImg = cv::imread(fileName.toStdString(), 0);
	//Call ED constructor
	ED testED = ED(testImg, SOBEL_OPERATOR, gradient_threshold, 
		anchor_threshold, anchor_width, min_legth_feature, 1.0, true); // apply ED algorithm--36-8-4

	image_anchor = testED.getAnchorPoints();
																	   //Output number of segments
	int noSegments = testED.getSegmentNo();
	printf("%s: Number of image feature edge segments:%i \n", __FUNCTION__, noSegments);
	//Get edges in segment form (getSortedSegments() gives segments sorted w.r.t. legnths) 
	vector<vector<cv::Point> > segments = testED.getSegments();

	if (do_consider_cirlcewane)
	{
		for (int i = 0;i<segments.size();i++)
		{
			if (segments[i].size()>2 && segments[i].size()<20)
			{
				if(abs(segments[i][0].x-segments[i][segments[i].size()-1].x)<=1 &&
					abs(segments[i][0].y - segments[i][segments[i].size() - 1].y) <= 1)
				{
					Polygon_2 polygon_;
					for (int j = 0; j < segments[i].size(); j++)
					{
						Point_2 p(segments[i][j].x,segments[i][j].y);
						polygon_.push_back(p);
					}
					vector<cv::Point> unsorted_seg,sorted_seg;
					for (int j = 0; j < segments[i].size(); j++)
					{
						cv::Point pleft(segments[i][j].x-1, segments[i][j].y);
						cv::Point pright(segments[i][j].x+1, segments[i][j].y);
						cv::Point pup(segments[i][j].x, segments[i][j].y+1);
						cv::Point pdown(segments[i][j].x, segments[i][j].y-1);
						
						vector<cv::Point>::iterator res0 = find(unsorted_seg.begin(), unsorted_seg.end(), pleft);
						vector<cv::Point>::iterator res1 = find(segments[i].begin(), segments[i].end(), pleft);
						if (res0 == unsorted_seg.end() && res1 == segments[i].end())
						{
							if (polygon_.bounded_side(Point_2(segments[i][j].x - 1, segments[i][j].y)) == CGAL::ON_UNBOUNDED_SIDE)
								unsorted_seg.push_back(pleft);
						}

						vector<cv::Point>::iterator res4 = find(unsorted_seg.begin(), unsorted_seg.end(), pup);
						vector<cv::Point>::iterator res5 = find(segments[i].begin(), segments[i].end(), pup);
						if (res4 == unsorted_seg.end() && res5 == segments[i].end())
						{
							if (polygon_.bounded_side(Point_2(segments[i][j].x, segments[i][j].y + 1)) == CGAL::ON_UNBOUNDED_SIDE)
								unsorted_seg.push_back(pup);
						}

						vector<cv::Point>::iterator res2 = find(unsorted_seg.begin(), unsorted_seg.end(), pright);
						vector<cv::Point>::iterator res3 = find(segments[i].begin(), segments[i].end(), pright);
						if (res2 == unsorted_seg.end() && res3 == segments[i].end())
						{
							if (polygon_.bounded_side(Point_2(segments[i][j].x + 1, segments[i][j].y)) == CGAL::ON_UNBOUNDED_SIDE)
								unsorted_seg.push_back(pright);
						}						

						vector<cv::Point>::iterator res6 = find(unsorted_seg.begin(), unsorted_seg.end(), pdown);
						vector<cv::Point>::iterator res7 = find(segments[i].begin(), segments[i].end(), pdown);
						if (res6 == unsorted_seg.end() && res7 == segments[i].end())
						{
							if (polygon_.bounded_side(Point_2(segments[i][j].x, segments[i][j].y-1)) == CGAL::ON_UNBOUNDED_SIDE)
								unsorted_seg.push_back(pdown);
						}
					}
					//sort
					if (!unsorted_seg.empty())
					{
						vector<cv::Point> out;
						if (find_onering_pixels(unsorted_seg[0], unsorted_seg, out))
						{
							sorted_seg.push_back(out[0]);
							sorted_seg.push_back(unsorted_seg[0]);
							sorted_seg.push_back(out[1]);
						}
						int it = 2;
						do 
						{
							vector<cv::Point> out1;
							if (find_onering_pixels(sorted_seg[it],unsorted_seg,out1))
							{
								vector<cv::Point>::iterator res0 = find(sorted_seg.begin(), sorted_seg.end(), out1[0]);		
								if (res0 == sorted_seg.end())
								{
									sorted_seg.push_back(out1[0]);
								}
								vector<cv::Point>::iterator res1 = find(sorted_seg.begin(), sorted_seg.end(), out1[1]);
								if (res1 == sorted_seg.end())
								{
									sorted_seg.push_back(out1[1]);
								}
							}
							it++;
							if (it>unsorted_seg.size()-1)
							{
								break;
							}
						} while (sorted_seg.size() < unsorted_seg.size());
					}
					segments[i] = sorted_seg;
				}
			}
		}
	}
#endif

	//cv::Mat cm_out;
	//// Apply the colormap:
	//cv::applyColorMap(testImg, cm_out, cv::COLORMAP_JET);
	//// Show the result:
	//cv::imshow("colormap out", cm_out);

#if 0
	//*********************** EDCOLOR Edge Segment Detection from Color Images **********************
	cv::Mat colorImg = cv::imread(fileName.toStdString());
	EDColor testEDColor = EDColor(colorImg, 36, 8, 1.0, true); //last parameter for validation
															   //Output number of segments
	int nocSegments = testEDColor.getSegmentNo();
	std::cout << "Number of image feature edge segments: " << nocSegments << std::endl;
	//Get edges in segment form (getSortedSegments() gives segments sorted w.r.t. legnths) 
	std::vector< std::vector<cv::Point> > segments = testEDColor.getSegments();
#endif

	//normalize
	int lengthw = input_image->width();
	int lengthh = input_image->height();
	double step_ = 2.0*IMAGEWIDTHSIZE / (lengthw - 1);

	vector<vector<PointTriple>> lines_;
	int size_pixels_fea = 0;
	for (int i = 0; i<segments.size(); i++)
	{
		vector<PointTriple> line_;
		for (int j = 0; j<segments[i].size(); j++)
		{
			double y = double(lengthh) - 1 - segments[i][j].y;
			Point_2 pimage_(segments[i][j].x, y);

			//image index from 0 to image-w/h-1
			double x_1 = step_ * segments[i][j].x;
			double y_1 = step_ * y / (2.0*imagemesh_dy);
			Point_2 pdomain_(x_1, y_1);

			PointTriple p_temp;
			p_temp.index_x = segments[i][j].x;
			p_temp.index_y = lengthh - 1 - segments[i][j].y;
			p_temp.imagedomain = pdomain_;
			p_temp.imagep = pimage_;
			p_temp.is_fixed = false;
			line_.push_back(p_temp);
		}
		size_pixels_fea += line_.size();
		lines_.push_back(line_);
	}
	std::cout << "pixel feature length: " << step_ << std::endl;
	std::cout << "total feature length: " << size_pixels_fea*step_ << std::endl;

#if 1
	//plus boundary
	for (int i = 0; i < lines_.size(); i++)
	{
		//head
		PointTriple p_temp;
		if (lines_[i][0].index_x == 1)
		{
			p_temp.index_y = lines_[i][0].index_y;
			p_temp.index_x = 0;
			p_temp.imagedomain = Point_2(0.0, lines_[i][0].imagedomain.y());
			p_temp.imagep = Point_2(0.0, lines_[i][0].imagep.y());
			p_temp.is_fixed = false;
			lines_[i].insert(lines_[i].begin(), p_temp);
		}
		else if (lines_[i][0].index_y == 1)
		{
			p_temp.index_y = 0;
			p_temp.index_x = lines_[i][0].index_x;
			p_temp.imagedomain = Point_2(lines_[i][0].imagedomain.x(), 0.0);
			p_temp.imagep = Point_2(lines_[i][0].imagep.x(), 0.0);
			p_temp.is_fixed = false;
			lines_[i].insert(lines_[i].begin(), p_temp);
		}
		else if (lines_[i][0].index_x == input_image->width() - 2)
		{
			p_temp.index_y = lines_[i][0].index_y;
			p_temp.index_x = input_image->width() - 1;
			p_temp.imagedomain = Point_2(1.0, lines_[i][0].imagedomain.y());
			p_temp.imagep = Point_2(lines_[i][0].imagep.x() + 1.0, lines_[i][0].imagep.y());
			p_temp.is_fixed = false;
			lines_[i].insert(lines_[i].begin(), p_temp);
		}
		else if (lines_[i][0].index_y == input_image->height() - 2)
		{
			p_temp.index_y = input_image->height() - 1;
			p_temp.index_x = lines_[i][0].index_x;
			p_temp.imagedomain = Point_2(lines_[i][0].imagedomain.x(), 1.0);
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
		else if (lines_[i][lines_[i].size() - 1].index_y == input_image->height() - 2)
		{
			p_temp.index_y = input_image->height() - 1;
			p_temp.index_x = lines_[i][lines_[i].size() - 1].index_x;
			p_temp.imagedomain = Point_2(lines_[i][lines_[i].size() - 1].imagedomain.x(), 1.0);
			p_temp.imagep = Point_2(lines_[i][lines_[i].size() - 1].imagep.x(), lines_[i][lines_[i].size() - 1].imagep.y() + 1.0);
			p_temp.is_fixed = false;
			lines_[i].push_back(p_temp);
		}

	}
	image_feature_lines_set = lines_;
#endif

#if 1
	//feature flag
	// build KD-tree
	int				k = 1;									// number of nearest neighbors
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
				vit->feature_type() = i + 1;
				image_feature_lines_set[i][j].vit = vit;
			}
			else
			{
				printf("%s: KD-tree search wrong: no finding \n", __FUNCTION__);
			}

			delete[] nnIdx_temp;							// clean things up
			delete[] dists_temp;
		}
	}

	delete kdTree_temp;

	feature_lines_modify = image_feature_lines_set;

	//classify feature points
	for (auto vit = originalMesh->vertices_begin(); vit != originalMesh->vertices_end(); vit++)
	{
		vector<int> inte = { vit->vertex_index() };
		vector<int> one_ring;
		from_interior_to_ccw_one_ring_cycle(inte, one_ring, &originalMesh);
		bool is_adjacent_tofeature = false;
		for (int i = 0; i<one_ring.size(); i++)
		{
			Mesh::Vertex_iterator vt = originalMesh->get_vertex_iterator(one_ring[i]);
			if (vt->is_feature())
			{
				is_adjacent_tofeature = true;
				break;
			}
		}
		if (vit->is_feature())
		{
			vit->tag() = 0;
		}
		else if (is_adjacent_tofeature)
		{
			vit->tag() = 1;
		}
		else
		{
			vit->tag() = -1;
		}
	}
	for (auto vit = originalMesh->vertices_begin(); vit != originalMesh->vertices_end(); vit++)
	{
		vector<int> inte = { vit->vertex_index() };
		vector<int> one_ring;
		from_interior_to_ccw_one_ring_cycle(inte, one_ring, &originalMesh);
		bool is_adjacent_tofeature = false;
		for (int i = 0; i < one_ring.size(); i++)
		{
			Mesh::Vertex_iterator vt = originalMesh->get_vertex_iterator(one_ring[i]);
			if (vt->tag() == 0 || vt->tag() == 1)
			{
				is_adjacent_tofeature = true;
				break;
			}
		}
		if (is_adjacent_tofeature && vit->tag() == -1)
		{
			vit->tag() = 2;
		}	
	}
	has_compute_feature = true;

	//test
	//QString dirub = out_file_name;
	//dirub.append("/");
	//dirub.append("feature_points.txt");
	//std::ofstream fout_ub;
	//fout_ub.open(dirub.toStdString());
	//if (fout_ub.is_open())
	//{
	//	vector<pair<int, int>> indexvec;
	//	for (int i = 0; i < image_feature_lines_set.size(); i++)
	//	{
	//		if (abs(image_feature_lines_set[i][0].index_x - image_feature_lines_set[i][image_feature_lines_set[i].size() - 1].index_x) < 2 &&
	//			abs(image_feature_lines_set[i][0].index_y - image_feature_lines_set[i][image_feature_lines_set[i].size() - 1].index_y) < 2)
	//		{
	//			indexvec.push_back(pair<int, int>(i, image_feature_lines_set[i].size()));
	//			//std::cout << "index: " << i << "- " << image_feature_lines_set[i].size() << std::endl;
	//		}
	//	}
	//	int index = 0;
	//	int size = -1;
	//	for (int k = 0; k < indexvec.size(); k++)
	//	{
	//		if (indexvec[k].second > size)
	//		{
	//			size = indexvec[k].second;
	//			index = indexvec[k].first;
	//		}
	//	}
	//	for (int j = 0; j < image_feature_lines_set[index].size(); j++)
	//	{
	//		fout_ub << image_feature_lines_set[index][j].imagedomain.x() - imagemesh_dx << " "
	//			<< image_feature_lines_set[index][j].imagedomain.y()*imagemesh_dy * 2 - imagemesh_dy << "\n";
	//	}
	//}
	//fout_ub.close();
	printf("%s: Image feature lines done \n", __FUNCTION__);
#endif
}

void Image_Approximation::smooth_and_sample_feature_curve_points()
{
#if 0
	//curve fitting
	QString dirc = out_file_name;
	dirc.append("/");
	dirc.append("fitted_points.txt");
	std::ifstream fin_0;
	fin_0.open(dirc.toStdString());
	vector<RGBPoint> fittedps;
	if (fin_0.is_open())
	{
		double x_;
		while (fin_0 >> x_)
		{
			double y_;
			fin_0 >> y_;
			RGBPoint p;
			p.x = x_; p.y = y_;
			featurecurve_originalps.push_back(p);
		}
	}
	fin_0.close();
	std::cout << "read test" << std::endl;

	for (int i = 0; i<feature_lines_modify[0].size(); i++)
	{
		Point_2 pt(feature_lines_modify[0][i].paradomain);
		featurecurve_sampleps.push_back(pt);
		if (i<feature_lines_modify[0].size() - 1)
		{
			Point_2 pplus(feature_lines_modify[0][i + 1].paradomain);

			for (int k = 1; k<8; k++)
			{
				Point_2 pmid1((k * pt.x() + (8.0 - k) * pplus.x()) / 8.0, (k * pt.y() + (8.0 - k) * pplus.y()) / 8.0);
				featurecurve_sampleps.push_back(pmid1);
			}
		}
	}
#endif

#if 0
	vector<pair<bool, vector<Point_2>>> smooth_samplesps;
	vector<pair<bool, vector<RGBPoint>>> smooth_originalps;

	int size_add = 3;
	double midshift_weight = 1.0;
	double shift_weight = 1.0;

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
			Point_2 pt(feature_lines_modify[k][i].paradomain);
			samplesps.push_back(pt);
		}
		smooth_samplesps.push_back(pair<bool, vector<Point_2>>(is_circle, samplesps));
	}
	for (int n = 0; n<smooth_samplesps.size(); n++)
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
				for (int i = 0; i < smooth_samplesps[n].second.size() - 1; i++)
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
	}

	for (int i = 0; i < feature_lines_modify.size(); i++)
	{
		vector<RGBPoint> originalps;
		bool is_circle = false;
		if (abs(feature_lines_modify[i][0].index_x - feature_lines_modify[i][feature_lines_modify[i].size() - 1].index_x) < 2 &&
			abs(feature_lines_modify[i][0].index_y - feature_lines_modify[i][feature_lines_modify[i].size() - 1].index_y) < 2)
		{
			is_circle = true;
		}
		for (int j = 0; j < feature_lines_modify[i].size(); j++)
		{

			double x = feature_lines_modify[i][j].imagedomain.x() - imagemesh_dx;
			double y = feature_lines_modify[i][j].imagedomain.y()*imagemesh_dy * 2 - imagemesh_dy;
			RGBPoint p;
			p.x = x; p.y = y;
			originalps.push_back(p);
		}
		smooth_originalps.push_back(pair<bool, vector<RGBPoint>>(is_circle, originalps));
	}

	for (int n = 0; n < smooth_originalps.size(); n++)
	{
		vector<RGBPoint> new_points;
		if (smooth_originalps[n].first)
		{
			//circle 
			for (int k = 0; k < size_add; k++)
			{
				new_points.clear();
				//add new mid points
				for (int i = 0; i < smooth_originalps[n].second.size(); i++)
				{
					new_points.push_back(smooth_originalps[n].second[i]);
					RGBPoint ppre1 = smooth_originalps[n].second[i];
					RGBPoint ppre2 = smooth_originalps[n].second[(i - 1 + smooth_originalps[n].second.size()) % smooth_originalps[n].second.size()];
					RGBPoint pnex1 = smooth_originalps[n].second[(i + 1) % smooth_originalps[n].second.size()];
					RGBPoint pnex2 = smooth_originalps[n].second[(i + 2) % smooth_originalps[n].second.size()];
					double midx = (smooth_originalps[n].second[(i + 1) % smooth_originalps[n].second.size()].x + smooth_originalps[n].second[i].x) / 2.0;
					double midy = (smooth_originalps[n].second[(i + 1) % smooth_originalps[n].second.size()].y + smooth_originalps[n].second[i].y) / 2.0;
					RGBPoint pc; pc.x = midx; pc.y = midy;

					double x = pc.x*(1 - midshift_weight) + midshift_weight / 4.0*(ppre1.x + ppre2.x + pnex1.x + pnex2.x);
					double y = pc.y*(1 - midshift_weight) + midshift_weight / 4.0*(ppre1.y + ppre2.y + pnex1.y + pnex2.y);
					RGBPoint p;
					p.x = x; p.y = y;
					new_points.push_back(p);
				}
				smooth_originalps[n].second = new_points;
				new_points.clear();
				//move other points
				for (int i = 0; i < smooth_originalps[n].second.size(); i++)
				{
					RGBPoint ppre1 = smooth_originalps[n].second[(i - 1 + smooth_originalps[n].second.size()) % smooth_originalps[n].second.size()];
					RGBPoint ppre2 = smooth_originalps[n].second[(i - 2 + smooth_originalps[n].second.size()) % smooth_originalps[n].second.size()];
					RGBPoint pc = smooth_originalps[n].second[i];
					RGBPoint pnex1 = smooth_originalps[n].second[(i + 1) % smooth_originalps[n].second.size()];
					RGBPoint pnex2 = smooth_originalps[n].second[(i + 2) % smooth_originalps[n].second.size()];
					double x = pc.x*(1 - shift_weight) + shift_weight / 4.0*(ppre1.x + ppre2.x + pnex1.x + pnex2.x);
					double y = pc.y*(1 - shift_weight) + shift_weight / 4.0*(ppre1.y + ppre2.y + pnex1.y + pnex2.y);
					RGBPoint p;
					p.x = x; p.y = y;
					new_points.push_back(p);
				}
				smooth_originalps[n].second.clear();
				smooth_originalps[n].second = new_points;
			}
		}
		else
		{
			//normal
			for (int k = 0; k < size_add; k++)
			{
				new_points.clear();
				//add new mid points
				for (int i = 0; i < smooth_originalps[n].second.size() - 1; i++)
				{
					new_points.push_back(smooth_originalps[n].second[i]);
					if (i == 0)
					{
						double midx = (smooth_originalps[n].second[i + 1].x + smooth_originalps[n].second[i].x) / 2.0;
						double midy = (smooth_originalps[n].second[i + 1].y + smooth_originalps[n].second[i].y) / 2.0;
						RGBPoint pc; pc.x = midx; pc.y = midy;

						RGBPoint ppre1 = smooth_originalps[n].second[i];
						RGBPoint pnex1 = smooth_originalps[n].second[i + 1];
						RGBPoint pnex2 = smooth_originalps[n].second[i + 2];

						double x = pc.x*(1 - midshift_weight) + midshift_weight / 3.0*(ppre1.x + pnex1.x + pnex2.x);
						double y = pc.y*(1 - midshift_weight) + midshift_weight / 3.0*(ppre1.y + pnex1.y + pnex2.y);
						RGBPoint p;
						p.x = x; p.y = y;
						new_points.push_back(p);
					}
					else if (i == smooth_originalps[n].second.size() - 2)
					{
						double midx = (smooth_originalps[n].second[i + 1].x + smooth_originalps[n].second[i].x) / 2.0;
						double midy = (smooth_originalps[n].second[i + 1].y + smooth_originalps[n].second[i].y) / 2.0;
						RGBPoint pc; pc.x = midx; pc.y = midy;

						RGBPoint ppre1 = smooth_originalps[n].second[i - 1];
						RGBPoint ppre2 = smooth_originalps[n].second[i];
						RGBPoint pnex1 = smooth_originalps[n].second[i + 1];

						double x = pc.x*(1 - midshift_weight) + midshift_weight / 3.0*(ppre1.x + ppre2.x + pnex1.x);
						double y = pc.y*(1 - midshift_weight) + midshift_weight / 3.0*(ppre1.y + ppre2.y + pnex1.y);
						RGBPoint p;
						p.x = x; p.y = y;
						new_points.push_back(p);
					}
					else
					{
						RGBPoint ppre1 = smooth_originalps[n].second[i];
						RGBPoint ppre2 = smooth_originalps[n].second[i - 1];
						RGBPoint pnex1 = smooth_originalps[n].second[i + 1];
						RGBPoint pnex2 = smooth_originalps[n].second[i + 2];
						double midx = (smooth_originalps[n].second[i + 1].x + smooth_originalps[n].second[i].x) / 2.0;
						double midy = (smooth_originalps[n].second[i + 1].y + smooth_originalps[n].second[i].y) / 2.0;
						RGBPoint pc; pc.x = midx; pc.y = midy;

						double x = pc.x*(1 - midshift_weight) + midshift_weight / 4.0*(ppre1.x + ppre2.x + pnex1.x + pnex2.x);
						double y = pc.y*(1 - midshift_weight) + midshift_weight / 4.0*(ppre1.y + ppre2.y + pnex1.y + pnex2.y);
						RGBPoint p;
						p.x = x; p.y = y;
						new_points.push_back(p);
					}
				}

				smooth_originalps[n].second = new_points;
				new_points.clear();
				//move other points
				for (int i = 0; i < smooth_originalps[n].second.size(); i++)
				{
					if (i == 0 || i == smooth_originalps[n].second.size() - 1)
					{
						new_points.push_back(smooth_originalps[n].second[i]);
					}
					else if (i == 1 || i == smooth_originalps[n].second.size() - 2)
					{
						RGBPoint ppre1 = smooth_originalps[n].second[i - 1];
						RGBPoint pc = smooth_originalps[n].second[i];
						RGBPoint pnex1 = smooth_originalps[n].second[i + 1];
						double x = pc.x*(1 - shift_weight) + shift_weight / 2.0*(ppre1.x + pnex1.x);
						double y = pc.y*(1 - shift_weight) + shift_weight / 2.0*(ppre1.y + pnex1.y);
						RGBPoint p;
						p.x = x; p.y = y;
						new_points.push_back(p);
					}
					else
					{
						RGBPoint ppre1 = smooth_originalps[n].second[i - 1];
						RGBPoint ppre2 = smooth_originalps[n].second[i - 2];
						RGBPoint pc = smooth_originalps[n].second[i];
						RGBPoint pnex1 = smooth_originalps[n].second[i + 1];
						RGBPoint pnex2 = smooth_originalps[n].second[i + 2];
						double x = pc.x*(1 - shift_weight) + shift_weight / 4.0*(ppre1.x + ppre2.x + pnex1.x + pnex2.x);
						double y = pc.y*(1 - shift_weight) + shift_weight / 4.0*(ppre1.y + ppre2.y + pnex1.y + pnex2.y);
						RGBPoint p;
						p.x = x; p.y = y;
						new_points.push_back(p);
					}
				}
				smooth_originalps[n].second.clear();
				smooth_originalps[n].second = new_points;
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
	for (int i = 0; i < smooth_originalps.size(); i++)
	{
		for (int j = 0; j < smooth_originalps[i].second.size(); j++)
		{
			featurecurve_originalps.push_back(smooth_originalps[i].second[j]);
		}
	}
#endif

#if 1
	vector<pair<bool, vector<PointTriple>>> smooth_originalps;

	int size_add = sample_feature_curve_rate;
	int nite_smooth = degree_ofsmoothing_feaP;
	double midshift_weight = 1.0;
	double shift_weight = 1.0;

	//Laplace uniform
	for (int i = 0; i < feature_lines_modify.size(); i++)
	{
		vector<PointTriple> originalps;
		bool is_circle = false;
		if (abs(feature_lines_modify[i][0].index_x - feature_lines_modify[i][feature_lines_modify[i].size() - 1].index_x) < 2 &&
			abs(feature_lines_modify[i][0].index_y - feature_lines_modify[i][feature_lines_modify[i].size() - 1].index_y) < 2)
		{
			is_circle = true;
		}
		for (int j = 0; j < feature_lines_modify[i].size(); j++)
		{
			double x = feature_lines_modify[i][j].imagedomain.x() - imagemesh_dx;
			double y = feature_lines_modify[i][j].imagedomain.y()*imagemesh_dy * 2 - imagemesh_dy;
			PointTriple p = feature_lines_modify[i][j];
			p.imagep = Point_2(x, y);
			p.is_fixed = true;
			originalps.push_back(p);
		}
		smooth_originalps.push_back(pair<bool, vector<PointTriple>>(is_circle, originalps));
	}

	for (int n = 0; n < smooth_originalps.size(); n++)
	{
		vector<PointTriple> new_points;
		if (smooth_originalps[n].first)
		{
			//circle 
			for (int k = 0; k < size_add; k++)
			{
				new_points.clear();
				//add new mid points
				for (int i = 0; i < smooth_originalps[n].second.size(); i++)
				{
					new_points.push_back(smooth_originalps[n].second[i]);
					Point_2 ppre1 = smooth_originalps[n].second[i].imagep;
					Point_2 ppre2 = smooth_originalps[n].second[(i - 1 + smooth_originalps[n].second.size()) % smooth_originalps[n].second.size()].imagep;
					Point_2 pnex1 = smooth_originalps[n].second[(i + 1) % smooth_originalps[n].second.size()].imagep;
					Point_2 pnex2 = smooth_originalps[n].second[(i + 2) % smooth_originalps[n].second.size()].imagep;
					double midx = (smooth_originalps[n].second[(i + 1) % smooth_originalps[n].second.size()].imagep.x() + smooth_originalps[n].second[i].imagep.x()) / 2.0;
					double midy = (smooth_originalps[n].second[(i + 1) % smooth_originalps[n].second.size()].imagep.y() + smooth_originalps[n].second[i].imagep.y()) / 2.0;
					double x = midx*(1 - midshift_weight) + midshift_weight / 4.0*(ppre1.x() + ppre2.x() + pnex1.x() + pnex2.x());
					double y = midy*(1 - midshift_weight) + midshift_weight / 4.0*(ppre1.y() + ppre2.y() + pnex1.y() + pnex2.y());
					PointTriple p;
					p.imagep = Point_2(x, y);
					p.is_fixed = false;
					new_points.push_back(p);
				}
				smooth_originalps[n].second = new_points;

#if 1//comment for testing
				//move other points
				for (int it = 0; it<nite_smooth; it++)
				{
					new_points.clear();
					for (int i = 0; i < smooth_originalps[n].second.size(); i++)
					{
						Point_2 ppre1 = smooth_originalps[n].second[(i - 1 + smooth_originalps[n].second.size()) % smooth_originalps[n].second.size()].imagep;
						Point_2 ppre2 = smooth_originalps[n].second[(i - 2 + smooth_originalps[n].second.size()) % smooth_originalps[n].second.size()].imagep;
						Point_2 ppre3 = smooth_originalps[n].second[(i - 3 + smooth_originalps[n].second.size()) % smooth_originalps[n].second.size()].imagep;
						Point_2 ppre4 = smooth_originalps[n].second[(i - 4 + smooth_originalps[n].second.size()) % smooth_originalps[n].second.size()].imagep;
						Point_2 pc = smooth_originalps[n].second[i].imagep;
						Point_2 pnex1 = smooth_originalps[n].second[(i + 1) % smooth_originalps[n].second.size()].imagep;
						Point_2 pnex2 = smooth_originalps[n].second[(i + 2) % smooth_originalps[n].second.size()].imagep;
						Point_2 pnex3 = smooth_originalps[n].second[(i + 3) % smooth_originalps[n].second.size()].imagep;
						Point_2 pnex4 = smooth_originalps[n].second[(i + 4) % smooth_originalps[n].second.size()].imagep;
						double x = pc.x()*(1 - shift_weight) + shift_weight / 8.0*(ppre1.x() + ppre2.x() + ppre3.x() + ppre4.x() + pnex1.x() + pnex2.x() + pnex3.x() + pnex4.x());
						double y = pc.y()*(1 - shift_weight) + shift_weight / 8.0*(ppre1.y() + ppre2.y() + ppre3.y() + ppre4.y() + pnex1.y() + pnex2.y() + pnex3.y() + pnex4.y());
						
						PointTriple p = smooth_originalps[n].second[i];
						p.imagep = Point_2(x, y);
						new_points.push_back(p);
					}
					smooth_originalps[n].second.clear();
					smooth_originalps[n].second = new_points;
				}
#endif
			}
		}
		else
		{
			//normal
			for (int k = 0; k < size_add; k++)
			{
				new_points.clear();
				//add new mid points
				for (int i = 0; i < smooth_originalps[n].second.size(); i++)
				{
					new_points.push_back(smooth_originalps[n].second[i]);
					if (i == smooth_originalps[n].second.size() - 1)
					{
						break;
					}
					if (i == 0)
					{
						double midx = (smooth_originalps[n].second[i + 1].imagep.x() + smooth_originalps[n].second[i].imagep.x()) / 2.0;
						double midy = (smooth_originalps[n].second[i + 1].imagep.y() + smooth_originalps[n].second[i].imagep.y()) / 2.0;
						Point_2 ppre1 = smooth_originalps[n].second[i].imagep;
						Point_2 pnex1 = smooth_originalps[n].second[i + 1].imagep;
						Point_2 pnex2 = smooth_originalps[n].second[i + 2].imagep;

						double x = midx*(1 - midshift_weight) + midshift_weight / 3.0*(ppre1.x() + pnex1.x() + pnex2.x());
						double y = midy*(1 - midshift_weight) + midshift_weight / 3.0*(ppre1.y() + pnex1.y() + pnex2.y());
						PointTriple p;
						p.imagep = Point_2(x, y);
						p.is_fixed = false;
						new_points.push_back(p);
					}
					else if (i == smooth_originalps[n].second.size() - 2)
					{
						double midx = (smooth_originalps[n].second[i + 1].imagep.x() + smooth_originalps[n].second[i].imagep.x()) / 2.0;
						double midy = (smooth_originalps[n].second[i + 1].imagep.y() + smooth_originalps[n].second[i].imagep.y()) / 2.0;
						Point_2 ppre1 = smooth_originalps[n].second[i - 1].imagep;
						Point_2 ppre2 = smooth_originalps[n].second[i].imagep;
						Point_2 pnex1 = smooth_originalps[n].second[i + 1].imagep;

						double x = midx*(1 - midshift_weight) + midshift_weight / 3.0*(ppre1.x() + ppre2.x() + pnex1.x());
						double y = midy*(1 - midshift_weight) + midshift_weight / 3.0*(ppre1.y() + ppre2.y() + pnex1.y());
						PointTriple p;
						p.imagep = Point_2(x, y);
						p.is_fixed = false;
						new_points.push_back(p);
					}
					else
					{
						Point_2 ppre1 = smooth_originalps[n].second[i].imagep;
						Point_2 ppre2 = smooth_originalps[n].second[i - 1].imagep;
						Point_2 pnex1 = smooth_originalps[n].second[i + 1].imagep;
						Point_2 pnex2 = smooth_originalps[n].second[i + 2].imagep;
						double midx = (smooth_originalps[n].second[i + 1].imagep.x() + smooth_originalps[n].second[i].imagep.x()) / 2.0;
						double midy = (smooth_originalps[n].second[i + 1].imagep.y() + smooth_originalps[n].second[i].imagep.y()) / 2.0;
						double x = midx*(1 - midshift_weight) + midshift_weight / 4.0*(ppre1.x() + ppre2.x() + pnex1.x() + pnex2.x());
						double y = midy*(1 - midshift_weight) + midshift_weight / 4.0*(ppre1.y() + ppre2.y() + pnex1.y() + pnex2.y());
						PointTriple p;
						p.imagep = Point_2(x, y);
						p.is_fixed = false;
						new_points.push_back(p);
					}
				}
				smooth_originalps[n].second = new_points;

				//move other points
				for (int it = 0; it<nite_smooth; it++)
				{
					new_points.clear();
					for (int i = 0; i < smooth_originalps[n].second.size(); i++)
					{
						if (i == 0 || i == smooth_originalps[n].second.size() - 1)
						{
							new_points.push_back(smooth_originalps[n].second[i]);
						}
						else if (i == 1 || i == smooth_originalps[n].second.size() - 2)
						{
							Point_2 ppre1 = smooth_originalps[n].second[i - 1].imagep;
							Point_2 pc = smooth_originalps[n].second[i].imagep;
							Point_2 pnex1 = smooth_originalps[n].second[i + 1].imagep;
							double x = pc.x()*(1 - shift_weight) + shift_weight / 2.0*(ppre1.x() + pnex1.x());
							double y = pc.y()*(1 - shift_weight) + shift_weight / 2.0*(ppre1.y() + pnex1.y());
							PointTriple p = smooth_originalps[n].second[i];
							p.imagep = Point_2(x, y);
							new_points.push_back(p);
						}
						else if (i >= 4 && i <= smooth_originalps[n].second.size() - 5)
						{

							Point_2 ppre1 = smooth_originalps[n].second[i - 1].imagep;
							Point_2 ppre2 = smooth_originalps[n].second[i - 2].imagep;
							Point_2 ppre3 = smooth_originalps[n].second[i - 3].imagep;
							Point_2 ppre4 = smooth_originalps[n].second[i - 4].imagep;
							Point_2 pc = smooth_originalps[n].second[i].imagep;
							Point_2 pnex1 = smooth_originalps[n].second[i + 1].imagep;
							Point_2 pnex2 = smooth_originalps[n].second[i + 2].imagep;
							Point_2 pnex3 = smooth_originalps[n].second[i + 3].imagep;
							Point_2 pnex4 = smooth_originalps[n].second[i + 4].imagep;
							double x = pc.x()*(1 - shift_weight) + shift_weight / 8.0*(ppre1.x() + ppre2.x() + ppre3.x() + ppre4.x() + pnex1.x() + pnex2.x() + pnex3.x() + pnex4.x());
							double y = pc.y()*(1 - shift_weight) + shift_weight / 8.0*(ppre1.y() + ppre2.y() + ppre3.y() + ppre4.y() + pnex1.y() + pnex2.y() + pnex3.y() + pnex4.y());
							PointTriple p = smooth_originalps[n].second[i];
							p.imagep = Point_2(x, y);
							new_points.push_back(p);
						}
						else
						{
							Point_2 ppre1 = smooth_originalps[n].second[i - 1].imagep;
							Point_2 ppre2 = smooth_originalps[n].second[i - 2].imagep;
							Point_2 pc = smooth_originalps[n].second[i].imagep;
							Point_2 pnex1 = smooth_originalps[n].second[i + 1].imagep;
							Point_2 pnex2 = smooth_originalps[n].second[i + 2].imagep;
							double x = pc.x()*(1 - shift_weight) + shift_weight / 4.0*(ppre1.x() + ppre2.x() + pnex1.x() + pnex2.x());
							double y = pc.y()*(1 - shift_weight) + shift_weight / 4.0*(ppre1.y() + ppre2.y() + pnex1.y() + pnex2.y());
							PointTriple p = smooth_originalps[n].second[i];
							p.imagep = Point_2(x, y);
							new_points.push_back(p);
						}
					}
					smooth_originalps[n].second.clear();
					smooth_originalps[n].second = new_points;
				}
			}
		}

	}	
	
	for (int i = 0; i < smooth_originalps.size(); i++)
	{
		for (int j = 0; j < smooth_originalps[i].second.size(); j++)
		{
			/*if (!smooth_originalps[i].second[j].is_fixed)
			{
			continue;
			}*/
			RGBPoint p;
			p.x = smooth_originalps[i].second[j].imagep.x();
			p.y = smooth_originalps[i].second[j].imagep.y();
			featurecurve_originalps.push_back(p);
		}
	}

	assert(feature_lines_modify.size() == smooth_originalps.size());
	for (int i = 0; i < feature_lines_modify.size(); i++)
	{
		int index_ = 0;
		for (int j = 0; j < smooth_originalps[i].second.size(); j++)
		{
			if (smooth_originalps[i].second[j].is_fixed)
			{
				assert(smooth_originalps[i].second[j].index_x == feature_lines_modify[i][index_].index_x&&
					smooth_originalps[i].second[j].index_y == feature_lines_modify[i][index_].index_y);
				double x = feature_lines_modify[i][index_].imagedomain.x() - imagemesh_dx;
				double y = feature_lines_modify[i][index_].imagedomain.y()*imagemesh_dy * 2 - imagemesh_dy;
				feature_lines_modify[i][index_].imagep = Point_2(x, y);
				feature_lines_modify[i][index_].image_newp = smooth_originalps[i].second[j].imagep;
				feature_lines_modify[i][index_].vit->smooth_point() = feature_lines_modify[i][index_].image_newp;
				index_++;
			}
		}
	}

#endif

#if 1
	//flip feature edge
	for (int i = 0; i < feature_lines_modify.size(); i++)
	{
		for (int j = 0; j < feature_lines_modify[i].size(); j++)
		{
			int star = feature_lines_modify[i][j].vit->vertex_index();
			int end_ = feature_lines_modify[i][(j + 1)%feature_lines_modify[i].size()].vit->vertex_index();

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
#endif
}

void Image_Approximation::project_featureps_onto_smoothps()
{
	for (int i = 0; i < feature_lines_modify.size(); i++)
	{
		for (int j = 0;j<feature_lines_modify[i].size();j++)
		{
			feature_lines_modify[i][j].vit->point() = Point(feature_lines_modify[i][j].image_newp.x(), feature_lines_modify[i][j].image_newp.y(),0.0);
		}
	}

	ConstrainedPara parame_(originalMesh, 1);
	parame_.compute_para_domain();

	//recompute domain
	for (Mesh::Vertex_iterator vit = originalMesh->vertices_begin(); vit != originalMesh->vertices_end(); vit++)
	{
		Point_2 domain_p(vit->point().x() + imagemesh_dx, (vit->point().y() + imagemesh_dy) / (2 * imagemesh_dy));
		vit->get_old_domain() = domain_p;
		vit->get_domain() = domain_p;
	}

	//update feature line domain
	for (int i = 0; i < feature_lines_modify.size(); i++)
	{
		for (int j = 0; j < feature_lines_modify[i].size(); j++)
		{
			Point_2 domain_p(feature_lines_modify[i][j].image_newp.x() + imagemesh_dx, (feature_lines_modify[i][j].image_newp.y() + imagemesh_dy) / (2 * imagemesh_dy));
			feature_lines_modify[i][j].imagedomain = domain_p;
			feature_lines_modify[i][j].paradomain = domain_p;
		}
	}
	//compute discrete curvature
	for (int i = 0; i < feature_lines_modify.size(); i++)
	{
		if (abs(feature_lines_modify[i][0].index_x - feature_lines_modify[i][feature_lines_modify[i].size() - 1].index_x) < 2 &&
			abs(feature_lines_modify[i][0].index_y - feature_lines_modify[i][feature_lines_modify[i].size() - 1].index_y) < 2)
		{
			//especially for a circle only consist of 8 points
			if (feature_lines_modify[i].size() <= 8)
			{
				feature_lines_modify[i][0].curvature = 125;
				feature_lines_modify[i][feature_lines_modify[i].size() / 2].curvature = 125;
			}

			for (int j = 0; j < feature_lines_modify[i].size() ; j++)
			{
				if (abs(feature_lines_modify[i][j].index_x-121)<2 && abs(feature_lines_modify[i][j].index_y-130)<2)
				{
					int t = 1;
				}

				if (feature_lines_modify[i].size()<=8)
				{
					if (j != 0 && j != feature_lines_modify[i].size() / 2)
					{
						feature_lines_modify[i][j].curvature = 0.1;
					}
				}
				Point_2 p0 = feature_lines_modify[i][(j - 3+feature_lines_modify[i].size())% feature_lines_modify[i].size()].image_newp;
				Point_2 p1 = feature_lines_modify[i][j].image_newp;
				Point_2 p2 = feature_lines_modify[i][(j + 3)% feature_lines_modify[i].size()].image_newp;
				/*double sita = acos((p2 - p1)*(p0-p1) / sqrt((p2 - p1).squared_length()*(p0 - p1).squared_length()));
				double cur_ = sin(sita / 2.0);*/
				double abc = sqrt((p0-p1).squared_length()*(p0-p2).squared_length()*(p1-p2).squared_length());
				double discrete_cur = 1.0 / (abc / 4.0 / abs(area(p0, p1, p2)));
				feature_lines_modify[i][j].constant_size = 200.0 / input_image->width();
				feature_lines_modify[i][j].curvature = discrete_cur / input_image->width()*200.0;
				//std::cout << feature_lines_modify[i][j].curvature <<"belong to "<<i<< std::endl;
			}		
		}
		else
		{
			for (int j = 0; j < feature_lines_modify[i].size(); j++)
			{
				if (j<3 || j> feature_lines_modify[i].size() - 4)
				{
					feature_lines_modify[i][j].curvature = 0.1;
					continue;
				}
				Point_2 p0 = feature_lines_modify[i][j - 3].image_newp;
				Point_2 p1 = feature_lines_modify[i][j].image_newp;
				Point_2 p2 = feature_lines_modify[i][j + 3].image_newp;
				/*double sita = acos((p2 - p1)*(p0 - p1) / sqrt((p2 - p1).squared_length()*(p1 - p0).squared_length()));
				double cur_ = sin(sita / 2.0);
				feature_lines_modify[i][j].curvature = cur_;*/
				double abc = sqrt((p0 - p1).squared_length()*(p0 - p2).squared_length()*(p1 - p2).squared_length());
				double discrete_cur = 1.0 / (abc / 4.0 / abs(area(p0, p1, p2)));
				feature_lines_modify[i][j].constant_size = 200.0 / input_image->width();
				feature_lines_modify[i][j].curvature = discrete_cur / input_image->width()*200.0;
				//std::cout << feature_lines_modify[i][j].curvature << "belong to " << i << std::endl;
			}
			
			//especially "9" shape feature points
			for (int j = 2; j < feature_lines_modify[i].size() - 2; j++)
			{
				if (abs(feature_lines_modify[i][0].index_x - feature_lines_modify[i][j].index_x) < 2 &&
					abs(feature_lines_modify[i][0].index_y - feature_lines_modify[i][j].index_y) < 2 ||
					abs(feature_lines_modify[i][j].index_x - feature_lines_modify[i][feature_lines_modify[i].size() - 1].index_x) < 2 &&
					abs(feature_lines_modify[i][j].index_y - feature_lines_modify[i][feature_lines_modify[i].size() - 1].index_y) < 2)
				{
					feature_lines_modify[i][j].curvature = 125;
					continue;
				}
			}			
		}
	}
}

void Image_Approximation::remedy_color_of_one_neighbor()
{
	vector<int> processed_points;
	 for (int i = 0;i<feature_lines_modify.size();i++)
	 {
		 for (int j = 0;j < feature_lines_modify[i].size();j++)
		 {
			 Mesh::Vertex_iterator vit = feature_lines_modify[i][j].vit;			 			
			 vector<int> inte = { vit->vertex_index() };
			 vector<int> one_ring;
			 from_interior_to_ccw_one_ring_cycle(inte, one_ring, &originalMesh);
			 for (int k = 0;k<one_ring.size();k++)
			 {
				 vector<int>::iterator res = find(processed_points.begin(), processed_points.end(), one_ring[k]);
				 if (res != processed_points.end())
				 {
					 continue;
				 }
				 if (!originalMesh->get_vertex_iterator(one_ring[k])->is_feature())
				 {
					processed_points.push_back(one_ring[k]);
					vector<int> inte1 = { one_ring[k]};
					vector<int> one_ring1;
					from_interior_to_ccw_one_ring_cycle(inte1, one_ring1, &originalMesh);

					bool does_exist_feature = false;
					bool does_exist_othertype_feature = false;
					vector<Mesh::Vertex_iterator> vfeanew;
					for (int it = 0;it<one_ring1.size();it++)
					{
						if (originalMesh->get_vertex_iterator(one_ring1[it])->is_feature() && 
							originalMesh->get_vertex_iterator(one_ring1[it])->vertex_index() != vit->vertex_index())
						{
							does_exist_feature = true;
							vfeanew.push_back(originalMesh->get_vertex_iterator(one_ring1[it]));
							if (originalMesh->get_vertex_iterator(one_ring1[it])->feature_type() !=vit->feature_type())
							{
								does_exist_othertype_feature = true;
								break;
							}							
						}
					}
					if (does_exist_othertype_feature)
					{
						continue;//no modifying
						std::cout <<__FUNCTION__<<": "<< "color beside feature without modifying" << std::endl;
					}
					if (does_exist_feature)
					{
						//winnow other side
						double r = 0, g = 0, b = 0;
						double num_plus = 0.0;
						bool is_plusv1 = false;

						Point_2 pf1 = vit->get_domain();
						for (int it = 0; it<one_ring1.size(); it++)
						{
							if (!originalMesh->get_vertex_iterator(one_ring1[it])->is_feature())
							{
								bool is_satisfying = true;
								for (int m = 0;m<vfeanew.size();m++)
								{
									Point_2 pf2 = vfeanew[m]->get_domain();
									if (CGAL::area(pf1, pf2, originalMesh->get_vertex_iterator(one_ring1[it])->get_domain()) *
										CGAL::area(pf1, pf2, originalMesh->get_vertex_iterator(one_ring[k])->get_domain()) < 0)
									{
										is_satisfying = false;
									}
								}
								if (is_satisfying)
								{
									is_plusv1 = true;
									num_plus += 1.0;
									r += originalMesh->get_vertex_iterator(one_ring1[it])->vertex_pixel_color().x();
									g += originalMesh->get_vertex_iterator(one_ring1[it])->vertex_pixel_color().y();
									b += originalMesh->get_vertex_iterator(one_ring1[it])->vertex_pixel_color().z();
								}
							}
						}
						if (is_plusv1)
						{
							r = r / num_plus;
							g = g / num_plus;
							b = b / num_plus;

							originalMesh->get_vertex_iterator(one_ring[k])->vertex_pixel_color() = Point_3(r, g, b);
							originalMesh->get_vertex_iterator(one_ring[k])->vertex_input_color() = Point_3(r, g, b);
						}

					}
					else
					{
						double r = 0, g = 0, b = 0;
						double num_plus = 0.0;
						for (int it = 0; it < one_ring1.size(); it++)
						{
							if (!originalMesh->get_vertex_iterator(one_ring1[it])->is_feature())
							{
								num_plus += 1.0;
								r += originalMesh->get_vertex_iterator(one_ring1[it])->vertex_pixel_color().x();
								g += originalMesh->get_vertex_iterator(one_ring1[it])->vertex_pixel_color().y();
								b += originalMesh->get_vertex_iterator(one_ring1[it])->vertex_pixel_color().z();
							}
						}
						r = r / num_plus;
						g = g / num_plus;
						b = b / num_plus;
						originalMesh->get_vertex_iterator(one_ring[k])->vertex_pixel_color() = Point_3(r, g, b);
						originalMesh->get_vertex_iterator(one_ring[k])->vertex_input_color() = Point_3(r, g, b);
					}
				 }
			 }
		 }
	 }

}

void Image_Approximation::matching_feature_with_polyline(double error_)
{
	if (bsimplyfeature)
	{
		return;
	}
	for (int i = 0; i < feature_lines_modify.size(); i++)
	{
		bool is_loop = false;
		if (feature_lines_modify[i].size() < 3)
		{
			is_loop = false;
		}
		else if (abs(feature_lines_modify[i][0].index_x - feature_lines_modify[i][feature_lines_modify[i].size() - 1].index_x) <= 1 &&
			abs(feature_lines_modify[i][0].index_y - feature_lines_modify[i][feature_lines_modify[i].size() - 1].index_y) <= 1)
		{
			is_loop = true;
		}

		if (is_loop)
		{
			if (feature_lines_modify[i].size() == 56)
			{
				int s = 1;
			}
#if 1
			vector<PointTriple> Points = feature_lines_modify[i];
			std::pair<int, double> maxDistance = findMaximumDistance(Points, error_);
			int index = maxDistance.first;
			vector<PointTriple>::iterator it = Points.begin();
			vector<PointTriple> path1t(Points.begin(), it + index + 1);
			vector<PointTriple> path2t(it + index, Points.end());
			path2t.push_back(Points[0]);

			simplifyWithRDP(path1t, error_);
			simplifyWithRDP(path2t, error_);

			int size1=0, size2=0;
			for (int k = 0;k<path1t.size();k++)
			{
				if (path1t[k].is_fixed)
				{
					size1++;
				}
			}
			for (int k = 0; k < path2t.size(); k++)
			{
				if (path2t[k].is_fixed)
				{
					size2++;
				}
			}
			if (size1 == 2 && size2 == 2)
			{
				vector<PointTriple>::iterator ik = Points.begin();
				vector<PointTriple> path1(Points.begin(), ik + index + 1);
				vector<PointTriple> path2(ik + index, Points.end());
				path2.push_back(Points[0]);

				if (path1.size() > path2.size())
				{
					vector<PointTriple> tPoints = path1;
					std::pair<int, double> tmaxDistance = findMaximumDistance(tPoints, error_);
					int tindex = tmaxDistance.first;
					vector<PointTriple>::iterator tit = tPoints.begin();
					vector<PointTriple> tpath1(tPoints.begin(), tit + tindex + 1);
					vector<PointTriple> tpath2(tit + tindex, tPoints.end());

					simplifyWithRDP(tpath1, error_);
					simplifyWithRDP(tpath2, error_);
					simplifyWithRDP(path2, error_);

					Points.clear();
					Points.insert(Points.begin(), tpath1.begin(), tpath1.end());
					Points.pop_back();
					Points.insert(Points.end(), tpath2.begin(), tpath2.end());
					Points.pop_back();
					Points.insert(Points.end(), path2.begin(), path2.end());
					feature_lines_modify[i] = Points;
					continue;
				}
				else
				{
					vector<PointTriple> tPoints = path2;
					std::pair<int, double> tmaxDistance = findMaximumDistance(tPoints, error_);
					int tindex = tmaxDistance.first;
					vector<PointTriple>::iterator tit = tPoints.begin();
					vector<PointTriple> tpath1(tPoints.begin(), tit + tindex + 1);
					vector<PointTriple> tpath2(tit + tindex, tPoints.end());

					simplifyWithRDP(path1, error_);
					simplifyWithRDP(tpath1, error_);
					simplifyWithRDP(tpath2, error_);

					Points.clear();
					Points.insert(Points.begin(), path1.begin(), path1.end());
					Points.pop_back();
					Points.insert(Points.end(), tpath1.begin(), tpath1.end());
					Points.pop_back();
					Points.insert(Points.end(), tpath2.begin(), tpath2.end());
					feature_lines_modify[i] = Points;
					continue;
				}
			}

			//simplify path1 and path2 together
			Points.clear();
			Points.insert(Points.begin(), path1t.begin(), path1t.end());
			Points.pop_back();
			Points.insert(Points.end(), path2t.begin(), path2t.end());
			feature_lines_modify[i] = Points;
#else
			//circle line
			int index_mid = -1, index_dis = -1;

			vector<PointTriple> Points = feature_lines_modify[i];
			vector<pair<int, double>> points_cur;
			for (int i = 0;i<Points.size();i++)
			{
				points_cur.push_back(pair<int, double>(i, Points[i].curvature));
			}
			sort(points_cur.begin(), points_cur.end(), sortdoublePair_SecondSmaller);
			index_mid = points_cur[int(points_cur.size() * 0.4)].first;
	
			double Mdis = -1;
			for (int i = 0; i < Points.size(); i++)
			{
				if ((Points[i].imagedomain - Points[index_mid].imagedomain).squared_length() > Mdis)
				{
					Mdis = (Points[i].imagedomain - Points[index_mid].imagedomain).squared_length();
					index_dis = i;
				}
			}
			int index_from = std::min(index_mid, index_dis);
			int index_to = std::max(index_mid, index_dis);
			int index_now = index_from;
			vector<PointTriple> path1, path2;
			path2.push_back(Points[index_to]);

			for (int i = 0; i < Points.size(); i++)
			{
				if (index_now >= index_from && index_now <= index_to)
				{
					path1.push_back(Points[index_now%Points.size()]);
				}
				else
				{
					path2.push_back(Points[index_now%Points.size()]);
				}
				index_now++;
			}
			path2.push_back(Points[index_from]);

			simplifyWithRDP(path1, error_);
			simplifyWithRDP(path2, error_);

			//simplify path1 and path2 together
			Points.clear();
			Points.insert(Points.begin(), path1.begin(), path1.end());
			Points.pop_back();
			Points.insert(Points.end(), path2.begin(), path2.end());
			feature_lines_modify[i] = Points;
#endif
		}
		else
		{
			simplifyWithRDP(feature_lines_modify[i], error_);
		}
	}
	int num = 0;
	for (int i = 0;i<feature_lines_modify.size();i++)
	{
		for (int j = 0;j<feature_lines_modify[i].size();j++)
		{
			if (feature_lines_modify[i][j].is_fixed)
			{
				num++;
			}
		}
	}
	std::cout << __FUNCTION__ << ": " << "points size: " << num << std::endl;

#if 1
	//find intersection segments
	struct Np
	{
		int line_t;
		int line_index;
		Point_2 p;
	};
	bool do_exist_intersect;
	do 
	{
		do_exist_intersect = false;
		vector<vector<Np>> extracted;
		for (int i = 0; i < feature_lines_modify.size(); i++)
		{
			vector<Np> ps;
			for (int j = 0; j < feature_lines_modify[i].size(); j++)
			{
				feature_lines_modify[i][j].is_headortail = false;
				if (abs(feature_lines_modify[i][0].index_x - feature_lines_modify[i][feature_lines_modify[i].size() - 1].index_x) <= 1 &&
					abs(feature_lines_modify[i][0].index_y - feature_lines_modify[i][feature_lines_modify[i].size() - 1].index_y) <= 1)
				{
					if ((j == 0 || j == feature_lines_modify[i].size() - 1) && feature_lines_modify[i][j].is_fixed)
					{
						feature_lines_modify[i][j].is_headortail = true;
					}
				}

				if (feature_lines_modify[i][j].is_fixed)
				{
					Np p; p.line_t = i; p.line_index = j; p.p = feature_lines_modify[i][j].paradomain;
					ps.push_back(p);
				}
			}
			extracted.push_back(ps);
		}

		vector<vector<Np>> candidates;//element has 4 Np: from-to and from-to
		for (int i = 0; i < extracted.size(); i++)
		{
			for (int j = 0; j < extracted[i].size()-1; j++)
			{
				Segment_2 seg1(extracted[i][j].p,extracted[i][j+1].p);
				for (int m = 0; m < extracted.size(); m++)
				{
					for (int n = 0; n < extracted[m].size() - 1; n++)
					{
						Segment_2 seg2(extracted[m][n].p, extracted[m][n + 1].p);
						PointTriple p1f, p1t, p2f, p2t;
						p1f = feature_lines_modify[extracted[i][j].line_t][extracted[i][j].line_index];
						p1t = feature_lines_modify[extracted[i][j+1].line_t][extracted[i][j+1].line_index];
						p2f = feature_lines_modify[extracted[m][n].line_t][extracted[m][n].line_index];
						p2t = feature_lines_modify[extracted[m][n+1].line_t][extracted[m][n+1].line_index];
						if (p1f.index_x== p2f.index_x&&p1f.index_y == p2f.index_y ||
							p1f.index_x == p2t.index_x&&p1f.index_y == p2t.index_y ||
							p1t.index_x == p2f.index_x&&p1t.index_y == p2f.index_y ||
							p1t.index_x == p2t.index_x&&p1t.index_y == p2t.index_y )
						{
							continue;
						}
						if (CGAL::do_intersect(seg1, seg2))
						{
							do_exist_intersect = true;
							bool doinsert = true;
							for (int idt = 0;idt<candidates.size();idt++)
							{
								if (candidates[idt][0].line_index == extracted[i][j].line_index && candidates[idt][0].line_t == extracted[i][j].line_t &&
									candidates[idt][2].line_index == extracted[m][n].line_index && candidates[idt][2].line_t == extracted[m][n].line_t ||
									candidates[idt][2].line_index == extracted[i][j].line_index && candidates[idt][2].line_t == extracted[i][j].line_t &&
									candidates[idt][0].line_index == extracted[m][n].line_index && candidates[idt][0].line_t == extracted[m][n].line_t)
								{
									doinsert = false;
									break;
								}
							}
							if (doinsert)
							{
								vector<Np> segpair = { extracted[i][j],extracted[i][j + 1],extracted[m][n],extracted[m][n + 1] };
								candidates.push_back(segpair);
							}			
						}
					}
				}
			}
		}
		std::cout << __FUNCTION__ << ": " << "intersection candidate size: " << candidates.size() << std::endl;
		/*if (candidates.size() == 1)
		{
			std::cout << "from-to: " << candidates[0][0].p.x() << "," << candidates[0][0].p.y()
				<< "|" << candidates[0][1].p.x() << "," << candidates[0][1].p.y() << std::endl;
			break;
		}*/
		if (do_exist_intersect)
		{
			for (int i = 0;i<candidates.size();i++)
			{
				PointTriple p1f, p1t, p2f, p2t;
				p1f = feature_lines_modify[candidates[i][0].line_t][candidates[i][0].line_index];
				p1t = feature_lines_modify[candidates[i][1].line_t][candidates[i][1].line_index];
				p2f = feature_lines_modify[candidates[i][2].line_t][candidates[i][2].line_index];
				p2t = feature_lines_modify[candidates[i][3].line_t][candidates[i][3].line_index];

				int num_line = candidates[i][0].line_t, index_from = candidates[i][0].line_index, index_to= candidates[i][1].line_index;
				if ((p1f.is_headortail || p1t.is_headortail) && (!p2f.is_headortail && !p2t.is_headortail))
				{
					num_line = candidates[i][2].line_t;
					index_from = candidates[i][2].line_index;
					index_to = candidates[i][3].line_index;
				}

				vector<PointTriple> frontpart,processpart,backpart;
				for (int k = 0;k<feature_lines_modify[num_line].size();k++)
				{
					if (k<index_from)
					{
						frontpart.push_back(feature_lines_modify[num_line][k]);
					}
					else if (k>index_to)
					{
						backpart.push_back(feature_lines_modify[num_line][k]);
					}
					else
					{
						processpart.push_back(feature_lines_modify[num_line][k]);
					}
				}
				double err = 1e-8;
				std::pair<int, double> maxDistance = findMaximumDistance(processpart, err);
				int index = maxDistance.first;
				vector<PointTriple>::iterator it = processpart.begin();
				vector<PointTriple> path1(processpart.begin(), it + index + 1);
				vector<PointTriple> path2(it + index, processpart.end());

				simplifyWithRDP(path1, 10.0,0);
				simplifyWithRDP(path2, 10.0,0);

				//simplify path1 and path2 together
				processpart.clear();
				processpart.insert(processpart.begin(), path1.begin(), path1.end());
				processpart.pop_back();
				processpart.insert(processpart.end(), path2.begin(), path2.end());

				feature_lines_modify[num_line] = frontpart;
				feature_lines_modify[num_line].insert(feature_lines_modify[num_line].end(), processpart.begin(), processpart.end());
				feature_lines_modify[num_line].insert(feature_lines_modify[num_line].end(), backpart.begin(), backpart.end());
			}		
		}
	} while (do_exist_intersect);
#endif

	//test
	//output feature polylines to matlab
	const QString mesh_p = "./test_out/feature_points.txt";
	std::ofstream fout_p;
	fout_p.open(mesh_p.toStdString());
	if (fout_p.is_open())
	{
		for (auto vit = originalMesh->vertices_begin(); vit != originalMesh->vertices_end(); vit++)
		{
			if (vit->is_feature())
			{
				fout_p << vit->get_domain().x() << " " << vit->get_domain().y() << "\n";
			}
		}
	}
	fout_p.close();
	//polylines
	const QString poly_l = "./test_out/polylines.txt";
	std::ofstream fout_l;
	fout_l.open(poly_l.toStdString());
	if (fout_l.is_open())
	{
		for (int i =0;i<feature_lines_modify.size();i++)
		{
			//int size_n = 0;
			//for (int j = 0; j < feature_lines_modify[i].size(); j++)
			//{
			//	if (feature_lines_modify[i][j].is_fixed)
			//	{
			//		size_n++;
			//	}
			//}
			//fout_l <<size_n <<" ";
			for (int j = 0;j<feature_lines_modify[i].size();j++)
			{
				if (feature_lines_modify[i][j].is_fixed)
				{
					fout_l << feature_lines_modify[i][j].paradomain.x() << " " << feature_lines_modify[i][j].paradomain.y() << "\n";
				}
			}
			//fout_l << "\n";
		}
	}
	fout_l.close();

	bsimplyfeature = true;
}

bool Image_Approximation::build_collinear_knots()
{
	if (do_compute_collinear_knots)
	{
		return 1;
	}

	matching_feature_with_polyline(simply_error);

	if (feature_lines_modify.size() == 0)
	{
		return false;
	}

	//extract fixed knots
	vector<vector<PointTriple>> fixed_lines;
	for (int i = 0; i < feature_lines_modify.size(); i++)
	{
		vector<PointTriple> fixed_knots;
		fixed_knots.clear();
		for (int j = 0; j < feature_lines_modify[i].size(); j++)
		{
			if (!feature_lines_modify[i][j].is_fixed)
			{
				continue;
			}
			fixed_knots.push_back(feature_lines_modify[i][j]);
		}
		if (fixed_knots.size() != 0)
		{
			fixed_lines.push_back(fixed_knots);
		}
	}

	//make non-repeat knots
	int index_ = 0;
	for (int i = 0; i < fixed_lines.size(); i++)
	{
		for (int j = 0; j < fixed_lines[i].size(); j++)
		{
			bool is_find = false;
			for (int k = 0; k < Norepeat_knot.size(); k++)
			{
				if (fixed_lines[i][j].index_x == Norepeat_knot[k].index_x &&
					fixed_lines[i][j].index_y == Norepeat_knot[k].index_y)
				{
					is_find = true;
					fixed_lines[i][j].index = Norepeat_knot[k].index;
					break;
				}
			}
			if (!is_find)
			{
				fixed_lines[i][j].index = index_;
				index_++;
				Norepeat_knot.push_back(fixed_lines[i][j]);
			}
		}
	}

	//tackle T-intersect
	for (int iti = 0; iti != fixed_lines.size(); iti++)
	{
		vector<PointTriple> insert_seg;
		for (int itj = 0; itj != fixed_lines[iti].size() - 1; itj++)
		{
			insert_seg.push_back(fixed_lines[iti][itj]);

			for (int k = 0; k<Norepeat_knot.size(); k++)
			{
				//test
				if (iti == 14 && itj == 0)
				{
					int s = 1;
				}

				Point_2 p1 = Norepeat_knot[k].imagedomain;
				Point_2 p2 = fixed_lines[iti][itj].imagedomain;
				Point_2 p3 = fixed_lines[iti][itj + 1].imagedomain;
				if (abs(p1.x() - p2.x())<1e-8 && abs(p1.y() - p2.y())<1e-8 ||
					abs(p1.x() - p3.x())<1e-8 && abs(p1.y() - p3.y())<1e-8)
				{
					continue;
				}
				if (abs(area(p1, p2, p3)) < 1e-16)
				{
					bool is_insert = false;
					if (abs(p2.x() - p3.x()) > 1e-8)
					{
						if (p1.x()>std::min(p2.x(), p3.x()) && p1.x()< std::max(p2.x(), p3.x()))
						{
							std::cout << __FUNCTION__ << ": " << "real area: " << setprecision(16) << area(p1, p2, p3) << "\n";
							is_insert = true;
						}
					}
					else
					{
						if (p1.y() > std::min(p2.y(), p3.y()) && p1.y() < std::max(p2.y(), p3.y()))
						{
							is_insert = true;
						}
					}
					if (is_insert)
					{
						insert_seg.push_back(Norepeat_knot[k]);
					}
				}
			}
		}
		insert_seg.push_back(fixed_lines[iti][fixed_lines[iti].size() - 1]);

		fixed_lines[iti] = insert_seg;
	}
	std::cout << __FUNCTION__ << ": " << "tackle T-intersect finished  " << std::endl;

	//whether new inserted p in corner triangles
	map<int, vector<Point_2>> corner_triangles;
	for (int i = 0; i < fixed_lines.size(); i++)
	{
		if (fixed_lines[i].size() > 2)
		{
			if (fixed_lines[i][0].index == fixed_lines[i][fixed_lines[i].size() - 1].index)
			{
				Point_2 center(0.0, 0.0);
				PointTriple p0 = fixed_lines[i][0];
				PointTriple p1 = fixed_lines[i][1];
				PointTriple p2 = fixed_lines[i][fixed_lines[i].size() - 2];

				Point_2 pt0,pt1, pt2;
				Vector_2 pnow1 = (p0.paradomain - center) + 1.0 / double(nknotinsert_collinear + 1)*(p1.paradomain - p0.paradomain);
				Vector_2 pnow2 = (p0.paradomain - center) + 1.0 / double(nknotinsert_collinear + 1)*(p2.paradomain - p0.paradomain);
				pt1 = Point_2(pnow1.x(), pnow1.y());
				pt2 = Point_2(pnow2.x(), pnow2.y());
				pt0 = p0.paradomain;

				vector<Point_2> tri = {pt0,pt1,pt2};
				corner_triangles[p0.index] = tri;
			}
		}
		for (int j = 1; j < fixed_lines[i].size()-1;j++)
		{
			Point_2 center(0.0, 0.0);
			PointTriple p0 = fixed_lines[i][j];
			PointTriple p1 = fixed_lines[i][j-1];
			PointTriple p2 = fixed_lines[i][j+1];

			Point_2 pt0, pt1, pt2;
			Vector_2 pnow1 = (p0.paradomain - center) + 1.0 / double(nknotinsert_collinear + 1)*(p1.paradomain - p0.paradomain);
			Vector_2 pnow2 = (p0.paradomain - center) + 1.0 / double(nknotinsert_collinear + 1)*(p2.paradomain - p0.paradomain);
			pt1 = Point_2(pnow1.x(), pnow1.y());
			pt2 = Point_2(pnow2.x(), pnow2.y());
			pt0 = p0.paradomain;

			vector<Point_2> tri = { pt0,pt1,pt2 };
			corner_triangles[p0.index] = tri;
		}
	}
	vector<int> refine_index;
	for (auto it = corner_triangles.begin();it!= corner_triangles.end();it++)
	{
		bool do_refine = false;
		for (int i = 0;i<Norepeat_knot.size();i++)
		{
			int ret = CGAL::bounded_side_2(it->second.begin(), it->second.end(), Norepeat_knot[i].paradomain, K());
			if (ret == CGAL::ON_BOUNDED_SIDE)
			{
				do_refine = true;
				break;
			}
		}
		if (do_refine)
		{
			refine_index.push_back(it->first);
		}
	}

	//insert new points
	vector<vector<PointTriple>>  new_lines;
	new_lines.clear();
	int new_indexx = -1, new_indexy = -1;
	for (int i = 0; i < fixed_lines.size(); i++)
	{
		vector<PointTriple> line_;
		line_.clear();
		for (int j = 0; j < fixed_lines[i].size() - 1; j++)
		{
			PointTriple p1 = fixed_lines[i][j];
			PointTriple p2 = fixed_lines[i][j + 1];
			//insert points

			bool is_plus_p1side = false;
			bool is_plus_p2side = false;
#if 0
			vector<int>::iterator res1 = find(refine_index.begin(), refine_index.end(), p1.index);
			vector<int>::iterator res2 = find(refine_index.begin(), refine_index.end(), p2.index);
			if (res1 != refine_index.end())
			{
				is_plus_p1side = true;
			}
			if ((fixed_lines[i][0].index_x != fixed_lines[i][fixed_lines[i].size() - 1].index_x ||
				fixed_lines[i][0].index_y != fixed_lines[i][fixed_lines[i].size() - 1].index_y) && j == 0)
			{
				is_plus_p1side = true;
			}
			if (res2 != refine_index.end())
			{
				is_plus_p2side = true;
			}
			if ((fixed_lines[i][0].index_x != fixed_lines[i][fixed_lines[i].size() - 1].index_x ||
				fixed_lines[i][0].index_y != fixed_lines[i][fixed_lines[i].size() - 1].index_y) && j == fixed_lines[i].size() - 2)
			{
				is_plus_p2side = true;
			}
#endif

			line_.push_back(p1);

			double coeff_ = 0.08;//0.08 original
			if (is_plus_p1side)
			{
				PointTriple pnew;
				pnew.index_x = new_indexx; pnew.index_y = new_indexy;
				new_indexy--;
				pnew.is_fixed = false;
				Point_2 p0(0.0, 0.0);
				double x, y;
				Vector_2 tempp = (p1.paradomain - p0) + (p2.paradomain - p1.paradomain)*coeff_ / double(nknotinsert_collinear + 1);
				pnew.paradomain = Point_2(tempp.x(), tempp.y());
				pnew.imagedomain = pnew.paradomain;
				line_.push_back(pnew);
			}

			for (int k = 0; k < nknotinsert_collinear; k++)
			{
				PointTriple pnew;
				pnew.index_x = new_indexx; pnew.index_y = new_indexy;
				if (k % 2 == 0)
				{
					new_indexx--;
				}
				else
				{
					new_indexy--;
				}
				pnew.is_fixed = false;
				Point_2 p0(0.0, 0.0);
				Vector_2 pt = (p1.paradomain - p0) + double(k + 1) / double(nknotinsert_collinear + 1)*(p2.paradomain - p1.paradomain);
				pnew.paradomain = Point_2(pt.x(), pt.y());
				pnew.imagedomain = pnew.paradomain;
				line_.push_back(pnew);
			}

			if (is_plus_p2side)
			{
				PointTriple pnew;
				pnew.index_x = new_indexx; pnew.index_y = new_indexy;
				new_indexy--;
				pnew.is_fixed = false;
				Point_2 p0(0.0, 0.0);
				double x, y;
				Vector_2 tempp = (p2.paradomain - p0) + (p1.paradomain - p2.paradomain)* coeff_ / double(nknotinsert_collinear + 1);
				pnew.paradomain = Point_2(tempp.x(), tempp.y());
				pnew.imagedomain = pnew.paradomain;
				line_.push_back(pnew);
			}
			
		}
		line_.push_back(fixed_lines[i][fixed_lines[i].size() - 1]);
		new_lines.push_back(line_);
	}

	if (Nite_add_collinear_knots != 0)
	{
		vector<pair<int, int>> add_collinear_knots;

		std::ifstream fin1;
		QString add_knots_dir = filename_foraddcollinear_knots;
		add_knots_dir.append("/forIte_Addcollinearknots.txt");
		fin1.open(add_knots_dir.toStdString());
		if (fin1.is_open())
		{	
			int number_seg;
			while (fin1 >> number_seg)
			{
				for (int i = 0; i < number_seg; i++)
				{
					int seg_s, seg_e;
					fin1 >> seg_s >> seg_e;
					add_collinear_knots.push_back(pair<int, int>(seg_s, seg_e));
				}
			}
		}	
		fin1.close();

		vector<pair<int, vector<int>>> add_index;
		for (int i = 0;i<add_collinear_knots.size();i++)
		{
			bool do_find = false;
			for (int j = 0;j<add_index.size();j++)
			{
				if (add_index[j].first == add_collinear_knots[i].first)
				{
					do_find = true;
					add_index[j].second.push_back(add_collinear_knots[i].second);
					break;
				}
			}
			if (!do_find)
			{
				vector<int> vec_ = {add_collinear_knots[i].second};
				add_index.push_back(pair<int,vector<int>>(add_collinear_knots[i].first,vec_));
			}
		}

		int nknotinsert_ite = 2;
		for (int i = 0; i < add_index.size(); i++)
		{
			vector<PointTriple> line_;
			for (int j = 0; j < new_lines[add_index[i].first].size(); j++)
			{
				vector<int>::iterator res = find(add_index[i].second.begin(),add_index[i].second.end(),j);
				line_.push_back(new_lines[add_index[i].first][j]);
				if (res != add_index[i].second.end())
				{
					PointTriple p1 = new_lines[add_index[i].first][j];
					PointTriple p2 = new_lines[add_index[i].first][j+1];
					//insert points
					for (int k = 0; k < nknotinsert_ite; k++)
					{
						PointTriple pnew;
						pnew.index_x = new_indexx; pnew.index_y = new_indexy;
						if (k % 2 == 0)
						{
							new_indexx--;
						}
						else
						{
							new_indexy--;
						}
						pnew.is_fixed = false;
						Point_2 p0(0.0, 0.0);
						Vector_2 pt = (p1.paradomain - p0) + double(k + 1) / double(nknotinsert_ite + 1)*(p2.paradomain - p1.paradomain);
						pnew.paradomain = Point_2(pt.x(), pt.y());
						pnew.imagedomain = pnew.paradomain;
						line_.push_back(pnew);
					}
				}	
			}
			new_lines[add_index[i].first] = line_;
		}
	}
	

	//compute index
	index_ = 0;
	Norepeat_knot.clear();
	for (int i = 0; i < new_lines.size(); i++)
	{
		for (int j = 0; j < new_lines[i].size(); j++)
		{
			bool is_find = false;
			for (int k = 0; k < Norepeat_knot.size(); k++)
			{
				if (new_lines[i][j].index_x == Norepeat_knot[k].index_x &&
					new_lines[i][j].index_y == Norepeat_knot[k].index_y)
				{
					is_find = true;
					new_lines[i][j].index = Norepeat_knot[k].index;
					break;
				}
			}
			if (!is_find)
			{
				new_lines[i][j].index = index_;
				index_++;
				Norepeat_knot.push_back(new_lines[i][j]);
			}
		}
	}

	collinear_knots_lines = new_lines;

	//predicate if exist boundary polyline
	vector<vector<PointTriple>> corner_bound;
	for (int i = 0; i < new_lines.size(); i++)
	{
		if (new_lines[i][0].index_x == new_lines[i][new_lines[i].size() - 1].index_x &&
			new_lines[i][0].index_y == new_lines[i][new_lines[i].size() - 1].index_y)
		{
			continue;
		}
		else
		{
			double length_ = sqrt((new_lines[i][0].imagedomain - new_lines[i][1].imagedomain).squared_length()) / 4.0;
			if (new_lines[i][0].index_x == 0 || new_lines[i][0].index_x == input_image->width() - 1)
			{
				double y = new_lines[i][0].imagedomain.y() + length_;
				if (y > 1.0)
				{
					y = new_lines[i][0].imagedomain.y() - length_;
				}
				Point_2 pt(new_lines[i][0].imagedomain.x(), y);
				PointTriple p1; p1.imagedomain = new_lines[i][1].imagedomain; p1.index = new_lines[i][1].index;
				PointTriple p0; p0.imagedomain = new_lines[i][0].imagedomain; p0.index = new_lines[i][0].index;
				PointTriple pb; pb.imagedomain = pt; pb.index = -1;
				PointTriple p2; p2.imagedomain = new_lines[i][2].imagedomain; p2.index = new_lines[i][2].index;
				if (nDeg ==2)
				{
					corner_bound.push_back({ pb,p0,p1,p2 });
				}
				if (nDeg == 3)
				{
					PointTriple p3; p3.imagedomain = new_lines[i][3].imagedomain; p3.index = new_lines[i][3].index;
					corner_bound.push_back({ pb,p0,p1,p2,p3 });
				}		
			}
			else if (new_lines[i][0].index_y == 0 || new_lines[i][0].index_y == input_image->height() - 1)
			{
				double x = new_lines[i][0].imagedomain.x() + length_;
				if (x > 1.0)
				{
					x = new_lines[i][0].imagedomain.x() - length_;
				}
				Point_2 pt(x, new_lines[i][0].imagedomain.y());
				PointTriple p1; p1.imagedomain = new_lines[i][1].imagedomain; p1.index = new_lines[i][1].index;
				PointTriple p0; p0.imagedomain = new_lines[i][0].imagedomain; p0.index = new_lines[i][0].index;
				PointTriple pb; pb.imagedomain = pt; pb.index = -1;
				PointTriple p2; p2.imagedomain = new_lines[i][2].imagedomain; p2.index = new_lines[i][2].index;
				if (nDeg == 2)
				{
					corner_bound.push_back({ pb,p0,p1,p2 });
				}
				if (nDeg == 3)
				{
					PointTriple p3; p3.imagedomain = new_lines[i][3].imagedomain; p3.index = new_lines[i][3].index;
					corner_bound.push_back({ pb,p0,p1,p2,p3 });
				}
			}

			length_ = sqrt((new_lines[i][new_lines[i].size() - 1].imagedomain - new_lines[i][new_lines[i].size() - 2].imagedomain).squared_length()) / 4.0;
			if (new_lines[i][new_lines[i].size() - 1].index_x == 0 ||
				new_lines[i][new_lines[i].size() - 1].index_x == input_image->width() - 1)
			{
				double y = new_lines[i][new_lines[i].size() - 1].imagedomain.y() + length_;
				if (y > 1.0)
				{
					y = new_lines[i][new_lines[i].size() - 1].imagedomain.y() - length_;
				}
				Point_2 pt(new_lines[i][new_lines[i].size() - 1].imagedomain.x(), y);

				PointTriple p2; p2.imagedomain = new_lines[i][new_lines[i].size() - 2].imagedomain; p2.index = new_lines[i][new_lines[i].size() - 2].index;
				PointTriple p1; p1.imagedomain = new_lines[i][new_lines[i].size() - 1].imagedomain; p1.index = new_lines[i][new_lines[i].size() - 1].index;
				PointTriple pb; pb.imagedomain = pt; pb.index = -1;
				PointTriple p3; p3.imagedomain = new_lines[i][new_lines[i].size() - 3].imagedomain; p3.index = new_lines[i][new_lines[i].size() - 3].index;
				if (nDeg == 2)
				{
					corner_bound.push_back({ pb,p1,p2,p3 });
				}
				if (nDeg == 3)
				{
					PointTriple p4; p4.imagedomain = new_lines[i][new_lines[i].size() - 4].imagedomain; p4.index = new_lines[i][new_lines[i].size() - 4].index;
					corner_bound.push_back({ pb,p1,p2,p3,p4 });
				}
			}
			else if (new_lines[i][new_lines[i].size() - 1].index_y == 0 ||
				new_lines[i][new_lines[i].size() - 1].index_y == input_image->height() - 1)
			{
				double x = new_lines[i][new_lines[i].size() - 1].imagedomain.x() + length_;
				if (x > 1.0)
				{
					x = new_lines[i][new_lines[i].size() - 1].imagedomain.x() - length_;
				}
				Point_2 pt(x, new_lines[i][new_lines[i].size() - 1].imagedomain.y());
				PointTriple p2; p2.imagedomain = new_lines[i][new_lines[i].size() - 2].imagedomain; p2.index = new_lines[i][new_lines[i].size() - 2].index;
				PointTriple p1; p1.imagedomain = new_lines[i][new_lines[i].size() - 1].imagedomain; p1.index = new_lines[i][new_lines[i].size() - 1].index;
				PointTriple pb; pb.imagedomain = pt; pb.index = -1;
				PointTriple p3; p3.imagedomain = new_lines[i][new_lines[i].size() - 3].imagedomain; p3.index = new_lines[i][new_lines[i].size() - 3].index;
				if (nDeg == 2)
				{
					corner_bound.push_back({ pb,p1,p2,p3 });
				}
				if (nDeg == 3)
				{
					PointTriple p4; p4.imagedomain = new_lines[i][new_lines[i].size() - 4].imagedomain; p4.index = new_lines[i][new_lines[i].size() - 4].index;
					corner_bound.push_back({ pb,p1,p2,p3,p4 });
				}
			}
		}
	}

	//make cdt
#if 1
	CDT feature_cdt;
	map<int, CDT::Vertex_handle> cdt_vertex_handle;
	for (int i = 0; i < Norepeat_knot.size(); i++)
	{
		CDT::Vertex_handle vh;
		double x = Norepeat_knot[i].imagedomain.x() * 2.0*imagemesh_dx - imagemesh_dx;
		double y = Norepeat_knot[i].imagedomain.y() * 2.0*imagemesh_dy - imagemesh_dy;
		vh = feature_cdt.insert(Point_2(x, y));
		vh->set_associated_index(Norepeat_knot[i].index);
		cdt_vertex_handle[Norepeat_knot[i].index] = vh;
	}

	for (int i = 0; i < new_lines.size(); i++)
	{
		for (int j = 0; j < new_lines[i].size() - 1; j++)
		{
			int edge_f = new_lines[i][j].index;
			int edge_t = new_lines[i][j + 1].index;

			//std::cout << new_lines[i][j].imagedomain.x() << "," << new_lines[i][j].imagedomain.y() << "\n";
			//std::cout << new_lines[i][j + 1].imagedomain.x() << "," << new_lines[i][j + 1].imagedomain.y() << "\n";
			//if (edge_f == 369 || edge_f == 370 || edge_t == 369 || edge_t == 370)
			{
				//std::cout << edge_f <<"  --  " << edge_t << "\n";
			}	

			feature_cdt.insert_constraint(cdt_vertex_handle.at(edge_f), cdt_vertex_handle.at(edge_t));
		}
	}
	std::cout << __FUNCTION__ << ": " << "insert collinear segments  " << std::endl;

	for (int i = 0; i < new_lines.size(); i++)
	{
		if (new_lines[i].size() >= 3)
		{
			if (new_lines[i][0].index == new_lines[i][new_lines[i].size() - 1].index)
			{
				int edge_f = new_lines[i][1].index;
				int edge_t = new_lines[i][new_lines[i].size() - 2].index;

				feature_cdt.insert_constraint(cdt_vertex_handle.at(edge_f), cdt_vertex_handle.at(edge_t));
			}
		}
		if (nknotinsert_collinear > 0) {
		for (int j = 0; j < new_lines[i].size();)
		{	
				if (new_lines[i].size() <= nknotinsert_collinear + 2)
				{
					break;
				}

				j++;
				if (j > new_lines[i].size() - 1 || !new_lines[i][j].is_fixed)
				{
					continue;
				}
				if (j < new_lines[i].size() - 1)
				{
					int edge_f = new_lines[i][j - 1].index;
					int edge_t = new_lines[i][j + 1].index;

					//std::cout << edge_f << "  --  " << edge_t << "\n";

					feature_cdt.insert_constraint(cdt_vertex_handle.at(edge_f), cdt_vertex_handle.at(edge_t));
				}
			}
		}
	}
	std::cout << __FUNCTION__ << ": " << "insert corner segments  " << std::endl;

	//insert corner
	int index = Norepeat_knot.size();
	int index_start0 = index;
	vector<Point_2> image_bound = { 
		Point_2(-imagemesh_dx, -imagemesh_dy) ,
		Point_2(-imagemesh_dx, 0),
		Point_2(-imagemesh_dx, imagemesh_dy), 
		Point_2(0.0, imagemesh_dy),
		Point_2(imagemesh_dx, imagemesh_dy),
		Point_2(imagemesh_dx, 0),
		Point_2(imagemesh_dx, -imagemesh_dy) ,
		Point_2(0.0, -imagemesh_dy) };

	vector<Point_2> domain_bound = { 
		Point_2(0.0, 0.0) , 
		Point_2(0.0, 0.5),
		Point_2(0.0, 1.0) ,
		Point_2(0.5, 1.0),
		Point_2(1.0, 1.0) , 
		Point_2(1.0, 0.5),
		Point_2(1.0, 0.0) ,
		Point_2(0.5, 0.0) };

	for (int i = 0; i < image_bound.size(); i++)
	{
		CDT::Vertex_handle vh;
		vh = feature_cdt.insert(image_bound[i]);
		vh->set_associated_index(index);
		cdt_vertex_handle[index] = vh;
		PointTriple p;
		p.imagedomain = domain_bound[i];
		p.paradomain = p.imagedomain;
		p.index = index;
		Norepeat_knot.push_back(p);
		index++;
	}
	for (int i = 0; i < image_bound.size(); i++)
	{
		feature_cdt.insert_constraint(cdt_vertex_handle.at(index_start0 + i%image_bound.size()),
			cdt_vertex_handle.at(index_start0 + (i + 1) % image_bound.size()));
	}
	std::cout << __FUNCTION__ << ": " << "insert corner constrains  " << std::endl;

	//insert feature corner bound
	index = Norepeat_knot.size();
	int index_start1 = index;
	for (int i = 0; i<corner_bound.size(); i++)
	{
		Point_2 extraimage_bound(corner_bound[i][0].imagedomain.x() - imagemesh_dx, corner_bound[i][0].imagedomain.y()*2.0*imagemesh_dy - imagemesh_dy);
		corner_bound[i][0].imagep = extraimage_bound;

		CDT::Vertex_handle vh;
		vh = feature_cdt.insert(extraimage_bound);
		vh->set_associated_index(index);
		cdt_vertex_handle[index] = vh;
		PointTriple p;
		p.imagedomain = corner_bound[i][0].imagedomain;
		p.paradomain = p.imagedomain;
		p.index = index;
		corner_bound[i][0].index = index;
		Norepeat_knot.push_back(p);
		index++;
	}
	for (int i = 0; i<corner_bound.size(); i++)
	{
		feature_cdt.insert_constraint(cdt_vertex_handle.at(corner_bound[i][0].index),
			cdt_vertex_handle.at(corner_bound[i][2].index));
		assert(corner_bound[i][0].index == index_start1 + i);
	}
	feature_bound_corner = corner_bound;
	std::cout << __FUNCTION__ << ": " << "insert feature segment beside to boundary" << std::endl;

	index = feature_cdt.number_of_vertices();
	double dis_fea = 2.0*remove_criteria_closeto_featureedge / input_image->width();
	//insert feature neighbor- fixed
#if 0
	for (int i = 0; i < fixed_lines.size(); i++)
	{
		if (fixed_lines[i][0].index_x != fixed_lines[i][fixed_lines[i].size() - 1].index_x ||
			fixed_lines[i][0].index_y != fixed_lines[i][fixed_lines[i].size() - 1].index_y)
		{
			if (!(fixed_lines[i][0].index_x == input_image->width() - 1 || fixed_lines[i][0].index_x == 0 
				|| fixed_lines[i][0].index_y == 0 || fixed_lines[i][0].index_y == input_image->height() - 1))
			{
				double x = fixed_lines[i][1].paradomain.x();
				double y = fixed_lines[i][1].paradomain.y();
				Point_2 from(x, y);
				x = fixed_lines[i][0].paradomain.x();
				y = fixed_lines[i][0].paradomain.y();
				Point_2 to(x, y);
				Vector_2 perp((to - from).y(), -(to - from).x());
				Vector_2 norpp = perp / sqrt(perp.squared_length()) * dis_fea;
				Vector_2 center = (to - Point_2(0, 0)) - (from - to) / sqrt((from - to).squared_length()) * dis_fea;

				Point_2 p1(center.x() + norpp.x(), center.y() + norpp.y());
				Point_2 p1image(p1.x() * 2.0*imagemesh_dx - imagemesh_dx, p1.y()* 2.0*imagemesh_dy - imagemesh_dy);
				Point_2 p2(center.x() - norpp.x(), center.y() - norpp.y());
				Point_2 p2image(p2.x() * 2.0*imagemesh_dx - imagemesh_dx, p2.y()* 2.0*imagemesh_dy - imagemesh_dy);
				CDT::Vertex_handle v1 = feature_cdt.insert(p1image);
				v1->set_associated_index(index);
				cdt_vertex_handle[index] = v1;

					PointTriple pt1;
					pt1.imagedomain = p1;
					pt1.paradomain = p1;
					pt1.index = index;
					Norepeat_knot.push_back(pt1);
				index++;

				CDT::Vertex_handle v2 = feature_cdt.insert(p2image);
				v2->set_associated_index(index);
				cdt_vertex_handle[index] = v2;
					PointTriple pt2;
					pt2.imagedomain = p2;
					pt2.paradomain = p2;
					pt2.index = index;
					Norepeat_knot.push_back(pt2);
				index++;	
			}
			if (!(fixed_lines[i][fixed_lines[i].size()-1].index_x == input_image->width() - 1 || 
				fixed_lines[i][fixed_lines[i].size() - 1].index_x == 0
				|| fixed_lines[i][fixed_lines[i].size() - 1].index_y == 0 || 
				fixed_lines[i][fixed_lines[i].size() - 1].index_y == input_image->height() - 1))
			{
				double x = fixed_lines[i][fixed_lines[i].size() - 2].paradomain.x();
				double y = fixed_lines[i][fixed_lines[i].size() - 2].paradomain.y();
				Point_2 from(x, y);
				x = fixed_lines[i][fixed_lines[i].size() - 1].paradomain.x();
				y = fixed_lines[i][fixed_lines[i].size() - 1].paradomain.y();
				Point_2 to(x, y);
				Vector_2 perp((to - from).y(), -(to - from).x());
				Vector_2 norpp = perp / sqrt(perp.squared_length()) * dis_fea;
				Vector_2 center = (to - Point_2(0, 0)) - (from - to) / sqrt((from - to).squared_length()) * dis_fea;

				Point_2 p1(center.x() + norpp.x(), center.y() + norpp.y());
				Point_2 p1image(p1.x() * 2.0*imagemesh_dx - imagemesh_dx, p1.y()* 2.0*imagemesh_dy - imagemesh_dy);
				Point_2 p2(center.x() - norpp.x(), center.y() - norpp.y());
				Point_2 p2image(p2.x() * 2.0*imagemesh_dx - imagemesh_dx, p2.y()* 2.0*imagemesh_dy - imagemesh_dy);
				CDT::Vertex_handle v1 = feature_cdt.insert(p1image);
				v1->set_associated_index(index);
				cdt_vertex_handle[index] = v1;

					PointTriple pt1;
					pt1.imagedomain = p1;
					pt1.paradomain = p1;
					pt1.index = index;
					Norepeat_knot.push_back(pt1);
				index++;

				CDT::Vertex_handle v2 = feature_cdt.insert(p2image);
				v2->set_associated_index(index);
				cdt_vertex_handle[index] = v2;
					PointTriple pt2;
					pt2.imagedomain = p2;
					pt2.paradomain = p2;
					pt2.index = index;
					Norepeat_knot.push_back(pt2);
				index++;
			}
		}

		for (int j = 0; j < fixed_lines[i].size() - 1; j++)
		{
			double x = fixed_lines[i][j].paradomain.x();
			double y = fixed_lines[i][j].paradomain.y();
			Point_2 from(x, y);
			x = fixed_lines[i][j + 1].paradomain.x();
			y = fixed_lines[i][j + 1].paradomain.y();
			Point_2 to(x, y);
			Vector_2 perp((to - from).y(), -(to - from).x());
			Vector_2 norpp = perp / sqrt(perp.squared_length()) * dis_fea;
			Point_2 center1((from.x() + 2.0*to.x()) / 3.0, (from.y() + 2.0*to.y()) / 3.);
			Point_2 center2((2.0*from.x() + to.x()) / 3.0, (2.0*from.y() + to.y()) / 3.);

			Point_2 p1 = center1 + norpp;
			Point_2 p1image(p1.x() * 2.0*imagemesh_dx - imagemesh_dx, p1.y()* 2.0*imagemesh_dy - imagemesh_dy);
			Point_2 p2 = center2 - norpp;
			Point_2 p2image(p2.x() * 2.0*imagemesh_dx - imagemesh_dx, p2.y()* 2.0*imagemesh_dy - imagemesh_dy);

			CDT::Vertex_handle v1 = feature_cdt.insert(p1image);
			v1->set_associated_index(index);
			cdt_vertex_handle[index] = v1;
			PointTriple pt1;
			pt1.imagedomain = p1;
			pt1.paradomain = p1;
			pt1.index = index;
			Norepeat_knot.push_back(pt1);
			index++;

			CDT::Vertex_handle v2 = feature_cdt.insert(p2image);
			v2->set_associated_index(index);
			cdt_vertex_handle[index] = v2;
			PointTriple pt2;
			pt2.imagedomain = p2;
			pt2.paradomain = p2;
			pt2.index = index;
			Norepeat_knot.push_back(pt2);
			index++;
		}			
	}
#endif
	//insert feature neighbor- movable
	if (Nite_now == 0 && do_insert_feature_neighbor_knots)
	{
		for (int i = 0; i < fixed_lines.size(); i++)
		{
			if (fixed_lines[i][0].index_x != fixed_lines[i][fixed_lines[i].size() - 1].index_x ||
				fixed_lines[i][0].index_y != fixed_lines[i][fixed_lines[i].size() - 1].index_y)
			{
				if (!(fixed_lines[i][0].index_x == input_image->width() - 1 || fixed_lines[i][0].index_x == 0
					|| fixed_lines[i][0].index_y == 0 || fixed_lines[i][0].index_y == input_image->height() - 1))
				{
					double x = fixed_lines[i][1].paradomain.x();
					double y = fixed_lines[i][1].paradomain.y();
					Point_2 from(x, y);
					x = fixed_lines[i][0].paradomain.x();
					y = fixed_lines[i][0].paradomain.y();
					Point_2 to(x, y);

					Vector_2 dirnorm = (to - from) / sqrt((to - from).squared_length()) * dis_fea;
					Point_2 p1(to.x() + dirnorm.x(), to.y() + dirnorm.y());
					Point_2 p1image(p1.x() * 2.0*imagemesh_dx - imagemesh_dx, p1.y()* 2.0*imagemesh_dy - imagemesh_dy);
					CDT::Vertex_handle v1 = feature_cdt.insert(p1image);
					v1->set_associated_index(index);
					cdt_vertex_handle[index] = v1;
					index++;

					Vector_2 perp((to - from).y(), -(to - from).x());
					Vector_2 norpp = perp / sqrt(perp.squared_length()) * dis_fea;
					Vector_2 center1 = (to - Point_2(0, 0)) + (from - to) / sqrt((from - to).squared_length()) * dis_fea;
					Point_2 p3(center1.x() + norpp.x(), center1.y() + norpp.y());
					Point_2 p3image(p3.x() * 2.0*imagemesh_dx - imagemesh_dx, p3.y()* 2.0*imagemesh_dy - imagemesh_dy);
					Point_2 p4(center1.x() - norpp.x(), center1.y() - norpp.y());
					Point_2 p4image(p4.x() * 2.0*imagemesh_dx - imagemesh_dx, p4.y()* 2.0*imagemesh_dy - imagemesh_dy);
#if !REMOVE_ONE_ONLY//only one p added
					CDT::Vertex_handle v3 = feature_cdt.insert(p3image);
					v3->set_associated_index(index);
					cdt_vertex_handle[index] = v3;
					index++;
#endif

					CDT::Vertex_handle v4 = feature_cdt.insert(p4image);
					v4->set_associated_index(index);
					cdt_vertex_handle[index] = v4;
					index++;
				}
				if (!(fixed_lines[i][fixed_lines[i].size() - 1].index_x == input_image->width() - 1 ||
					fixed_lines[i][fixed_lines[i].size() - 1].index_x == 0
					|| fixed_lines[i][fixed_lines[i].size() - 1].index_y == 0 ||
					fixed_lines[i][fixed_lines[i].size() - 1].index_y == input_image->height() - 1))
				{
					double x = fixed_lines[i][fixed_lines[i].size() - 2].paradomain.x();
					double y = fixed_lines[i][fixed_lines[i].size() - 2].paradomain.y();
					Point_2 from(x, y);
					x = fixed_lines[i][fixed_lines[i].size() - 1].paradomain.x();
					y = fixed_lines[i][fixed_lines[i].size() - 1].paradomain.y();
					Point_2 to(x, y);

					Vector_2 dirnorm = (to - from) / sqrt((to - from).squared_length()) * dis_fea;
					Point_2 p1(to.x() + dirnorm.x(), to.y() + dirnorm.y());
					Point_2 p1image(p1.x() * 2.0*imagemesh_dx - imagemesh_dx, p1.y()* 2.0*imagemesh_dy - imagemesh_dy);
					CDT::Vertex_handle v1 = feature_cdt.insert(p1image);
					v1->set_associated_index(index);
					cdt_vertex_handle[index] = v1;
					index++;

					Vector_2 perp((to - from).y(), -(to - from).x());
					Vector_2 norpp = perp / sqrt(perp.squared_length()) * dis_fea;
					Vector_2 center1 = (to - Point_2(0, 0)) + (from - to) / sqrt((from - to).squared_length()) * dis_fea;
					Point_2 p3(center1.x() + norpp.x(), center1.y() + norpp.y());
					Point_2 p3image(p3.x() * 2.0*imagemesh_dx - imagemesh_dx, p3.y()* 2.0*imagemesh_dy - imagemesh_dy);
					Point_2 p4(center1.x() - norpp.x(), center1.y() - norpp.y());
					Point_2 p4image(p4.x() * 2.0*imagemesh_dx - imagemesh_dx, p4.y()* 2.0*imagemesh_dy - imagemesh_dy);
#if !REMOVE_ONE_ONLY//only one p added
					CDT::Vertex_handle v3 = feature_cdt.insert(p3image);
					v3->set_associated_index(index);
					cdt_vertex_handle[index] = v3;
					index++;
#endif

					CDT::Vertex_handle v4 = feature_cdt.insert(p4image);
					v4->set_associated_index(index);
					cdt_vertex_handle[index] = v4;
					index++;
				}
			}

			for (int j = 0; j < fixed_lines[i].size() - 1; j++)
			//for (int j = 1; j < fixed_lines[i].size() - 2; j++)
			{
				double x = fixed_lines[i][j].paradomain.x();
				double y = fixed_lines[i][j].paradomain.y();
				Point_2 from(x, y);
				x = fixed_lines[i][j + 1].paradomain.x();
				y = fixed_lines[i][j + 1].paradomain.y();
				Point_2 to(x, y);
				Vector_2 perp((to - from).y(), -(to - from).x());
				Vector_2 norpp = perp / sqrt(perp.squared_length()) * dis_fea;
				Point_2 center1((from.x() + 2.0*to.x()) / 3.0, (from.y() + 2.0*to.y()) / 3.);
				Point_2 center2((2.0*from.x() + to.x()) / 3.0, (2.0*from.y() + to.y()) / 3.);

				Point_2 p3 = center1 - norpp;
				Point_2 p3image(p3.x() * 2.0*imagemesh_dx - imagemesh_dx, p3.y()* 2.0*imagemesh_dy - imagemesh_dy);
				Point_2 p4 = center2 + norpp;
				Point_2 p4image(p4.x() * 2.0*imagemesh_dx - imagemesh_dx, p4.y()* 2.0*imagemesh_dy - imagemesh_dy);

#if !REMOVE_ONE_ONLY//only one p added
				CDT::Vertex_handle v3 = feature_cdt.insert(p3image);
				v3->set_associated_index(index);
				cdt_vertex_handle[index] = v3;
				index++;
#endif

				CDT::Vertex_handle v4 = feature_cdt.insert(p4image);
				v4->set_associated_index(index);
				cdt_vertex_handle[index] = v4;
				index++;
			}

			//adding one knot in the position with large rotating angle
#if 0
			if (fixed_lines[i][0].index_x != fixed_lines[i][fixed_lines[i].size() - 1].index_x ||
				fixed_lines[i][0].index_y != fixed_lines[i][fixed_lines[i].size() - 1].index_y)
			{
				for (int j = 1; j < fixed_lines[i].size() - 1; j++)
				{
					double x = fixed_lines[i][j].paradomain.x();
					double y = fixed_lines[i][j].paradomain.y();
					Point_2 mid(x, y);
					x = fixed_lines[i][j + 1].paradomain.x();
					y = fixed_lines[i][j + 1].paradomain.y();
					Point_2 nex(x, y);
					x = fixed_lines[i][j - 1].paradomain.x();
					y = fixed_lines[i][j - 1].paradomain.y();
					Point_2 pre(x, y);

					Vector_2 v1 = (pre - mid) / sqrt((pre - mid).squared_length());
					Vector_2 v2 = (nex - mid) / sqrt((nex - mid).squared_length());

					if (v1*v2 > 0)
					{
						Vector_2 vtemp = (v1 + v2) / sqrt((v1 + v2).squared_length()) * dis_fea;

						Point_2 p3 = mid - vtemp;
						Point_2 p3image(p3.x() * 2.0*imagemesh_dx - imagemesh_dx, p3.y()* 2.0*imagemesh_dy - imagemesh_dy);

						CDT::Vertex_handle v3 = feature_cdt.insert(p3image);
						v3->set_associated_index(index);
						cdt_vertex_handle[index] = v3;
						index++;
					}
				}
			}
			else
			{
				for (int j = 0; j < fixed_lines[i].size(); j++)
				{
					double x = fixed_lines[i][j].paradomain.x();
					double y = fixed_lines[i][j].paradomain.y();
					Point_2 mid(x, y);
					x = fixed_lines[i][(j + 1) % fixed_lines[i].size()].paradomain.x();
					y = fixed_lines[i][(j + 1) % fixed_lines[i].size()].paradomain.y();
					Point_2 nex(x, y);
					x = fixed_lines[i][(j - 1 + fixed_lines[i].size()) % fixed_lines[i].size()].paradomain.x();
					y = fixed_lines[i][(j - 1 + fixed_lines[i].size()) % fixed_lines[i].size()].paradomain.y();
					Point_2 pre(x, y);

					Vector_2 v1 = (pre - mid) / sqrt((pre - mid).squared_length());
					Vector_2 v2 = (nex - mid) / sqrt((nex - mid).squared_length());

					if (v1*v2 > 0)
					{
						Vector_2 vtemp = (v1 + v2) / sqrt((v1 + v2).squared_length()) * dis_fea;

						Point_2 p3 = mid - vtemp;
						Point_2 p3image(p3.x() * 2.0*imagemesh_dx - imagemesh_dx, p3.y()* 2.0*imagemesh_dy - imagemesh_dy);

						CDT::Vertex_handle v3 = feature_cdt.insert(p3image);
						v3->set_associated_index(index);
						cdt_vertex_handle[index] = v3;
						index++;
					}
				}
			}
#endif
		}
	}

	std::cout << __FUNCTION__ << ": " << "feature-knot finished, number : " << feature_cdt.number_of_vertices() << std::endl;

	QString dir_featureknot_edge = out_file_name;
	dir_featureknot_edge.append("/midprocess_featureknotsedge.txt");

	//write constrained edge into txt file
	std::ofstream fout;
	const string str_name = dir_featureknot_edge.toStdString();
	fout.open(str_name);
	if (fout.is_open())
	{
		for (int i = 0; i < new_lines.size(); i++)
		{
			fout << new_lines[i].size() - 1 << " ";
			for (int j = 0; j < new_lines[i].size() - 1; j++)
			{
				fout << new_lines[i][j].index << " " << new_lines[i][j + 1].index << " ";
			}
			fout << "\n";
		}

		for (int i = 0; i < new_lines.size(); i++)
		{
			if (nknotinsert_collinear > 0) {
			for (int j = 0; j < new_lines[i].size();)
			{
				
					if (new_lines[i].size() <= nknotinsert_collinear + 2)
					{
						break;
					}
					j++;
					if (j > new_lines[i].size() - 1 || !new_lines[i][j].is_fixed)
					{
						continue;
					}
					if (j < new_lines[i].size() - 1)
					{
						fout << 1 << " ";
						fout << new_lines[i][j - 1].index << " " << new_lines[i][j + 1].index << "\n";
					}
				}
			}
			if (new_lines[i].size() >= 3)
			{
				if (new_lines[i][0].index == new_lines[i][new_lines[i].size() - 1].index)
				{
					fout << 1 << " ";
					fout << new_lines[i][new_lines[i].size() - 2].index << " " << new_lines[i][1].index << "\n";
				}
			}
		}

		//boundary
		fout << image_bound.size() << " ";
		for (int i = 0; i < image_bound.size(); i++)
		{
			fout << index_start0 + i%image_bound.size() << " " << index_start0 + (i + 1) % image_bound.size() << " ";
		}

		for (int i = 0; i < corner_bound.size(); i++)
		{
			fout << 1 << " ";
			fout << corner_bound[i][0].index << " " << corner_bound[i][2].index << "\n";
		}
	}
	fout.close();

	QString dir_featureknot_cdt = out_file_name;
	dir_featureknot_cdt.append("/midprocess_featureknotscdt.obj");

	//write CDT- to obj file
	const string obj_name = dir_featureknot_cdt.toStdString();
	std::ofstream fout_obj;
	fout_obj.open(obj_name);
	if (fout_obj.is_open())
	{
		for (int i = 0; i < feature_cdt.number_of_vertices(); i++)
		{
			CDT::Vertex_handle vh = cdt_vertex_handle.at(i);
			if (vh->get_associated_index() == i)
			{
				double x = vh->point().x();
				double y = vh->point().y();
				double z = 0.0;
				fout_obj << "v" << " " << x << " " << y << " " << z << "\n";
			}
		}
		for (auto fit = feature_cdt.faces_begin(); fit != feature_cdt.faces_end(); fit++)
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

	std::cout << "/////////////////////////////build collinear knots finished" << simply_error << std::endl;

#endif

	do_compute_collinear_knots = true;
	return true;
}

void Image_Approximation::optimize_knots_triangulation()
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
	if (Nite_now == 0)
	{
		opti_knottri->init_greedy_mesh(Norepeat_knot.size() + nknotinsert_opti);
	}
	if (Nite_now == 0)
		opti_knottri->optimize_mesh(niterate_opti, dir_allknot.toLatin1(), 1);
	else
		opti_knottri->optimize_mesh(4, dir_allknot.toLatin1(), 1);

	//knot mesh
	knot_mesh = new Mesh;
	read_mesh(&knot_mesh, dir_allknot);
	nknot = knot_mesh->size_of_vertices();

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
			Point_2 p;

			double x = vit->point().x();
			double y = vit->point().y();
			p = Point_2((x + imagemesh_dx) / (2.0*imagemesh_dx), (y + imagemesh_dy) / (2.0*imagemesh_dy));

			vit->get_domain() = p;
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

	/**********************************************make cdt***************************************************/
	//from knot mesh to cdt
	knots_config = new CDT;
	map<int, CDT::Vertex_handle> vertex_handle_cdt;
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
	//add feature corner constrains
	for (int i = 0; i < collinear_knots_lines.size(); i++)
	{
		if (collinear_knots_lines[i].size() >= 3)
		{
			if (collinear_knots_lines[i][0].index == collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index)
			{
				int edge_f = collinear_knots_lines[i][1].index;
				int edge_t = collinear_knots_lines[i][collinear_knots_lines[i].size() - 2].index;

				knots_config->insert_constraint(vertex_handle_cdt.at(edge_f), vertex_handle_cdt.at(edge_t));
			}
		}
		if (nknotinsert_collinear > 0) {
		for (int j = 0; j < collinear_knots_lines[i].size();)
		{
			
				if (collinear_knots_lines[i].size() <= nknotinsert_collinear + 2)
				{
					break;
				}
				j++;
				if (j > collinear_knots_lines[i].size() - 1 || !collinear_knots_lines[i][j].is_fixed)
				{
					continue;
				}
				if (j < collinear_knots_lines[i].size() - 1)
				{
					int edge_f = collinear_knots_lines[i][j - 1].index;
					int edge_t = collinear_knots_lines[i][j + 1].index;

					knots_config->insert_constraint(vertex_handle_cdt.at(edge_f), vertex_handle_cdt.at(edge_t));
				}
			}
		}
	}
	for (int i = 0; i<feature_bound_corner.size(); i++)
	{
		knots_config->insert_constraint(vertex_handle_cdt.at(feature_bound_corner[i][0].index), vertex_handle_cdt.at(feature_bound_corner[i][2].index));
	}
	/////////////second step/////////////////
	vector<int> knots_closeto_feature;
	vector<int> knots_in_feature_corner;
	for (auto vit = knot_mesh->vertices_begin(); vit != knot_mesh->vertices_end(); vit++)
	{
		bool is_remove = false;
		if (vit->vertex_index() >= Norepeat_knot.size())
		{
			//remove knots in feature corner triangle
			for (int i = 0; i < collinear_knots_lines.size(); i++)
			{
				if (collinear_knots_lines[i].size() >= 3)
				{
					if (collinear_knots_lines[i][0].index == collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index)
					{
						vector<Point_2> triangles = { collinear_knots_lines[i][collinear_knots_lines[i].size() - 2].paradomain,
							collinear_knots_lines[i][0].paradomain,
							collinear_knots_lines[i][1].paradomain,
						};
						Point_2 pdomain = vit->get_domain();
						int ret = CGAL::bounded_side_2(triangles.begin(), triangles.end(), pdomain, K());
						if (ret == CGAL::ON_BOUNDED_SIDE || is_on_polygon_convex_bound(pdomain, triangles))
						{
							is_remove = true;
							knots_in_feature_corner.push_back(vit->vertex_index());
							printf("%s: remove knot locating in feature corner \n", __FUNCTION__);
						}
					}
				}
				if (nknotinsert_collinear > 0) {
				for (int j = 0; j < collinear_knots_lines[i].size();)
				{					
						if (collinear_knots_lines[i].size() <= nknotinsert_collinear + 2)
						{
							break;
						}
						j++;
						if (j > collinear_knots_lines[i].size() - 1 || !collinear_knots_lines[i][j].is_fixed)
						{
							continue;
						}
						if (j < collinear_knots_lines[i].size() - 1)
						{
							vector<Point_2> triangles = { collinear_knots_lines[i][j - 1].paradomain,
								collinear_knots_lines[i][j].paradomain,
								collinear_knots_lines[i][j + 1].paradomain,
							};
							Point_2 pdomain = vit->get_domain();
							int ret = CGAL::bounded_side_2(triangles.begin(), triangles.end(), pdomain, K());
							if (ret == CGAL::ON_BOUNDED_SIDE || is_on_polygon_convex_bound(pdomain, triangles))
							{
								is_remove = true;
								knots_in_feature_corner.push_back(vit->vertex_index());
								std::cout << __FUNCTION__ << ": " << "remove knot locating in feature corner" << std::endl;
							}
						}
						if (is_remove)
						{
							break;
						}
					}
				}
			}

			//remove knots in feature bound corner triangle
			for (int i = 0; i < feature_bound_corner.size(); i++)
			{
				vector<Point_2> triangles = { feature_bound_corner[i][0].imagedomain,
					feature_bound_corner[i][1].imagedomain,
					feature_bound_corner[i][2].imagedomain
				};
				Point_2 pdomain = vit->get_domain();
				int ret = CGAL::bounded_side_2(triangles.begin(), triangles.end(), pdomain, K());
				if (ret == CGAL::ON_BOUNDED_SIDE || is_on_polygon_convex_bound(pdomain, triangles))
				{
					is_remove = true;
					knots_in_feature_corner.push_back(vit->vertex_index());
					std::cout << __FUNCTION__ << ": " << "remove knot locating in bound corner" << std::endl;
					break;
				}
			}

#if 1
			//remove knots pretty close to feature corner triangle edge
			for (int i = 0; i < collinear_knots_lines.size(); i++)
			{
				if (collinear_knots_lines[i].size() >= 3)
				{
					if (collinear_knots_lines[i][0].index == collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index)
					{
						Point_2 linef(collinear_knots_lines[i][collinear_knots_lines[i].size() - 2].paradomain);
						Point_2 linet(collinear_knots_lines[i][1].paradomain);
						if (vit->get_domain().x() > std::min(linef.x(), linet.x()) && vit->get_domain().x() < std::max(linef.x(), linet.x()) ||
							vit->get_domain().y() > std::min(linef.y(), linet.y()) && vit->get_domain().y() < std::max(linef.y(), linet.y()))
						{
							if (dis_between_p_line(linef, linet, vit->get_domain()) < remove_criteria_closeto_featureedge / input_image->width())
							{
								is_remove = true;
								knots_closeto_feature.push_back(vit->vertex_index());
								std::cout << __FUNCTION__ << ": " << "new knot: " << vit->vertex_index() << "-coord:(" << vit->get_domain().x() << "," << vit->get_domain().y()
									<< ") is almost on feature line, area: "
									<< abs(CGAL::area(vit->get_domain(), linet, linef)) << ". so, remove it" << std::endl;
								break;
							}
						}
					}
				}
				if (nknotinsert_collinear > 0) {
				for (int j = 0; j < collinear_knots_lines[i].size();)
				{
					
						if (collinear_knots_lines[i].size() <= nknotinsert_collinear + 2)
						{
							break;
						}

						j++;
						if (j > collinear_knots_lines[i].size() - 1 || !collinear_knots_lines[i][j].is_fixed)
						{
							continue;
						}
						if (j < collinear_knots_lines[i].size() - 1)
						{
							Point_2 linef(collinear_knots_lines[i][j - 1].paradomain);
							Point_2 linet(collinear_knots_lines[i][j + 1].paradomain);
							if (vit->get_domain().x() > std::min(linef.x(), linet.x()) && vit->get_domain().x() < std::max(linef.x(), linet.x()) ||
								vit->get_domain().y() > std::min(linef.y(), linet.y()) && vit->get_domain().y() < std::max(linef.y(), linet.y()))
							{
								if (dis_between_p_line(linef, linet, vit->get_domain()) < remove_criteria_closeto_featureedge / input_image->width())
								{
									is_remove = true;
									knots_closeto_feature.push_back(vit->vertex_index());
									std::cout << __FUNCTION__ << ": " << "new knot: " << vit->vertex_index() << "-coord:(" << vit->get_domain().x() << "," << vit->get_domain().y()
										<< ") is almost on feature line, area: "
										<< abs(CGAL::area(vit->get_domain(), linet, linef)) << ". so, remove it" << std::endl;
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
			}

			//remove knots pretty close to feature bound corner triangle edge
			for (int i = 0; i < feature_bound_corner.size(); i++)
			{
				Point_2 linef(feature_bound_corner[i][0].imagedomain);
				Point_2 linet(feature_bound_corner[i][2].imagedomain);
				if (vit->get_domain().x() > std::min(linef.x(), linet.x()) && vit->get_domain().x() < std::max(linef.x(), linet.x()) ||
					vit->get_domain().y() > std::min(linef.y(), linet.y()) && vit->get_domain().y() < std::max(linef.y(), linet.y()))
				{
					if (dis_between_p_line(linef, linet, vit->get_domain()) < remove_criteria_closeto_featureedge / input_image->width())
					{
						is_remove = true;
						knots_closeto_feature.push_back(vit->vertex_index());
						std::cout << __FUNCTION__ << ": " << "new knot: " << vit->vertex_index() << "-coord:(" << vit->get_domain().x() << "," << vit->get_domain().y()
							<< ") is almost on feature line, area: "
							<< abs(CGAL::area(vit->get_domain(), linet, linef)) << ". so, remove it" << std::endl;
						break;
					}
				}
			}

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
					if (j>collinear_knots_lines[i].size() - 1|| !collinear_knots_lines[i][j].is_fixed )
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
#endif
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

	std::cout << __FUNCTION__ << ": " << "before all edge constraints"  << std::endl;

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
#if 1
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

		vector<int>::iterator res0_corner = find(knots_in_feature_corner.begin(), knots_in_feature_corner.end(), index0);
		vector<int>::iterator res1_corner = find(knots_in_feature_corner.begin(), knots_in_feature_corner.end(), index1);
		vector<int>::iterator res2_corner = find(knots_in_feature_corner.begin(), knots_in_feature_corner.end(), index2);

		vector<int>::iterator res0_closeto = find(knots_closeto_feature.begin(), knots_closeto_feature.end(), index0);
		vector<int>::iterator res1_closeto = find(knots_closeto_feature.begin(), knots_closeto_feature.end(), index1);
		vector<int>::iterator res2_closeto = find(knots_closeto_feature.begin(), knots_closeto_feature.end(), index2);

		//std::cout << __FUNCTION__ << ": " << "current constraint: " << index0 << "-" << index1 << "-" << index2 << "\n";

		if (res0_closeto == knots_closeto_feature.end() && res1_closeto == knots_closeto_feature.end()
			&& res0_corner == knots_in_feature_corner.end() && res1_corner == knots_in_feature_corner.end())
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
			&& res0_corner == knots_in_feature_corner.end() && res2_corner == knots_in_feature_corner.end())
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
			&& res2_corner == knots_in_feature_corner.end() && res1_corner == knots_in_feature_corner.end())
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
	}
#endif

	std::cout << "/////////////////////////////optimize knots mesh finished, number of knots " << nknot << "-" << knots_config->number_of_vertices() << std::endl;
}

void Image_Approximation::optimize_knots_triangulation_delaunay()
{
	/***************************************************by Yanyang**********************************************/
	QString dir_featureknot_cdt;
	QString dir_featureknot_edge;
	QString dir_number_fixed_knots;

	QString dir_allknot = out_file_name;
	dir_allknot.append("/midprocess_optiknots.obj");

	build_collinear_knots();

	//knot mesh
	knot_mesh = new Mesh;
	read_mesh(&knot_mesh, dir_allknot);
	nknot = knot_mesh->size_of_vertices();

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
			Point_2 p;

			double x = vit->point().x();
			double y = vit->point().y();
			p = Point_2((x + imagemesh_dx) / (2.0*imagemesh_dx), (y + imagemesh_dy) / (2.0*imagemesh_dy));

			vit->get_domain() = p;
		}
	}
	knot_mesh->computeCornerVerticesType();
	knot_mesh->compute_all_face_domain_area();

	/**********************************************make cdt***************************************************/
	//from knot mesh to cdt
	knots_config = new CDT;
	map<int, CDT::Vertex_handle> vertex_handle_cdt;
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
	for (auto vit = knot_mesh->vertices_begin(); vit != knot_mesh->vertices_end(); vit++)
	{
		if (vit->vertex_index() >= Norepeat_knot.size())
		{
			knots_config->insert(vit->get_domain());
		}			
	}
#if 1
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
	}
#endif

	std::cout << "/////////////////////////////optimize knots mesh finished, number of knots " << nknot << "-" << knots_config->number_of_vertices() << std::endl;
}


bool Image_Approximation::is_parameterization()
{
	return bparameterization;
}

void Image_Approximation::compute_constrained_parameterization()
{
	if (!bparameterization)
	{
		//std::cout << "\n" << "/////////////////////////////parameterization start" << std::endl;
		//compute feature poly
		matching_feature_with_polyline(simply_error);
		//fix feature
		for (int i = 0; i < feature_lines_modify.size(); i++)
		{
			for (int j = 0; j < feature_lines_modify[i].size(); j++)
			{
				Mesh::Vertex_iterator vit = feature_lines_modify[i][j].vit;
				vit->get_domain() = feature_lines_modify[i][j].paradomain;
			}
		}

		ConstrainedPara parame_(originalMesh,0);
		//parame_.mesh_deformation();
		parame_.compute_para_domain();
		bparameterization = true;

		std::cout << "/////////////////////////////parameterization finished" << std::endl;
	}
}

void Image_Approximation::create_kd_tree()
{
	if (!is_build_domain_kd_tree)
	{
		//domain
		ANNpointArray	dataPts;
		dataPts = annAllocPts(originalMesh->size_of_vertices(), 2);
		Vertex_iterator dstIt = originalMesh->vertices_begin();
		int i = 0;
		for (; dstIt != originalMesh->vertices_end(); dstIt++)
		{
			dataPts[i][0] = dstIt->get_domain().x();
			dataPts[i][1] = dstIt->get_domain().y();
			i++;
		}
		kdTree_newdomain = new ANNkd_tree(					// build search structure
			dataPts,					// the data points
			originalMesh->size_of_vertices(),						// number of points
			2);						// dimension of space


									//old domain
		ANNpointArray	dataPtsold;
		dataPtsold = annAllocPts(originalMesh->size_of_vertices(), 2);
		Vertex_iterator dstItold = originalMesh->vertices_begin();
		i = 0;
		for (; dstItold != originalMesh->vertices_end(); dstItold++)
		{
			dataPtsold[i][0] = dstItold->get_old_domain().x();
			dataPtsold[i][1] = dstItold->get_old_domain().y();
			i++;
		}
		kdTree_olddomain = new ANNkd_tree(					// build search structure
			dataPtsold,					// the data points
			originalMesh->size_of_vertices(),						// number of points
			2);						// dimension of space


		is_build_domain_kd_tree = true;
	}
}

void Image_Approximation::knots_transform()
{
	if (!knots_config)
	{
		return;
	}
	OneStepInfo oneStepInfo;
	statisticsInfos.infos.push_back(oneStepInfo);

	for (auto vit = knots_config->vertices_begin(); vit != knots_config->vertices_end(); vit++)
	{
		KnotData data;
		data.pt = Point_2(vit->point().x(), vit->point().y());
		data.index = vit->get_associated_index();
		data.flag.bBoundary = is_on_domain_bound(vit->point());
		data.flag.bCorner = is_on_domain_corner(vit->point());
		dataMaps[vit->get_associated_index()] = data;
	}
	//statisticsInfos.infos[ite_now - 1].timeCost.knotTime = 0.0;

}

void Image_Approximation::generate_configs_ofsplitbasis()
{
	G1_constrains.clear();

	//inte set of basis are on edge
	for (int i = 0; i < collinear_knots_lines.size(); i++)
	{
		if (nknotinsert_collinear > 0) {
			if (collinear_knots_lines[i].size() >= 3)
			{
				if (collinear_knots_lines[i][0].index == collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index)
				{
					vector<unsigned int> corner_seq;
					int id_cn = -1;
					if (nDeg == 2)
					{
						id_cn = 2;
						corner_seq.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 3].index);
						corner_seq.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 2].index);
						corner_seq.push_back(collinear_knots_lines[i][0].index);
						corner_seq.push_back(collinear_knots_lines[i][1].index);
						corner_seq.push_back(collinear_knots_lines[i][2].index);
						G1Corner cons;
						cons.inte_side1.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 2].index);
						cons.inte_side1.push_back(collinear_knots_lines[i][0].index);
						cons.inte_cen.push_back(collinear_knots_lines[i][0].index);
						cons.inte_cen.push_back(collinear_knots_lines[i][0].index);
						cons.inte_side2.push_back(collinear_knots_lines[i][0].index);
						cons.inte_side2.push_back(collinear_knots_lines[i][1].index);
						G1_constrains.push_back(cons);
					}
					else if (nDeg == 3)
					{
						id_cn = 3;
						corner_seq.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 4].index);
						corner_seq.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 3].index);
						corner_seq.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 2].index);
						corner_seq.push_back(collinear_knots_lines[i][0].index);
						corner_seq.push_back(collinear_knots_lines[i][1].index);
						corner_seq.push_back(collinear_knots_lines[i][2].index);
						corner_seq.push_back(collinear_knots_lines[i][3].index);
						G1Corner cons;
						cons.inte_side1.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 2].index);
						cons.inte_side1.push_back(collinear_knots_lines[i][0].index);
						cons.inte_side1.push_back(collinear_knots_lines[i][0].index);

						cons.inte_cen.push_back(collinear_knots_lines[i][0].index);
						cons.inte_cen.push_back(collinear_knots_lines[i][0].index);
						cons.inte_cen.push_back(collinear_knots_lines[i][0].index);

						cons.inte_side2.push_back(collinear_knots_lines[i][0].index);
						cons.inte_side2.push_back(collinear_knots_lines[i][0].index);
						cons.inte_side2.push_back(collinear_knots_lines[i][1].index);
						G1_constrains.push_back(cons);
					}
					from_corner_triple_to_constrains(nDeg, id_cn, corner_seq, rupture_edge_basis_config);
				}
				else
				{
					int j = collinear_knots_lines[i].size() - 1;
					unsigned int v_2 = collinear_knots_lines[i][j - 2].index;
					unsigned int v_1 = collinear_knots_lines[i][j - 1].index;
					unsigned int v = collinear_knots_lines[i][j].index;

					RuptureKnotConfig rcd20;
					rcd20.inte = { v_1,v };
					rcd20.constrain = { v_2,v };
					rupture_edge_basis_config.push_back(rcd20);

					j = 0;
					v = collinear_knots_lines[i][j].index;
					unsigned int v1 = collinear_knots_lines[i][j+1].index;
					unsigned int v2 = collinear_knots_lines[i][j+2].index;

					RuptureKnotConfig rcd21;
					rcd21.inte = { v,v1 };
					rcd21.constrain = { v,v2 };
					rupture_edge_basis_config.push_back(rcd21);
				}
			}

			int former;
			for (int j = 0; j < collinear_knots_lines[i].size();)
			{
				if (collinear_knots_lines[i][j].is_fixed)
				{
					former = j;
				}
				j++;
				if (j >= collinear_knots_lines[i].size() || !collinear_knots_lines[i][j].is_fixed)
				{
					continue;
				}
				int  nknotins = j - former - 1;

				for (int id = 1; id < nDeg + 1; id++)
				{
					for (int it = 0; it < nknotins - id + 1; it++)
					{
						unsigned int left_, right_;
						vector<unsigned int> inte;
						left_ = collinear_knots_lines[i][former + it].index;
						for (int in = 1; in < id + 1; in++)
						{
							inte.push_back(collinear_knots_lines[i][former + it + in].index);
						}
						right_ = collinear_knots_lines[i][former + it + id + 1].index;
						RuptureKnotConfig rc;
						rc.inte = inte; rc.constrain = { left_,right_ }; rc.split_spare_index = -1;
						rupture_edge_basis_config.push_back(rc);
					}
				}

				if (j < collinear_knots_lines[i].size() - nDeg && j >(nDeg - 1))
				{
					vector<unsigned int> corner_seq;
					int id_cn = -1;
					if (nDeg == 2)
					{
						id_cn = 2;
						corner_seq.push_back(collinear_knots_lines[i][j - 2].index);
						corner_seq.push_back(collinear_knots_lines[i][j - 1].index);
						corner_seq.push_back(collinear_knots_lines[i][j].index);
						corner_seq.push_back(collinear_knots_lines[i][j + 1].index);
						corner_seq.push_back(collinear_knots_lines[i][j + 2].index);
						G1Corner cons;
						cons.inte_side1.push_back(collinear_knots_lines[i][j - 1].index);
						cons.inte_side1.push_back(collinear_knots_lines[i][j].index);
						cons.inte_cen.push_back(collinear_knots_lines[i][j].index);
						cons.inte_cen.push_back(collinear_knots_lines[i][j].index);
						cons.inte_side2.push_back(collinear_knots_lines[i][j].index);
						cons.inte_side2.push_back(collinear_knots_lines[i][j + 1].index);
						G1_constrains.push_back(cons);

					}
					else if (nDeg == 3)
					{
						id_cn = 3;
						corner_seq.push_back(collinear_knots_lines[i][j - 3].index);
						corner_seq.push_back(collinear_knots_lines[i][j - 2].index);
						corner_seq.push_back(collinear_knots_lines[i][j - 1].index);
						corner_seq.push_back(collinear_knots_lines[i][j].index);
						corner_seq.push_back(collinear_knots_lines[i][j + 1].index);
						corner_seq.push_back(collinear_knots_lines[i][j + 2].index);
						corner_seq.push_back(collinear_knots_lines[i][j + 3].index);
						G1Corner cons;
						cons.inte_side1.push_back(collinear_knots_lines[i][j - 1].index);
						cons.inte_side1.push_back(collinear_knots_lines[i][j].index);
						cons.inte_side1.push_back(collinear_knots_lines[i][j].index);

						cons.inte_cen.push_back(collinear_knots_lines[i][j].index);
						cons.inte_cen.push_back(collinear_knots_lines[i][j].index);
						cons.inte_cen.push_back(collinear_knots_lines[i][j].index);

						cons.inte_side2.push_back(collinear_knots_lines[i][j].index);
						cons.inte_side2.push_back(collinear_knots_lines[i][j].index);
						cons.inte_side2.push_back(collinear_knots_lines[i][j + 1].index);
						G1_constrains.push_back(cons);
					}
					from_corner_triple_to_constrains(nDeg, id_cn, corner_seq, rupture_edge_basis_config);
				}
			}
		}
		else
		{
			//if (collinear_knots_lines[i].size() >= 3)
			{
				if (collinear_knots_lines[i][0].index == collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index)
				{
					unsigned int vnow = collinear_knots_lines[i][0].index;
					unsigned int vnet = collinear_knots_lines[i][1].index;
					unsigned int vpre = collinear_knots_lines[i][collinear_knots_lines[i].size() - 2].index;

					RuptureKnotConfig rcd20;
					rcd20.inte = { vnow,vnow };
					rcd20.constrain = { vpre,vnet };
					rupture_edge_basis_config.push_back(rcd20);
				}
			}

			for (int i = 0; i < collinear_knots_lines.size(); i++)
			{
				for (int j = 1; j < collinear_knots_lines[i].size()-1; j++)
				{
					unsigned int vnow = collinear_knots_lines[i][j].index;
					unsigned int vnet = collinear_knots_lines[i][(j + 1) % collinear_knots_lines[i].size()].index;
					unsigned int vpre = collinear_knots_lines[i][(j - 1 + collinear_knots_lines[i].size()) % collinear_knots_lines[i].size()].index;

					RuptureKnotConfig rcd20;
					rcd20.inte = { vnow,vnow };
					rcd20.constrain = { vpre,vnet };
					rupture_edge_basis_config.push_back(rcd20);
				}
			}

			for (int i = 0; i < collinear_knots_lines.size(); i++)
			{
				for (int j = 0; j < collinear_knots_lines[i].size() - 1; j++)
				{
					unsigned int vnow = collinear_knots_lines[i][j].index;
					unsigned int vnet = collinear_knots_lines[i][(j + 1) % collinear_knots_lines[i].size()].index;

					RuptureKnotConfig rcd2;
					rcd2.inte = { vnow,vnet };
					rcd2.constrain = { vnow,vnet };
					rupture_edge_basis_config.push_back(rcd2);
				}
			}
		}
	}

	for (int i = 0; i < feature_bound_corner.size(); i++)
	{
		vector<unsigned int> corner_seq;
		int id_cn = -1;
		if (nDeg == 2)
		{
			id_cn = 2;
			corner_seq.push_back(feature_bound_corner[i][0].index);
			corner_seq.push_back(feature_bound_corner[i][1].index);
			corner_seq.push_back(feature_bound_corner[i][2].index);
			corner_seq.push_back(feature_bound_corner[i][3].index);
		}
		else if (nDeg == 3)
		{
			id_cn = 3;
			corner_seq.push_back(feature_bound_corner[i][0].index);
			corner_seq.push_back(feature_bound_corner[i][1].index);
			corner_seq.push_back(feature_bound_corner[i][2].index);
			corner_seq.push_back(feature_bound_corner[i][3].index);
			corner_seq.push_back(feature_bound_corner[i][4].index);
		}
		from_corner_triple_to_constrains(nDeg, id_cn, corner_seq, rupture_edge_basis_config);
	}
}

void Image_Approximation::link_triangulation_procedure()
{
	if (dataMaps.empty())
		return;
	if (ltp != NULL)
	{
		delete ltp;
		ltp = NULL;
	}
	ltp = new CKnotsTriangulation(originalMesh, &dataMaps, nDeg);
	ltp->set_kdtree(kdTree_newdomain);
	ltp->set_optimization_algorithm(optAlg);
	ltp->set_correspondence(&correspondence);
	ltp->set_corner_indices(&cornerIds);
	ltp->set_collinear_knots_config(collinear_knots_lines, feature_bound_corner,
		rupture_edge_basis_config,nknotinsert_collinear);

	alltconfigs.clear();
	ltp->link_triangulation_procedure(*knots_config, alltconfigs);//knots_config has corner multi-p
	std::cout << __FUNCTION__ << ": " << "caution: knots_config has corner repeated-points,number of knots: " << knots_config->number_of_vertices() << std::endl;

	//sort the configs such that we can read data from file
	sort(alltconfigs[nDeg].begin(), alltconfigs[nDeg].end(), sortConfigGreater);
}

void Image_Approximation::remove_degraded_simplex(vector<TConfig2> &configs)
{
	//remove simplex degrade
	vector<TConfig2> new_configs;
	for (int i = 0; i < configs.size(); i++)
	{
		vector<Point_2> simplex, convex;
		for (int j = 0; j < configs[i].tconfig.size(); j++)
		{
			simplex.push_back(dataMaps.at(configs[i].tconfig[j]).pt);
		}
		CGAL::convex_hull_2(simplex.begin(), simplex.end(), back_inserter(convex));
		Polygon_2 polygon_(convex.begin(), convex.end());
		if (abs(polygon_.area()) > 1e-12)
		{
			new_configs.push_back(configs[i]);
		}
	}

	//remove triangle degrade
	vector<TConfig2> renew_configs;
	for (int i = 0; i < new_configs.size(); i++)
	{
		//test
		/*vector<unsigned> vecin = { 14,15,15,15,257 };
		sort(vecin.begin(), vecin.end());
		vector<unsigned> vectemp = new_configs[i];
		sort(vectemp.begin(), vectemp.end());
		if (equal(vecin.begin(), vecin.end(), vectemp.begin()))
		{
		continue;
		}*/

		if (new_configs[i].tconfig[0] != new_configs[i].tconfig[1] &&
			new_configs[i].tconfig[0] != new_configs[i].tconfig[2] &&
			new_configs[i].tconfig[1] != new_configs[i].tconfig[2])
		{
			renew_configs.push_back(new_configs[i]);
		}
	}
	configs = renew_configs;
}

void Image_Approximation::merge_linearly_dependent_simplex(vector<TConfig2> &configs)
{
	//compute area
	for (int i = 0; i < configs.size(); i++)
	{
		Point_2 p1=dataMaps[configs[i].tconfig[0]].pt;
		Point_2 p2=dataMaps[configs[i].tconfig[1]].pt;
		Point_2 p3=dataMaps[configs[i].tconfig[2]].pt;
		configs[i].tri_area = abs(CGAL::area(p1, p2, p3));
	}

#if 0
	//merge
	for (int i = 0;i<configs.size()-1;i++)
	{
		vector<unsigned> temp1 = configs[i].tconfig;
		sort(temp1.begin(),temp1.end());
		for (int j = i+1;j<configs.size();j++)
		{
			vector<unsigned> temp2 = configs[j].tconfig;
			sort(temp2.begin(), temp2.end());
			if (equal(temp1.begin(),temp1.end(),temp2.begin()))
			{
				//check whether exist feature basis
				int type_inte = 0;
				vector<unsigned> inte1 = configs[i].inte;
				vector<unsigned> inte2 = configs[j].inte;
				sort(inte1.begin(), inte1.end());
				sort(inte2.begin(), inte2.end());
				for (int k = 0;k<rupture_edge_basis_config.size();k++)
				{
					vector<unsigned> fea1 = rupture_edge_basis_config[k].inte;
					sort(fea1.begin(), fea1.end());
					if (equal(fea1.begin(),fea1.end(),inte1.begin()))
					{
						type_inte = 1;
						break;
					}
					if (equal(fea1.begin(), fea1.end(), inte2.begin()))
					{
						type_inte = 2; break;
					}
				}
				for (int k = 0; k < rupture_corner_basis_config.size(); k++)
				{
					vector<unsigned> fea1 = { rupture_corner_basis_config[k][0],rupture_corner_basis_config[k][1] };
					vector<unsigned> fea2 = { rupture_corner_basis_config[k][1],rupture_corner_basis_config[k][1] };
					vector<unsigned> fea3 = { rupture_corner_basis_config[k][1],rupture_corner_basis_config[k][2] };
					sort(fea1.begin(), fea1.end());
					sort(fea2.begin(), fea2.end());
					sort(fea3.begin(), fea3.end());
					if (equal(fea1.begin(), fea1.end(), inte1.begin())||
						equal(fea2.begin(), fea2.end(), inte1.begin())||
						equal(fea3.begin(), fea3.end(), inte1.begin()))
					{
						type_inte = 1; break;
					}
					if (equal(fea1.begin(), fea1.end(), inte2.begin()) ||
						equal(fea2.begin(), fea2.end(), inte2.begin()) ||
						equal(fea3.begin(), fea3.end(), inte2.begin()))
					{
						type_inte = 2; break;
					}
				}
				if (type_inte == 0)
				{
					configs[j].tconfig = configs[i].tconfig;
				}
				//else/* if (type_inte == 2)*/
				//{
				//	configs[i].tconfig = configs[j].tconfig;
				//}
			}
		}
	}
#endif
}

void Image_Approximation::splite_rupture_basis()
{
	vector<SplineBasisConfigs> basisConfigs = bSplineBasis.basisConfigs;
	vector<SplineBasisConfigs> newbasisConfigs;
	for (int ib = 0; ib < basisConfigs.size(); ib++)
	{
		bool is_find = false;
		vector<unsigned> temp = basisConfigs[ib].config;
		sort(temp.begin(), temp.end());
		for (int it = 0; it < rupture_edge_basis_config.size(); it++)
		{
			vector<unsigned> intenow = rupture_edge_basis_config[it].inte;
			sort(intenow.begin(),intenow.end());
			if (equal(intenow.begin(),intenow.end(),temp.begin()) && intenow.size() == temp.size())
			{
				if (intenow[0]==3 && intenow[1] == 4)
				{
					int s = 1;
				}

				is_find = true;

				SplineBasisConfigs splitbasis_pside, splitbasis_nside, splitbasis_twoside;
				splitbasis_pside.config = basisConfigs[ib].config;
				splitbasis_nside.config = basisConfigs[ib].config;
				splitbasis_twoside.config = basisConfigs[ib].config;

				bool is_inte_same = true;
				for (int its = 1; its < basisConfigs[ib].config.size(); its++)
				{
					if (basisConfigs[ib].config[its] != basisConfigs[ib].config[0])
					{
						is_inte_same = false;
						break;
					}
				}
				Point_2 p_corner = dataMaps.at(basisConfigs[ib].config[0]).pt;
				if (is_inte_same)
				{					
					//split basis by corner triangle					
					for (int k = 0; k < basisConfigs[ib].tconfigs.size(); k++)
					{
						if (basisConfigs[ib].tconfigs[k].tconfig[0] == basisConfigs[ib].tconfigs[k].tconfig[1] ||
							basisConfigs[ib].tconfigs[k].tconfig[0] == basisConfigs[ib].tconfigs[k].tconfig[2] ||
							basisConfigs[ib].tconfigs[k].tconfig[1] == basisConfigs[ib].tconfigs[k].tconfig[2])
						{
							continue;
						}
						vector<unsigned>::iterator res0 = find(basisConfigs[ib].tconfigs[k].tconfig.begin(), basisConfigs[ib].tconfigs[k].tconfig.end(),
							rupture_edge_basis_config[it].constrain.first);
						vector<unsigned>::iterator res2 = find(basisConfigs[ib].tconfigs[k].tconfig.begin(), basisConfigs[ib].tconfigs[k].tconfig.end(),
							rupture_edge_basis_config[it].constrain.second);
						if (res0 != basisConfigs[ib].tconfigs[k].tconfig.end() && res2 != basisConfigs[ib].tconfigs[k].tconfig.end())
						{
							Point_2 p_fir = dataMaps.at(*res0).pt;
							Point_2 p_sed = dataMaps.at(*res2).pt;
							splitbasis_nside.bCCWorientation = area(p_fir, p_corner, p_sed)>0 ? true : false;
							splitbasis_nside.tconfigs.push_back(basisConfigs[ib].tconfigs[k]);
						}
						else
						{											
							splitbasis_pside.tconfigs.push_back(basisConfigs[ib].tconfigs[k]);
						}
					}
					splitbasis_pside.bCCWorientation = !splitbasis_nside.bCCWorientation;

					int index_n = newbasisConfigs.size();
					if (splitbasis_nside.tconfigs.size() > 0)
					{
						newbasisConfigs.push_back(splitbasis_nside);
					}
					int index_p = newbasisConfigs.size();
					if (splitbasis_pside.tconfigs.size() > 0)
					{
						newbasisConfigs.push_back(splitbasis_pside);
					}
					if (splitbasis_nside.tconfigs.size() > 0 && splitbasis_pside.tconfigs.size() > 0)
					{
						basis_point_pair.push_back(pair<int, int>(index_n, index_p));
					}
					else
					{
						std::cout << __FUNCTION__ << ": " << "interior index: " << basisConfigs[ib].config[0] << "-" << basisConfigs[ib].config[1] << "--------------corner-";
						std::cout << __FUNCTION__ << ": " << "wrong split: " << "\n";
					}
				}
				else
				{
					Point_2 line_f, line_t,line2_f;
					//split basis by line
					if (rupture_edge_basis_config[it].bInverse_order)
					{
						line_t = dataMaps[rupture_edge_basis_config[it].constrain.first].pt;
						line_f = dataMaps[rupture_edge_basis_config[it].constrain.second].pt;
					}
					else
					{
						line_f = dataMaps[rupture_edge_basis_config[it].constrain.first].pt;
						line_t = dataMaps[rupture_edge_basis_config[it].constrain.second].pt;
					}					
					if (rupture_edge_basis_config[it].split_spare_index != -1)
					{
						//split basis by broken lines
						line2_f = dataMaps[rupture_edge_basis_config[it].split_spare_index].pt;
					}
					
					vector<int> simplex_types;//-1: oneside; 1: otherside; 0: twoside; 
					for (int k = 0; k < basisConfigs[ib].tconfigs.size(); k++)
					{
						int simplex_type;
						vector<int> point_type; //-1: oneside; 1: otherside; 0: online;
						for (int i = 0; i < 3; i++)
						{
							Point_2 p = dataMaps[basisConfigs[ib].tconfigs[k].tconfig[i]].pt;
							if (abs(area(p, line_f, line_t)) < 1e-16)
							{
								point_type.push_back(0);
							}
							else if (area(p, line_f, line_t) > 1e-16)
							{
								point_type.push_back(1);
							}
							else if (area(p, line_f, line_t) < -1e-16)
							{
								point_type.push_back(-1);
							}
						}
						if (!(point_type[0] > 0) && !(point_type[1] > 0) && !(point_type[2] > 0))
						{
							simplex_type = -1;							
						}
						else if (!(point_type[0] < 0) && !(point_type[1] < 0) && !(point_type[2] < 0))
						{
							simplex_type = 1;
						}
						else
						{
							simplex_type = 0;					
						}

						if (rupture_edge_basis_config[it].split_spare_index != -1 && simplex_type == 0)
						{
							vector<int> point_type2; //-1: oneside; 1: otherside; 0: online;
							for (int i = 0; i < 3; i++)
							{
								Point_2 p = dataMaps[basisConfigs[ib].tconfigs[k].tconfig[i]].pt;
								if (abs(area(p, line_t, line2_f)) < 1e-16)
								{
									point_type2.push_back(0);
								}
								else if (area(p, line_t, line2_f) > 1e-16)
								{
									point_type2.push_back(1);
								}
								else if (area(p, line_t, line2_f) < -1e-16)
								{
									point_type2.push_back(-1);
								}
							}
							if ((point_type[0] != -1) && (point_type[1] != -1) && (point_type[2] != -1)
								&& (point_type2[0] != -1) && (point_type2[1] != -1) && (point_type2[2] != -1))
							{
								simplex_type = 1;
							}
							else
							{
								simplex_type = -1;
							}
						}			
						simplex_types.push_back(simplex_type);
						if (simplex_type == 1)
						{
							splitbasis_pside.tconfigs.push_back(basisConfigs[ib].tconfigs[k]);
						}
						else if (simplex_type == -1)
						{
							splitbasis_nside.tconfigs.push_back(basisConfigs[ib].tconfigs[k]);
						}
						else
						{
							splitbasis_twoside.tconfigs.push_back(basisConfigs[ib].tconfigs[k]);
						}
					}

					splitbasis_pside.bCCWorientation = true;
					splitbasis_nside.bCCWorientation = false;

					int index_n = newbasisConfigs.size();
					if (splitbasis_nside.tconfigs.size() > 0)
					{
						newbasisConfigs.push_back(splitbasis_nside);
					}
					int index_p = newbasisConfigs.size();
					if (splitbasis_pside.tconfigs.size() > 0)
					{
						newbasisConfigs.push_back(splitbasis_pside);
					}
					if (splitbasis_twoside.tconfigs.size() > 0)
					{
						newbasisConfigs.push_back(splitbasis_twoside);
					}

					bool has_pside = false, has_nside = false, has_twoside = false;
					for (int k = 0; k < simplex_types.size(); k++)
					{
						if (simplex_types[k] == -1)
						{
							has_nside = true;
						}
						if (simplex_types[k] == 1)
						{
							has_pside = true;
						}
						if (simplex_types[k] == 0)
						{
							has_twoside = true;
						}
					}
					int basis_type;
					if (has_pside && !has_nside && !has_twoside ||
						has_nside && !has_pside && !has_twoside)
					{
						basis_type = 1;
					}
					else if (has_pside && has_nside && !has_twoside)
					{
						basis_type = 2;
					}
					else if (has_twoside)
					{
						basis_type = 3;
					}
					if (basis_type != 2)
					{
						std::cout << __FUNCTION__ << ": " << "interior index: ";
						for (int k = 0; k < rupture_edge_basis_config[it].inte.size(); k++)
						{
							std::cout << __FUNCTION__ << ": " << rupture_edge_basis_config[it].inte[k] << "-";
						}
						std::cout << __FUNCTION__ << ": " << "-------basis type: " << basis_type << "\n";
					}
					if (basis_type == 2)
					{
						basis_point_pair.push_back(pair<int, int>(index_n, index_p));
					}
				}
				break;
			}
		}
		if (!is_find)
		{
			newbasisConfigs.push_back(basisConfigs[ib]);
		}

		//test
		/*if (temp[0] == 2 && temp[1] == 3)
		{
			std::cout << "interior index: " << basisConfigs[ib].config[0] << "-" << basisConfigs[ib].config[1] << "\n";
			std::cout << "basis index: " << ib << "\n";
			for (int k = 0; k < basisConfigs[ib].tconfigs.size(); k++)
			{
				for (int ki = 0; ki < basisConfigs[ib].tconfigs[k].tconfig.size(); ki++)
				{
					std::cout << basisConfigs[ib].tconfigs[k].tconfig[ki] << ",";
				}
				std::cout << "\n";
			}
		}*/
	}

	bSplineBasis.basisConfigs = newbasisConfigs;
	std::cout << __FUNCTION__ << ": " << "----number of old basis configs: " << basisConfigs.size() << std::endl;
	std::cout << __FUNCTION__ << ": " << "----number of splitter basis configs: " << newbasisConfigs.size() << std::endl;
}

void Image_Approximation::verify_basis_normalizng()
{
	bool do_verify_derivate = false;

	vector<DomainValue> mesh_data_map;
	mesh_data_map.resize(originalMesh->size_of_vertices() + 
		supplementdomaindata.size() + featureregion_sampleps.size()+ featurecurve_sampleps.size());
	Mesh::Vertex_iterator vit;
	for (vit = originalMesh->vertices_begin(); vit != originalMesh->vertices_end(); vit++)
	{
		int index = vit->vertex_index();
		DomainValue p;
		p.pt = vit->get_domain();
		mesh_data_map[index] = p;
	}
	for (int i = 0; i < featurecurve_sampleps.size(); i++)
	{
		DomainValue p;
		p.pt = featurecurve_sampleps[i];
		int index_now = originalMesh->size_of_vertices() + i;
		mesh_data_map[index_now] = p;
	}
	for (int i = 0; i < featureregion_sampleps.size(); i++)
	{
		DomainValue p;
		p.pt = featureregion_sampleps[i];
		int index_now = originalMesh->size_of_vertices() + featurecurve_sampleps.size() + i;
		mesh_data_map[index_now] = p;
	}
	for (int i = 0; i < supplementdomaindata.size(); i++)
	{
		DomainValue p;
		p.pt = supplementdomaindata[i];
		int index_now = originalMesh->size_of_vertices() + featurecurve_sampleps.size() + featureregion_sampleps.size() + i;
		mesh_data_map[index_now] = p;
	}
	int number_threads;
#pragma omp parallel for
	for (int mit = 0; mit < mesh_data_map.size(); mit++)
	{
		number_threads = omp_get_num_threads();
		double basis_value_sum_ = 0.0;
		double basis_dx_sum_ = 0.0;
		double basis_dy_sum_ = 0.0;
		double basis_dxx_sum_ = 0.0;
		double basis_dxy_sum_ = 0.0;
		double basis_dyy_sum_ = 0.0;

		for (int i = 0; i < bSplineBasis.basisMergeInfos.size(); i++)
		{
			for (int j = 0; j < bSplineBasis.basisMergeInfos[i].basisMerge.size(); j++)
			{
				int loc = bSplineBasis.basisMergeInfos[i].basisMerge[j];
				if (bSplineBasis.basisConfigs[loc].supports.size() != NULL)
				{
					map<unsigned int, DomainValue>::iterator res = bSplineBasis.basisConfigs[loc].supports.find(mit);
					if (res != bSplineBasis.basisConfigs[loc].supports.end())
					{
						basis_value_sum_ += res->second.value;
						if (do_verify_derivate)
						{
							basis_dx_sum_ += res->second.dx;
							basis_dy_sum_ += res->second.dy;
							basis_dxx_sum_ += res->second.dxx;
							basis_dxy_sum_ += res->second.dxy;
							basis_dyy_sum_ += res->second.dyy;
						}
					}
				}
			}
		}
		mesh_data_map[mit].value = basis_value_sum_;
		if (do_verify_derivate)
		{
			mesh_data_map[mit].dx = basis_dx_sum_;
			mesh_data_map[mit].dy = basis_dy_sum_;
			mesh_data_map[mit].dxx = basis_dxx_sum_;
			mesh_data_map[mit].dxy = basis_dxy_sum_;
			mesh_data_map[mit].dyy = basis_dyy_sum_;
		}
	}
	
	int nw_value_e4 = 0;
	int nw_value_e8 = 0;
	int nw_d1_e4 = 0;
	int nw_d2_e4 = 0;
	for (int i = 0; i < mesh_data_map.size(); i++)
	{
		if (abs(mesh_data_map[i].value - 1) > 1e-4)
		{
			nw_value_e4++;
			std::cout << __FUNCTION__ << ": " << "basis value wrong:index:" << i << "-value: " << mesh_data_map[i].value
				<< " wrong points coord:(" << mesh_data_map[i].pt.x() << "," << mesh_data_map[i].pt.y() << ")" << std::endl;
		}
		if (abs(mesh_data_map[i].value - 1) > 1e-8)
		{
			nw_value_e8++;
			wrong_points.push_back(mesh_data_map[i].pt);
			/*std::cout << __FUNCTION__ << ": " << "basis value wrong:index:" << i << "-value: " << mesh_data_map[i].value<<setprecision(10)
			<< " wrong points coord:(" << mesh_data_map[i].pt.x() << "," << mesh_data_map[i].pt.y() << ")" << std::endl;*/
		}
		if (do_verify_derivate)
		{
			if (abs(mesh_data_map[i].dx) > 0.1 || abs(mesh_data_map[i].dy) > 0.1)
			{
				std::cout << __FUNCTION__ << ": " << "basis dx/dy wrong:index:" << i << "-dx: " << mesh_data_map[i].dx << "-dy: " << mesh_data_map[i].dy
					<< " wrong points coord:(" << mesh_data_map[i].pt.x() << "," << mesh_data_map[i].pt.y() << ")" << std::endl;
			}
			if (abs(mesh_data_map[i].dxx) > 0.1 || abs(mesh_data_map[i].dxy) > 0.1 || abs(mesh_data_map[i].dyy) > 0.1)
			{
				//std::cout << __FUNCTION__ << ": " << "basis dxx/dxy/dyy wrong:index:" << mit->first << "-dxx: " << mit->second.dxx << "-dxy: " << mit->second.dxy
				//<< " wrong points coord:(" << mit->second.pt.x() << "," << mit->second.pt.y() << ")" << std::endl;
			}
			if (abs(mesh_data_map[i].dx) > 1e-4 || abs(mesh_data_map[i].dy) > 1e-4)
			{
				nw_d1_e4++;
			}
			if (abs(mesh_data_map[i].dxx) > 1e-4 || abs(mesh_data_map[i].dxy) > 1e-4 || abs(mesh_data_map[i].dyy) > 1e-4)
			{
				nw_d2_e4++;
				//wrong_points.push_back(mesh_data_map[i].pt);
			}
		}
	}
	std::cout << __FUNCTION__ << ": " << "wrong number value(error > 1e-4):  " << nw_value_e4 << std::endl;
	std::cout << __FUNCTION__ << ": " << "wrong number value(error > 1e-8):  " << nw_value_e8 << std::endl;
	if (do_verify_derivate)
	{
		std::cout << __FUNCTION__ << ": " << "wrong number dx/dy(error > 1e-4):  " << nw_d1_e4 << std::endl;
		std::cout << __FUNCTION__ << ": " << "wrong number dxx/y(error > 1e-4):  " << nw_d2_e4 << std::endl;
	}
	std::cout << __FUNCTION__ << ": " << "the number of parallel compute threads: " << number_threads << "\n";

	//test sample point if existing wrong value
	//int tempp = 4382;
	//for (auto it = alltconfig3s.begin(); it != alltconfig3s.end(); it++)
	//{
	//	map<unsigned, DomainValue>::iterator res = it->supports.find(tempp);
	//	if (res != it->supports.end())
	//	{
	//		if (res->second.value > 1e-5)
	//		{
	//			std::cout << __FUNCTION__ << ": " << "index: " << tempp << "-coord: " << res->second.pt.x() << "," << res->second.pt.y() << "-value: " << res->second.value << " -config: ";
	//			for (int k = 0; k < it->tconfig.size(); k++)
	//			{
	//				std::cout << __FUNCTION__ << ": " << it->tconfig[k] << " ";
	//			}
	//			std::cout << __FUNCTION__ << ": " << "\n";
	//		}				
	//	}
	//}
}

void Image_Approximation::compute_supplement_5Ddata()
{
	for (int i = 0; i < supplementdomaindata.size(); i++)
	{
		vector<int> triangle;
		RGBPoint pt;
		compute_originalcolor_at_domainpoint(supplementdomaindata[i], triangle, pt);
		supplement5Ddata.push_back(pt);
	}
}

void Image_Approximation::generate_G1_pairs()
{
	for (int it = 0; it < G1_constrains.size(); it++)
	{
		G1_constrains[it].index_cen_pair1 = -1;
		G1_constrains[it].index_cen_pair2 = -1;
		G1_constrains[it].index_side1 = -1;
		G1_constrains[it].index_side2 = -1;

		if (it == 29)
		{
			int a = 1;
		}

		vector<unsigned> inte_side1 = G1_constrains[it].inte_side1;
		vector<unsigned> inte_center = G1_constrains[it].inte_cen;
		vector<unsigned> inte_side2 = G1_constrains[it].inte_side2;
		sort(inte_side1.begin(), inte_side1.end());
		sort(inte_side2.begin(), inte_side2.end());
		sort(inte_center.begin(), inte_center.end());
		for (int i = 0; i < bSplineBasis.basisMergeInfos.size(); i++)
		{
			for (int j = 0; j < bSplineBasis.basisMergeInfos[i].basisMerge.size(); j++)
			{
				int loc = bSplineBasis.basisMergeInfos[i].basisMerge[j];
				vector<unsigned> intenow = bSplineBasis.basisConfigs[loc].config;
				sort(intenow.begin(), intenow.end());
				if (equal(intenow.begin(), intenow.end(), inte_side1.begin()) && intenow.size() == inte_side1.size())
				{
					G1_constrains[it].index_side1 = i;
				}
				if (equal(intenow.begin(), intenow.end(), inte_side2.begin()) && intenow.size() == inte_side2.size())
				{
					G1_constrains[it].index_side2 = i;
				}
				if (equal(intenow.begin(), intenow.end(), inte_center.begin()) && intenow.size() == inte_center.size())
				{
					if (G1_constrains[it].index_cen_pair1 == -1)
					{
						G1_constrains[it].index_cen_pair1 = i;
					}
					else
					{
						G1_constrains[it].index_cen_pair2 = i;
					}
				}
			}
		}
	}

}

void Image_Approximation::generate_control_mesh()
{
	cmInfo.vertices.clear();
	cmInfo.edges.clear();
	cmInfo.faces.clear();
	vector<vector<TConfig2>> temp = alltconfigs;
	restore_index(correspondence, temp[nDeg - 1]);
	restore_index(correspondence, temp[nDeg - 2]);
	compute_centroid_triangulation(temp, alltconfigs, dataMaps, cmInfo);

	double length = imagemesh_dx / input_image->width() / 2.0;

	//real vertices
	int realCpNum = 0;
	for (unsigned i = 0; i < bSplineBasis.basisMergeInfos.size(); i++)
	{
		if (bSplineBasis.basisMergeInfos[i].index != -1)
		{
			bool is_onfeature = false;
			for (int it = 0; it < basis_point_pair.size(); it++)
			{
				if (basis_point_pair[it].first == i || basis_point_pair[it].second == i)
				{
					is_onfeature = true;

					RGBPoint pfir, cpd1, psed, cpd2;
					Vector_2 temp_d;
					vector<unsigned int> verify_config1, verify_config2;
					int fir = basis_point_pair[it].first;
					if (bSplineBasis.basisMergeInfos[fir].index != -1)
					{
						int loc = bSplineBasis.basisMergeInfos[fir].basisMerge[0];
						verify_config1 = bSplineBasis.basisConfigs[loc].config;
						bSplineBasis.basisConfigs[loc].controlPt.basis_index = loc;
						pfir = bSplineBasis.basisConfigs[loc].controlPt;

						Vector_2 pc = 0.5*((dataMaps[verify_config1[0]].pt - Point_2(0, 0)) + (dataMaps[verify_config1[1]].pt - Point_2(0, 0)));
						Vector_2 pd(0, 0);
						int num = 0;
						for (int k = 0; k < bSplineBasis.basisConfigs[loc].tconfigs.size(); k++)
						{
							for (int i = 0; i < bSplineBasis.basisConfigs[loc].tconfigs[k].tconfig.size(); i++)
							{
								pd = pd + (dataMaps[bSplineBasis.basisConfigs[loc].tconfigs[k].tconfig[i]].pt - Point_2(0, 0));
								num++;
							}
						}
						pd = pd / num;
						temp_d = (pd - pc) / sqrt((pd - pc).squared_length())*length;
						cpd1 = pfir;
						cpd1.x = pfir.x + temp_d.x(); cpd1.y = pfir.y + temp_d.y();
					}
					int sed = basis_point_pair[it].second;
					if (bSplineBasis.basisMergeInfos[sed].index != -1)
					{
						int loc = bSplineBasis.basisMergeInfos[sed].basisMerge[0];
						verify_config2 = bSplineBasis.basisConfigs[loc].config;
						bSplineBasis.basisConfigs[loc].controlPt.basis_index = loc;
						psed = bSplineBasis.basisConfigs[loc].controlPt;

						cpd2 = psed;
						cpd2.x = psed.x - temp_d.x(); cpd2.y = psed.y - temp_d.y();
					}
					//verify
					sort(verify_config1.begin(), verify_config1.end());
					sort(verify_config2.begin(), verify_config2.end());
					if (!equal(verify_config1.begin(), verify_config1.end(), verify_config2.begin()))
					{
						std::cout << __FUNCTION__ << ": " << "control points (edge basis) pair mismatch" << verify_config1[0] << "," << verify_config1[1] <<
							"--" << verify_config2[0] << "," << verify_config2[1] << "\n";
					}
					
					if (bSplineBasis.basisMergeInfos[fir].index != -1 &&
						bSplineBasis.basisMergeInfos[sed].index != -1)
					{
						pair<RGBPoint, RGBPoint> cpp(pfir, psed);
						pair<RGBPoint, RGBPoint> cpp_trans(cpd1, cpd2);

						int loc = bSplineBasis.basisMergeInfos[fir].basisMerge[0];
						for (unsigned k = 0; k < cmInfo.vertices.size(); k++)
						{
							vector<unsigned int> scr_, temp_;
							scr_ = cmInfo.vertices[k].interior; temp_ = bSplineBasis.basisConfigs[loc].config;
							sort(scr_.begin(), scr_.end());
							sort(temp_.begin(), temp_.end());
							if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
							{
								cmInfo.vertices[k].do_display = true;
								cmInfo.vertices[k].flag = 1;
								cmInfo.vertices[k].vertex = bSplineBasis.basisConfigs[loc].controlPt;
								cmInfo.vertices[k].is_on_feature = true;
								cmInfo.vertices[k].control_point_pair = cpp;
								cmInfo.vertices[k].cpp_little_trans = cpp_trans;
								cmInfo.vertices[k].feature_mergebasis_id = pair<int, int>(fir,sed);
								realCpNum++;
								break;
							}
						}
					}				
					break;
				}
			}

			if (!is_onfeature)
			{
				for (unsigned j = 0; j < bSplineBasis.basisMergeInfos[i].basisMerge.size(); j++)
				{
					int loc = bSplineBasis.basisMergeInfos[i].basisMerge[j];

					for (unsigned k = 0; k < cmInfo.vertices.size(); k++)
					{
						vector<unsigned int> scr_, temp_;
						scr_ = cmInfo.vertices[k].interior; temp_ = bSplineBasis.basisConfigs[loc].config;
						sort(scr_.begin(), scr_.end());
						sort(temp_.begin(), temp_.end());
						if (std::equal(scr_.begin(), scr_.end(), temp_.begin()))
						{
							cmInfo.vertices[k].do_display = true;
							cmInfo.vertices[k].flag = 1;
							cmInfo.vertices[k].vertex = bSplineBasis.basisConfigs[loc].controlPt;
							cmInfo.vertices[k].is_on_feature = false;
							cmInfo.vertices[k].merge_basis_id = i;

#if 0
							//test
							bSplineBasis.basisConfigs[loc].controlPt.r = bSplineBasis.basisConfigs[loc].initial_points[2];
							bSplineBasis.basisConfigs[loc].controlPt.g = bSplineBasis.basisConfigs[loc].initial_points[3];
							bSplineBasis.basisConfigs[loc].controlPt.b = bSplineBasis.basisConfigs[loc].initial_points[4];
							cmInfo.vertices[k].vertex = bSplineBasis.basisConfigs[loc].controlPt;
#endif

							realCpNum++;
							break;
						}
					}
				}
			}			
		}
	}

#if 0
	//test
#pragma omp parallel for
	for (int it = 0; it < fittedMesh->size_of_vertices(); it++)
	{
		Mesh::Vertex_iterator vit = fittedMesh->get_vertex_iterator(it);
		RGBPoint tep; tep.x = 0; tep.y = 0; tep.r = 0; tep.g = 0; tep.b = 0;
		for (int i = 0; i < bSplineBasis.basisMergeInfos.size(); i++)
		{
			int flag = bSplineBasis.basisMergeInfos[i].index;
			if (flag != -1)
			{
				for (int j = 0; j < bSplineBasis.basisMergeInfos[i].basisMerge.size(); j++)
				{
					int index_basis = bSplineBasis.basisMergeInfos[i].basisMerge[j];
					map<unsigned int, DomainValue>::iterator ite = bSplineBasis.basisConfigs[index_basis].supports.find(vit->vertex_index());
					if (ite != bSplineBasis.basisConfigs[index_basis].supports.end())
					{
						tep.x = tep.x + ite->second.value * bSplineBasis.basisConfigs[index_basis].controlPt.x;
						tep.y = tep.y + ite->second.value * bSplineBasis.basisConfigs[index_basis].controlPt.y;
						tep.r = tep.r + ite->second.value * bSplineBasis.basisConfigs[index_basis].controlPt.r;
						tep.g = tep.g + ite->second.value * bSplineBasis.basisConfigs[index_basis].controlPt.g;
						tep.b = tep.b + ite->second.value * bSplineBasis.basisConfigs[index_basis].controlPt.b;
					}
				}
			}
		}

		vit->vertex_pixel_color() = Point_3(tep.r, tep.g, tep.b);
		double gray_ = 0.2989*tep.r + 0.5870*tep.g + 0.1140*tep.b;//for surface display
		vit->point() = Point(tep.x, tep.y, gray_);
	}
#endif

	//auxiliary vertices
	int nProcessed = realCpNum;
	while (nProcessed < cmInfo.vertices.size())
	{
		for (unsigned k = 0; k < cmInfo.vertices.size(); k++)
		{
			if (cmInfo.vertices[k].flag == -1)
			{
				vector<unsigned> neighbors;
				for (set<CMEdgeInfo>::iterator it = cmInfo.edges.begin(); it != cmInfo.edges.end(); it++)
				{
					if (it->first() == k && cmInfo.vertices[it->second()].flag != -1)
						neighbors.push_back(it->second());
					else if (it->second() == k && cmInfo.vertices[it->first()].flag != -1)
						neighbors.push_back(it->first());
				}

				if (!neighbors.empty())
				{
					double x = 0, y = 0, r = 0, g = 0, b = 0;
					for (unsigned i = 0; i < neighbors.size(); i++)
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
					cmInfo.vertices[k].do_display = false;
					nProcessed++;
				}
			}
		}
	}

	//compute control points sequences exclude boundary feature
	for (int i = 0; i < collinear_knots_lines.size(); i++)
	{
		vector<vector<unsigned int>> interior_configs;
		if (nknotinsert_collinear > 0)
		{
			if (collinear_knots_lines[i].size() >= 3)
			{
				if (collinear_knots_lines[i][0].index == collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index)
				{
					vector<unsigned int> inte_config;
					if (nDeg == 2)
					{
						inte_config.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 2].index);
						inte_config.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

						inte_config.push_back(collinear_knots_lines[i][0].index);
						inte_config.push_back(collinear_knots_lines[i][0].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

						inte_config.push_back(collinear_knots_lines[i][0].index);
						inte_config.push_back(collinear_knots_lines[i][1].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

					}
					else if (nDeg == 3)
					{
						inte_config.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 3].index);
						inte_config.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 2].index);
						inte_config.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

						inte_config.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 2].index);
						inte_config.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index);
						inte_config.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

						inte_config.push_back(collinear_knots_lines[i][0].index);
						inte_config.push_back(collinear_knots_lines[i][0].index);
						inte_config.push_back(collinear_knots_lines[i][0].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

						inte_config.push_back(collinear_knots_lines[i][0].index);
						inte_config.push_back(collinear_knots_lines[i][0].index);
						inte_config.push_back(collinear_knots_lines[i][1].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

						inte_config.push_back(collinear_knots_lines[i][0].index);
						inte_config.push_back(collinear_knots_lines[i][1].index);
						inte_config.push_back(collinear_knots_lines[i][2].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();
					}
				}
				else
				{
					vector<unsigned int> inte_config;
					inte_config.push_back(collinear_knots_lines[i][0].index);
					inte_config.push_back(collinear_knots_lines[i][1].index);
					interior_configs.push_back(inte_config);
					inte_config.clear();
					inte_config.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size()-2].index);
					inte_config.push_back(collinear_knots_lines[i][collinear_knots_lines[i].size()-1].index);
					interior_configs.push_back(inte_config);
				}
			}
		}
		else
		{
			//if (collinear_knots_lines[i].size() >= 3)
			{
				if (collinear_knots_lines[i][0].index == collinear_knots_lines[i][collinear_knots_lines[i].size() - 1].index)
				{
					vector<unsigned int> inte_config;
						inte_config.push_back(collinear_knots_lines[i][0].index);
						inte_config.push_back(collinear_knots_lines[i][1].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();
				}
			}			
		}

		int former;
		if (nknotinsert_collinear > 0) {
			for (int j = 0; j < collinear_knots_lines[i].size();)
			{
				if (collinear_knots_lines[i][j].is_fixed)
				{
					former = j;
				}
				j++;
				if (j >= collinear_knots_lines[i].size() || !collinear_knots_lines[i][j].is_fixed)
				{
					continue;
				}
				int  nknotins = j - former - 1;

				for (int it = 0; it < nknotins - nDeg + 1; it++)
				{
					vector<unsigned int> inte;
					for (int in = 1; in < nDeg + 1; in++)
					{
						inte.push_back(collinear_knots_lines[i][former + it + in].index);
					}
					interior_configs.push_back(inte);
				}

				if (j < collinear_knots_lines[i].size() - (nDeg-1) && j > (nDeg-2)) {
					vector<unsigned int> inte_config;
					if (nDeg == 2)
					{
						inte_config.push_back(collinear_knots_lines[i][j - 1].index);
						inte_config.push_back(collinear_knots_lines[i][j].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

						inte_config.push_back(collinear_knots_lines[i][j].index);
						inte_config.push_back(collinear_knots_lines[i][j].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

						inte_config.push_back(collinear_knots_lines[i][j].index);
						inte_config.push_back(collinear_knots_lines[i][j + 1].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();
					}
					else if (nDeg == 3)
					{
						inte_config.push_back(collinear_knots_lines[i][j - 2].index);
						inte_config.push_back(collinear_knots_lines[i][j - 1].index);
						inte_config.push_back(collinear_knots_lines[i][j].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

						inte_config.push_back(collinear_knots_lines[i][j - 1].index);
						inte_config.push_back(collinear_knots_lines[i][j].index);
						inte_config.push_back(collinear_knots_lines[i][j].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

						inte_config.push_back(collinear_knots_lines[i][j].index);
						inte_config.push_back(collinear_knots_lines[i][j].index);
						inte_config.push_back(collinear_knots_lines[i][j].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

						inte_config.push_back(collinear_knots_lines[i][j].index);
						inte_config.push_back(collinear_knots_lines[i][j].index);
						inte_config.push_back(collinear_knots_lines[i][j + 1].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();

						inte_config.push_back(collinear_knots_lines[i][j].index);
						inte_config.push_back(collinear_knots_lines[i][j + 1].index);
						inte_config.push_back(collinear_knots_lines[i][j + 2].index);
						interior_configs.push_back(inte_config);
						inte_config.clear();
					}
				}
			}
		}
		else
		{
			for (int j = 1; j < collinear_knots_lines[i].size() - 1; j++)
			{
				vector<unsigned int> inte_config;
				inte_config.push_back(collinear_knots_lines[i][j].index);
				inte_config.push_back(collinear_knots_lines[i][j].index);
				interior_configs.push_back(inte_config);
				inte_config.clear();
			}

			for (int j = 0; j < collinear_knots_lines[i].size() - 1; j++)
			{
				vector<unsigned int> inte_config;
				inte_config.push_back(collinear_knots_lines[i][j].index);
				inte_config.push_back(collinear_knots_lines[i][j + 1].index);
				interior_configs.push_back(inte_config);
				inte_config.clear();
			}
		}

		vector<CMVertexInfo> cp_seq;
		for (int j = 0;j<interior_configs.size();j++)
		{
			vector<unsigned> inte_now = interior_configs[j];
			sort(inte_now.begin(), inte_now.end());
			for (int k = 0;k<cmInfo.vertices.size();k++)
			{
				vector<unsigned> inte_v = cmInfo.vertices[k].interior;
				sort(inte_v.begin(), inte_v.end());
				if (equal(inte_now.begin(),inte_now.end(),inte_v.begin()) && inte_now.size() == inte_v.size())
				{
					cmInfo.vertices[k].feacp_type = i;
					CMVertexInfo vnow = cmInfo.vertices[k]; vnow.flag = k;//use flag for edit in this moment
					cp_seq.push_back(vnow);
					break;
				}
			}
		}
		control_points_seqs.push_back(cp_seq);
	}
}

void Image_Approximation::rasterize_image(int magnified_level)
{
	int sz = magnified_level;

	double dx = 1.0 / sz / (input_image->width() - 1);
	double dy = 1.0 / sz / (input_image->height() - 1);

	cv::Mat magnifiedImage(sz * input_image->height() - sz + 1, sz * input_image->width() - sz + 1, CV_8UC3);

	vector<Point_2> sample_ps;
	double tracey = 1.0;
	for (int i = 0; i < sz * input_image->height() - sz + 1; i++)
	{
		double tracex = 0.0;
		for (int j = 0; j < sz * input_image->width() - sz + 1; j++)
		{
			tracex = std::min(tracex,1.0);
			tracey = std::max(tracey,0.0);

			Point_2 temp(tracex, tracey);
			sample_ps.push_back(temp);

			tracex += dx;
			if (tracex > 1.0)
			{
				break;
			}
		}
		tracey += -dy;
		if (tracey < 0.0)
		{
			break;
		}
	}

	//compute basis value
	BSplineBasis new_bsplinebasis = bSplineBasis;
	vector<TConfig3> new_alltconfig3s = alltconfig3s;
	new_alltconfig3s.clear();
	compute_sample_basis(sample_ps, new_bsplinebasis, dataMaps, new_alltconfig3s);

	//combine basis value and control point
	vector<RGBPoint> color_value;
	color_value.resize(sample_ps.size());
#pragma omp parallel for
	for (int key = 0; key < color_value.size(); key++)
	{
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

	int index_now = 0;
	tracey = 1.0;
	for (int i = 0; i < sz * input_image->height() - sz + 1; i++)
	{
		double tracex = 0.0;
		for (int j = 0; j < sz * input_image->width() - sz + 1; j++)
		{
			tracex = std::min(tracex, 1.0);
			tracey = std::max(tracey, 0.0);

			RGBPoint pt = color_value[index_now];
			index_now++;
			int r_ = int(pt.r* 255.0 + 0.5);
			int g_ = int(pt.g* 255.0 + 0.5);
			int b_ = int(pt.b* 255.0 + 0.5);
			r_ = std::max(r_, 0);
			r_ = std::min(r_, 255);
			g_ = std::max(g_, 0);
			g_ = std::min(g_, 255);
			b_ = std::max(b_, 0);
			b_ = std::min(b_, 255);
			magnifiedImage.at<cv::Vec3b>(i, j)[0] = b_;
			magnifiedImage.at<cv::Vec3b>(i, j)[1] = g_;
			magnifiedImage.at<cv::Vec3b>(i, j)[2] = r_;
			tracex += dx;
			if (tracex > 1.0)
			{
				break;
			}
		}
		tracey += -dy;
		if (tracey < 0.0)
		{
			break;
		}
	}

	QString file_name = out_file_name;
	file_name.append("raster_image.png");
	cv::imwrite(file_name.toStdString(), magnifiedImage);
}

void Image_Approximation::compute_mesh_error()
{
	::compute_mesh_error(fittedMesh, originalMesh, statisticsInfos.infos[0]);

	Mesh::Vertex_iterator v_it;
	double range_min = 0.0, range_max = 1.0;
	for (v_it = originalMesh->vertices_begin(); v_it != originalMesh->vertices_end(); v_it++)
	{
		double cur_m = v_it->vertex_error();

		if (cur_m > range_max)
		{
			cur_m = range_max;
		}
		if (cur_m < range_min)
		{
			cur_m = range_min;
		}
		cur_m = 2.5*(cur_m - range_min) / (range_max - range_min);
		Point_3 color_(cur_m, cur_m, cur_m);
		v_it->vertex_color() = color_;
	}

	int lw = input_image->width();
	int lh = input_image->height();
	double m_dx = IMAGEWIDTHSIZE;
	double pixel_l = 2.0*IMAGEWIDTHSIZE / lw;
	double m_dy = pixel_l * lh / 2.0;

	cv::Mat cm_in(lh, lw, CV_8UC3);
	for (auto vit = originalMesh->vertices_begin(); vit != originalMesh->vertices_end(); vit++)
	{
		int i = vit->vertex_coordinate().first;
		int j = vit->vertex_coordinate().second;
		int r_ = int(vit->vertex_color().x()* 255.0 + 0.5);
		int g_ = int(vit->vertex_color().y()* 255.0 + 0.5);
		int b_ = int(vit->vertex_color().z()* 255.0 + 0.5);
		r_ = std::max(r_, 0);
		r_ = std::min(r_, 255);
		g_ = std::max(g_, 0);
		g_ = std::min(g_, 255);
		b_ = std::max(b_, 0);
		b_ = std::min(b_, 255);
		cm_in.at<cv::Vec3b>(lh - 1 - j, i)[0] = b_;
		cm_in.at<cv::Vec3b>(lh - 1 - j, i)[1] = g_;
		cm_in.at<cv::Vec3b>(lh - 1 - j, i)[2] = r_;
	}
	cv::Mat cm_out;
	cv::applyColorMap(cm_in, cm_out, cv::COLORMAP_JET);
	QString file_name = out_file_name;
	file_name.append("/fitted_image_error.png");
	cv::imwrite(file_name.toStdString(), cm_out);

	knot_mesh->map_facet_index_to_iterator();
	knot_mesh->compute_all_face_domain_area();
	knot_mesh->set_all_facets_selected(0);
	std::cout << __FUNCTION__ << ": " << "mesh error finished" << std::endl;

	//insert_new_knots_through_threshold();
	insert_collinear_knots_through_error();
}

void Image_Approximation::insert_new_knots_through_error()
{
	if (nknotinsert_ite > knot_mesh->size_of_vertices() - 20)
	{
		nknotinsert_ite = knot_mesh->size_of_vertices() - 20;
	}

	//assign points in knot triangles
	vector<vector<int>> knots_pixels;
	knots_pixels.resize(knot_mesh->size_of_facets());
	vector<pair<Point2withIndex,int>> mesh_data_map;
	mesh_data_map.resize(fittedMesh->size_of_vertices());
	Mesh::Vertex_iterator vit;
	for (vit = fittedMesh->vertices_begin(); vit != fittedMesh->vertices_end(); vit++)
	{
		Point2withIndex p;
		p.point2 = vit->get_domain();
		p.index = vit->vertex_index();
		mesh_data_map[vit->vertex_index()] = pair<Point2withIndex, int>(p,-1);
	}
	std::cout << __FUNCTION__ << ": " << "pixels on triangle s" << std::endl;
	#pragma omp parallel for
	for (int mit = 0; mit < mesh_data_map.size(); mit++)
	{
		for (auto fit = knot_mesh->facets_begin(); fit != knot_mesh->facets_end(); fit++)
		{
			if (fit->domain_area() < 1e-8)
			{
				continue;
			}
			Mesh::Halfedge_around_facet_circulator chi = fit->facet_begin();
			vector<Point_2> triangles;
			triangles.push_back(chi->vertex()->get_domain());
			chi++;
			triangles.push_back(chi->vertex()->get_domain());
			chi++;
			triangles.push_back(chi->vertex()->get_domain());

			if (CGAL::bounded_side_2(triangles.begin(), triangles.end(), mesh_data_map[mit].first.point2) != CGAL::ON_UNBOUNDED_SIDE ||
				is_on_polygon_convex_bound(mesh_data_map[mit].first.point2, triangles))
			{
				mesh_data_map[mit].second = fit->facet_index();
				break;
			}
		}
	}
	for (int mit = 0; mit < mesh_data_map.size(); mit++)
	{
		if (mesh_data_map[mit].second != -1)
		{
			knots_pixels[mesh_data_map[mit].second].push_back(mesh_data_map[mit].first.index);
		}
		else
		{
			if (!fittedMesh->get_vertex_iterator(mesh_data_map[mit].first.index)->is_feature())
			{
				std::cout << __FUNCTION__ << ": " << "uncounted pixel not on feature" << std::endl;
			}	
		}
	}

	//compute error
	//sort error
	double max_error = 0.0;
	for (auto fit = knot_mesh->facets_begin(); fit != knot_mesh->facets_end(); fit++)
	{
		fit->facet_error() = 0.0;
		fit->facet_flag().bSelected = 0;

		if (fit->domain_area() < 1e-8 || knots_pixels[fit->facet_index()].size() < 3)
		{
			continue;
		}

		double er_ = 0.0;
		double num = 0.0;
		for (int i = 0; i < knots_pixels[fit->facet_index()].size(); i++)
		{
			bool is_plus = true;
			int ver_index = knots_pixels[fit->facet_index()][i];
			if (fittedMesh->get_vertex_iterator(ver_index)->is_feature())
			{
				is_plus = false;
			}
			vector<int> inte = { ver_index };
			vector<int> one_ring;
			from_interior_to_ccw_one_ring_cycle(inte, one_ring, &fittedMesh);
			for (int j = 0; j < one_ring.size(); j++)
			{
				if (fittedMesh->get_vertex_iterator(one_ring[j])->is_feature())
				{
					is_plus = false;
					break;
				}
			}
			if (is_plus)
			{
				num = num + 1;
				er_ += fittedMesh->get_vertex_iterator(ver_index)->vertex_error();
			}
		}
		if (num > 0.1)
		{
			er_ = er_ / num;
		}

		if (er_ > max_error)
		{
			max_error = er_;
		}

		fit->facet_error() = er_;
		faceerror_sort.push_back(pair<int, double>(fit->facet_index(), fit->facet_error()));
	}
	sort(faceerror_sort.begin(), faceerror_sort.end(), sortdoublePair_SecondSmaller);
	knot_mesh->set_max_error(max_error);
	std::cout << __FUNCTION__ << ": " << "max error in the triangle of knot mesh: " << max_error << std::endl;

	//sort area
	vector<pair<int, double>> facearea_sort;
	int num_cut = 3 * nknotinsert_ite;
	for (int i = 0; i < num_cut; i++)
	{
		Mesh::Facet_iterator fi = knot_mesh->get_facet_iterator(faceerror_sort[i].first);
		facearea_sort.push_back(pair<int, double>(fi->facet_index(), fi->domain_area()));
	}
	sort(facearea_sort.begin(), facearea_sort.end(), sortdoublePair_SecondSmaller);

	//insert new knot
	vector<Point_2> insert_ps;
	set<int> free_knotindex;
#if 0
	for (int i = 0; i < nknotinsert_ite; i++)
	{
		Mesh::Facet_iterator fi = knot_mesh->get_facet_iterator(facearea_sort[i].first);
		fi->facet_flag().bSelected = 1;

		//centroid
		Mesh::Halfedge_around_facet_circulator chi = fi->facet_begin();
		Point_2 p0 = chi->vertex()->get_domain();
		if (chi->vertex()->vertex_index() >= Norepeat_knot.size())
		{
			free_knotindex.insert(chi->vertex()->vertex_index());
		}
		chi++;
		Point_2 p1 = chi->vertex()->get_domain();
		if (chi->vertex()->vertex_index() >= Norepeat_knot.size())
		{
			free_knotindex.insert(chi->vertex()->vertex_index());
		}
		chi++;
		Point_2 p2 = chi->vertex()->get_domain();
		if (chi->vertex()->vertex_index() >= Norepeat_knot.size())
		{
			free_knotindex.insert(chi->vertex()->vertex_index());
		}

		Point_2 pcd((p0.x() + p1.x() + p2.x()) / 3.0, (p0.y() + p1.y() + p2.y()) / 3.0);
		Point_2 pcentorid((p0.x() + p1.x() + p2.x()) / 3.0 * 2.0*imagemesh_dx - imagemesh_dx,
			(p0.y() + p1.y() + p2.y()) / 3.0 * 2.0*imagemesh_dy - imagemesh_dy);
		fi->error_knot() = pcd;
		insert_ps.push_back(pcentorid);
		//max error position
		//double max_err = -1;
		//Point_2 tempp(0, 0);
		//for (int k = 0; k < knots_pixels[facearea_sort[i].first].size(); k++)
		//{
		//	int ver_index = knots_pixels[facearea_sort[i].first][k];
		//	double er_now = fittedMesh->get_vertex_iterator(ver_index)->vertex_error();
		//	if (er_now > max_err)
		//	{
		//		max_err = er_now;
		//		tempp = fittedMesh->get_vertex_iterator(ver_index)->get_domain();
		//	}
		//}

		//double weight_ = 0.3;
		//fi->error_knot() = Point_2(weight_*tempp.x() + (1 - weight_)*pcd.x(), weight_*tempp.y() + (1 - weight_)*pcd.y());
		//Point_2 perror(tempp.x() * 2.0*imagemesh_dx - imagemesh_dx, tempp.y() * 2.0*imagemesh_dy - imagemesh_dy);
		//Point_2 pout(weight_*perror.x() + (1 - weight_)*pcentorid.x(), weight_*perror.y() + (1 - weight_)*pcentorid.y());
		//insert_ps.push_back(pout);
	}
#endif
	for (auto fit = knot_mesh->facets_begin();fit != knot_mesh->facets_end();fit++)
	{
		fit->facet_flag().bSelected = 0;
	}

	for (int i = 0; i < nknotinsert_ite; i++)
	{
		Mesh::Facet_iterator fi = knot_mesh->get_facet_iterator(faceerror_sort[i].first);
		if (fi->facet_flag().bSelected == 1)
		{
			continue;
		}
		else
		{
			fi->facet_flag().bSelected = 1;
		}
		//std::cout << "\n";
		//centroid
		Mesh::Halfedge_around_facet_circulator chi = fi->facet_begin();
		Point_2 p0 = chi->vertex()->get_domain();
		if (chi->vertex()->vertex_index() >= Norepeat_knot.size())
		{
			free_knotindex.insert(chi->vertex()->vertex_index());
			//std::cout << " face vertices: "<< chi->vertex()->vertex_index();
		}
		chi++;
		Point_2 p1 = chi->vertex()->get_domain();
		if (chi->vertex()->vertex_index() >= Norepeat_knot.size())
		{
			free_knotindex.insert(chi->vertex()->vertex_index());
			//std::cout << "-" << chi->vertex()->vertex_index();
		}
		chi++;
		Point_2 p2 = chi->vertex()->get_domain();
		if (chi->vertex()->vertex_index() >= Norepeat_knot.size())
		{
			free_knotindex.insert(chi->vertex()->vertex_index());
			//std::cout << "-" << chi->vertex()->vertex_index();
		}
		//std::cout << "\n";

		Point_2 pcd((p0.x() + p1.x() + p2.x()) / 3.0, (p0.y() + p1.y() + p2.y()) / 3.0);
		Point_2 pcentorid((p0.x() + p1.x() + p2.x()) / 3.0 * 2.0*imagemesh_dx - imagemesh_dx,
			(p0.y() + p1.y() + p2.y()) / 3.0 * 2.0*imagemesh_dy - imagemesh_dy);
		fi->error_knot() = pcd;
		insert_ps.push_back(pcentorid);
	}
	//std::cout << "number of inserting new knots: " << num_cut << std::endl;

	//make cdt
	if (new_knot_config != NULL)
	{
		delete new_knot_config;
		new_knot_config = NULL;
	}
	new_knot_config = new CDT;
	map<int, CDT::Vertex_handle> vertex_handle_cdt;

	//construct new knot mesh: move local three knots neighboring new center knot
	////first: re-rank knot index
	int index_decrease = 0;
	//std::cout << "number free: " << free_knotindex.size() << std::endl;
	for (int id = 0; id<knot_mesh->size_of_vertices(); id++)
	{
		Mesh::Vertex_iterator vit = knot_mesh->get_vertex_iterator(id);
		set<int>::iterator res = find(free_knotindex.begin(), free_knotindex.end(), id);
		if (res == free_knotindex.end())
		{
			vit->vertex_index() = id - index_decrease;
		}
		else
		{
			vit->vertex_index() = knot_mesh->size_of_vertices() - free_knotindex.size() + index_decrease;
			index_decrease++;
		}
		//std::cout << id<<"-now index: " << vit->vertex_index() << std::endl;
	}
	//make initial edges
	for (auto eit = knot_mesh->edges_begin(); eit != knot_mesh->edges_end(); eit++)
	{
		Point_2 p1(eit->vertex()->point().x() - imagemesh_dx, eit->vertex()->point().y()*2.0*imagemesh_dy - imagemesh_dy);
		Point_2 p2(eit->opposite()->vertex()->point().x() - imagemesh_dx, eit->opposite()->vertex()->point().y()*2.0*imagemesh_dy - imagemesh_dy);
		CDT::Vertex_handle v1 = new_knot_config->insert(p1);
		v1->set_associated_index(eit->vertex()->vertex_index());
		v1->set_new_status(false);

		CDT::Vertex_handle v2 = new_knot_config->insert(p2);
		v2->set_associated_index(eit->opposite()->vertex()->vertex_index());
		v2->set_new_status(false);

		new_knot_config->insert_constraint(v1, v2);
	}
	//write number of fixed knots
	std::ofstream fout1;
	QString dir_number_fixed_knots = out_file_name;
	dir_number_fixed_knots.append("/forIte_numberfixedknots.txt");
	const string str_name1 = dir_number_fixed_knots.toStdString();
	fout1.open(str_name1);
	if (fout1.is_open())
	{
		fout1 << knot_mesh->size_of_vertices() - free_knotindex.size() << " ";
	}
	fout1.close();
	//write constrained edges
	std::ofstream fout;
	QString dir_featureknot_edge = out_file_name;
	dir_featureknot_edge.append("/forIte_featureknotsedge.txt");
	const string str_name = dir_featureknot_edge.toStdString();
	fout.open(str_name);
	if (fout.is_open())
	{
		fout << knot_mesh->size_of_halfedges() / 2 << " ";
		for (auto eit = knot_mesh->edges_begin(); eit != knot_mesh->edges_end(); eit++)
		{
			set<int>::iterator res1 = find(free_knotindex.begin(), free_knotindex.end(), eit->vertex()->vertex_index());
			set<int>::iterator res2 = find(free_knotindex.begin(), free_knotindex.end(), eit->opposite()->vertex()->vertex_index());
			if (res1 == free_knotindex.end() && res2 == free_knotindex.end())
			{
				fout << eit->vertex()->vertex_index() << " " << eit->opposite()->vertex()->vertex_index() << " ";
			}
		}
	}
	fout.close();

	int index_now = new_knot_config->number_of_vertices();
	for (int i = 0; i < insert_ps.size(); i++)
	{
		CDT::Vertex_handle v1 = new_knot_config->insert(insert_ps[i]);
		v1->set_associated_index(index_now);
		v1->set_new_status(1);
		index_now++;
	}
	vertex_handle_cdt.clear();
	for (auto vit = new_knot_config->vertices_begin(); vit != new_knot_config->vertices_end(); vit++)
	{
		vertex_handle_cdt[vit->get_associated_index()] = vit->handle();
	}

	//output to file
#if 1
	//if (is_updating_knot_mesh_afterfitting)
	{
		QString dir_featureknot_cdt = out_file_name;
		dir_featureknot_cdt.append("/forIte_knot_mesh.obj");
		const string obj_name = dir_featureknot_cdt.toStdString();
		std::ofstream fout_obj;
		fout_obj.open(obj_name);
		if (fout_obj.is_open())
		{
			//std::cout << "number of all knots now: " << new_knot_config->number_of_vertices() << std::endl;
			for (int i = 0; i < new_knot_config->number_of_vertices(); i++)
			{
				map<int, CDT::Vertex_handle>::iterator vh = vertex_handle_cdt.find(i);
				if (vh != vertex_handle_cdt.end() && vh->second->get_associated_index() == i)
				{
					double x = vh->second->point().x();
					double y = vh->second->point().y();
					double z = 0.0;
					fout_obj << "v" << " " << x << " " << y << " " << z << "\n";
				}
				else
				{
					std::cout << __FUNCTION__ << ": " << "wrong: no finding: " << i << std::endl;
				}
			}
			//std::cout << "p f " << new_knot_config->number_of_vertices() << std::endl;
			for (auto fit = new_knot_config->faces_begin(); fit != new_knot_config->faces_end(); fit++)
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
#endif
}

void Image_Approximation::insert_new_knots_through_threshold(Point_2 psta,
	Point_2 pend,bool do_select_region)
{
	insert_pixels.clear();
	insert_regions.clear();

	Point_2 p1(psta.x() + imagemesh_dx, (psta.y() + imagemesh_dy) / (2 * imagemesh_dy));
	Point_2 p2(pend.x() + imagemesh_dx, (pend.y() + imagemesh_dy) / (2 * imagemesh_dy));
	vector<Point_2> points;
	points.push_back(p1);
	points.push_back(Point_2(p1.x(), p2.y()));
	points.push_back(p2);
	points.push_back(Point_2(p2.x(), p1.y()));

	map<pair<int, int>, PixelAttribute> pixels_attri;
	vector<pair<int, int>> select_ps;
	for (auto vit = fittedMesh->vertices_begin(); vit != fittedMesh->vertices_end(); vit++)
	{
		bool is_in_quad = false;
		int res = CGAL::bounded_side_2(points.begin(), points.end(), vit->get_domain());
		if (res == CGAL::ON_BOUNDED_SIDE)
			is_in_quad = true;
		if (do_select_region && !is_in_quad)
		{
			continue;
		}

		PixelAttribute pixelnow;
		//pixelnow.insert_status = 1;
		if (vit->vertex_error()*255.0 > err_thres && (!vit->is_feature()))
		{
			pixelnow.do_select = true;
			select_ps.push_back(vit->vertex_coordinate());
			pixelnow.error_ = vit->vertex_error()*255.0;
		}	
		else
		{
			pixelnow.do_select = false;
			if (vit->is_feature())
			{
				pixelnow.error_ = 0.0;
				//pixelnow.insert_status = 0;
			}
			else
			{
				pixelnow.error_ = vit->vertex_error()*255.0;
			}
		}		
		pixelnow.coord = vit->vertex_coordinate();
		pixelnow.vindex = vit->vertex_index();
		pixels_attri[vit->vertex_coordinate()]=pixelnow;
	}

	std::cout << __FUNCTION__ << ": " << "pixels selected through error threshold: "<< select_ps.size() << std::endl;

	if (select_ps.empty())
	{
		return;
	}

	//compute regions
	vector<vector<pair<int, int>>> err_regions;
	vector<pair<int, int>>::iterator itenow = select_ps.begin();
	do 
	{
		vector<pair<int, int>> region_;
		search_neighboring_pixels(*itenow,region_,2, select_ps);
		for (int i = 0;i<region_.size();i++)
		{
			vector<pair<int, int>>::iterator res = find(select_ps.begin(), select_ps.end(), region_[i]);
			if (res != select_ps.end())
			{
				select_ps.erase(res);
			}
		}		
		err_regions.push_back(region_);
		if (select_ps.empty())
		{
			break;
		}
		else
		{
			itenow = select_ps.begin();
		}	
		//std::cout << "region now: size-" << region_.size() << "   pixels left:-" << select_ps.size() << "\n";
	} while (err_regions.size() < (input_image->width()*input_image->height()));

	for (auto it = err_regions.begin();it!=err_regions.end();it++)
	{
#if 0
		bool bhas_fea = false;
		for (int i = 0; i < it->size(); i++)
		{
			map<pair<int, int>, PixelAttribute>::iterator res1 = pixels_attri.find((*it)[i]);
			if (res1 != pixels_attri.end())
			{
				if (res1->second.insert_status == 0)
				{
					bhas_fea = true;
					break;
				}
			}
		}
		if (!bhas_fea)
#endif
		{
			if (it->size() > 13)
			{
				insert_regions.push_back(*it);
			}
		}	
	}

	std::cout << __FUNCTION__ << ": " << "compute regions end" << std::endl;

	if (insert_regions.empty())
	{
		return;
	}

	//compute the number of inserted knots
	vector<pair<int, double>> regions_err_level;
	for (int it = 0; it < insert_regions.size(); it++)
	{
		double sum_region = 0.0;
		for (int j = 0; j < insert_regions[it].size(); j++)
		{
			map<pair<int, int>, PixelAttribute>::iterator res1 = pixels_attri.find(insert_regions[it][j]);
			if (res1 != pixels_attri.end())
			{
				sum_region += pixels_attri.at(insert_regions[it][j]).error_;
			}			
		}
		double ave_err = sum_region / insert_regions[it].size();
		regions_err_level.push_back(pair<int, double>(it, ave_err));
	}
	sort(regions_err_level.begin(), regions_err_level.end(), sortdoublePair_SecondSmaller);
	map<int, int> region_num_knots;
	for (int it = 0;it<regions_err_level.size();it++)
	{
		//region_num_knots[regions_err_level[it].first] = 6 + it * 10 / regions_err_level.size();
		region_num_knots[regions_err_level[it].first] = 8 + it * 14 / regions_err_level.size();
	}

	std::cout << __FUNCTION__ << ": " << "compute the number of knots to be inserted" << std::endl;

	//insert new knot
	vector<Point_2> insert_ps;
	set<int> free_knotindex;
	for (int it = 0; it < insert_regions.size(); it++)
	{
		int inserted_internal = region_num_knots.at(it);

		//compute spot error
		vector<PixelAttribute> pixels;	
		for (int j = 0;j<insert_regions[it].size();j++)
		{
			double sum_err = 0.0;
			int half_internal = inserted_internal / 2;
			for (int m = -half_internal;m<half_internal;m++)
			{
				for (int n = -half_internal;n<half_internal;n++)
				{
					pair<int, int> xy(insert_regions[it][j].first + m, insert_regions[it][j].second+ n);
					map<pair<int, int>, PixelAttribute>::iterator res1 = pixels_attri.find(xy);
					if (res1 != pixels_attri.end())
					{
						sum_err += res1->second.error_;
					}
				}
			}
			PixelAttribute pixel_now = pixels_attri.at(insert_regions[it][j]);
			pixel_now.err_spotsum = sum_err;
			pixels.push_back(pixel_now);
		}

		//compute knot position
		for (int k = 0;k<input_image->width();k++)
		{
			if (pixels.empty())
			{
				break;
			}
			sort(pixels.begin(), pixels.end(), sortspoterr_Smaller);
			pair<int, int> coord_now = pixels[0].coord;
			insert_pixels.push_back(coord_now);
			insert_ps.push_back(fittedMesh->get_vertex_iterator(pixels[0].vindex)->get_domain());

			//screen neighbor pixels
			vector<PixelAttribute> pixels_left;
			for (auto ik = pixels.begin(); ik != pixels.end(); ik++)
			{
				if (!(abs((*ik).coord.first-coord_now.first)<inserted_internal &&
					abs((*ik).coord.second - coord_now.second)<inserted_internal))
				{
					pixels_left.push_back(*ik);
				}
			}
			pixels = pixels_left;
		}
		//std::cout << "inserting region:"<<it<<"-inserting number all: "<< insert_pixels.size()<<"-inserting internal:" << inserted_internal << "\n";
	}
	if (insert_ps.empty())
	{
		return;
	}
	std::cout << __FUNCTION__ << ": " << "number of new inserted knots:" << insert_ps.size() << "\n";

	//looking for knot face
	for (int mit = 0; mit < insert_ps.size(); mit++)
	{
		bool success_insert = 0;
		for (auto fit = knot_mesh->facets_begin(); fit != knot_mesh->facets_end(); fit++)
		{
			if (fit->domain_area() < 1e-7)
			{
				continue;
			}
			Mesh::Halfedge_around_facet_circulator chi = fit->facet_begin();
			vector<Point_2> triangles;
			triangles.push_back(chi->vertex()->get_domain());
			chi++;
			triangles.push_back(chi->vertex()->get_domain());
			chi++;
			triangles.push_back(chi->vertex()->get_domain());

			if (CGAL::bounded_side_2(triangles.begin(), triangles.end(), insert_ps[mit]) != CGAL::ON_UNBOUNDED_SIDE ||
				is_on_polygon_convex_bound(insert_ps[mit], triangles))
			{
				fit->facet_flag().bSelected = 1;
				fit->error_knot() = insert_ps[mit];

				Mesh::Halfedge_around_facet_circulator chi = fit->facet_begin();
				if (chi->vertex()->vertex_index() >= Norepeat_knot.size())
				{
					free_knotindex.insert(chi->vertex()->vertex_index());
				}
				chi++;
				if (chi->vertex()->vertex_index() >= Norepeat_knot.size())
				{
					free_knotindex.insert(chi->vertex()->vertex_index());
				}
				chi++;
				if (chi->vertex()->vertex_index() >= Norepeat_knot.size())
				{
					free_knotindex.insert(chi->vertex()->vertex_index());
				}

				Point_2 pcd(insert_ps[mit].x() * 2.0*imagemesh_dx - imagemesh_dx,
					insert_ps[mit].y() * 2.0*imagemesh_dy - imagemesh_dy);
				insert_ps[mit] = pcd;

				success_insert = 1;
				break;
			}
		}
		if (!success_insert)
		{
			std::cout << __FUNCTION__ << ": " << "failed inserting new knot: position-x-" << insert_ps[mit].x() << "-y-" << insert_ps[mit].y() << "\n";
		}
	}

	//make cdt
	if (new_knot_config != NULL)
	{
		delete new_knot_config;
		new_knot_config = NULL;
	}
	new_knot_config = new CDT;
	map<int, CDT::Vertex_handle> vertex_handle_cdt;

	//construct new knot mesh: move local three knots neighboring new center knot
	////first: re-rank knot index
	int index_decrease = 0;
	//std::cout << "number free: " << free_knotindex.size() << std::endl;
	for (int id = 0; id < knot_mesh->size_of_vertices(); id++)
	{
		Mesh::Vertex_iterator vit = knot_mesh->get_vertex_iterator(id);
		set<int>::iterator res = find(free_knotindex.begin(), free_knotindex.end(), id);
		if (res == free_knotindex.end())
		{
			vit->vertex_index() = id - index_decrease;
		}
		else
		{
			vit->vertex_index() = knot_mesh->size_of_vertices() - free_knotindex.size() + index_decrease;
			index_decrease++;
		}
		//std::cout << id<<"-now index: " << vit->vertex_index() << std::endl;
	}
	//make initial edges
	for (auto eit = knot_mesh->edges_begin(); eit != knot_mesh->edges_end(); eit++)
	{
		Point_2 p1(eit->vertex()->point().x() - imagemesh_dx, eit->vertex()->point().y()*2.0*imagemesh_dy - imagemesh_dy);
		Point_2 p2(eit->opposite()->vertex()->point().x() - imagemesh_dx, eit->opposite()->vertex()->point().y()*2.0*imagemesh_dy - imagemesh_dy);
		CDT::Vertex_handle v1 = new_knot_config->insert(p1);
		v1->set_associated_index(eit->vertex()->vertex_index());
		v1->set_new_status(false);

		CDT::Vertex_handle v2 = new_knot_config->insert(p2);
		v2->set_associated_index(eit->opposite()->vertex()->vertex_index());
		v2->set_new_status(false);

		new_knot_config->insert_constraint(v1, v2);
	}
	//write number of fixed knots
	std::ofstream fout1;
	QString dir_number_fixed_knots = out_file_name;
	dir_number_fixed_knots.append("/forIte_numberfixedknots.txt");
	const string str_name1 = dir_number_fixed_knots.toStdString();
	fout1.open(str_name1);
	if (fout1.is_open())
	{
		fout1 << knot_mesh->size_of_vertices() - free_knotindex.size() << " ";
	}
	fout1.close();
	//write constrained edges
	std::ofstream fout;
	QString dir_featureknot_edge = out_file_name;
	dir_featureknot_edge.append("/forIte_featureknotsedge.txt");
	const string str_name = dir_featureknot_edge.toStdString();
	fout.open(str_name);
	if (fout.is_open())
	{
		fout << knot_mesh->size_of_halfedges() / 2 << " ";
		for (auto eit = knot_mesh->edges_begin(); eit != knot_mesh->edges_end(); eit++)
		{
			set<int>::iterator res1 = find(free_knotindex.begin(), free_knotindex.end(), eit->vertex()->vertex_index());
			set<int>::iterator res2 = find(free_knotindex.begin(), free_knotindex.end(), eit->opposite()->vertex()->vertex_index());
			if (res1 == free_knotindex.end() && res2 == free_knotindex.end())
			{
				fout << eit->vertex()->vertex_index() << " " << eit->opposite()->vertex()->vertex_index() << " ";
			}
		}
	}
	fout.close();

	int index_now = new_knot_config->number_of_vertices();
	for (int i = 0; i < insert_ps.size(); i++)
	{
		CDT::Vertex_handle v1 = new_knot_config->insert(insert_ps[i]);
		v1->set_associated_index(index_now);
		v1->set_new_status(1);
		index_now++;
	}
	vertex_handle_cdt.clear();
	for (auto vit = new_knot_config->vertices_begin(); vit != new_knot_config->vertices_end(); vit++)
	{
		vertex_handle_cdt[vit->get_associated_index()] = vit->handle();
	}

	//output to file
#if 1
	//if (is_updating_knot_mesh_afterfitting)
	{
		QString dir_featureknot_cdt = out_file_name;
		dir_featureknot_cdt.append("/forIte_knot_mesh.obj");
		const string obj_name = dir_featureknot_cdt.toStdString();
		std::ofstream fout_obj;
		fout_obj.open(obj_name);
		if (fout_obj.is_open())
		{
			//std::cout << "number of all knots now: " << new_knot_config->number_of_vertices() << std::endl;
			for (int i = 0; i < new_knot_config->number_of_vertices(); i++)
			{
				map<int, CDT::Vertex_handle>::iterator vh = vertex_handle_cdt.find(i);
				if (vh != vertex_handle_cdt.end() && vh->second->get_associated_index() == i)
				{
					double x = vh->second->point().x();
					double y = vh->second->point().y();
					double z = 0.0;
					fout_obj << "v" << " " << x << " " << y << " " << z << "\n";
				}
				else
				{
					std::cout << __FUNCTION__ << ": " << "wrong: no finding: " << i << std::endl;
				}
			}
			//std::cout << "p f " << new_knot_config->number_of_vertices() << std::endl;
			for (auto fit = new_knot_config->faces_begin(); fit != new_knot_config->faces_end(); fit++)
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
#endif
}

void Image_Approximation::insert_collinear_knots_through_error()
{
	threshold_error = threshold_error/ double(input_image->width() - 1);

	vector<Point_2>	refine_points;
	for (int i = 0; i < originalMesh->size_of_vertices(); i++)
	{
		Mesh::Vertex_iterator vit = originalMesh->get_vertex_iterator(i);
		Mesh::Vertex_iterator vnewit = fittedMesh->get_vertex_iterator(i);

		if (!vit->is_feature())
		{
			continue;
		}

		double error_ = sqrt(pow(vit->smooth_point().x() - vnewit->point().x(), 2) +
			pow(vit->smooth_point().y() - vnewit->point().y(), 2));

		if (error_ > threshold_error)
		{
			refine_points.push_back(vit->get_domain());
		}
	}
	std::cout << __FUNCTION__ << ": " << refine_points.size() << "\n";
	if (refine_points.empty())
	{
		return;
	}

	vector<pair<int, int>> insert_knot_index;
	for (int i = 0;i<collinear_knots_lines.size();i++)
	{
		for (int j = 0;j<collinear_knots_lines[i].size()-1;j++)
		{
			Point_2 former = collinear_knots_lines[i][j].paradomain;
			Point_2 later = collinear_knots_lines[i][j + 1].paradomain;
			for (int n =0;n<refine_points.size();n++)
			{
				if (abs(area(former,later,refine_points[n]))>1e-12)
				{
					continue;
				}
				bool x_check = false;
				if (abs(former.x()-later.x())>1e-6&& 
					refine_points[n].x()>std::min(former.x(),later.x())-1e-8&&
					refine_points[n].x()<std::max(former.x(), later.x())+1e-8)
				{
					x_check = true;
				}
				bool y_check = false;
				if (abs(former.y() - later.y()) > 1e-6&&
					refine_points[n].y() > std::min(former.y(), later.y()) - 1e-8 &&
					refine_points[n].y() < std::max(former.y(), later.y()) + 1e-8)
				{
					y_check = true;
				}
				if (x_check || y_check)
				{
					insert_knot_index.push_back(pair<int, int>(i,j));
					std::cout << "collinear knots to be inserted: " << i << "-" << j << "\n";
					break;
				}
			}
		}
	}

	//write number of fixed knots
	std::ofstream fout1;
	QString add_knots_dir = filename_foraddcollinear_knots;
	add_knots_dir.append("/forIte_Addcollinearknots.txt");
	const string str_name1 = add_knots_dir.toStdString();
	fout1.open(str_name1);
	if (fout1.is_open())
	{
		fout1 << insert_knot_index.size() << "\n";
		for (int i =0;i<insert_knot_index.size();i++)
		{
			fout1 <<  insert_knot_index[i].first<< " "<<insert_knot_index[i].second<<"\n";
		}	
	}
	fout1.close();
}