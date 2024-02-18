#include "simplify.h"

#include <vector>
#include <map>
#include <set>
#include <deque>

using namespace std;
using namespace Eigen;

namespace SUB_SIMPLIFY {

	enum collapse_method {
		COLLAPSE_TO_V1,
		COLLAPSE_TO_V2,
		COLLAPSE_TO_MEAN,
		NO_COLLAPSE_NOW
	};

	inline Vector4f homogenous(const Vector3f &vec) {
		return Vector4f(vec[0], vec[1], vec[2], 1);
	}

	float error(const Vector4f &homo, const Matrix4f &Q) {
		return abs((float)(homo.transpose() * Q * homo));
	}

	double thres_add = 0.0;
	double thres_max = 0.0;
	double dis_pixel = 0.0;

	class contraction;

	vector<vertex*> verteces;
	vector<Matrix4f> initialQ;
	vector<vector<face*> > vertexToFaces;
	vector<set<int> > vertexToEdges;
	set<int> facesToRemove;
	map<int, contraction*> edges;
	map<pair<double, int>, contraction*> valued_edgeId;
	map<int, int> fea_e_pairs;
	map<int, int> fea_v_pairs;
	std::vector<std::vector<int>> in_feature_lines;
	std::vector<std::vector<int>> new_feature_lines;
	map<vertpair, vertpair> final_fea_e_pairs;

	class contraction {
	public:
		int			e_id;
		vertpair	vp;
		int			keep;
		int			remove;
		Matrix4f	Q;
		collapse_method method;
		double		resultError;
		double		combo_error;//for feature
		bool		bfea_pair;

		contraction(
			const vertpair &vertp,bool bfeapair) {
			vp = vertp;
			bfea_pair = bfeapair;
			findMinError();
		}

		void findMinError() {
			Vector4f loc_v1 = homogenous(verteces[vp.first]->loc);
			Vector4f loc_v2 = homogenous(verteces[vp.second]->loc);
			Vector4f loc_vm = (loc_v1 + loc_v2) / 2;
			Q = initialQ[vp.first] + initialQ[vp.second];

			float scale_ = 1e10;
			float err_v1 = scale_*error(loc_v1, Q),
				err_v2 = scale_*error(loc_v2, Q),
				err_vm = scale_*error(loc_vm, Q);

			if (err_v1 < err_v2) {
				method = err_vm < err_v1 ? COLLAPSE_TO_MEAN : COLLAPSE_TO_V1;
			}
			else {
				method = err_vm < err_v2 ? COLLAPSE_TO_MEAN : COLLAPSE_TO_V2;
			}

			/*special situation */
			if (verteces[vp.second]->type.is_corner || verteces[vp.first]->type.is_corner)
			{
				method = COLLAPSE_TO_V1;
				if (verteces[vp.second]->type.is_corner)
				{
					method = COLLAPSE_TO_V2;
				}
			}
			else if (verteces[vp.second]->type.bound_type != -1 || verteces[vp.first]->type.bound_type != -1)
			{
				if (verteces[vp.second]->type.bound_type != -1&& verteces[vp.second]->type.feature_type == -1
					&& verteces[vp.first]->type.feature_type != -1 ||
					verteces[vp.first]->type.bound_type != -1 && verteces[vp.first]->type.feature_type == -1 
					&& verteces[vp.second]->type.feature_type != -1)
				{
					resultError = thres_max;
					method = NO_COLLAPSE_NOW;
					return;
				}
				else if (verteces[vp.second]->type.bound_type != -1 && verteces[vp.first]->type.bound_type == -1)
				{
					method = COLLAPSE_TO_V2;
				}
				else if (verteces[vp.second]->type.bound_type== -1 && verteces[vp.first]->type.bound_type != -1)
				{
					method = COLLAPSE_TO_V1;
				}
				else
				{
					if (verteces[vp.second]->type.bound_type != verteces[vp.first]->type.bound_type)
					{
						resultError = thres_max;
						method = NO_COLLAPSE_NOW;
						return;
					}
				}
			}
			else if (verteces[vp.second]->type.feature_type >= 0 || verteces[vp.first]->type.feature_type >= 0)
			{
				if (verteces[vp.second]->type.feature_type >= 0 && verteces[vp.first]->type.feature_type == -1)
				{
					method = COLLAPSE_TO_V2;
				}
				else if (verteces[vp.second]->type.feature_type == -1 && verteces[vp.first]->type.feature_type >= 0)
				{
					method = COLLAPSE_TO_V1;
				}
				else
				{
					if (verteces[vp.second]->type.feature_type != verteces[vp.first]->type.feature_type ||
						!bfea_pair)
					{
						resultError = thres_max;
						method = NO_COLLAPSE_NOW;
						return;
					}
					else 
					{
						method = verteces[vp.first]->type.is_fea_corner ? COLLAPSE_TO_V1: COLLAPSE_TO_V2;
					}
				}
			}

			if (method == COLLAPSE_TO_V1) {
				resultError = err_v1;
			}
			else if (method == COLLAPSE_TO_V2) {
				resultError = err_v2;
			}
			else {
				resultError = err_vm;
			}
		}

		bool contains(int vid) {
			return vid == vp.first || vid == vp.second;
		}

		void erase_current_edge() {
			auto cur_edge = edges.find(e_id);
			auto cur_Vedge = valued_edgeId.find(pair<double, int>(cur_edge->second->combo_error,e_id));

			int fir = cur_edge->second->vp.first;
			for (auto it = vertexToEdges[fir].begin(); it != vertexToEdges[fir].end(); it++)
			{
				if (*it == e_id)
				{
					vertexToEdges[fir].erase(it);
					break;
				}
			}
			int sed = cur_edge->second->vp.second;
			for (auto it = vertexToEdges[sed].begin(); it != vertexToEdges[sed].end(); it++)
			{
				if (*it == e_id)
				{
					vertexToEdges[sed].erase(it);
					break;
				}
			}
			if (cur_edge->first == 13205)
			{
				int s = 1;
			}
			edges.erase(cur_edge);
			valued_edgeId.erase(cur_Vedge);
		}

		void update_error()
		{
			if (fea_e_pairs.find(e_id) != fea_e_pairs.end())
			{
				//for feature edge pair
				pair<double, int> old_id1(edges.at(e_id)->combo_error, e_id);
				auto res1 = valued_edgeId.find(old_id1);
				valued_edgeId.erase(res1);

				int e2 = fea_e_pairs.at(e_id);
				pair<double, int> old_id2(edges.at(e2)->combo_error, e2);
				auto res2 = valued_edgeId.find(old_id2);
				valued_edgeId.erase(res2);

				double err_ = edges.at(e_id)->resultError + edges.at(e2)->resultError;
				edges.at(e_id)->combo_error =err_;
				pair<double, int> new_id1(err_, e_id);
				valued_edgeId[new_id1] = edges.at(e_id);

				edges.at(e2)->combo_error = err_;
				pair<double, int> new_id2(err_, e2);
				valued_edgeId[new_id2] = edges.at(e2);
			}
			else
			{
				//for current edge
				pair<double, int> old_id(edges.at(e_id)->combo_error, e_id);

				auto res = valued_edgeId.find(old_id);
				valued_edgeId.erase(res);

				edges.at(e_id)->combo_error = edges.at(e_id)->resultError;
				pair<double, int> new_id(edges.at(e_id)->combo_error, e_id);
				valued_edgeId[new_id] = edges.at(e_id);
			}					
		}

		void do_collapase(const set<int> &current_faces2Remove,
			const set<int> &vertex2face_need_process,
			const vector<face*> &keeped_faces)
		{
			/* Update Q for keep vertex */
			initialQ[keep] = Q;

			/* Update loc */
			verteces[keep]->loc_back = verteces[keep]->loc;
			verteces[keep]->type_back = verteces[keep]->type;
			verteces[remove]->bremoved = true;

			/* Update f */
			for (int i = 0; i < keeped_faces.size(); i++) {
				face* f = keeped_faces[i];
				f->verts_back = f->verts;
				f->connectivity_back = f->connectivity;
			}
			/* Update vertexToFaces */
			vertexToFaces[keep] = keeped_faces;
			vertexToFaces[remove].clear();
			for (auto ik = vertex2face_need_process.begin();
				ik != vertex2face_need_process.end(); ik++)
			{
				for (auto ie = vertexToFaces[*ik].begin(); ie != vertexToFaces[*ik].end(); ie++)
				{
					if (find(current_faces2Remove.begin(), current_faces2Remove.end(), 
						(*ie)->id) != current_faces2Remove.end())
					{
						vertexToFaces[*ik].erase(ie);
						break;
					}
				}
			}

			erase_current_edge();

			for (auto it = current_faces2Remove.begin(); it != current_faces2Remove.end(); it++)
			{
				facesToRemove.insert(*it);
			}

			/* Update edges error */
			deque<int> edgesToRemove;
			for (auto it = vertexToEdges[keep].begin(); it != vertexToEdges[keep].end(); it++)
			{
				if (edges.find(*it) == edges.end())
				{
					std::cout << "wrong when update vertexToEdges" << endl;
				}
				if (edges.at(*it)->vp == vp)
				{
					continue;
				}
				edges.at(*it)->findMinError();
				edges.at(*it)->update_error();
			}
			for (auto it = vertexToEdges[remove].begin(); it != vertexToEdges[remove].end(); it++)
			{
				if (edges.at(*it)->vp == vp)
				{
					continue;
				}
				vertpair possible;
				if (edges.at(*it)->vp.first == remove) {
					possible = makeVertpair(edges.at(*it)->vp.second, keep);
				}
				else {
					possible = makeVertpair(edges.at(*it)->vp.first, keep);
				}

				bool do_find = false;
				auto res_it = vertexToEdges[keep].begin();
				for (auto it1 = vertexToEdges[keep].begin(); it1 != vertexToEdges[keep].end(); it1++)
				{
					if (edges.at(*it1)->vp == possible)
					{
						do_find = true;
						res_it = it1;
						break;
					}
				}

				if (!do_find)
				{
					vertexToEdges[keep].insert(*it);

					edges.at(*it)->vp = possible;
					edges.at(*it)->findMinError();
					edges.at(*it)->update_error();
					continue;
				}

				if (edges.at(*it)->bfea_pair) {

					if (edges.at(*res_it)->bfea_pair)
					{
						std::cout << "feature intersection v:" << possible.first << "," << possible.second << std::endl;
						std::cout <<",remove:"<<remove<<","<< verteces[remove]->type.feature_type << "--";
						std::cout << ",keep:" << keep << "," << verteces[keep]->type.feature_type << "--";
						std::cout << verteces[possible.first]->type.feature_type << "-" <<
							verteces[possible.second]->type.feature_type << std::endl;
						std::cout << verteces[possible.first]->loc[0] << ", " <<
							verteces[possible.first]->loc[1] << ", " <<
							verteces[possible.first]->loc[2] << std::endl;
					}
					edgesToRemove.push_back(*res_it);

					vertexToEdges[keep].insert(*it);	
					edges.at(*it)->vp = possible;
					edges.at(*it)->findMinError();
					edges.at(*it)->update_error();
				}
				else {
					edgesToRemove.push_back(*it);
				}
			}
			/* Update vertexToEdges, edges, valued_edgeId */
			for (int i = 0; i < edgesToRemove.size(); i++) {
				map<int, contraction*>::iterator res = edges.find(edgesToRemove[i]);
				if (res != edges.end())
				{
					int fir = edges.at(edgesToRemove[i])->vp.first;
					for (auto it = vertexToEdges[fir].begin(); it != vertexToEdges[fir].end(); it++)
					{
						if (*it == edgesToRemove[i])
						{
							vertexToEdges[fir].erase(it);
							break;
						}
					}
					int sed = edges.at(edgesToRemove[i])->vp.second;
					for (auto it = vertexToEdges[sed].begin(); it != vertexToEdges[sed].end(); it++)
					{
						if (*it == edgesToRemove[i])
						{
							vertexToEdges[sed].erase(it);
							break;
						}
					}

					/* Update edges and  valued_edgeId*/
					pair<double, int> old_id(res->second->combo_error, res->first);
					map<pair<double, int>, contraction*>::iterator resv = valued_edgeId.find(old_id);
					valued_edgeId.erase(resv);

					edges.erase(res);
				}				
			}
			vertexToEdges[remove].clear();
		}

		bool able2collapse(set<int> &faces2Remove,
			set<int> &vertex2face_need_process,
			vector<face*> &keeped_faces)
		{
			if (method == NO_COLLAPSE_NOW)
			{
				return 0;
			}

			if (method == COLLAPSE_TO_MEAN) {
				verteces[keep]->loc = (verteces[remove]->loc + verteces[keep]->loc) / 2.0;
			}

			/* Find all old faces on v2 */
			bool bNonmanifold = false;
			vector<face*> faces = vertexToFaces[remove];
			vector<face*> faces_pushback2keep = vertexToFaces[keep];
			for (int i = 0; i < faces.size(); i++) {
				face* f = faces[i];

				/* If v1 and v2 are on this face, we mark it for removal. If
				* only v2 is in this face, we replace v2 with v1. */
				int v1idx = -1, v2idx = -1;
				for (int j = 0; j < f->verts.size(); j++) {
					if (f->verts[j]->id == keep) v1idx = j;
					else if (f->verts[j]->id == remove) v2idx = j;
				}

				if (v1idx == -1) {
					f->verts[v2idx] = verteces[keep];
					for (int j = 0; j < f->connectivity.size(); j++) {
						if (f->connectivity[j].first == remove)
							f->connectivity[j] = makeVertpair(keep, f->connectivity[j].second);
						else if (f->connectivity[j].second == remove)
							f->connectivity[j] = makeVertpair(keep, f->connectivity[j].first);
					}
					bool balready_exist = false;
					std::vector<int> in_;
					for (int n = 0; n < 3; n++)
					{
						in_.push_back(f->verts[n]->id);
					}
					for (auto ik = faces_pushback2keep.begin(); ik != faces_pushback2keep.end(); ik++)
					{
						std::vector<int> exist_;
						int id_in = 0;
						for (int n = 0; n < 3; n++)
						{
							exist_.push_back((*ik)->verts[n]->id);
							if (n == 0)
							{
								for (int l = 0;l<3;l++)
								{
									if (in_[l] == exist_[n])
									{
										id_in = l;
										break;
									}
								}
							}
						}
						bool bsame = true;
						for (int n = 0;n < 3;n++)
						{
							if (in_[(id_in+n)%3] != exist_[n])
							{
								bsame = false;
							}
						}
						if (bsame)
						{
							balready_exist = true;
							break;
						}

						std::vector<int> in2 = in_;
						sort(in2.begin(), in2.end());
						sort(exist_.begin(), exist_.end());
						if (equal(in2.begin(), in2.end(), exist_.begin()))
						{
							bNonmanifold = true;
							break;
						}
					}
					if (!balready_exist)
					{
						faces_pushback2keep.push_back(f);
					}
					else
					{
						for (int j = 0; j < f->verts.size(); j++) {
							int id_p = f->verts[j]->id;
							if (id_p != remove && id_p != keep)
								vertex2face_need_process.insert(id_p);
						}
						faces2Remove.insert(f->id);
						//std::cout << "already exist" << std::endl;
					}
				}
				else {
					for (int j = 0; j < f->verts.size(); j++) {
						int id_p = f->verts[j]->id;
						if (id_p != remove && id_p != keep)
							vertex2face_need_process.insert(id_p);
					}
					faces2Remove.insert(f->id);
				}
			}
			for (auto it = faces_pushback2keep.begin(); it != faces_pushback2keep.end(); it++)
			{
				set<int>::iterator res = find(faces2Remove.begin(), faces2Remove.end(), (*it)->id);
				if (res == faces2Remove.end())
				{
					keeped_faces.push_back(*it);
				}
			}

			//check edge conflict
			bool is_conflict = false;
			for (auto it = vertexToEdges[remove].begin(); it != vertexToEdges[remove].end(); it++)
			{
				if (edges.at(*it)->vp == vp)
				{
					continue;
				}
				vertpair possible;
				if (edges.at(*it)->vp.first == remove) {
					possible = makeVertpair(edges.at(*it)->vp.second, keep);
				}
				else {
					possible = makeVertpair(edges.at(*it)->vp.first, keep);
				}

				bool do_find = false;
				auto res_it = vertexToEdges[keep].begin();
				for (auto it1 = vertexToEdges[keep].begin(); it1 != vertexToEdges[keep].end(); it1++)
				{
					if (edges.at(*it1)->vp == possible)
					{
						do_find = true;
						res_it = it1;
						break;
					}
				}

				if (do_find && edges.at(*it)->bfea_pair
					&&edges.at(*res_it)->bfea_pair)
				{
					is_conflict = true;
				}
			}

			//check fold-over
			bool has_foldover_tri = false;
			for (auto it = keeped_faces.begin(); it != keeped_faces.end(); it++)
			{
				Point_2 p1((*it)->verts[0]->loc[0], (*it)->verts[0]->loc[1]);
				Point_2 p2((*it)->verts[1]->loc[0], (*it)->verts[1]->loc[1]);
				Point_2 p3((*it)->verts[2]->loc[0], (*it)->verts[2]->loc[1]);
				//double areaaaa = CGAL::area(p1, p2, p3);
					if (CGAL::area(p1, p2, p3) < 1e-16)
					{
						has_foldover_tri = true;
						break;
					}
			}

			bool bprocess = false;

			if (!has_foldover_tri && !bNonmanifold && !is_conflict)
			{
				//check distance of simplified feature edges
				if (bfea_pair && !verteces[keep]->added)
				{				
					bool b_another_pair = true;
					for (int m = 0; m < new_feature_lines.size(); m++)
					{
						auto res1 = find(new_feature_lines[m].begin(), new_feature_lines[m].end(), keep);
						auto res2 = find(new_feature_lines[m].begin(), new_feature_lines[m].end(), remove);
						if (res1 != new_feature_lines[m].end() && res2 != new_feature_lines[m].end())
						{
							b_another_pair = false;
							if (new_feature_lines[m].size() > MIN_NUMBER_OF_FEAPOLY)
							{
								bool bfill = false;
								vector<int> internal_vs;
								for (int n = 0;n<in_feature_lines[m].size();n++)
								{
									if (in_feature_lines[m][n] == keep || in_feature_lines[m][n] == remove)
									{
										bfill = !bfill;
										internal_vs.push_back(in_feature_lines[m][n]);
									}
									if (bfill)
									{
										internal_vs.push_back(in_feature_lines[m][n]);
									}
								}
								
								int next_;
								if (*res2 != *new_feature_lines[m].rbegin())
								{
									res2++;
									next_ = *res2;
									if (*res2 == keep)
									{
										res2--; 
										if (*res2 == *new_feature_lines[m].begin())
										{
											next_ = *new_feature_lines[m].rbegin();
										}
										else
										{
											res2--;
											next_ = *res2;
										}					
									}
								}
								else
								{
									if (keep == *new_feature_lines[m].begin())
									{
										res2--;
										next_ = *res2;
									}
									else
									{
										next_ = *new_feature_lines[m].begin();
									}
								}

								bool boversize = false;
								for (int n = 0;n<internal_vs.size();n++)
								{
									Point_2 p1(verteces[internal_vs[n]]->loc_ori[0], verteces[internal_vs[n]]->loc_ori[1]);
									Point_2 p2(verteces[keep]->loc_back[0], verteces[keep]->loc_back[1]);
									Point_2 p3(verteces[next_]->loc_back[0], verteces[next_]->loc_back[1]);
									double dis = abs(CGAL::area(p1, p2, p3))*2.0 / sqrt((p2 - p3).squared_length());
									if (dis > DIS_FEATURE_POLY*dis_pixel)
									{
										boversize = true;
										break;
									}
								}
								if (!boversize)
								{
									bprocess = true;
									break;
								}
							}
						}
					}
					if (b_another_pair)
					{
						std::cout << "wrong\n";
					}
				}
				else
					bprocess = true;
			}

			if (!bprocess)
			{
				verteces[keep]->loc = verteces[keep]->loc_back;

				vector<face*> faces_ = vertexToFaces[remove];
				for (int i = 0; i < faces_.size(); i++) {
					face* f = faces_[i];
					f->verts = f->verts_back;
					f->connectivity = f->connectivity_back;
				}

				return 0;
			}
			else
			{
				return 1;
			}
		}

		bool perform()
		{
			keep = vp.first, remove = vp.second;
			if (method == COLLAPSE_TO_V2) {
				keep = vp.second;
				remove = vp.first;
			}

			int keep2, remove2;

			set<int> cur_faces2Remove;
			set<int> vertex2face_need_process;
			vector<face*> keeped_faces;

			bool bcollapse = able2collapse(cur_faces2Remove, vertex2face_need_process, keeped_faces);
			if (bcollapse)
			{			
				contraction *fea_pair_e;
				int e2_id;
				if (bfea_pair)
				{					
					e2_id = fea_e_pairs.at(e_id);
					fea_pair_e = edges.at(e2_id);

					bool bcorresponding = false;//same location
					if (fea_v_pairs.at(vp.first) == fea_pair_e->vp.first &&
						fea_v_pairs.at(vp.second) == fea_pair_e->vp.second)
					{
						bcorresponding = true;
					}
					if (method == COLLAPSE_TO_V1)
					{
						fea_pair_e->method = bcorresponding ? COLLAPSE_TO_V1 : COLLAPSE_TO_V2;
					}
					else if (method == COLLAPSE_TO_V2)
					{
						fea_pair_e->method = bcorresponding ? COLLAPSE_TO_V2 : COLLAPSE_TO_V1;
					}
					else
					{
						fea_pair_e->method = method;
					}

					keep2 = fea_pair_e->vp.first;
					remove2 = fea_pair_e->vp.second;
					if (fea_pair_e->method == COLLAPSE_TO_V2/* ||
						!bcorresponding && method == COLLAPSE_TO_MEAN*/) {
						keep2 = fea_pair_e->vp.second;
						remove2 = fea_pair_e->vp.first;
					}
					fea_pair_e->keep = keep2;
					fea_pair_e->remove = remove2;

					set<int> e2_cur_faces2Remove;
					set<int> e2_vertex2face_need_process;
					vector<face*> e2_keeped_faces;
					bool bfeature_pair_collapsed = fea_pair_e->able2collapse(e2_cur_faces2Remove, 
						e2_vertex2face_need_process, e2_keeped_faces);

					if (!bfeature_pair_collapsed)
					{
						verteces[keep]->loc = verteces[keep]->loc_back;
						verteces[keep]->type = verteces[keep]->type_back;

						vector<face*> faces_ = vertexToFaces[remove];
						for (int i = 0; i < faces_.size(); i++) {
							face* f = faces_[i];
							f->verts = f->verts_back;
							f->connectivity = f->connectivity_back;
						}
						return 0;
					}
					else
					{																		
						fea_pair_e->do_collapase(e2_cur_faces2Remove,
							e2_vertex2face_need_process, e2_keeped_faces);
					}
				}
				do_collapase(cur_faces2Remove, vertex2face_need_process, keeped_faces);

				if (bfea_pair)
				{
					//update fea_e_pairs
					auto res = fea_e_pairs.find(e_id);
					fea_e_pairs.erase(res);
					auto res0 = fea_e_pairs.find(e2_id);
					fea_e_pairs.erase(res0);

					for (auto it = fea_e_pairs.begin(); it != fea_e_pairs.end(); it++)
					{
						vertpair e1 = edges.at(it->first)->vp;
						if (e1.first == 0 && e1.second == 4445)
						{
							int s = 1;
						}
					}

					//update fea_v_pairs
					auto res1 = fea_v_pairs.find(vp.first);
					auto res2 = fea_v_pairs.find(vp.second);
					auto res3 = fea_v_pairs.find(fea_pair_e->vp.first);
					auto res4 = fea_v_pairs.find(fea_pair_e->vp.second);
					fea_v_pairs.erase(res1);
					fea_v_pairs.erase(res2);
					fea_v_pairs.erase(res3);
					fea_v_pairs.erase(res4);
					fea_v_pairs.insert(pair<int, int>(keep, keep2));
					fea_v_pairs.insert(pair<int, int>(keep2, keep));
				}
				return 1;
			}
			else
			{
				return 0;
			}
		}
	};

	contraction min_edge() {
		map<pair<double, int>, contraction*>::iterator bestv = valued_edgeId.begin();
		map<int, contraction*>::iterator bestit = edges.find(bestv->first.second);
		contraction best = *(bestit->second);
		return best;
	}

	void simplifyMesh(MeshSimplify &mesh, float factor) {

		cout << "Begin simplify:-------------------------------" << "\n";

		/*
		  1. Compute the Q matrices for all the initial vertices.
		  2. Select all valid pairs
		*/
		verteces.resize(mesh.max_vertex_id);
		initialQ.resize(mesh.max_vertex_id);	
		vertexToFaces.resize(mesh.max_vertex_id);
		vertexToEdges.resize(mesh.max_vertex_id);
		for (int i = 0; i < mesh.max_vertex_id; i++)
		{
			verteces[i] = NULL;
			initialQ[i] = Matrix4f::Zero();
			vertexToFaces[i] = vector<face*>();
			vertexToEdges[i] = set<int>();
		}

		set<vertpair> edgeset;

		for (int i = 0; i < mesh.faces.size(); i++) {
			face *f = mesh.faces[i];

			Vector3f v1 = f->verts[0]->loc,
				v2 = f->verts[1]->loc,
				v3 = f->verts[2]->loc;

			if (i == 0)
			{
				double dis1 = sqrt(pow(v1[0] - v2[0],2) + pow(v1[1] - v2[1],2));
				double dis2 = sqrt(pow(v1[0] - v3[0], 2) + pow(v1[1] - v3[1], 2));
				dis_pixel = std::min(dis1, dis2);
			}

			/* equation thanks to Paul Bourke http://paulbourke.net/geometry/planeeq/ */
			Vector4f p;
			p[0] = v1[1] * (v2[2] - v3[2]) + v2[1] * (v3[2] - v1[2]) + v3[1] * (v1[2] - v2[2]);
			p[1] = v1[2] * (v2[0] - v3[0]) + v2[2] * (v3[0] - v1[0]) + v3[2] * (v1[0] - v2[0]);
			p[2] = v1[0] * (v2[1] - v3[1]) + v2[0] * (v3[1] - v1[1]) + v3[0] * (v1[1] - v2[1]);
			p[3] = -(v1[0] * (v2[1] * v3[2] - v3[1] * v2[2]) +
				v2[0] * (v3[1] * v1[2] - v1[1] * v3[2]) +
				v3[0] * (v1[1] * v2[2] - v2[1] * v1[2]));

			Matrix4f pp = p * p.transpose();

			for (auto v_iter = f->verts.begin();
				v_iter != f->verts.end(); v_iter++) {
				vertex* vert = *v_iter;

				verteces[vert->id] = vert;

				vertexToFaces[vert->id].push_back(f);

				/* Add the plane represented by this face on this vertex. */
				initialQ[vert->id] += pp;
			}

			/* Add edges to set */
			for (int j = 0; j < f->connectivity.size(); j++) {
				vertpair edge = f->connectivity[j];

				edgeset.insert(edge);
			}
		}

		/*
		  3. Compute the optimal contraction target v' for each valid pair (v1, v2).
			 The error v'^T (Q1+Q2) v' of this target vertex becomes the cost of
			 contracting that pair.
		  4. Place all the pairs in a heap keyed on cost with the minimum cost
			 pair at the top.
		*/	

		deque<contraction> store_data;

		cout << "create feature pairs \n";
		map<vertpair,int> fea_edge_set;
		int i = 0;
		for (auto edge = edgeset.begin(); edge != edgeset.end(); edge++, i++) {
			if (verteces[edge->first]->type.feature_type != -1 &&
				verteces[edge->second]->type.feature_type != -1)
			{
				fea_edge_set.insert(pair<vertpair, int>(*edge, i));
			}
		}
		//calculate feature edges pair
		for (int m = 0; m < in_feature_lines.size(); m++)
		{
			for (int n = 0; n < in_feature_lines[m].size(); n++)
			{
				int next_v = in_feature_lines[m][(n + 1) % in_feature_lines[m].size()];
				vertpair e1 = makeVertpair(in_feature_lines[m][n], next_v);
				auto res1 = fea_edge_set.find(e1);
				if (res1 == fea_edge_set.end())
				{
					verteces[next_v]->type.is_fea_corner = true;
					verteces[fea_v_pairs.at(next_v)]->type.is_fea_corner = true;
					verteces[in_feature_lines[m][n]]->type.is_fea_corner = true;
					verteces[fea_v_pairs.at(in_feature_lines[m][n])]->type.is_fea_corner = true;

					verteces[next_v]->type_back.is_fea_corner = true;
					verteces[fea_v_pairs.at(next_v)]->type_back.is_fea_corner = true;
					verteces[in_feature_lines[m][n]]->type_back.is_fea_corner = true;
					verteces[fea_v_pairs.at(in_feature_lines[m][n])]->type_back.is_fea_corner = true;
				}
				else
				{					
					int id1 = res1->second;

					//get another edge
					int v1 = fea_v_pairs.at(in_feature_lines[m][n]);
					int v2 = fea_v_pairs.at(next_v);
					vertpair e2 = makeVertpair(v2, v1);
					auto res2 = fea_edge_set.find(e2);
					if (res2 != fea_edge_set.end())
					{
						int id2 = res2->second;
						fea_e_pairs.insert(pair<int, int>(id1, id2));
						fea_e_pairs.insert(pair<int, int>(id2, id1));
					}
					else
						std::cout << "something wrong when initialize feature-edge\n";
				}
			}
		}

		cout << "store data \n";
		std::map<int, int> fea_v_pairs_backup = fea_v_pairs;

		i = 0;
		clock_t t1 = clock();
		for (auto edge = edgeset.begin(); edge != edgeset.end(); edge++,i++) {
			vertexToEdges[edge->first].insert(i);
			vertexToEdges[edge->second].insert(i);
			bool bfeap = false;
			if (fea_e_pairs.find(i) != fea_e_pairs.end())
			{
				bfeap = true;
			}
			store_data.push_back(contraction(*edge,bfeap));
			store_data[i].e_id = i;
			edges[i] = &store_data[i];			
		}
		i = 0;
		for (auto edge = edgeset.begin(); edge != edgeset.end(); edge++, i++) {
			double error_ = store_data[i].resultError;
			if (fea_e_pairs.find(i) != fea_e_pairs.end())
			{
				error_ += store_data[fea_e_pairs.at(i)].resultError;
			}
			store_data[i].combo_error = error_;
			pair<double, int> id_(error_, i);
			valued_edgeId[id_] = &store_data[i];
		}		

		//max and min energy
		thres_max = valued_edgeId.rbegin()->first.first;

		i = 0;
		for (auto it = valued_edgeId.begin();it != valued_edgeId.end();it++,i++)
		{
			if (i == valued_edgeId.size()-1)
			{
				thres_add = it->first.first;
				break;
			}
		}

		clock_t t2 = clock();
		cout << "vertex to edges time: " << t2 - t1 << "\n";
		//make_heap(edges.begin(), edges.end());

		/*
		5. Iteratively remove the pair (v1,v2) of least cost from the heap, contract
		   this pair, and update the costs of all valid pairs involving v1.
		*/
		cout << "begin loop \n";

		int loop_id = 0;
		int fail_n = 0;
		int target_edges = factor > 1? (int)factor: (int)(factor * (float)edges.size());
		while (edges.size() > target_edges) {

			//clock_t from_ = clock();	

			contraction best = min_edge();

			//clock_t to_ = clock();
			//std::cout << "pop timing: " << (to_ - from_) << std::endl;
			bool bsuccess = best.perform();
			//clock_t to_1 = clock();
			//std::cout << "perform timing: " << (to_1 - to_) << std::endl;

			thres_max = valued_edgeId.rbegin()->first.first;
			if (!bsuccess)
			{
				thres_add = max(thres_add,(valued_edgeId.begin()->first.first+thres_max)/2.0);
				std::map<pair<double, int>, contraction*>::iterator res = valued_edgeId.begin();
				res->second->resultError += thres_add;
				res->second->update_error();
				fail_n++;
			}

			if (best.bfea_pair && bsuccess)
			{
				int v_remove;
				if (fea_v_pairs.find(best.vp.first) != fea_v_pairs.end())
				{
					v_remove = best.vp.second;
				}
				if (fea_v_pairs.find(best.vp.second) != fea_v_pairs.end())
				{
					v_remove = best.vp.first;
				}
				for (int m = 0;m<new_feature_lines.size();m++)
				{
					int v_pair = fea_v_pairs_backup.at(v_remove);
					auto res1 = find(new_feature_lines[m].begin(), new_feature_lines[m].end(), v_remove);
					auto res2 = find(new_feature_lines[m].begin(), new_feature_lines[m].end(), v_pair);
					if (res1 != new_feature_lines[m].end() || res2 != new_feature_lines[m].end())
					{
						if (res1 != new_feature_lines[m].end())
						{
							new_feature_lines[m].erase(res1);
						}
						else
						{
							new_feature_lines[m].erase(res2);
						}
						break;
					}		
				}
			}

			loop_id++;
			if (loop_id %10000 == 0)
			{
			
				cout <<loop_id<< " loop: " << edges.size()<<"-fail "<<fail_n
					<<"-min "<< valued_edgeId.begin()->first.first
					<< "-max " << valued_edgeId.rbegin()->first.first << " \n";
				fail_n = 0;
			}

			if (edges.size() <= 30084)
			{
				int s = 1;
				//break;
			}
		}

		cout <<"end: " << edges.size() << " \n";

		vector<face*> newfaces;
		for (int i = 0; i < mesh.faces.size(); i++) {
			if (facesToRemove.find(mesh.faces[i]->id) == facesToRemove.end()) {
				newfaces.push_back(mesh.faces[i]);
			}
		}
		mesh.faces.assign(newfaces.begin(), newfaces.end());

		for (auto it = fea_e_pairs.begin(); it != fea_e_pairs.end(); it++)
		{
			vertpair e1 = edges.at(it->first)->vp;
			vertpair e2 = edges.at(it->second)->vp;

			if (fea_v_pairs.find(e1.first) == fea_v_pairs.end() ||
				fea_v_pairs.find(e1.second) == fea_v_pairs.end())
			{
				std::cout << "not feature p\n";
			}
			else
				final_fea_e_pairs[e1] = e2;
		}
	}

	void get_new_featurelines(std::vector<std::vector<int>> &new_feature)
	{
		new_feature = new_feature_lines;
	}
	void get_ori_featurelines(std::vector<std::vector<Point_2>> &feature)
	{
		feature.clear();
		for (int i = 0;i<in_feature_lines.size();i++)
		{
			vector<Point_2> data;
			for (int j = 0;j<in_feature_lines[i].size();j++)
			{
				Point_2 p(verteces[in_feature_lines[i][j]]->loc_ori[0],
					verteces[in_feature_lines[i][j]]->loc_ori[1]);
				data.push_back(p);
			}
			feature.push_back(data);
		}
	}

	void get_feature_e_pairs(std::map<vertpair, vertpair> &fea_e_pa)
	{
		fea_e_pa = final_fea_e_pairs;
	}

	void get_feature_v_pairs(std::map<int,int> &fea_v_pa)
	{
		fea_v_pa = fea_v_pairs;
	}

	void simplify(std::string infile_mesh, std::string infile_feature,
		MeshSimplify &meshout, double level_)
	{
		std::string infile = infile_mesh;

		std::string infile1 = infile_feature;
		loadFeatureFile(infile1, in_feature_lines);

		std::map<int, int> fea_pairs;
		loadObjFile(infile, meshout, in_feature_lines, fea_pairs);
		//saveObjFile("test.obj", meshout);

		clock_t from_ = clock();
		new_feature_lines = in_feature_lines;
		fea_v_pairs = fea_pairs;
		simplifyMesh(meshout, level_);
		clock_t to_ = clock();
		std::cout << "timing: " << (to_ - from_) / 1000.0 << std::endl;

		//saveObjFile("initialmesh_sim_tt.obj", meshout);

		std::vector<SUB_SIMPLIFY::vertex*> vertices;
		int v_id = 0;
		map<int, int> id_trans;
		for (int i = 0; i < meshout.vertices.size(); i++)
		{
			if (!meshout.vertices[i]->bremoved)
			{
				bool bsaved = true;
				int v_pair;
				if (meshout.vertices[i]->type.feature_type != -1)
				{
					v_pair = fea_v_pairs.at(i);
					for (int m = 0; m < new_feature_lines.size(); m++)
					{
						if ((v_pair == *new_feature_lines[m].rbegin() ||
							v_pair == *new_feature_lines[m].begin()))
						{							
							vertpair e = makeVertpair(*new_feature_lines[m].rbegin(), *new_feature_lines[m].begin());
							if (final_fea_e_pairs.find(e) == final_fea_e_pairs.end())
							{
								bsaved = false;
								break;
							}
						}
					}
				}				
				if (bsaved)
				{
					vertices.push_back(meshout.vertices[i]);
					meshout.vertices[i]->id = v_id;
					id_trans[i] = v_id;
					v_id++;
				}
				else
				{
					id_trans[i] = meshout.vertices[v_pair]->id;
					meshout.vertices[i]->loc = meshout.vertices[v_pair]->loc;
					meshout.vertices[i]->loc_ori = meshout.vertices[v_pair]->loc_ori;
					meshout.vertices[i]->id = meshout.vertices[v_pair]->id;
				}
			}
		}
		meshout.vertices = vertices;

		for (int i = 0;i<new_feature_lines.size();i++)
		{
			for (int j = 0;j<new_feature_lines[i].size();j++)
			{
				new_feature_lines[i][j] = id_trans.at(new_feature_lines[i][j]);
			}
		}

		map<int, int> new_fea_v_pairs;
		for (auto it = fea_v_pairs.begin();it != fea_v_pairs.end();it++)
		{
			new_fea_v_pairs[id_trans.at(it->first)] = id_trans.at(it->second);
		}
		fea_v_pairs = new_fea_v_pairs;

		map<vertpair, vertpair> out_fea_e_pairs;
		for (auto it = final_fea_e_pairs.begin();it != final_fea_e_pairs.end();it++)
		{
			vertpair e1 = makeVertpair(id_trans.at(it->first.first),id_trans.at(it->first.second));
			vertpair e2 = makeVertpair(id_trans.at(it->second.first), id_trans.at(it->second.second));
			out_fea_e_pairs[e1] = e2;
		}
		final_fea_e_pairs = out_fea_e_pairs;

		//saveObjFile("initialmesh_sim.obj", meshout);
	}

	void simplifyTest() {

		MeshSimplify globalMesh;

		std::string infile = "initialmesh.obj";
		std::string outfile = "initialmesh_sim.obj";

		std::string infile1 = "feature.txt";
		std::vector<std::vector<int>> feature_lines;
		loadFeatureFile(infile1, feature_lines);

		std::map<int, int> fea_pairs;

		loadObjFile(infile, globalMesh,feature_lines,fea_pairs);

		saveObjFile("test.obj", globalMesh);

		clock_t from_ = clock();
		fea_v_pairs = fea_pairs;
		in_feature_lines = feature_lines;
		simplifyMesh(globalMesh, 0.01);
		clock_t to_ = clock();
		std::cout << "timing: " << (to_ - from_) / 1000.0 << std::endl;

		saveObjFile(outfile, globalMesh);
	}


}