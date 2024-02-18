#include "loop.h"

namespace SUBDIVISION {
	
	LOOP::LOOP(SUB_SIMPLIFY::MeshSimplify *mesh1,
		SUB_SIMPLIFY::MeshSimplify *mesh2, bool blimit) {
		mesh_in = mesh1;
		mesh_out = mesh2;
		bLimitPos = blimit;
	}

	void LOOP::classify_v(SUB_SIMPLIFY::MeshSimplify *mesh,
		int v,int &type,pair<int,int> &chain2)
	{
		if (mesh->vertices[v]->type.is_fea_corner ||
			mesh->vertices[v]->type.is_corner)
		{
			type =  CORNER_V;
			return;
		}
		if (mesh->vertices[v]->type.feature_type == -1 &&
			mesh->vertices[v]->type.bound_type == -1)
		{
			type = SMOOTH_V;
			return;
		}
		if (mesh->vertices[v]->type.bound_type != -1)
		{
			int ncorner = 0;
			for (auto it = mesh->vertices[v]->neighbors.begin();
				it != mesh->vertices[v]->neighbors.end(); it++)
			{
				if (mesh->vertices[*it]->type.is_corner)
				{
					ncorner++;
					if (chain2.first == -1)
					{
						chain2.first = *it;
					}
					else if (chain2.second == -1)
					{
						chain2.second = *it;
					}
				}
			}
			for (auto it = mesh->vertices[v]->neighbors.begin();
				it != mesh->vertices[v]->neighbors.end(); it++)
			{
				if (!mesh->vertices[*it]->type.is_corner &&
					mesh->vertices[*it]->type.bound_type == mesh->vertices[v]->type.bound_type)
				{
					if (chain2.first == -1)
					{
						chain2.first = *it;
					}
					else if (chain2.second == -1)
					{
						chain2.second = *it;
					}
				}
			}
			if (ncorner == 0)
			{
				type = CREASE_V;
				return;
			}
			else if (ncorner == 1)
			{
				type = CREASE_V_ONE_CORNER_NEIBOR;
				return;
			}
			if (ncorner == 2)
			{
				type = CREASE_V_TWO_CORNER_NEIBOR;
				return;
			}
			else
				std::cout << "!!!!!!!!!!!!!!be attention\n";
		}
		int nfeacorner = 0;		
		for (auto it = mesh->vertices[v]->neighbors.begin();
			it != mesh->vertices[v]->neighbors.end(); it++)
		{
			SUB_SIMPLIFY::vertpair vp = SUB_SIMPLIFY::makeVertpair(v, *it);
			if (mesh->vertices[*it]->type.is_fea_corner &&
				fea_e_pair.find(vp)!= fea_e_pair.end())
			{
				nfeacorner++;
				if (chain2.first == -1)
				{
					chain2.first = *it;
				}
				else if (chain2.second == -1)
				{
					chain2.second = *it;
				}		
			}
		}
		if (nfeacorner == 2)
		{
			type = CREASE_V_TWO_CORNER_NEIBOR;
			return;
		}
		for (auto it = fea_e_pair.begin(); it != fea_e_pair.end(); it++)
		{
			int v_nei = it->first.first == v ? it->first.second : -1;
			v_nei = it->first.second == v ? it->first.first : v_nei;

			if (v_nei != -1)
			{
				if (mesh->vertices[v_nei]->type.is_fea_corner)
				{
					continue;
				}
				if (chain2.first == -1)
				{
					chain2.first = v_nei;
				}
				else if (chain2.second == -1)
				{
					chain2.second = v_nei;
				}		
			}
		}
		if (nfeacorner == 0)
		{
			type = CREASE_V;	
			return;
		}	
		if (nfeacorner == 1)
		{
			type = CREASE_V_ONE_CORNER_NEIBOR;
			return;
		}	
	}

	int LOOP::classify_e(SUB_SIMPLIFY::vertpair e)
	{
		bool bfea = fea_e_pair.find(e) != fea_e_pair.end() ? true:false;

		if (mesh_in->vertices[e.first]->type.is_corner &&
			mesh_in->vertices[e.second]->type.bound_type != -1 ||
			mesh_in->vertices[e.second]->type.is_corner &&
			mesh_in->vertices[e.first]->type.bound_type != -1)
		{
			return CREASE_E_ONE_CORNER_NEIBOR;
		}

		if (mesh_in->vertices[e.first]->type.bound_type != -1 &&
			mesh_in->vertices[e.first]->type.bound_type ==
			mesh_in->vertices[e.second]->type.bound_type)
		{
			return CREASE_E_OTHER;
		}

		if (bfea)
		{
			if (mesh_in->vertices[e.first]->type.feature_type != -1 &&
				mesh_in->vertices[e.first]->type.feature_type ==
				mesh_in->vertices[e.second]->type.feature_type)
			{
				if (mesh_in->vertices[e.first]->type.is_fea_corner ||
					mesh_in->vertices[e.second]->type.is_fea_corner)
				{
					return CREASE_E_ONE_CORNER_NEIBOR;
				}
				return CREASE_E_OTHER;
			}
		}

		return SMOOTH_E;
	}

	float LOOP::calculateAlpha(int n) {
		float an = 5 / 8.0 - pow(3.0 + 2.0*cos(6.2831853f / (float)n), 2) / 64.0;
		float alpha = (float)n*(1 - an) / an;
		return alpha;
	}

	float LOOP::calculatelimitW(int n) {		
		float an = 5 / 8.0 - pow(3.0 + 2.0*cos(6.2831853f / (float)n), 2) / 64.0;
		float w_ = 3.0*float(n) / (8.0*an);
		return w_;
	}

	void LOOP::limit_pos()
	{
		//assign neighbors
//#pragma omp parallel for
		for (int i = 0; i < mesh_out->faces.size(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				int v = mesh_out->faces[i]->verts[j]->id;
				int vp = mesh_out->faces[i]->verts[(j + 1) % 3]->id;
				int vc = mesh_out->faces[i]->verts[(j + 2) % 3]->id;
				mesh_out->vertices[v]->neighbors.insert(vp);
				mesh_out->vertices[v]->neighbors.insert(vc);
			}
		}

		// scan all vertices and update its v
		//std::vector<SUB_SIMPLIFY::vertex*> new_vertices(mesh_out->vertices.size(),NULL);
		for (int i = 0; i < mesh_out->vertices.size(); i++)
		{
			mesh_out->vertices[i]->loc_back = mesh_out->vertices[i]->loc;
		}
		for (int i = 0; i < mesh_out->vertices.size(); i++)
		{
			Eigen::Vector3f newpos(0, 0, 0);
			int type_;
			pair<int, int> chain2(-1, -1);
			classify_v(mesh_out,i, type_, chain2);
			int n_ = 0;
			float w_;
			switch (type_)
			{
			case SMOOTH_V:
				for (auto it = mesh_out->vertices[i]->neighbors.begin();
					it != mesh_out->vertices[i]->neighbors.end(); it++)
				{
					newpos += mesh_out->vertices[*it]->loc_back;
					n_++;
				}
				w_ = calculatelimitW(n_);
				newpos = (newpos + w_*mesh_out->vertices[i]->loc_back) / (w_ + n_);
				break;
			case CREASE_V:
				newpos = 1.0 / 6.0*mesh_out->vertices[chain2.first]->loc_back
					+ 1.0 / 6.0*mesh_out->vertices[chain2.second]->loc_back
					+ 4.0 / 6.0* mesh_out->vertices[i]->loc_back;
				break;
			case CORNER_V:
				newpos += mesh_out->vertices[i]->loc_back;
				break;
			case CREASE_V_ONE_CORNER_NEIBOR:
				newpos = 3.0 / 12.0*mesh_out->vertices[chain2.first]->loc_back
					+ 2.0 / 12.0*mesh_out->vertices[chain2.second]->loc_back
					+ 7.0 / 12.0* mesh_out->vertices[i]->loc_back;
				break;
			case CREASE_V_TWO_CORNER_NEIBOR:
				newpos = 1.0 / 4.0*mesh_out->vertices[chain2.first]->loc_back
					+ 1.0 / 4.0*mesh_out->vertices[chain2.second]->loc_back
					+ 2.0 / 4.0* mesh_out->vertices[i]->loc_back;
				break;
			}

			mesh_out->vertices[i]->loc = newpos;
		}
	}

	void LOOP::subdivide() {

		mesh_out->vertices.clear();
		mesh_out->faces.clear();

		std::vector<Tri> triplete;

		//assign neighbors
//#pragma omp parallel for
		for (int i = 0;i<mesh_in->faces.size();i++)
		{
			for (int j = 0;j<3;j++)
			{
				int v = mesh_in->faces[i]->verts[j]->id;
				int vp = mesh_in->faces[i]->verts[(j+1)%3]->id;
				int vc = mesh_in->faces[i]->verts[(j+2)%3]->id;
				mesh_in->vertices[v]->neighbors.insert(vp);
				mesh_in->vertices[v]->neighbors.insert(vc);
			}
		}
		clock_t start = clock();
		// scan all vertices and update its v
		for (int i = 0;i<mesh_in->vertices.size();i++)
		{
			SUB_SIMPLIFY::vertex *new_v = new SUB_SIMPLIFY::vertex;
			new_v->id = i;
			new_v->type = mesh_in->vertices[i]->type;

			Eigen::Vector3f newpos(0, 0, 0);			
			Eigen::Vector3f newcolor(0, 0, 0);
			int type_;
			pair<int,int> chain2(-1,-1);
			classify_v(mesh_in,i,type_,chain2);
			int n_ = 0;
			float alpha;
			switch (type_)
			{
			case SMOOTH_V:	
				for (auto it = mesh_in->vertices[i]->neighbors.begin();
					it != mesh_in->vertices[i]->neighbors.end(); it++)
				{
					newpos += mesh_in->vertices[*it]->loc;
					if (bsubdivided_color)
						newcolor += mesh_in->vertices[*it]->color;
					n_++;
				}
				alpha = calculateAlpha(n_);
				newpos = (newpos + alpha*mesh_in->vertices[i]->loc) / (alpha + n_);
				if (bsubdivided_color)
					newcolor = (newcolor + alpha * mesh_in->vertices[i]->color) / (alpha + n_);
				//mat
				for (auto it = mesh_in->vertices[i]->neighbors.begin();
					it != mesh_in->vertices[i]->neighbors.end(); it++)
				{
					int nei_id = mesh_in->vertices[*it]->id;
					triplete.push_back(Tri(nei_id, i, double(1.0 / (alpha + n_))));
				}
				triplete.push_back(Tri(i, i, double(alpha / (alpha + n_))));
				break;
			case CREASE_V:
				newpos = 1.0 / 8.0*mesh_in->vertices[chain2.first]->loc
					+ 1.0 / 8.0*mesh_in->vertices[chain2.second]->loc
					+ 6.0 / 8.0* mesh_in->vertices[i]->loc;
				if (bsubdivided_color)
				newcolor = 1.0 / 8.0 * mesh_in->vertices[chain2.first]->color
					+ 1.0 / 8.0 * mesh_in->vertices[chain2.second]->color
					+ 6.0 / 8.0 * mesh_in->vertices[i]->color;
				triplete.push_back(Tri(mesh_in->vertices[chain2.first]->id, i, 1.0/8));
				triplete.push_back(Tri(mesh_in->vertices[chain2.second]->id, i, 1.0 / 8));
				triplete.push_back(Tri(i, i, 6.0 / 8));
				break;
			case CORNER_V:
				newpos += mesh_in->vertices[i]->loc;
				if (bsubdivided_color)
					newcolor += mesh_in->vertices[i]->color;
				triplete.push_back(Tri(i, i, 1.0));
				break;
			case CREASE_V_ONE_CORNER_NEIBOR:
				newpos = 3.0 / 16.0*mesh_in->vertices[chain2.first]->loc
					+ 2.0 / 16.0*mesh_in->vertices[chain2.second]->loc
					+ 11.0 / 16.0* mesh_in->vertices[i]->loc;
				if (bsubdivided_color)
					newcolor = 3.0 / 16.0 * mesh_in->vertices[chain2.first]->color
					+ 2.0 / 16.0 * mesh_in->vertices[chain2.second]->color
					+ 11.0 / 16.0 * mesh_in->vertices[i]->color;
				triplete.push_back(Tri(mesh_in->vertices[chain2.first]->id, i, 3.0 / 16.0));
				triplete.push_back(Tri(mesh_in->vertices[chain2.second]->id, i, 2.0 / 16.0));
				triplete.push_back(Tri(i, i, 11.0 / 16.0));
				break;
			case CREASE_V_TWO_CORNER_NEIBOR:
				newpos = 3.0 / 16.0*mesh_in->vertices[chain2.first]->loc
					+ 3.0 / 16.0*mesh_in->vertices[chain2.second]->loc
					+ 10.0 / 16.0* mesh_in->vertices[i]->loc;
				if (bsubdivided_color)
					newcolor = 3.0 / 16.0 * mesh_in->vertices[chain2.first]->color
					+ 3.0 / 16.0 * mesh_in->vertices[chain2.second]->color
					+ 10.0 / 16.0 * mesh_in->vertices[i]->color;
				triplete.push_back(Tri(mesh_in->vertices[chain2.first]->id, i, 3.0 / 16.0));
				triplete.push_back(Tri(mesh_in->vertices[chain2.second]->id, i, 3.0 / 16.0));
				triplete.push_back(Tri(i, i, 10.0 / 16.0));
				break;
			}

			new_v->loc = newpos;
			if (bsubdivided_color)
				new_v->color = newcolor;
			mesh_out->vertices.push_back(new_v);
		}

		// scan all edges and create vertex on each edge
		map<SUB_SIMPLIFY::vertpair, int> e2v;
		int v_id = mesh_in->vertices.size();
		for (int i = 0;i<mesh_in->faces.size();i++)
		{
			for (int k = 0;k<3;k++)
			{
				int v1 = mesh_in->faces[i]->verts[k]->id;
				int v2 = mesh_in->faces[i]->verts[(k+1)%3]->id;
				int v3 = mesh_in->faces[i]->verts[(k+2)%3]->id;
				
				SUB_SIMPLIFY::vertpair e = SUB_SIMPLIFY::makeVertpair(v1, v2);

				if (e2v.find(e) != e2v.end())
				{
					if (classify_e(e) != SMOOTH_E)
					{
						std::cout << "be attention !!!!!!!!!wrong\n";
						classify_e(e);
					}
					mesh_out->vertices[e2v.at(e)]->loc +=
						1/8.0*mesh_in->vertices[v3]->loc;
					if (bsubdivided_color)
						mesh_out->vertices[e2v.at(e)]->color +=
						1 / 8.0 * mesh_in->vertices[v3]->color;
					triplete.push_back(Tri(mesh_in->vertices[v3]->id, mesh_out->vertices[e2v.at(e)]->id, 1.0 /8));
				}
				else
				{
					SUB_SIMPLIFY::vertex *new_v = new SUB_SIMPLIFY::vertex;
					new_v->id = v_id;
					if (mesh_in->vertices[v1]->type.bound_type !=-1 &&
						mesh_in->vertices[v2]->type.bound_type != -1)
					{
						if (mesh_in->vertices[v1]->type.is_corner ||
							mesh_in->vertices[v2]->type.is_corner)
							new_v->type.bound_type = mesh_in->vertices[v1]->type.is_corner ?
							mesh_in->vertices[v2]->type.bound_type :
							mesh_in->vertices[v1]->type.bound_type;
						else
							new_v->type.bound_type = mesh_in->vertices[v1]->type.bound_type ==
								mesh_in->vertices[v2]->type.bound_type ?
								mesh_in->vertices[v1]->type.bound_type : -1;		
					}
					else
					{
						new_v->type.bound_type = -1;
					}

					new_v->type.feature_type = fea_e_pair.find(e) == fea_e_pair.end() ?
						-1 : mesh_in->vertices[v1]->type.feature_type;
					new_v->type.is_corner = false;
					new_v->type.is_fea_corner = false;

					Eigen::Vector3f newpos(0, 0, 0);
					Eigen::Vector3f newcolor(0, 0, 0);

					int type_ = classify_e(e);
					switch (type_)
					{
					case SMOOTH_E:
						newpos = 3 / 8.0*mesh_in->vertices[v1]->loc
							+ 3 / 8.0*mesh_in->vertices[v2]->loc
							+ 1 / 8.0*mesh_in->vertices[v3]->loc;
						if (bsubdivided_color)
							newcolor = 3 / 8.0 * mesh_in->vertices[v1]->color
							+ 3 / 8.0 * mesh_in->vertices[v2]->color
							+ 1 / 8.0 * mesh_in->vertices[v3]->color;
						triplete.push_back(Tri(mesh_in->vertices[v1]->id, v_id, 3.0 / 8.0));
						triplete.push_back(Tri(mesh_in->vertices[v2]->id, v_id, 3.0 / 8.0));
						triplete.push_back(Tri(mesh_in->vertices[v3]->id, v_id, 1.0 / 8.0));
						break;
					case CREASE_E_OTHER:
						newpos = 0.5*mesh_in->vertices[v1]->loc
							+ 0.5*mesh_in->vertices[v2]->loc;
						if (bsubdivided_color)
							newcolor = 0.5 * mesh_in->vertices[v1]->color
							+ 0.5 * mesh_in->vertices[v2]->color;
						triplete.push_back(Tri(mesh_in->vertices[v1]->id, v_id, 0.5));
						triplete.push_back(Tri(mesh_in->vertices[v2]->id, v_id, 0.5));
						break;
					case CREASE_E_ONE_CORNER_NEIBOR:
						if (mesh_in->vertices[v1]->type.is_fea_corner ||
							mesh_in->vertices[v1]->type.is_corner)
						{
							newpos = 0.75*mesh_in->vertices[v1]->loc
								+ 0.25*mesh_in->vertices[v2]->loc;
							if (bsubdivided_color)
								newcolor = 0.75 * mesh_in->vertices[v1]->color
								+ 0.25 * mesh_in->vertices[v2]->color;
							triplete.push_back(Tri(mesh_in->vertices[v1]->id, v_id, 0.75));
							triplete.push_back(Tri(mesh_in->vertices[v2]->id, v_id, 0.25));
						}
						else
						{
							newpos = 0.25*mesh_in->vertices[v1]->loc
								+ 0.75*mesh_in->vertices[v2]->loc;
							if (bsubdivided_color)
								newcolor = 0.25 * mesh_in->vertices[v1]->color
								+ 0.75 * mesh_in->vertices[v2]->color;
							triplete.push_back(Tri(mesh_in->vertices[v1]->id, v_id, 0.25));
							triplete.push_back(Tri(mesh_in->vertices[v2]->id, v_id, 0.75));
						}						
						break;
					}
		
					new_v->loc = newpos;
					if (bsubdivided_color)
						new_v->color = newcolor;
					e2v[e] = v_id;
					mesh_out->vertices.push_back(new_v);

					bool bfea = fea_e_pair.find(e) != fea_e_pair.end() ? true : false;
					if (bfea)
					{
						if (feature_lines != NULL) {
							for (auto in = feature_lines->begin(); in != feature_lines->end(); in++)
							{
								auto res1 = find(in->begin(), in->end(), e.first);
								auto res2 = find(in->begin(), in->end(), e.second);
								if (res1 == in->end() || res2 == in->end())
								{
									continue;
								}
								if (res1 != in->begin())
								{
									res1--;
									if (res1 == res2)
									{
										res1++;
										in->insert(res1, v_id);
									}
									else
									{
										in->insert(res2, v_id);
									}
								}
								else
								{
									res2--;
									if (res1 == res2)
									{
										res2++;
										in->insert(res2, v_id);
									}
									else
									{
										in->insert(res1, v_id);
									}
								}
							}
						}
					}
					
					v_id++;
				}
			}
		}
		
		// Create new faces
		for (int i = 0; i < mesh_in->faces.size(); i++)
		{
			int v1 = mesh_in->faces[i]->verts[0]->id;
			int v2 = mesh_in->faces[i]->verts[1]->id;
			int v3 = mesh_in->faces[i]->verts[2]->id;

			SUB_SIMPLIFY::vertpair e12 = SUB_SIMPLIFY::makeVertpair(v1, v2);
			SUB_SIMPLIFY::vertpair e13 = SUB_SIMPLIFY::makeVertpair(v1, v3);
			SUB_SIMPLIFY::vertpair e23 = SUB_SIMPLIFY::makeVertpair(v3, v2);

			int v12 = e2v.at(e12);
			int v13 = e2v.at(e13);
			int v23 = e2v.at(e23);

			SUB_SIMPLIFY::face *f1 = new SUB_SIMPLIFY::face;
			SUB_SIMPLIFY::face *f2 = new SUB_SIMPLIFY::face;
			SUB_SIMPLIFY::face *f3 = new SUB_SIMPLIFY::face;
			SUB_SIMPLIFY::face *f4 = new SUB_SIMPLIFY::face;
			f1->verts = { mesh_out->vertices[v12],mesh_out->vertices[v23],mesh_out->vertices[v13]};
			f2->verts = { mesh_out->vertices[v1],mesh_out->vertices[v12],mesh_out->vertices[v13] };
			f3->verts = { mesh_out->vertices[v2],mesh_out->vertices[v23],mesh_out->vertices[v12] };
			f4->verts = { mesh_out->vertices[v3],mesh_out->vertices[v13],mesh_out->vertices[v23] };
			mesh_out->faces.push_back(f1);
			mesh_out->faces.push_back(f2);
			mesh_out->faces.push_back(f3);
			mesh_out->faces.push_back(f4);
		}
		
		clock_t finish = clock();
		timing_sub_s += (double(finish - start)) / CLOCKS_PER_SEC;
		std::cout << "real timing:" << timing_sub_s << "\n";

		//create subdivision mat
		mat_subdivision.resize(mesh_in->vertices.size(),mesh_out->vertices.size());
		mat_subdivision.setFromTriplets(triplete.begin(), triplete.end());

		//update feature pairs
		FeaturePair new_fea_e_pair;
		for (auto it = fea_e_pair.begin();it != fea_e_pair.end();it++)
		{
			SUB_SIMPLIFY::vertpair e1 = it->first;
			SUB_SIMPLIFY::vertpair e2 = it->second;

			int e2_v1 = (*fea_v_pair).at(e1.first);
			int e2_v2 = (*fea_v_pair).at(e1.second);
			assert(e2_v1 == e2.first || e2_v1 == e2.second);
			assert(e2_v2 == e2.first || e2_v2 == e2.second);
			int e1_mid = e2v.at(e1);
			int e2_mid = e2v.at(e2);
			(*fea_v_pair)[e1_mid] = e2_mid;
			(*fea_v_pair)[e2_mid] = e1_mid;
			SUB_SIMPLIFY::vertpair e11 = SUB_SIMPLIFY::makeVertpair(e1.first, e1_mid);
			SUB_SIMPLIFY::vertpair e12 = SUB_SIMPLIFY::makeVertpair(e1.second, e1_mid);
			SUB_SIMPLIFY::vertpair e21 = SUB_SIMPLIFY::makeVertpair(e2_v1, e2_mid);
			SUB_SIMPLIFY::vertpair e22 = SUB_SIMPLIFY::makeVertpair(e2_v2, e2_mid);

			new_fea_e_pair[e11] = e21;
			new_fea_e_pair[e21] = e11;
			new_fea_e_pair[e12] = e22;
			new_fea_e_pair[e22] = e12;
		}
		fea_e_pair = new_fea_e_pair;


		if (bLimitPos)
		{
			limit_pos();
		}
	}
}