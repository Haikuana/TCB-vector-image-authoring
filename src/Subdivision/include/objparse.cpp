#include "objparse.h"

#include <cstdio>
#include <iterator>
#include <vector>
#include <map>
#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace SUB_SIMPLIFY {

	struct objface {
		unsigned int id;
		vector<unsigned int> vids;
	};

	typedef pair<face*, face*> tripair;
	vertpair makeVertpair(int v1, int v2) {
		if (v1 < v2) {
			return vertpair(v1, v2);
		}
		else {
			return vertpair(v2, v1);
		}
	}
	map<vertpair, tripair> neighborLookup;

	vector<vertex*> vertices;
	vector<struct objface> faces;

	void parseLine(string line) {
		if (line[0] == 'v' && line[1] == ' ') {
			vertex *v = new vertex;

			float x, y, z;
			float r, g, b;
			int corner, bound, feature;
			sscanf(line.c_str(), "v %f %f %f vc %f %f %f vt %d %d %d", 
				&x, &y, &z, &r,&g,&b,&corner,&bound,&feature);
			v->loc_back = Vector3f(x, y, z);
			v->loc = v->loc_back;
			v->loc_ori = v->loc_back;
			v->color = Vector3f(r, g, b);
			v->id = vertices.size();
			v->normal = Vector3f::Zero();
			v->type.is_corner = corner;
			/*if (bcorner)
			{
				std::cout << "corner" << endl;
			}*/
			v->type.bound_type = bound;
			v->type.feature_type = -1;
			v->type.is_fea_corner = false;
			v->type_back = v->type;
			vertices.push_back(v);

		}
		else if (line[0] == 'f' && line[1] == ' ') {
			istringstream tokenizer(line);
			vector<string> tokens;
			copy(istream_iterator<string>(tokenizer),
				istream_iterator<string>(),
				back_inserter<vector<string> >(tokens));

			struct objface f;
			f.id = faces.size() + 1;
			for (unsigned int i = 1; i < tokens.size(); i++) {
				string token = tokens[i];
				unsigned int vid;
				sscanf(token.c_str(), "%d", &vid);
				f.vids.push_back(vid);
			}

			faces.push_back(f);
		}
	}

	void insertFaceForVertPair(int v1, int v2, face *f) {
		vertpair vp = makeVertpair(v1, v2);
		tripair tp = neighborLookup[vp];
		if (tp.first == NULL) {
			tp.first = f;
		}
		else {
			tp.second = f;
		}
		neighborLookup[vp] = tp;
	}

	void insertFaceIntoNeighborMap(face *f) {
		insertFaceForVertPair(f->verts[0]->id, f->verts[1]->id, f);
		insertFaceForVertPair(f->verts[0]->id, f->verts[2]->id, f);
		insertFaceForVertPair(f->verts[1]->id, f->verts[2]->id, f);
	}

	bool assignNeighbors() {
		bool manifold = true;
		for (auto it = neighborLookup.begin(); it != neighborLookup.end(); it++) {
			vertpair vp = (*it).first;
			tripair tp = (*it).second;
			if (tp.first != NULL || tp.second != NULL) {
				if (tp.first != NULL)
				{				
					tp.first->connectivity.push_back(vp);
					tp.first->connectivity_back.push_back(vp);
				}
				if (tp.second != NULL)
				{
					tp.second->connectivity.push_back(vp);
					tp.second->connectivity_back.push_back(vp);
				}
				if (tp.first != NULL && tp.second != NULL)
				{
					tp.first->neighbors.push_back(tp.second);
					tp.second->neighbors.push_back(tp.first);
				}
			}
			else {
				manifold = false;
			}
		}
		return manifold;
	}

	void calculateFaceNormal(face *f) {
		Vector3f e1 = f->verts[0]->loc - f->verts[1]->loc,
			e2 = f->verts[0]->loc - f->verts[2]->loc;
		f->normal = e1.cross(e2);
		f->normal.normalize();
	}

	void get_neighbor_average(int level, int vid, set<int> &vertices, vector<set<int>> vv_neighbors)
	{
		if (level == 0)
		{
			return;
		}

		for (auto it = vv_neighbors[vid].begin(); it != vv_neighbors[vid].end(); it++)
		{
			vertices.insert(*it);
			get_neighbor_average(level - 1, *it, vertices, vv_neighbors);
		}
	}

	void objfacesToMesh(MeshSimplify& mesh, 
		std::vector<std::vector<int>> feature_lines,
		std::map<int,int> &fea_pairs) {

		//test
#if 0
		ofstream fout;
		fout.open("fea_test.obj");
		if (fout.is_open())
		{
			for (int m = 0; m < feature_lines.size(); m++)
			{
				for (int n = 0; n < feature_lines[m].size(); n++)
				{
					fout << "v " << vertices[feature_lines[m][n]]->loc[0] << " " <<
						vertices[feature_lines[m][n]]->loc[1] << " " <<0
						/*vertices[feature_lines[m][n]]->loc[2]*/ << "\n";
				}
			}
			fout.close();
		}
#endif

		int v_id = vertices.size();
		for (int i = 0; i < faces.size(); i++) {
			struct objface objf = faces[i];
			face *f = new face();
			f->id = objf.id;

			for (int j = 0; j < 3; j++) {
				f->verts[j] = vertices[objf.vids[j] - 1];
			}

			//int exist_ = 0;
			//for (int j = 0; j < 3; j++) {
			//	if (f->verts[j]->id == 1802 || f->verts[j]->id == 1803
			//		|| f->verts[j]->id == 1865)
			//	{
			//		exist_++;
			//	}
			//}
			//if (exist_ == 3)
			//{
			//	int tt = 1;
			//}

			if (!feature_lines.empty())
			{
				for (int m = 0;m<feature_lines.size();m++)
				{
					std::vector<int> v_find;
					int id_n = 0;
					for (int n = 0;n<feature_lines[m].size();n++)
					{
						for (int j = 0; j < 3; j++) {						
							if (f->verts[j]->id == feature_lines[m][n])
							{
								v_find.push_back(feature_lines[m][n]);
								id_n = n;
							}					
						}
					}
					if (v_find.empty())
					{
						continue;
					}
					for (int n = 0;n<v_find.size();n++)
					{
						vertices[v_find[n]]->type.feature_type = m;
						vertices[v_find[n]]->type_back.feature_type = m;
					}
					if (v_find.size() == 1)
					{
						int find_j;
						for (int j = 0; j < 3; j++) {
							if (f->verts[j]->id == v_find[0])
							{
								find_j = j;
								break;
							}
						}
						int v_id_fea1= -1;
						int v_id_fea2= -1;

						Point_2 p3(vertices[f->verts[(find_j + 1) % 3]->id]->loc[0],
							vertices[f->verts[(find_j + 1) % 3]->id]->loc[1]);
						Point_2 p4(vertices[f->verts[(find_j + 2) % 3]->id]->loc[0],
							vertices[f->verts[(find_j + 2) % 3]->id]->loc[1]);

						Point_2 ps(vertices[feature_lines[m][feature_lines[m].size()-1]]->loc[0],
							vertices[feature_lines[m][feature_lines[m].size() - 1]]->loc[1]);
						Point_2 pe(vertices[feature_lines[m][0]]->loc[0],
							vertices[feature_lines[m][0]]->loc[1]);

						if (id_n < feature_lines[m].size()-1 || 
							(ps-pe).squared_length()<2.1*(p3-p4).squared_length())
						{
							v_id_fea2 = feature_lines[m][(id_n + 1)% feature_lines[m].size()];
						}
						if (id_n > 0 ||
							(ps - pe).squared_length() < 2.1*(p3 - p4).squared_length())
						{
							v_id_fea1 = feature_lines[m][(id_n - 1+ feature_lines[m].size()) % feature_lines[m].size()];
						}
						Point_2 p0(vertices[v_find[0]]->loc[0], vertices[v_find[0]]->loc[1]);
						Point_2 p1(p0); Point_2 p2(p0);
						if (v_id_fea1 != -1)
						{
							p1 = Point_2(vertices[v_id_fea1]->loc[0], vertices[v_id_fea1]->loc[1]);
						}
						if (v_id_fea2 != -1)
						{
							p2 = Point_2(vertices[v_id_fea2]->loc[0], vertices[v_id_fea2]->loc[1]);
						}						
						

						bool bcorrect_side = false;
						if (int(CGAL::area(p1, p2, p3) > 1e-8)+ 
							int(CGAL::area(p1, p0, p3) > 1e-8 )+
							int(CGAL::area(p0, p2, p3) > 1e-8) >1)
						{
							bcorrect_side = true;
						}

						if (bcorrect_side)
						{
							if (fea_pairs.find(v_find[0]) != fea_pairs.end())
							{
								for (int j = 0; j < 3; j++) {
									if (f->verts[j]->id == v_find[0])
									{
										f->verts[j] = vertices[fea_pairs.at(v_find[0])];
									}
								}
							}
							else
							{
								vertex *v = new vertex;
								*v = *(vertices[v_find[0]]);
								v->loc[2] = vertices[f->verts[(find_j + 1) % 3]->id]->loc[2];
								v->color = vertices[f->verts[(find_j + 1) % 3]->id]->color;
								//v->loc[2] = v->loc[2] - 0.01;
								v->loc_back = v->loc;
								v->id = v_id;
								v->added = true;
								v->type.feature_type = m;
								v->type_back = v->type;
								vertices.push_back(v);

								for (int j = 0; j < 3; j++) {
									if (f->verts[j]->id == v_find[0])
									{
										f->verts[j] = v;
									}
								}

								fea_pairs.insert(pair<int, int>(v_id, v_find[0]));
								fea_pairs.insert(pair<int, int>(v_find[0], v_id));

								v_id++;
							}			
						}
					}
					else if (v_find.size() >= 2)
					{
						int front_ = v_find[0], back_ = v_find[1];
						if (front_ == feature_lines[m][0]&&
							back_ == feature_lines[m][feature_lines[m].size()-1])
						{
							front_ = v_find[1];
							back_ = v_find[0];
						}

						for (int j = 0; j < 3; j++) {
							if (f->verts[j]->id == front_ && 
								f->verts[(j+1)%3]->id == back_)
							{
								for (int k= 0;k<v_find.size();k++)
								{
									if (fea_pairs.find(v_find[k]) != fea_pairs.end())
									{
										for (int l = 0; l < 3; l++) {
											if (f->verts[l]->id == v_find[k])
											{
												f->verts[l] = vertices[fea_pairs.at(v_find[k])];
											}
										}
									}
									else
									{
										vertex *v = new vertex;
										*v = *(vertices[v_find[k]]);
										v->loc[2] = vertices[f->verts[(j + 2) % 3]->id]->loc[2];
										v->color = vertices[f->verts[(j + 2) % 3]->id]->color;
										//v->loc[2] = v->loc[2] - 0.01;
										v->loc_back = v->loc;
										v->id = v_id;
										v->added = true;
										v->type.feature_type = m;
										v->type_back = v->type;
										vertices.push_back(v);

										for (int l = 0; l < 3; l++) {
											if (f->verts[l]->id == v_find[k])
											{
												f->verts[l] = v;
											}
										}
			
										fea_pairs.insert(pair<int, int>(v_id, v_find[k]));
										fea_pairs.insert(pair<int, int>(v_find[k], v_id));

										v_id++;
									}					
								}
								break;
							}
						}
					}
				
}
			}
	
			f->verts_back = f->verts;

			calculateFaceNormal(f);
			mesh.faces.push_back(f);
			insertFaceIntoNeighborMap(f);
		}

#if 1
		if (!feature_lines.empty()) {
			//recalculate vertex position
			vector<set<int>> vv_neighbors(vertices.size());
			for (int i = 0; i < mesh.faces.size(); i++)
			{
				face* f = mesh.faces[i];
				for (int j = 0; j < f->verts.size(); j++)
				{
					int v1 = f->verts[j]->id;
					int v2 = f->verts[(j + 1) % 3]->id;
					vv_neighbors[v1].insert(v2);
					vv_neighbors[v2].insert(v1);
				}
			}
			for (int i = 0; i < vertices.size(); i++)
			{
				set<int> one_ring = vv_neighbors[vertices[i]->id];
				bool beside_fea = false;
				double z_ = 0.0;
				int n_ = 0;
				for (auto it = one_ring.begin(); it != one_ring.end(); it++)
				{
					if (vertices[*it]->type.feature_type >= 0)
					{
						beside_fea = true;
					}
					else
					{
						z_ += vertices[*it]->loc[2];
						n_++;
					}
				}
				if (beside_fea && vertices[i]->type.feature_type == -1 && n_ > 0)
				{
					vertices[i]->loc[2] = 0.2* vertices[i]->loc[2] + 0.8* z_ / float(n_);
					vertices[i]->loc_back = vertices[i]->loc;
				}

			}
			for (int i = 0; i < vertices.size(); i++)
			{
				if (vertices[i]->type.feature_type >= 0)
				{
					set<int> one_ring = vv_neighbors[vertices[i]->id];
					double z_ = 0.0;
					int n_ = 0;
					for (auto it = one_ring.begin(); it != one_ring.end(); it++)
					{
						if (vertices[*it]->type.feature_type == -1)
						{
							z_ += vertices[*it]->loc[2];
							n_++;
						}
					}
					if (n_ > 0)
					{
						vertices[i]->loc[2] = z_ / float(n_);
						vertices[i]->loc_back = vertices[i]->loc;
					}
				}
			}
		}
#endif

		mesh.vertices = vertices;
		mesh.manifold = assignNeighbors();
		mesh.max_vertex_id = vertices.size();
	}

	void loadObjFile(std::string infile, MeshSimplify& mesh, 
		std::vector<std::vector<int>> feature_lines, std::map<int, int> &fea_pairs) {
		fstream fin;
		fin.open(infile);
		string line;
		while (fin.good()) {
			getline(fin, line);
			parseLine(line);
		}

		objfacesToMesh(mesh,feature_lines,fea_pairs);

		cout << "Loaded mesh: " << endl
			<< "  " << vertices.size() << " vertices." << endl
			<< "  " << faces.size() << " faces in OBJ file." << endl
			<< "  new global mesh size: " << mesh.faces.size() << endl
			<< "  global mesh has " << (mesh.manifold ? "no " : "") << "boundaries" << endl;
	}

	void loadFeatureFile(std::string infile,
		std::vector<std::vector<int>> &feature_lines)
	{
		std::ifstream fin;
		fin.open(infile);
		if (fin.is_open())
		{
			int line_size;
			while (fin >> line_size)
			{
				std::vector<int> polyline(line_size);
				for (int i = 0; i < line_size; i++)
				{
					fin >> polyline[i];
				}
				feature_lines.push_back(polyline);			
			}
			fin.close();
		}		
	}

	void saveObjFile(std::string outfile,const MeshSimplify& mesh) {
		ofstream fout;
		fout.open(outfile);
		if (fout.is_open())
		{
			for (auto v=mesh.vertices.begin();v != mesh.vertices.end();v++)
			{
				fout << "v " << (*v)->loc[0] << " " << (*v)->loc[1] << " " << (*v)->loc[2] << "\n";
			}
			for (auto f = mesh.faces.begin(); f != mesh.faces.end(); f++)
			{
				fout << "f ";
				for (auto fv = (*f)->verts.begin(); fv != (*f)->verts.end(); fv++)
				{
					fout << (*fv)->id+1 << " ";
				}	
				fout << "\n";
			}
			fout.close();

			cout << "Save mesh: " << endl
				<< "  " << mesh.vertices.size() << " vertices." << endl
				<< "  " << mesh.faces.size() << " faces in OBJ file." << endl;
		}
	}

}

