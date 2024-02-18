#include "editing.h"
#include <QtWidgets/QMessageBox>
#include <QDir>

namespace EDITGUI {

	VecEditing::VecEditing()
	{
		render_vecimage = NULL;
		is_fixed_modelview = true;

		edit_color[0] = 0.5; edit_color[1] = 0.0; edit_color[2] = 0.0;
		edit_color_backg[0] = 0.5; edit_color_backg[1] = 0.0; edit_color_backg[2] = 0.0;

		authoring_right_color[0] = 0.5; authoring_right_color[1] = 0.0; authoring_right_color[2] = 0.0;
		authoring_left_color[0] = 0.5; authoring_left_color[1] = 0.0; authoring_left_color[2] = 0.0;
	}

	VecEditing::~VecEditing()
	{
		if (render_vecimage != NULL)
		{
			delete render_vecimage;
			render_vecimage = NULL;
		}
	}

	bool VecEditing::load_editing_data(QString filename)
	{
		operation_mode = 1;

		infilename = filename;

		QPoint size_;
		Image_Approximation appimage;
		bool success_ = appimage.read_editdata(filename, fm, cps_seqs, cm,size_);

		image_wid_ = size_.x();
		image_hei_ = size_.y();

		//convert fitted mesh data to CM
		render_vecimage = new RGBPoint[fm.faces.size() * 3];
		int it = 0;
		for (auto fit = fm.faces.begin(); fit != fm.faces.end(); fit++)
		{
			for (int k = 0; k < fit->face.size(); k++)
			{
				RGBPoint p = fm.vertices[fit->face[k]];
				render_vecimage[it] = (p);
				it++;
			}
		}

		//complete feature polylines		
		for (int i = 0;i<cps_seqs.size();i++)
		{
			if (cps_seqs[i].empty())
				continue;

			bool is_cycle = false;
			int id1 = cps_seqs[i][0].index;
			int id2 = cps_seqs[i][cps_seqs[i].size() - 1].index;
			if (id1 == id2)
			{
				continue;
			}
			for (auto it = cm.edges.begin();it != cm.edges.end();it++)
			{
				if (it->first() == id1 && it->second() == id2 ||
					it->first() == id2 && it->second() == id1)
				{
					is_cycle = true;
					break;
				}
			}
			if (is_cycle)
			{
				cps_seqs[i].push_back(cps_seqs[i][0]);
			}
		}

		//remedy auxiliary cps
		for (int i = 0; i < cps_seqs.size(); i++)
		{
			for (int j = 0;j<cps_seqs[i].size();j++)
			{
				Vector_2 vecV;
				Point_2 pfir, psed;
				bool bcycle = cps_seqs[i][0].index == cps_seqs[i][cps_seqs[i].size() - 1].index;
				if (j == 0 && !bcycle)
				{
					pfir = cps_seqs[i][j].point2; psed = cps_seqs[i][j + 1].point2;
					vecV = cps_seqs[i][j + 1].point2 - cps_seqs[i][j].point2;
					vecV = Vector_2(-vecV.y(), vecV.x()) / sqrt(vecV.squared_length());
				}
				else if (j == cps_seqs[i].size() - 1 && !bcycle)
				{
					pfir = cps_seqs[i][j - 1].point2; psed = cps_seqs[i][j].point2;
					vecV = cps_seqs[i][j].point2 - cps_seqs[i][j - 1].point2;
					vecV = Vector_2(-vecV.y(), vecV.x()) / sqrt(vecV.squared_length());
				}
				else
				{
					if (j == cps_seqs[i].size() - 1)
					{
						break;
					}

					Vector_2 vecV1, vecV2;
					pfir = cps_seqs[i][j].point2; psed = cps_seqs[i][j + 1].point2;
					if (j == 0)
					{
						vecV1 = cps_seqs[i][cps_seqs[i].size()-2].point2 - cps_seqs[i][j].point2;						
					}
					else
					{
						vecV1 = cps_seqs[i][j - 1].point2 - cps_seqs[i][j].point2;
					}					
					vecV2 = psed - pfir;
					vecV1 = vecV1 / sqrt(vecV1.squared_length());								
					vecV2 = vecV2 / sqrt(vecV2.squared_length());
					vecV = vecV1 + vecV2;
					if (vecV.squared_length()<1e-16)
					{
						vecV = Vector_2(-vecV1.y(), vecV1.x());
					}
					else
					{
						vecV = vecV / sqrt(vecV.squared_length());
					}					
				}

				bool bCCW = cm.vertices[cps_seqs[i][j].index].bCCWorientation;
				RGBPoint pcenter = cm.vertices[cps_seqs[i][j].index].cp_pair.first;
				RGBPoint ptrans1 = cm.vertices[cps_seqs[i][j].index].cp_pair.second;
				RGBPoint ptrans2 = cm.vertices[cps_seqs[i][j].spare_index].cp_pair.second;
				double dis = sqrt(pow(ptrans1.x-pcenter.x, 2) + pow(ptrans1.y-pcenter.y, 2));
				ptrans1.x = pcenter.x - vecV.x()*dis;
				ptrans1.y = pcenter.y - vecV.y()*dis;
				ptrans2.x = pcenter.x + vecV.x()*dis;
				ptrans2.y = pcenter.y + vecV.y()*dis;

				Point_2 test_p(pcenter.x - vecV.x(), pcenter.y - vecV.y());
				if (CGAL::area(test_p, pfir, psed) > 0 && bCCW ||
					CGAL::area(test_p, pfir, psed) < 0 && !bCCW)
				{
					cm.vertices[cps_seqs[i][j].index].cp_pair.second = ptrans1;
					cm.vertices[cps_seqs[i][j].spare_index].cp_pair.second = ptrans2;
				}
				else
				{
					cm.vertices[cps_seqs[i][j].index].cp_pair.second = ptrans2;
					cm.vertices[cps_seqs[i][j].spare_index].cp_pair.second = ptrans1;
				}
			}
		}

		//back-up cps
		for (int i = 0;i<cm.vertices.size();i++)
		{
			cm.vertices[i].previous_pair = cm.vertices[i].cp_pair;
			cm.vertices[i].previous_cp = cm.vertices[i].cp;
			if (cm.vertices[i].is_on_feature)
			{
				cm.vertices[i].cp = cm.vertices[i].cp_pair.first;
			}
		}

#if 0
		QString filename1 = "E:/Projects/ImageApproxiTCB/ImageApproxiEdit/build/bin/Results/Amango_ref/mango.edit";
		EditFM						fm1;
		EditCM						cm1;
		CPs							cps_seqs1;
		bool success_1 = appimage.read_editdata(filename1, fm1, cps_seqs1, cm1, size_);

		set<int> updating_cp_candidates;
		for (int i = 0; i < cm.vertices.size(); i++)
		{
			updating_cp_candidates.insert(i);
			double xd_,yd_,rd_, gd_, bd_;
			if (cm.vertices[i].is_on_feature)
			{
				rd_ = cm1.vertices[i].cp_pair.first.r - cm.vertices[i].cp_pair.first.r;
				gd_ = cm1.vertices[i].cp_pair.first.g - cm.vertices[i].cp_pair.first.g;
				bd_ = cm1.vertices[i].cp_pair.first.b - cm.vertices[i].cp_pair.first.b;
				xd_ = cm1.vertices[i].cp_pair.first.x - cm.vertices[i].cp_pair.first.x;
				yd_ = cm1.vertices[i].cp_pair.first.y - cm.vertices[i].cp_pair.first.y;
			}
			else
			{
				rd_ = cm1.vertices[i].cp.r - cm.vertices[i].cp.r;
				gd_ = cm1.vertices[i].cp.g - cm.vertices[i].cp.g;
				bd_ = cm1.vertices[i].cp.b - cm.vertices[i].cp.b;
				xd_ = cm1.vertices[i].cp.x - cm.vertices[i].cp.x;
				yd_ = cm1.vertices[i].cp.y - cm.vertices[i].cp.y;
			}
			cm.vertices[i].deviation.x = xd_;
			cm.vertices[i].deviation.y = yd_;
			cm.vertices[i].deviation.r = rd_;
			cm.vertices[i].deviation.g = gd_;
			cm.vertices[i].deviation.b = bd_;
		}
		set_edit_data(updating_cp_candidates);

		//test
		Polygon_2 polygon_;
		for (int i = 0; i < cps_seqs[0].size() - 1; i++)
		{
			polygon_.push_back(cps_seqs[0][i].point2);
		}
		std::vector<double> bbox = { 100,-100,100,-100 };//-x,x,-y,y
		for (int i = 0; i < cm.vertices.size(); i++)
		{
			bbox[0] = std::min(cm.vertices[i].cp.x, bbox[0]);
			bbox[1] = std::max(cm.vertices[i].cp.x, bbox[1]);
			bbox[2] = std::min(cm.vertices[i].cp.y, bbox[2]);
			bbox[3] = std::max(cm.vertices[i].cp.y, bbox[3]);
		}
		std::vector<Point_2> corners = {
			Point_2(bbox[0],bbox[2]),
			Point_2(bbox[0],bbox[3]),
			Point_2(bbox[1],bbox[3]),
			Point_2(bbox[1],bbox[2]) };
		if (polygon_.is_simple())
		{
			for (int i = 0; i < cm.vertices.size(); i++)
			{
				if (!cm.vertices[i].is_on_feature)
				{
					Point_2 pt(cm.vertices[i].cp.x,
						cm.vertices[i].cp.y);
					if (polygon_.bounded_side(pt) != CGAL::ON_BOUNDED_SIDE)
					{
						cm.vertices[i].bdisplay = false;

						/*cm.vertices[i].cp.is_on_bound = true;
						double min_d = 1e5;
						int id_t = -1;
						for (int t = 0; t < 4; t++)
						{
							Point_2 p1 = corners[t];
							Point_2 p2 = corners[(t + 1) % 4];
							double dis_now = abs(CGAL::area(p1, p2, pt)) / sqrt((p2 - p1).squared_length());
							if (dis_now < min_d)
							{
								min_d = dis_now;
								id_t = t;
							}
						}
						Point_2 p1 = corners[id_t];
						Point_2 p2 = corners[(id_t + 1) % 4];
						Vector_2 proj = ((pt - p1) * (p2 - p1)) / (p2 - p1).squared_length() * (p2 - p1) + (p1 - Point_2(0, 0));
						cm.vertices[i].cp.x = proj.x();
						cm.vertices[i].cp.y = proj.y();*/
					}
				}
			}
		}
#endif // 0


		return success_;
	}

	bool VecEditing::load_authoring_data(QString filename)
	{
		operation_mode = 0;

		CPs_new.clear();
		domain_mesh.clear();

		infilename = filename;

		std::set<Vpair> edge_constraints;
		map<int, CDT::Vertex_handle> cdt_vertex_handle;

		QFile file(filename);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			QTextStream in(&file);
			QString line = in.readLine();
			if (line.isNull())
				return false;

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
					vh = domain_mesh.insert(Point_2(cp.x,cp.y));
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

		for (auto it = edge_constraints.begin(); it != edge_constraints.end(); it++)
		{
			domain_mesh.insert_constraint(cdt_vertex_handle.at(it->first), cdt_vertex_handle.at(it->second));
		}

		update_authoring_data();

		return true;
	}

	bool VecEditing::convert2concise(QString filename)
	{
		operation_mode = 0;
		infilename = filename;

		QPoint size_;
		Image_Approximation appimage;
		bool success_ = appimage.read_editdata(filename, fm, cps_seqs, cm, size_);

		double error_ = 0.002;
		for (int i = 0; i < cps_seqs.size(); i++)
		{
			if (cps_seqs[i].empty())
				continue;
			cps_seqs[i].pop_back();
			vector<Point2withIndex> Points = cps_seqs[i];		
			std::pair<int, double> maxDistance = findMaximumDistance(Points);
			int index = maxDistance.first;
			vector<Point2withIndex>::iterator it = Points.begin();
			vector<Point2withIndex> path1t(Points.begin(), it + index + 1);
			vector<Point2withIndex> path2t(it + index, Points.end());
			path2t.push_back(Points[0]);
			path1t = simplifyWithRDP(path1t, error_);
			path2t = simplifyWithRDP(path2t, error_);
			//simplify path1 and path2 together
			Points.clear();
			Points.insert(Points.begin(), path1t.begin(), path1t.end());
			Points.pop_back();
			Points.insert(Points.end(), path2t.begin(), path2t.end());
			Points.pop_back();
			cps_seqs[i] = Points;
		}

		//remedy auxiliary cps		
		for (int i = 0; i < cps_seqs.size(); i++)
		{
			vector<AuthoringP2> cp_new;
			for (int j = 0; j < cps_seqs[i].size(); j++)
			{
				AuthoringP2 pnow;
				pnow.point2 = cps_seqs[i][j].point2;				
				pnow.is_cycle = true;

				bool is_pair = false;
				Vector_2 vecV;
				Point_2 pfir, psed;
				Vector_2 vecV1, vecV2;
				bool bcycle = true;
				if (j == 0 && !bcycle)
				{
					pfir = cps_seqs[i][j].point2; psed = cps_seqs[i][j + 1].point2;
					vecV = cps_seqs[i][j + 1].point2 - cps_seqs[i][j].point2;
					vecV = Vector_2(-vecV.y(), vecV.x()) / sqrt(vecV.squared_length());
				}
				else if (j == cps_seqs[i].size() - 1 && !bcycle)
				{
					pfir = cps_seqs[i][j - 1].point2; psed = cps_seqs[i][j].point2;
					vecV = cps_seqs[i][j].point2 - cps_seqs[i][j - 1].point2;
					vecV = Vector_2(-vecV.y(), vecV.x()) / sqrt(vecV.squared_length());
				}
				else
				{		
					is_pair = true;
					Point_2 ppre, pcen, pnext;
					ppre = cps_seqs[i][(j - 1 + cps_seqs[i].size()) % cps_seqs[i].size()].point2;
					pcen = cps_seqs[i][j].point2;
					pnext = cps_seqs[i][(j + 1) % cps_seqs[i].size()].point2;

					pfir =pcen; psed = pnext;

					vecV1 = pcen - ppre;
					vecV1 = Vector_2(-vecV1.y(), vecV1.x()) / sqrt(vecV1.squared_length());
					vecV2 = pnext - pcen;
					vecV2 = Vector_2(-vecV2.y(), vecV2.x()) / sqrt(vecV2.squared_length());

					vecV = vecV1 + vecV2;
				}

				bool bCCW = cm.vertices[cps_seqs[i][j].index].bCCWorientation;
				RGBPoint pcenter = cm.vertices[cps_seqs[i][j].index].cp_pair.first;
				RGBPoint ptrans1 = cm.vertices[cps_seqs[i][j].index].cp_pair.second;
				RGBPoint ptrans2 = cm.vertices[cps_seqs[i][j].spare_index].cp_pair.second;
				double dis = sqrt(pow(ptrans1.x - pcenter.x, 2) + pow(ptrans1.y - pcenter.y, 2));
				ptrans1.x = pcenter.x - vecV.x() * dis;
				ptrans1.y = pcenter.y - vecV.y() * dis;
				ptrans2.x = pcenter.x + vecV.x() * dis;
				ptrans2.y = pcenter.y + vecV.y() * dis;
				
				Point_2 test_p(pcenter.x - vecV.x(), pcenter.y - vecV.y());
				if (CGAL::area(test_p, pfir, psed) > 0 && bCCW ||
					CGAL::area(test_p, pfir, psed) < 0 && !bCCW)
				{
					cm.vertices[cps_seqs[i][j].index].cp_pair.second = ptrans1;
					cm.vertices[cps_seqs[i][j].spare_index].cp_pair.second = ptrans2;
					pnow.v_positive = cm.vertices[cps_seqs[i][j].spare_index].cp_pair.first;					
					pnow.v_negative = cm.vertices[cps_seqs[i][j].index].cp_pair.first;
				}
				else
				{
					cm.vertices[cps_seqs[i][j].index].cp_pair.second = ptrans2;
					cm.vertices[cps_seqs[i][j].spare_index].cp_pair.second = ptrans1;
					pnow.v_positive = cm.vertices[cps_seqs[i][j].index].cp_pair.first;
					pnow.v_negative = cm.vertices[cps_seqs[i][j].spare_index].cp_pair.first;
				}

				pnow.v_positive.x = ptrans2.x;
				pnow.v_positive.y = ptrans2.y;
				pnow.v_negative.x = ptrans1.x;
				pnow.v_negative.y = ptrans1.y;

				cp_new.push_back(pnow);
			}
			CPs_new.push_back(cp_new);
		}

		std::vector<double> bbox = { 1e10,-1e10,1e10,-1e10 };//-x,x,-y,y
		for (int i = 0; i < cm.vertices.size(); i++)
		{
			bbox[0] = std::min(bbox[0], cm.vertices[i].cp.x);
			bbox[1] = std::max(bbox[1], cm.vertices[i].cp.x);
			bbox[2] = std::min(bbox[2], cm.vertices[i].cp.y);
			bbox[3] = std::max(bbox[3], cm.vertices[i].cp.y);
		}
		vector<Point_2> image_bound = {
		Point_2(bbox[0], bbox[2]) ,
		Point_2(bbox[0],bbox[2]+(bbox[3]-bbox[2])*1.0/3.0),
		Point_2(bbox[0],bbox[2] + (bbox[3] - bbox[2]) * 2.0 / 3.0),
		Point_2(bbox[0], bbox[3]),
		Point_2(bbox[0] + (bbox[1] - bbox[0]) * 1.0 / 3.0, bbox[3]),
		Point_2(bbox[0] + (bbox[1] - bbox[0]) * 2.0 / 3.0, bbox[3]),
		Point_2(bbox[1], bbox[3]),
		Point_2(bbox[1], bbox[2] + (bbox[3] - bbox[2]) * 2.0 / 3.0),
		Point_2(bbox[1], bbox[2] + (bbox[3] - bbox[2]) * 1.0 / 3.0),
		Point_2(bbox[1], bbox[2]) ,
		Point_2(bbox[0] + (bbox[1] - bbox[0]) * 2.0 / 3.0, bbox[2]),
		Point_2(bbox[0] + (bbox[1] - bbox[0]) * 1.0 / 3.0, bbox[2]) };

		map<int, CDT::Vertex_handle> cdt_vertex_handle;
		int id_ = 0;
		for (int i = 0; i < image_bound.size(); i++)
		{
			CDT::Vertex_handle vh;
			vh = domain_mesh.insert(image_bound[i]);
			vh->set_associated_index(id_);

			RGBPoint p; p.x = image_bound[i].x();
			p.y = image_bound[i].y();
			p.r = 0.78; p.g = 0.89; p.b = 0.93;
			p.is_on_bound = true;
			p.is_on_feature = false;
			vh->set_cp(p);
			cdt_vertex_handle[id_] = vh;
			id_++;
		}
		for (int i = 0; i < image_bound.size(); i++)
		{
			domain_mesh.insert_constraint(cdt_vertex_handle.at(i),
				cdt_vertex_handle.at((i + 1) % image_bound.size()));
		}

		for (int i = 0; i < CPs_new.size(); i++)
		{
			for (int j = 0; j < CPs_new[i].size(); j++)
			{
				CDT::Vertex_handle vh;
				vh = domain_mesh.insert(CPs_new[i][j].point2);
				vh->set_associated_index(id_);
				
				RGBPoint p; p.x = CPs_new[i][j].point2.x();
				p.y = CPs_new[i][j].point2.y();
				p.r = 0.78; p.g = 0.89; p.b = 0.93;
				p.is_on_bound = false;
				p.is_on_feature = true;
				vh->set_cp(p);

				vh->set_cseq_index(Vpair(i,j));
				cdt_vertex_handle[id_] = vh;
				CPs_new[i][j].id = id_;
				id_++;
			}
		}

		for (int i = 0; i < CPs_new.size(); i++)
		{
			for (int j = 0; j < CPs_new[i].size(); j++)
			{
				int edge_f = CPs_new[i][j].id;
				int edge_t = CPs_new[i][(j + 1)% CPs_new[i].size()].id;
				domain_mesh.insert_constraint(cdt_vertex_handle.at(edge_f), cdt_vertex_handle.at(edge_t));
			}
		}

		update_authoring_data();

		return 1;
	}

	bool VecEditing::save_authoring_data(QString* filename)
	{
		QString outfilename;
		if (filename == NULL)
		{
			outfilename = infilename;
		}
		else
		{
			outfilename = *filename;
		}

		QFile file(outfilename);
		if (file.open(QIODevice::WriteOnly))
		{
			QTextStream out(&file);
			out.setRealNumberPrecision(10);

			//control points sequences
			out << "CPSequence\n";
			for (int i = 0; i < CPs_new.size(); i++)
			{
				out << "cps ";
				for (int j = 0; j < CPs_new[i].size(); j++)
				{
					out << CPs_new[i][j].point2.x()
						<< " " << CPs_new[i][j].point2.y()
						<< " " << CPs_new[i][j].is_cycle
						<< " " << CPs_new[i][j].id
						<< " " << CPs_new[i][j].v_positive.x
						<< " " << CPs_new[i][j].v_positive.y
						<< " " << CPs_new[i][j].v_positive.r
						<< " " << CPs_new[i][j].v_positive.g
						<< " " << CPs_new[i][j].v_positive.b
						<< " " << CPs_new[i][j].v_negative.x
						<< " " << CPs_new[i][j].v_negative.y
						<< " " << CPs_new[i][j].v_negative.r
						<< " " << CPs_new[i][j].v_negative.g
						<< " " << CPs_new[i][j].v_negative.b
						<< " ";
				}
				out << "\n";
			}
			out << "End\n";

			//domain mesh
			out << "DomainMesh\n";
			for (auto vit = domain_mesh.vertices_begin(); vit != domain_mesh.vertices_end(); vit++)
			{
				out << "v "
					<< vit->get_associated_index()<<" "
					<< vit->get_cseq_index().first << " "
					<< vit->get_cseq_index().second << " "
					<< vit->get_cp().x << " "
					<< vit->get_cp().y << " "
					<< vit->get_cp().r << " "
					<< vit->get_cp().g << " "
					<< vit->get_cp().b << " "
					<< vit->get_cp().is_on_bound << " "
					<< vit->get_cp().is_on_feature
					<< "\n";							
			}
			for (auto fit = domain_mesh.finite_edges_begin(); fit != domain_mesh.finite_edges_end(); fit++)
			{
				CT::Face_handle f = fit->first;
				int index = fit->second;

				int v0 = f->vertex(f->ccw(index))->get_associated_index();
				int v1 = f->vertex(f->cw(index))->get_associated_index();

				if(domain_mesh.is_constrained(*fit))
					out << "e " << v0<<" "<<v1 << "\n";
			}
			out << "End\n";
			file.close();

			return 1;
		}

		return 0;
	}

	void VecEditing::update_authoring_data()
	{
		split_mesh_with_feature(domain_mesh, authorcm, CPs_new);

		subdivision(4);
		//SUB_SIMPLIFY::saveObjFile("test.obj", mesh_sub);

		if (render_vecimage != NULL)
		{
			delete render_vecimage;			
		}	
		render_vecimage = new RGBPoint[mesh_sub.faces.size() * 3];
		int it = 0;
		for (auto fit = mesh_sub.faces.begin(); fit != mesh_sub.faces.end(); fit++)
		{
			for (int k = 0; k < (*fit)->verts.size(); k++)
			{
				RGBPoint p;
				p.x = (*fit)->verts[k]->loc[0];
				p.y = (*fit)->verts[k]->loc[1];
				p.r = (*fit)->verts[k]->color[0];
				p.g = (*fit)->verts[k]->color[1];
				p.b = (*fit)->verts[k]->color[2];
				render_vecimage[it] = (p);
				it++;
			}
		}
	}

	void VecEditing::split_mesh_with_feature(CDT &domain_, AuthoringCM &cm_, ACPs &cps_new)
	{
		cm_.vertices.clear();
		cm_.faces.clear();

		fea_v_pair.clear();
		fea_e_pair.clear();
		mesh_sub.vertices.clear();
		mesh_sub.faces.clear();

		std::map<int, std::vector<int>> id_trans;
		int id_ = 0, idnew = 1;
		for (auto it = domain_.vertices_begin(); it != domain_.vertices_end(); it++, id_++)
		{
			if (idnew == 1001)
				int sds =  1;
			if (it->get_cp().is_on_feature)
			{
				Vpair cpsid = it->get_cseq_index();
				bool is_cfeature = false;
				if (!cps_new[cpsid.first][cpsid.second].is_cycle)
				{
					if (cpsid.second == 0 || cpsid.second == cps_new[cpsid.first].size() - 1)
						is_cfeature = true;
				}

				if (is_cfeature)
				{
					std::vector<int> ids = { idnew,idnew };
					id_trans[id_] = ids;

					double color_[3];
					color_[0] = (cps_new[cpsid.first][cpsid.second].v_positive.r + cps_new[cpsid.first][cpsid.second].v_negative.r) / 2;
					color_[1] = (cps_new[cpsid.first][cpsid.second].v_positive.g + cps_new[cpsid.first][cpsid.second].v_negative.g) / 2;
					color_[2] = (cps_new[cpsid.first][cpsid.second].v_positive.b + cps_new[cpsid.first][cpsid.second].v_negative.b) / 2;
					cps_new[cpsid.first][cpsid.second].v_positive.r = color_[0];
					cps_new[cpsid.first][cpsid.second].v_positive.g = color_[1];
					cps_new[cpsid.first][cpsid.second].v_positive.b = color_[2];

					AuthoringV v;
					v.is_on_feature = true;
					v.cp = cps_new[cpsid.first][cpsid.second].v_positive;
					v.cp.x = cps_new[cpsid.first][cpsid.second].point2.x();
					v.cp.y = cps_new[cpsid.first][cpsid.second].point2.y();
					v.pair = cps_new[cpsid.first][cpsid.second].v_positive;
					v.cpseq_id = cpsid;
					cm_.vertices.push_back(v);

					//for subdivision
					if (1) {
						SUB_SIMPLIFY::vertex* new_v = new SUB_SIMPLIFY::vertex;
						RGBPoint pc = v.cp;
						new_v->id = idnew - 1;
						new_v->loc = Eigen::Vector3f(pc.x, pc.y, 0);
						new_v->type.feature_type = cpsid.first;
						new_v->type.is_fea_corner = is_cfeature;
						new_v->type.bound_type = -1;
						new_v->type.is_corner = 0;
						new_v->color = Eigen::Vector3f(pc.r, pc.g, pc.b);
						mesh_sub.vertices.push_back(new_v);

						fea_v_pair[idnew-1] = idnew-1;
					}
				}
				else
				{
					std::vector<int> ids = { idnew,idnew + 1 };
					id_trans[id_] = ids;

					AuthoringV v;
					v.is_on_feature = true;
					v.cp = cps_new[cpsid.first][cpsid.second].v_positive;
					v.cp.x = cps_new[cpsid.first][cpsid.second].point2.x();
					v.cp.y = cps_new[cpsid.first][cpsid.second].point2.y();
					v.pair = cps_new[cpsid.first][cpsid.second].v_positive;
					v.cpseq_id = cpsid;
					cm_.vertices.push_back(v);

					//for subdivision
					if (1) {
						SUB_SIMPLIFY::vertex* new_v = new SUB_SIMPLIFY::vertex;
						RGBPoint pc = v.cp;
						new_v->id = idnew - 1;
						new_v->loc = Eigen::Vector3f(pc.x, pc.y, 0);
						new_v->type.feature_type = cpsid.first;
						new_v->type.is_fea_corner = is_cfeature;
						new_v->type.bound_type = -1;
						new_v->type.is_corner = 0;
						new_v->color = Eigen::Vector3f(pc.r, pc.g, pc.b);
						mesh_sub.vertices.push_back(new_v);

						fea_v_pair[idnew - 1] = idnew;
						fea_v_pair[idnew] = idnew - 1;
					}
					AuthoringV v1;
					v1.is_on_feature = true;
					v1.cp = cps_new[cpsid.first][cpsid.second].v_negative;
					v1.cp.x = cps_new[cpsid.first][cpsid.second].point2.x();
					v1.cp.y = cps_new[cpsid.first][cpsid.second].point2.y();
					v1.pair = cps_new[cpsid.first][cpsid.second].v_negative;
					v1.cpseq_id = cpsid;
					cm_.vertices.push_back(v1);

					//for subdivision
					if (1) {
						SUB_SIMPLIFY::vertex* new_v = new SUB_SIMPLIFY::vertex;
						RGBPoint pc = v1.cp;
						new_v->id = idnew;
						new_v->loc = Eigen::Vector3f(pc.x, pc.y, 0);
						new_v->type.feature_type = cpsid.first;
						new_v->type.is_fea_corner = is_cfeature;
						new_v->type.bound_type = -1;
						new_v->type.is_corner = 0;
						new_v->color = Eigen::Vector3f(pc.r, pc.g, pc.b);
						mesh_sub.vertices.push_back(new_v);
					}
					idnew++;					
				}
			}
			else
			{
				AuthoringV v;
				v.is_bound = it->get_cp().is_on_bound;
				RGBPoint p = it->get_cp();
				v.cp = p;
				cm_.vertices.push_back(v);
				std::vector<int> ids = { idnew};
				id_trans[id_] = ids;

				//for subdivision
				if (1) {
					SUB_SIMPLIFY::vertex* new_v = new SUB_SIMPLIFY::vertex;
					RGBPoint pc = v.cp;
					new_v->id = idnew - 1;
					new_v->loc = Eigen::Vector3f(pc.x, pc.y, 0);
					new_v->type.feature_type = -1;
					new_v->type.is_fea_corner = 0;
					new_v->type.bound_type = it->get_cp().is_on_bound ? (id_) / 3 : -1;
					new_v->type.is_corner = it->get_cp().is_on_bound && id_ % 3 == 0;
					new_v->color = Eigen::Vector3f(pc.r, pc.g, pc.b);
					mesh_sub.vertices.push_back(new_v);
				}
			}
			idnew++;
		}

		std::map<int,int> positive_edges;
		for (int i = 0; i < cps_new.size(); i++)
		{	
			for (int j = 0; j < cps_new[i].size(); j++)
			{
				if (!cps_new[i].begin()->is_cycle && j == cps_new[i].size()-1)
				{
					continue;
				}
				positive_edges[cps_new[i][j].id] = cps_new[i][(j + 1) % cps_new[i].size()].id;
			}
		}

		std::map<Vpair, std::set<Vpair>> edge_neigh;
		for (auto f = domain_.faces_begin(); f != domain_.faces_end(); f++)
		{
			std::vector<int> facet;
			int v0 = f->vertex(0)->get_associated_index();
			int v1 = f->vertex(1)->get_associated_index();
			int v2 = f->vertex(2)->get_associated_index();
			Vpair e0 = make_epair(v0, v1);
			Vpair e1 = make_epair(v1, v2);
			Vpair e2 = make_epair(v0, v2);
			edge_neigh[e0].insert(e1);
			edge_neigh[e0].insert(e2);
			edge_neigh[e1].insert(e0);
			edge_neigh[e1].insert(e2);
			edge_neigh[e2].insert(e0);
			edge_neigh[e2].insert(e1);
		}

		std::map<Vpair, Vpair> edge_split;
		//for edges on feature
		for (auto f = domain_.faces_begin(); f != domain_.faces_end(); f++)
		{			
			std::vector<int> facet;		
			std::vector<bool> bfeature;
			for (int t = 0; t < 3; t++)
			{
				int v = f->vertex(t)->get_associated_index();
				facet.push_back(v);
				bfeature.push_back(f->vertex(t)->get_cp().is_on_feature);					
			}
			
			for (int t = 0; t < 3; t++)
			{
				if (!bfeature[t] && !bfeature[(t + 1) % 3])
				{
					assert(id_trans.at(facet[t]).size() == 1 && id_trans.at(facet[(t + 1) % 3]).size() == 1);
					int vf = id_trans.at(facet[t])[0];
					int vt = id_trans.at(facet[(t + 1) % 3])[0];
					std::pair< Vpair, Vpair> epair = make_dpair(facet[t], vf, facet[(t + 1) % 3], vt);
					if(edge_split.find(epair.first) == edge_split.end())
						edge_split[epair.first] = epair.second;
				}		
				else
				{
					std::pair< Vpair, Vpair> epair = make_dpair(facet[t], -1, facet[(t + 1) % 3], -1);
					if (edge_split.find(epair.first) == edge_split.end())
						edge_split[epair.first] = epair.second;
				}
			}

			if (bfeature[0] + bfeature[1] + bfeature[2] < 2)
				continue;

			std::vector<int> type_e(3, -1);//edge-0,1; 1,2; 2,0;
			for (int t = 0; t < 3; t++)
			{
				if (positive_edges.find(facet[t]) != positive_edges.end())
				{
					if (positive_edges.at(facet[t]) == facet[(t+1)%3])
					{
						type_e[t] = 1;//positive
					}
					else if (positive_edges.at(facet[t]) == facet[(t + 2) % 3])
					{
						type_e[(t+2)%3] = 0;//negative
					}
				}
			}
			std::vector<int> type_v(3,-1);
			for (int t = 0; t < 3; t++)
			{
				if (type_e[t] == 1)
				{
					type_v[t] = 1;
					type_v[(t+1)%3] = 1;
				}
				else if (type_e[t] == 0)
				{
					type_v[t] = 0;
					type_v[(t + 1) % 3] = 0;
				}
			}
			for (int t = 0; t < 3; t++)
			{
				int vf = -1, vt = -1;
				if(!bfeature[t])
					vf = id_trans.at(facet[t])[0];
				if (!bfeature[(t + 1) % 3])
					vt = id_trans.at(facet[(t + 1) % 3])[0];
				if (type_v[t] == 1)
					vf = id_trans.at(facet[t])[0];
				else if (type_v[t] == 0)
					vf = id_trans.at(facet[t])[1];
				if (type_v[(t + 1) % 3] == 1)
					vt = id_trans.at(facet[(t + 1) % 3])[0];
				else if (type_v[(t + 1) % 3] == 0)
					vt = id_trans.at(facet[(t + 1) % 3])[1];

				assert(facet[t] != vf && facet[(t + 1) % 3] != vt);

				if (type_e[t] != -1)
				{
					Vpair ef(facet[t],facet[(t + 1) % 3]);
					Vpair et(vf,vt);			
					assert(-1!= vf && -1 != vt);
					edge_split[ef] = et;
				}
				else
				{
					std::pair< Vpair, Vpair> epair = make_dpair(facet[t], vf, facet[(t + 1) % 3], vt);

					if (edge_split.find(epair.first) != edge_split.end())
					{
						Vpair cur_e = edge_split.at(epair.first);
						if (epair.second.first != -1)
						{
							cur_e.first = epair.second.first;
						}
						if (epair.second.second != -1)
						{
							cur_e.second = epair.second.second;
						}
						edge_split[epair.first] = cur_e;
					}
					else
					{
						edge_split[epair.first] = epair.second;
					}
				}				
			}
		}		
		bool all_processed;
		do
		{
			all_processed = true;
			for (auto e = edge_split.begin(); e != edge_split.end(); e++)
			{
				if (e->second.first == -1 || e->second.second == -1)
				{
					all_processed = false;
					std::set<Vpair> neigh = edge_neigh.at(e->first);

					for (auto in = neigh.begin(); in != neigh.end(); in++)
					{
						if (e->second.first == -1)
						{
							if (in->first == e->first.first)
							{
								if (edge_split.at(*in).first != -1)
								{
									e->second.first = edge_split.at(*in).first;
								}
							}
							else if(in->second == e->first.first)
							{
								if (edge_split.at(*in).second != -1)
								{
									e->second.first = edge_split.at(*in).second;
								}
							}
						}
						if (e->second.second == -1)
						{
							if (in->first == e->first.second)
							{
								if (edge_split.at(*in).first != -1)
								{
									e->second.second = edge_split.at(*in).first;
								}
							}
							else if (in->second == e->first.second)
							{
								if (edge_split.at(*in).second != -1)
								{
									e->second.second = edge_split.at(*in).second;
								}
							}
						}
					}
				}
			}
			if (all_processed)
				break;
		} while (1);
		std::set<Vpair> feature_edges;
		for (auto f = domain_.faces_begin(); f != domain_.faces_end(); f++)
		{
			std::vector<int> facet;
			for (int t = 0; t < 3; t++)
			{
				int v = f->vertex(t)->get_associated_index();
				facet.push_back(v);
			}
			std::vector<int> type_e(3, -1);//edge-0,1; 1,2; 2,0;			
			for (int t = 0; t < 3; t++)
			{
				if (positive_edges.find(facet[t]) != positive_edges.end())
				{
					if (positive_edges.at(facet[t]) == facet[(t + 1) % 3])
					{
						type_e[t] = 1;//positive
					}
					else if (positive_edges.at(facet[t]) == facet[(t + 2) % 3])
					{
						type_e[(t + 2) % 3] = 0;//negative
					}
				}
			}
			std::vector<Vpair> tri_e;
			for (int t = 0; t < 3; t++)
			{				
				Vpair e,ebackup(-1,-1);
				if (type_e[t] == -1)
				{
					e = make_epair(facet[t], facet[(t + 1) % 3]);
				}
				else
				{
					e = Vpair(facet[t], facet[(t + 1) % 3]);
					ebackup = Vpair(facet[(t + 1) % 3], facet[t]);
				}
				Vpair e_trans = edge_split.at(e);

				//for subdivision
				if(ebackup.first != -1 && ebackup.second != -1)
				{
					Vpair e_trans_backup = edge_split.at(ebackup);
					Vpair e1new = make_epair(e_trans.second - 1, e_trans.first - 1);
					Vpair e2new = make_epair(e_trans_backup.second - 1, e_trans_backup.first - 1);
					fea_e_pair[e1new] = e2new;
					fea_e_pair[e2new] = e1new;
				}				

				Vpair e_new;
				if (e.first == facet[t])
					e_new = e_trans;
				else
					e_new = Vpair(e_trans.second,e_trans.first);
				if (type_e[t] != -1)
				{
					feature_edges.insert(make_epair(e_trans.second-1, e_trans.first-1));
				}
				tri_e.push_back(e_new);
			}
			CMFaceInfo fn;
			for (int t = 0; t < 3; t++)
			{
				assert(tri_e[t].second == tri_e[(t+1)%3].first);
				fn.face.push_back(tri_e[t].second - 1);
			}	
			cm_.faces.push_back(fn);

			//for subdivision
			if (1) {
				SUB_SIMPLIFY::face* f1 = new SUB_SIMPLIFY::face;
				f1->verts = { mesh_sub.vertices[fn.face[0]],
					mesh_sub.vertices[fn.face[1]],
					mesh_sub.vertices[fn.face[2]] };
				mesh_sub.faces.push_back(f1);
			}
		}
		cm_.edges = feature_edges;
	}

	void VecEditing::subdivision(int times)
	{
		bool subdivided_color = true;
		for (int i = 0; i < times; i++)
		{
			SUB_SIMPLIFY::MeshSimplify mesh;
			SUBDIVISION::LOOP Cloop(&mesh_sub, &mesh);

			Cloop.set_blimit(false);
			Cloop.set_subdivided_color(subdivided_color);
			Cloop.set_fea_e_pair(fea_e_pair);
			Cloop.set_fea_v_pair(&fea_v_pair);

			Cloop.subdivide();
			Cloop.get_fea_e_pair(fea_e_pair);

			mesh_sub = mesh;
		}
	}


	bool VecEditing::save_editing_data(QString *filename)
	{
		QString outfilename;
		if (filename == NULL)
		{
			outfilename = infilename;
		}
		else
		{
			outfilename = *filename;
		}

		QFile file(outfilename);
		if (file.open(QIODevice::WriteOnly))
		{
			QTextStream out(&file);
			out.setRealNumberPrecision(10);
			
			out << "InputImage" << " " << image_wid_ << " " << image_hei_ << "\nEnd\n";

			//control mesh
			out << "ControlMesh\n";
			for (unsigned i = 0; i < cm.vertices.size(); i++)
			{
				out << "vertex " << cm.vertices[i].cp.x << " " << cm.vertices[i].cp.y << " "
					<< cm.vertices[i].cp.r << " " << cm.vertices[i].cp.g << " "
					<< cm.vertices[i].cp.b << "\n";
				out << "cppair " << cm.vertices[i].cp_pair.first.x << " "
					<< cm.vertices[i].cp_pair.first.y << " "
					<< cm.vertices[i].cp_pair.first.r << " "
					<< cm.vertices[i].cp_pair.first.g << " "
					<< cm.vertices[i].cp_pair.first.b << " ";
				out << cm.vertices[i].cp_pair.second.x << " "
					<< cm.vertices[i].cp_pair.second.y << " "
					<< cm.vertices[i].cp_pair.second.r << " "
					<< cm.vertices[i].cp_pair.second.g << " "
					<< cm.vertices[i].cp_pair.second.b << "\n";
				out << "support ";
				for (auto jt = cm.vertices[i].supports.begin();
					jt != cm.vertices[i].supports.end(); jt++)
				{
					out << jt->first << " " << jt->second << " ";
				}
				out << "\n";
				out << "other " << cm.vertices[i].bCCWorientation << " " <<
					cm.vertices[i].is_on_feature<<" "<<cm.vertices[i].is_bound <<" "
					<< cm.vertices[i].domain_midEdge.x() << " " << cm.vertices[i].domain_midEdge.y() << "\n";
			}
			for (auto eit = cm.edges.begin(); eit != cm.edges.end(); eit++)
				out << "edge " << (eit->first()) << " " << (eit->second()) << "\n";
			out << "End\n";

			//control points sequences
			out << "CPSequence\n";
			for (int i = 0; i < cps_seqs.size(); i++)
			{
				out << "cps ";
				for (int j = 0; j < cps_seqs[i].size(); j++)
				{
					out << cm.vertices[cps_seqs[i][j].index].cp.x
						<< " " << cm.vertices[cps_seqs[i][j].index].cp.y
						<<" "<< cps_seqs[i][j].index 
						<< " " << cps_seqs[i][j].spare_index << " ";
				}
				out << "\n";
			}
			out << "End\n";

			//fitted mesh
			out << "FittedMesh\n";
			for (auto vit = fm.vertices.begin(); vit != fm.vertices.end(); vit++)
			{
				out << "v " << vit->x << " " << vit->y << " "
					<< vit->r << " " << vit->g << " "
					<< vit->b << " " << vit->is_on_feature << "\n";
			}
			for (auto fit = fm.faces.begin(); fit != fm.faces.end(); fit++)
			{
				out << "f ";
				for (int k = 0; k < fit->face.size(); k++)
				{
					out << fit->face[k] << " ";
				}
				out << "\n";
			}
			out << "End\n";

			file.close();

			return 1;
		}

		return 0;
	}

	bool VecEditing::generate_output_directory(QString allDir)
	{
		QStringList dirList = allDir.split("/");
		QString dir = ".";

		for (unsigned i = 0; i < dirList.size(); i++)
		{
			bool ret = false;

			QDir outputDir(dir);
			dir.append("/");
			dir.append(dirList.at(i));

			QString dirName = dirList.at(i);

			if (outputDir.exists(dirName))
				ret = true;
			else
				ret = outputDir.mkdir(dirName);

			if (!ret)
				return false;
		}

		return true;
	}

	void VecEditing::update_edited_vecimage(const set<int> &edit_data) {

		//has some problems for color sum, need add sum mode!
//#pragma omp parallel for
		for (auto it = edit_data.begin();it!= edit_data.end();it++)
		{
			int id_ = *it;
			for (auto ik = cm.vertices[id_].supports.begin();ik!= cm.vertices[id_].supports.end();ik++)
			{
				fm.vertices[ik->first].x += cm.vertices[id_].deviation.x*ik->second;
				fm.vertices[ik->first].y += cm.vertices[id_].deviation.y*ik->second;
				fm.vertices[ik->first].r += cm.vertices[id_].deviation.r*ik->second;
				fm.vertices[ik->first].g += cm.vertices[id_].deviation.g*ik->second;
				fm.vertices[ik->first].b += cm.vertices[id_].deviation.b*ik->second;
			}
		}
		int it = 0;
		for (auto fit = fm.faces.begin(); fit != fm.faces.end(); fit++)
		{
			for (int k = 0; k < fit->face.size(); k++)
			{
				render_vecimage[it] = fm.vertices[fit->face[k]];
				it++;
			}
		}
	}

	void VecEditing::set_edit_data(set<int>& edit_data_in)
	{
		edit_data = edit_data_in;
		edit_data_in.clear();
	}

	void VecEditing::interpolated_vecimage(int internal_) {
		for (auto it = edit_data.begin(); it != edit_data.end(); it++)
		{
			int id_ = *it;
			cm.vertices[id_].cp.x += cm.vertices[id_].deviation.x / internal_;
			cm.vertices[id_].cp.y += cm.vertices[id_].deviation.y / internal_;
			cm.vertices[id_].cp.r += cm.vertices[id_].deviation.r / internal_;
			cm.vertices[id_].cp.g += cm.vertices[id_].deviation.g / internal_;
			cm.vertices[id_].cp.b += cm.vertices[id_].deviation.b / internal_;
			cm.vertices[id_].cp_pair.first.x += cm.vertices[id_].deviation.x / internal_;
			cm.vertices[id_].cp_pair.first.y += cm.vertices[id_].deviation.y / internal_;
			cm.vertices[id_].cp_pair.first.r += cm.vertices[id_].deviation.r / internal_;
			cm.vertices[id_].cp_pair.first.g += cm.vertices[id_].deviation.g / internal_;
			cm.vertices[id_].cp_pair.first.b += cm.vertices[id_].deviation.b / internal_;
			cm.vertices[id_].cp_pair.second.x += cm.vertices[id_].deviation.x / internal_;
			cm.vertices[id_].cp_pair.second.y += cm.vertices[id_].deviation.y / internal_;
			cm.vertices[id_].cp_pair.second.r += cm.vertices[id_].deviation.r / internal_;
			cm.vertices[id_].cp_pair.second.g += cm.vertices[id_].deviation.g / internal_;
			cm.vertices[id_].cp_pair.second.b += cm.vertices[id_].deviation.b / internal_;
		}
		for (auto it = edit_data.begin(); it != edit_data.end(); it++)
		{
			int id_ = *it;
			for (auto ik = cm.vertices[id_].supports.begin(); ik != cm.vertices[id_].supports.end(); ik++)
			{
				fm.vertices[ik->first].x += cm.vertices[id_].deviation.x / internal_ * ik->second;
				fm.vertices[ik->first].y += cm.vertices[id_].deviation.y / internal_ * ik->second;
				fm.vertices[ik->first].r += cm.vertices[id_].deviation.r / internal_ * ik->second;
				fm.vertices[ik->first].g += cm.vertices[id_].deviation.g / internal_ * ik->second;
				fm.vertices[ik->first].b += cm.vertices[id_].deviation.b / internal_ * ik->second;
			}
		}
		int it = 0;
		for (auto fit = fm.faces.begin(); fit != fm.faces.end(); fit++)
		{
			for (int k = 0; k < fit->face.size(); k++)
			{
				render_vecimage[it] = fm.vertices[fit->face[k]];
				it++;
			}
		}
	}

	//model view
	void  VecEditing::GetCameraPosAndOrientation(vector<vector<double>> &pos_orient)
	{
		if (model_position_orientation.size() != 0)
		{
			pos_orient = model_position_orientation;
		}
		//else
		//{
		//	//std::cout << "read from txt" << std::endl;
		//	pos_orient.resize(2);
		//	pos_orient[0].resize(3);
		//	pos_orient[1].reserve(4);
		//	std::ifstream fin;
		//	QString orientation_f = out_file_name;
		//	orientation_f.append("/camera_pos_orientation.txt");
		//	fin.open(orientation_f.toStdString());
		//	if (fin.is_open())
		//	{
		//		for (int i = 0; i < 3; i++)
		//		{
		//			fin >> pos_orient[0][i];
		//		}
		//		for (int i = 0; i < 4; i++)
		//		{
		//			fin >> pos_orient[1][i];
		//		}
		//	}
		//	fin.close();
		//}
	}

	void  VecEditing::SetCameraPosAndOrientation(vector<vector<double>> pos_orient)
	{
		model_position_orientation = pos_orient;
	}

	bool & VecEditing::is_fixed_model_view()
	{
		return is_fixed_modelview;
	}


}