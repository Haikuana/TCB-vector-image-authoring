#include "draw_aux.h"

namespace EDITGUI {

	using namespace std;
	using namespace qglviewer;

	void draw_vecImage(RGBPoint* veci, int size_)
	{
		if (veci == NULL)
			return;

		glBegin(GL_TRIANGLES);
		for (int it = 0; it < size_; it += 3)
		{
			RGBPoint v1 = veci[it], v2 = veci[it + 1], v3 = veci[it + 2];
#if 1
			bool is_on_feature = false;
			if (!v1.is_on_feature && (v2.is_on_feature || v3.is_on_feature))
			{
				is_on_feature = true;
				glColor3d(v1.r, v1.g, v1.b);
			}
			else if (!v2.is_on_feature && (v1.is_on_feature || v3.is_on_feature))
			{
				is_on_feature = true;
				glColor3d(v2.r, v2.g, v2.b);
			}
			else if (!v3.is_on_feature && (v1.is_on_feature || v2.is_on_feature))
			{
				is_on_feature = true;
				glColor3d(v3.r, v3.g, v3.b);
			}
			if (is_on_feature)
			{
				glVertex2d(v1.x, v1.y);
				glVertex2d(v2.x, v2.y);
				glVertex2d(v3.x, v3.y);
			}
			else
#endif
			{
				glColor3d(v1.r, v1.g, v1.b);
				glVertex2d(v1.x, v1.y);
				glColor3d(v2.r, v2.g, v2.b);
				glVertex2d(v2.x, v2.y);
				glColor3d(v3.r, v3.g, v3.b);
				glVertex2d(v3.x, v3.y);
			}
		}
		glEnd();
	}

	void draw_control_mesh_points(EditCM *cmInfo, int type)
	{
		//glColor3d(194 / 255.0, 116 / 255.0, 167 / 255.0);
		glPointSize(3.0);
		for (int i = 0; i < cmInfo->vertices.size(); i++)
		{
			if (type == 32)
			{
				if (cmInfo->vertices[i].is_on_feature)
				{
					/////////////////////////////////////////////////one side
					//segment
					glLineWidth(1.0);
					glColor3d(0.5, 0.5, 0.5);
					glBegin(GL_LINES);
					glVertex2d(cmInfo->vertices[i].cp_pair.first.x, cmInfo->vertices[i].cp_pair.first.y);
					glVertex2d(cmInfo->vertices[i].cp_pair.second.x, cmInfo->vertices[i].cp_pair.second.y);
					glEnd();

					//background ring
					glPointSize(6);
					glBegin(GL_POINTS);
					glColor3d(0.0, 0.3, 0.0);
					glVertex2d(cmInfo->vertices[i].cp_pair.second.x, cmInfo->vertices[i].cp_pair.second.y);
					glEnd();

					//auxiliary
					glPointSize(5);
					glBegin(GL_POINTS);
					glColor3d(cmInfo->vertices[i].cp_pair.first.r,
						cmInfo->vertices[i].cp_pair.first.g,
						cmInfo->vertices[i].cp_pair.first.b);
					glVertex2d(cmInfo->vertices[i].cp_pair.second.x, cmInfo->vertices[i].cp_pair.second.y);
					glEnd();
				}
				else
				{
					if (cmInfo->vertices[i].bdisplay) {
						glPointSize(5);
						glBegin(GL_POINTS);
						glColor3d(cmInfo->vertices[i].cp.r, cmInfo->vertices[i].cp.g, cmInfo->vertices[i].cp.b);
						glVertex2d(cmInfo->vertices[i].cp.x, cmInfo->vertices[i].cp.y);
						glEnd();
					}
				}
			}

		}
	}

	void draw_control_mesh_edges(EditCM *cmInfo, int type, Point_2 psize)
	{
		std::set<CMEdgeInfo>::iterator it = cmInfo->edges.begin();
		for (; it != cmInfo->edges.end(); it++)
		{
			glLineWidth(2.5f);
			glColor3d(119 / 255.0, 189 / 255.0, 191 / 255.0);
			if (cmInfo->vertices[it->first()].is_on_feature && cmInfo->vertices[it->second()].is_on_feature)
			{
				/*glLineWidth(3.5f);
				glColor3d(0, 0, 0);*/
				continue;
			}

			if (!cmInfo->vertices[it->first()].bdisplay || !cmInfo->vertices[it->second()].bdisplay)
			{
				continue;
			}
#if 0
			if (!cmInfo->vertices[it->first()].is_on_feature && cmInfo->vertices[it->second()].is_on_feature)
			{
				glColor3d(cmInfo->vertices[it->first()].cp.r,
					cmInfo->vertices[it->first()].cp.g, cmInfo->vertices[it->first()].cp.b);
				glBegin(GL_LINES);
				glVertex2d(cmInfo->vertices[it->first()].domain_midEdge.x()*2.0*psize.x() - psize.x(),
					cmInfo->vertices[it->first()].domain_midEdge.y()*2.0*psize.y() - psize.y());
				glVertex2d(cmInfo->vertices[it->second()].domain_midEdge.x()*2.0*psize.x() - psize.x(),
					cmInfo->vertices[it->second()].domain_midEdge.y()*2.0*psize.y() - psize.y());
				glEnd();
			}
			else if (cmInfo->vertices[it->first()].is_on_feature && !cmInfo->vertices[it->second()].is_on_feature)
			{
				glColor3d(cmInfo->vertices[it->second()].cp.r,
					cmInfo->vertices[it->second()].cp.g, cmInfo->vertices[it->second()].cp.b);
				glBegin(GL_LINES);
				glVertex2d(cmInfo->vertices[it->first()].domain_midEdge.x()*2.0*psize.x() - psize.x(),
					cmInfo->vertices[it->first()].domain_midEdge.y()*2.0*psize.y() - psize.y());
				glVertex2d(cmInfo->vertices[it->second()].domain_midEdge.x()*2.0*psize.x() - psize.x(),
					cmInfo->vertices[it->second()].domain_midEdge.y()*2.0*psize.y() - psize.y());
				glEnd();
			}
			else
			{
				glBegin(GL_LINES);
				glColor3d(cmInfo->vertices[it->first()].cp.r,
					cmInfo->vertices[it->first()].cp.g, cmInfo->vertices[it->first()].cp.b);
				glVertex2d(cmInfo->vertices[it->first()].domain_midEdge.x()*2.0*psize.x() - psize.x(),
					cmInfo->vertices[it->first()].domain_midEdge.y()*2.0*psize.y() - psize.y());
				glColor3d(cmInfo->vertices[it->second()].cp.r,
					cmInfo->vertices[it->second()].cp.g, cmInfo->vertices[it->second()].cp.b);
				glVertex2d(cmInfo->vertices[it->second()].domain_midEdge.x()*2.0*psize.x() - psize.x(),
					cmInfo->vertices[it->second()].domain_midEdge.y()*2.0*psize.y() - psize.y());
				glEnd();
			}

#else
			glBegin(GL_LINES);
			glVertex2d(cmInfo->vertices[it->first()].cp.x, cmInfo->vertices[it->first()].cp.y);
			glVertex2d(cmInfo->vertices[it->second()].cp.x, cmInfo->vertices[it->second()].cp.y);
			glEnd();
#endif
		}
		it = cmInfo->edges.begin();
		for (; it != cmInfo->edges.end(); it++)
		{
			if (cmInfo->vertices[it->first()].is_on_feature && cmInfo->vertices[it->second()].is_on_feature)
			{
				glLineWidth(4.5f);
				glColor3d(0, 0, 0);
#if 0
				glBegin(GL_LINES);
				glVertex2d(cmInfo->vertices[it->first()].domain_midEdge.x()*2.0*psize.x() - psize.x(),
					cmInfo->vertices[it->first()].domain_midEdge.y()*2.0*psize.y() - psize.y());
				glVertex2d(cmInfo->vertices[it->second()].domain_midEdge.x()*2.0*psize.x() - psize.x(),
					cmInfo->vertices[it->second()].domain_midEdge.y()*2.0*psize.y() - psize.y());
				glEnd();
#else
				glBegin(GL_LINES);
				glVertex2d(cmInfo->vertices[it->first()].cp.x, cmInfo->vertices[it->first()].cp.y);
				glVertex2d(cmInfo->vertices[it->second()].cp.x, cmInfo->vertices[it->second()].cp.y);
				glEnd();
#endif
			}
		}
	}


	void draw_control_mesh_points(AuthoringCM* cmInfo, int type)
	{
		//glColor3d(194 / 255.0, 116 / 255.0, 167 / 255.0);
		glPointSize(3.0);
		for (int i = 0; i < cmInfo->vertices.size(); i++)
		{
			if (type == 32)
			{
				if (cmInfo->vertices[i].is_on_feature)
				{
					/////////////////////////////////////////////////one side
					//segment
					glLineWidth(1.0);
					glColor3d(0.5, 0.5, 0.5);
					glBegin(GL_LINES);
					glVertex2d(cmInfo->vertices[i].cp.x, cmInfo->vertices[i].cp.y);
					glVertex2d(cmInfo->vertices[i].pair.x, cmInfo->vertices[i].pair.y);
					glEnd();

					//background ring
					glPointSize(6);
					glBegin(GL_POINTS);
					glColor3d(0.0, 0.3, 0.0);
					glVertex2d(cmInfo->vertices[i].pair.x, cmInfo->vertices[i].pair.y);
					glEnd();

					//auxiliary
					glPointSize(5);
					glBegin(GL_POINTS);
					glColor3d(cmInfo->vertices[i].pair.r,
						cmInfo->vertices[i].pair.g,
						cmInfo->vertices[i].pair.b);
					glVertex2d(cmInfo->vertices[i].pair.x, cmInfo->vertices[i].pair.y);
					glEnd();
				}
				else
				{
					glPointSize(5);
					glBegin(GL_POINTS);
					glColor3d(cmInfo->vertices[i].cp.r, cmInfo->vertices[i].cp.g, cmInfo->vertices[i].cp.b);
					glVertex2d(cmInfo->vertices[i].cp.x, cmInfo->vertices[i].cp.y);
					glEnd();
				}
			}

		}
	}

	void draw_control_mesh_edges(AuthoringCM* cmInfo, int type, Point_2 psize)
	{		
		for (auto it = cmInfo->faces.begin(); it != cmInfo->faces.end(); it++)
		{
			for (int i = 0; i < 3; i++)
			{
				glLineWidth(2.0f);				
				glBegin(GL_LINES);
				glColor3d(cmInfo->vertices[it->face[i]].cp.r, 
					cmInfo->vertices[it->face[i]].cp.g,
					cmInfo->vertices[it->face[i]].cp.b);
				glVertex2d(cmInfo->vertices[it->face[i]].cp.x, cmInfo->vertices[it->face[i]].cp.y);
				glColor3d(cmInfo->vertices[it->face[(i + 1) % 3]].cp.r,
					cmInfo->vertices[it->face[(i + 1) % 3]].cp.g,
					cmInfo->vertices[it->face[(i + 1) % 3]].cp.b);
				glVertex2d(cmInfo->vertices[it->face[(i+1)%3]].cp.x, cmInfo->vertices[it->face[(i + 1) % 3]].cp.y);
				glEnd();
			}
		}
	}

	void draw_control_mesh_edges(CMInfo* cmInfo, AuthoringCM* cm_)
	{
		glColor3d(119 / 255.0, 189 / 255.0, 191 / 255.0);
		for (auto eit = cmInfo->edges.begin(); eit != cmInfo->edges.end(); eit++)
		{
			for (int i = 0; i < 3; i++)
			{
				glLineWidth(2.0f);
				glBegin(GL_LINES);	
				
				bool has_feature1 = false;
				bool has_feature2 = false;
				Point_2 ps1(cmInfo->vertices[eit->first()].vertex.x, cmInfo->vertices[eit->first()].vertex.y);
				Point_2 ps2(cmInfo->vertices[eit->second()].vertex.x, cmInfo->vertices[eit->second()].vertex.y);
				for (auto it = cm_->edges.begin(); it != cm_->edges.end(); it++)
				{
					Point_2 pt1(cm_->vertices[it->first].cp.x, cm_->vertices[it->first].cp.y);
					Point_2 pt2(cm_->vertices[it->second].cp.x, cm_->vertices[it->second].cp.y);

					if (abs(CGAL::area(pt1,pt2,ps1))<1e-8)
						has_feature1 = true;
					if (abs(CGAL::area(pt1, pt2, ps2)) < 1e-8)
						has_feature2 = true;
				}
				if (!has_feature1 && !has_feature2) 
				{
					glColor3d(cmInfo->vertices[eit->first()].vertex.r,
						cmInfo->vertices[eit->first()].vertex.g,
						cmInfo->vertices[eit->first()].vertex.b);
					glVertex2d(ps1.x(),ps1.y());
					glColor3d(cmInfo->vertices[eit->second()].vertex.r,
						cmInfo->vertices[eit->second()].vertex.g,
						cmInfo->vertices[eit->second()].vertex.b);
					glVertex2d(ps2.x(), ps2.y());
				}
				else
				{
					if(has_feature1)
						glColor3d(cmInfo->vertices[eit->second()].vertex.r,
							cmInfo->vertices[eit->second()].vertex.g,
							cmInfo->vertices[eit->second()].vertex.b);
					else
						glColor3d(cmInfo->vertices[eit->first()].vertex.r,
							cmInfo->vertices[eit->first()].vertex.g,
							cmInfo->vertices[eit->first()].vertex.b);
					glVertex2d(ps1.x(), ps1.y());
					glVertex2d(ps2.x(), ps2.y());
				}
				
				glEnd();
			}
		}	
		/*glPointSize(10.0);
		for (auto vit = cmInfo->vertices.begin(); vit != cmInfo->vertices.end(); vit++)
		{
			glBegin(GL_POINTS);
			glColor3d(vit->vertex.r, vit->vertex.g, vit->vertex.b);
			glVertex2d(vit->vertex.x, vit->vertex.y);
			glEnd();
		}*/
	}

	void draw_features(AuthoringCM* cmInfo)
	{
		for (auto it = cmInfo->edges.begin(); it != cmInfo->edges.end(); it++)
		{
			glLineWidth(3.0f);
			//glColor3d(0.2, 0.63, 0.2);
			glColor3d(0,0,0);
			glBegin(GL_LINES);
			glVertex2d(cmInfo->vertices[it->first].cp.x, cmInfo->vertices[it->first].cp.y);
			glVertex2d(cmInfo->vertices[it->second].cp.x, cmInfo->vertices[it->second].cp.y);
			glEnd();
		}
	}
}
