#include "Corefuncs/sample_drawfunc.h"

void draw_image(QImage *image_)
{
	if (image_ == 0)
	{
		return;
	}
	int lw = image_->width();
	int lh = image_->height();
	double m_dx = IMAGEWIDTHSIZE;
	double pixel_l = 2.0*IMAGEWIDTHSIZE / lw;
	double m_dy = pixel_l * lh / 2.0;
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	for (unsigned int i = 0; i < lw; i++)
	{
		double x = -m_dx + (double(i) + 0.5)*pixel_l;
		for (unsigned int j = 0; j < lh; j++)
		{
			double y = -m_dy + (double(j) + 0.5)*pixel_l;
			QRgb color_ = image_->pixel(i, lh - 1 - j);
			glColor3d(qRed(color_) / 255.0, qGreen(color_) / 255.0, qBlue(color_) / 255.0);
			draw_quad(x, y, pixel_l);
		}
	}
}

void draw_image_anchor(vector<cv::Point2i> anchorp, QImage *image_)
{
	int lw = image_->width();
	int lh = image_->height();
	double m_dx = IMAGEWIDTHSIZE;
	double pixel_l = 2.0*IMAGEWIDTHSIZE / lw;
	double m_dy = pixel_l * lh / 2.0;
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	for (int i = 0;i<anchorp.size();i++)
	{
		double x = -m_dx + (double(anchorp[i].x) + 0.5)*pixel_l;
		double y = -m_dy + (double(lh - 1 - anchorp[i].y) + 0.5)*pixel_l;
		glColor3d(1.0,0,0);
		draw_quad(x, y, pixel_l);
	}
}

void draw_image_feature_lines(vector<vector<PointTriple>> polys_, int image_wid, int image_hei)
{
	if (polys_.size() == 0)
	{
		return;
	}
	double m_dx = IMAGEWIDTHSIZE;
	double pixel_l = 2.0*IMAGEWIDTHSIZE / double(image_wid);
	double m_dy = pixel_l * double(image_hei) / 2.0;
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3d(0.0, 0.0, 0.0);

	for (int i = 0; i<polys_.size(); i++)
	{
		for (int j = 0; j<polys_[i].size(); j++)
		{
			//glColor3d(polys_[i][j].vit->vertex_pixel_color().x(), polys_[i][j].vit->vertex_pixel_color().x(), polys_[i][j].vit->vertex_pixel_color().x());
			double x = -m_dx + (polys_[i][j].imagep.x() + 0.5)*pixel_l;
			double y = -m_dy + (polys_[i][j].imagep.y() + 0.5)*pixel_l;
			draw_quad(x, y, pixel_l);
		}
	}
}

void draw_quad(double x, double y, double pisel_l)
{
	glBegin(GL_POLYGON);
	glVertex2d(x - pisel_l * 0.5, y - pisel_l * 0.5);
	glVertex2d(x + pisel_l * 0.5, y - pisel_l * 0.5);
	glVertex2d(x + pisel_l * 0.5, y + pisel_l * 0.5);
	glVertex2d(x - pisel_l * 0.5, y + pisel_l * 0.5);
	glEnd();
}

void draw_image_grid(QImage *image_)
{
	if (image_ == 0)
	{
		return;
	}
	int lw = image_->width();
	int lh = image_->height();
	double m_dx = IMAGEWIDTHSIZE;
	double pixel_l = 2.0*IMAGEWIDTHSIZE / (lw - 1);
	double m_dy = pixel_l * (lh - 1) / 2.0;
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	for (unsigned int i = 0; i < lw; i++)
	{
		double x = -m_dx + (double(i) + 0.5)*pixel_l;
		for (unsigned int j = 0; j < lh; j++)
		{
			double y = -m_dy + (double(j) + 0.5)*pixel_l;
			glColor3d(0.7, 0.7, 0.7);
			draw_quad(x, y, pixel_l);
		}
	}
}

void draw_image_knots_point(Mesh *knot_config_)
{
	if (!knot_config_)
	{
		return;
	}
	glPointSize(5.0);
	glBegin(GL_POINTS);
	glColor3f(1.0, 1.0, 0.0);
	for (Mesh::Vertex_iterator vit = knot_config_->vertices_begin(); vit != knot_config_->vertices_end(); vit++)
	{
		glVertex3d(vit->point().x(), vit->point().y(), vit->point().z());
	}
	glEnd();
}

void draw_image_knots_tri(Mesh *knot_config_)
{
	if (!knot_config_)
	{
		return;
	}
	glColor3f(0.8, 0.0, 0.8);
	glLineWidth(1.0);
	for (Mesh::Facet_iterator fit = knot_config_->facets_begin(); fit != knot_config_->facets_end(); fit++)
	{
		Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
		glBegin(GL_LINE_LOOP);
		do
		{
			glVertex3d(hc->vertex()->point().x(), hc->vertex()->point().y(), hc->vertex()->point().z());
		} while (++hc != hs);
		glEnd();
	}
}

void draw_parameterization(Mesh * surface_, bool is_image_domain)
{
	glLineWidth(0.1);
	glPointSize(0.1);
	glColor3f(0.3, 0.3, 0.3);
	for (Mesh::Facet_iterator fit = surface_->facets_begin(); fit != surface_->facets_end(); fit++)
	{
		Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
		glBegin(GL_LINE_LOOP);
		do
		{
			if (is_image_domain)
			{
				glVertex2d(hc->vertex()->get_old_domain().x(), hc->vertex()->get_old_domain().y());
			}
			else
			{
				glColor3d(hc->vertex()->vertex_input_color().x(),
					hc->vertex()->vertex_input_color().y(), 
					hc->vertex()->vertex_input_color().z());
				glVertex2d(hc->vertex()->get_domain().x(), hc->vertex()->get_domain().y());
			}
		} while (++hc != hs);
		glEnd();
	}

	/*glPointSize(4.0);
	glBegin(GL_POINTS);
	glColor3f(0.4, 0.4, 0.4);
	for (Mesh::Vertex_iterator vit = surface_->vertices_begin(); vit != surface_->vertices_end(); vit++)
	{
		if (is_image_domain)
		{
			glVertex2d(vit->get_old_domain().x(), vit->get_old_domain().y());
		}
		else
		{
			glColor3d(vit->vertex_pixel_color().x(), vit->vertex_pixel_color().y(), vit->vertex_pixel_color().z());
			glVertex2d(vit->get_domain().x(), vit->get_domain().y());
		}

	}
	glEnd();*/
}

void draw_parameter_image(Mesh * surface_)
{
	glBegin(GL_TRIANGLES);
	for (Mesh::Facet_iterator fit = surface_->facets_begin(); fit != surface_->facets_end(); fit++)
	{
		Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
		do
		{
			glColor3d(hc->vertex()->vertex_pixel_color().x(), hc->vertex()->vertex_pixel_color().y(),
				hc->vertex()->vertex_pixel_color().z());
			glVertex2d(hc->vertex()->get_domain().x(), hc->vertex()->get_domain().y());
		} while (++hc != hs);
	}
	glEnd();
}

void draw_knots_mesh_pf(Mesh *knot_config_)
{
	if (!knot_config_)
	{
		return;
	}
	glPointSize(4.0);
	glBegin(GL_POINTS);
	glColor3f(0.2, 0.2, 0.2);
	for (Mesh::Vertex_iterator vit = knot_config_->vertices_begin(); vit != knot_config_->vertices_end(); vit++)
	{
		glVertex2d(vit->point().x(), vit->point().y());
	}
	glEnd();

	glColor3f(0.0, 0.0, 0.0);
	glLineWidth(1.0);
	for (Mesh::Facet_iterator fit = knot_config_->facets_begin(); fit != knot_config_->facets_end(); fit++)
	{
		Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
		glBegin(GL_LINE_LOOP);
		do
		{
			glVertex2d(hc->vertex()->point().x(), hc->vertex()->point().y());
		} while (++hc != hs);
		glEnd();
	}
}

void draw_knot_face_error(Mesh * mesh)
{
	glBegin(GL_POINTS);
	glPointSize(5.0);
	glColor3d(0.0, 1.0, 0.0);
	for (auto fit = mesh->facets_begin(); fit != mesh->facets_end(); fit++)
	{
		if (fit->facet_flag().bSelected == 1)
		{
			glVertex2d(fit->error_knot().x(),fit->error_knot().y());
		}
	}
	glEnd();

	glBegin(GL_TRIANGLES);
	for (Mesh::Facet_iterator fit = mesh->facets_begin(); fit != mesh->facets_end(); fit++)
	{
		Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
		if (fit->domain_area() < 1e-9)
		{
			glColor3d(0.5, 0.0, 0.0);
		}
		else 
		{
			glColor3d(fit->facet_error() / mesh->get_max_error(), fit->facet_error() / mesh->get_max_error(), fit->facet_error() / mesh->get_max_error());
		}
		do
		{
			glVertex2d(hc->vertex()->get_domain().x(), hc->vertex()->get_domain().y());
		} while (++hc != hs);
	}
	glEnd();
}

void draw_domain_knots_pointcdt(CDT *knot_config_, Point_2 psize)
{
	if (!knot_config_)
	{
		return;
	}
	glPointSize(3.5);
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_POINTS);
	for (auto vit = knot_config_->vertices_begin(); vit != knot_config_->vertices_end(); vit++)
	{
		//glPointSize(2.0);
		//glColor3f(0.0, 0.0, 1.0);
		if (vit->get_new_status())
		{
			if (psize.x() > 0)
			{
				glVertex2d((vit->point().x() + psize.x()) / (2.0*psize.x()),
					(vit->point().y() + psize.y()) / (2.0*psize.y()));
			}
			else
			glVertex2d(vit->point().x(), vit->point().y());
		}	
	}
	glEnd();
}

void draw_domain_knots_tricdt(CDT *knot_config_,Point_2 psize)
{
	if (!knot_config_)
	{
		return;
	}
	glColor3f(0.0, 0.0, 0.0);
	glLineWidth(1.0);
	for (auto fit = knot_config_->faces_begin(); fit != knot_config_->faces_end(); fit++)
	{
		glBegin(GL_LINE_LOOP);
		if (psize.x()>0)
		{
			glVertex2d((fit->vertex(0)->point().x()+psize.x())/(2.0*psize.x()), 
				(fit->vertex(0)->point().y() + psize.y()) / (2.0*psize.y()));
			glVertex2d((fit->vertex(1)->point().x() + psize.x()) / (2.0*psize.x()), 
				(fit->vertex(1)->point().y() + psize.y()) / (2.0*psize.y()));
			glVertex2d((fit->vertex(2)->point().x() + psize.x()) / (2.0*psize.x()), 
				(fit->vertex(2)->point().y() + psize.y()) / (2.0*psize.y()));
		}
		else
		{
			glVertex2d(fit->vertex(0)->point().x(), fit->vertex(0)->point().y());
			glVertex2d(fit->vertex(1)->point().x(), fit->vertex(1)->point().y());
			glVertex2d(fit->vertex(2)->point().x(), fit->vertex(2)->point().y());
		}	
		glEnd();
	}
}

void draw_domain_knots_pointmesh(Mesh *knot_config_)
{
	if (!knot_config_)
	{
		return;
	}
	glPointSize(5.0);
	glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 0.4);
	for (Mesh::Vertex_iterator vit = knot_config_->vertices_begin(); vit != knot_config_->vertices_end(); vit++)
	{
		glVertex2d(vit->get_domain().x(), vit->get_domain().y());
	}
	glEnd();
}

void draw_domain_knots_trimesh(Mesh *knot_config_)
{
	if (!knot_config_)
	{
		return;
	}
	glColor3f(0.0, 0.0, 0.0);
	glLineWidth(1.0);
	for (Mesh::Facet_iterator fit = knot_config_->facets_begin(); fit != knot_config_->facets_end(); fit++)
	{
		Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
		glBegin(GL_LINE_LOOP);
		do
		{
			glVertex2d(hc->vertex()->get_domain().x(), hc->vertex()->get_domain().y());
		} while (++hc != hs);
		glEnd();
	}
}

void draw_feature_poly(vector<vector<PointTriple>> polys_)
{
	//test
	/*glColor3d(0.0, 0.0, 0.0);
	glBegin(GL_POINTS);
	for (int k = 0; k < polys_.size(); k++)
	{
	for (int i = 0; i < polys_[k].size(); i++)
	{
	glVertex2d(polys_[k][i].paradomain.x(), polys_[k][i].paradomain.y());
	}
	}
	glEnd();*/

	glLineWidth(3.0);
	glColor3f(0.0, 0.46, 0.0);

	for (int k = 0; k<polys_.size(); k++)
	{
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < polys_[k].size(); i++)
		{
			if (polys_[k][i].is_fixed)
			{
				glVertex2d(polys_[k][i].paradomain.x(), polys_[k][i].paradomain.y());
			}
		}
		glEnd();
	}

	/*glColor3f(1.0, 0.0, 1.0);
	glBegin(GL_POINTS);
	glPointSize(6.0);
	for (int k = 0; k < polys_.size(); k++)
	{
		for (int i = 0; i < polys_[k].size(); i++)
		{
			if (polys_[k][i].is_fixed)
			{
				glVertex2d(polys_[k][i].paradomain.x(), polys_[k][i].paradomain.y());
			}
		}
	}
	glEnd();*/
}

void draw_collinear_knots_lines(vector<vector<PointTriple>> polys_)
{
	/*glLineWidth(2.0);
	glColor3f(0.0, 0.0, 0.0);

	for (int k = 0; k<polys_.size(); k++)
	{
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < polys_[k].size(); i++)
		{
			if (polys_[k][i].is_fixed)
			{
				glVertex2d(polys_[k][i].imagedomain.x(), polys_[k][i].imagedomain.y());
			}
		}
		glEnd();
	}*/

	glPointSize(4.0);
	glColor3f(1.0, 1.0, 0.0);
	glBegin(GL_POINTS);
	for (int k = 0; k < polys_.size(); k++)
	{
		for (int i = 0; i < polys_[k].size(); i++)
		{
			glVertex2d(polys_[k][i].imagedomain.x(), polys_[k][i].imagedomain.y());
		}
	}
	glEnd();
}

void draw_domain_boundary()
{
	glLineWidth(2.0);
	glColor3f(0.0, 0.0, 0.8);

	glBegin(GL_LINE_LOOP);
	//glBegin(GL_POLYGON);
	glVertex2d(0.0, 0.0);
	glVertex2d(0.0, 1.0);
	glVertex2d(1.0, 1.0);
	glVertex2d(1.0, 0.0);
	glEnd();
}

void draw_image_boundary(double dw, double dh)
{
	glLineWidth(2.0);
	glColor3f(0.5, 0.5, 0.5);
	//glColor3f(0.0, 0.0, 0.8);
	dw += 0.005; dh += 0.005;

	glBegin(GL_LINE_LOOP);
	//glBegin(GL_POLYGON);
	glVertex2d(-dw, -dh);
	glVertex2d(-dw, dh);
	glVertex2d(dw, dh);
	glVertex2d(dw, -dh);
	glEnd();
}

void draw_feature_points(vector<vector<PointTriple>> polys_)
{
	glPointSize(5.0);

	vector<Point_3> color_;
	color_.push_back(Point_3(1, 0, 0));
	color_.push_back(Point_3(0, 1, 0));
	color_.push_back(Point_3(0, 0, 1));
	color_.push_back(Point_3(1, 0, 1));
	color_.push_back(Point_3(0, 1, 1));
	color_.push_back(Point_3(0.5, 0, 0));
	color_.push_back(Point_3(0, 0.5, 0));
	color_.push_back(Point_3(0, 0, 0.5));
	color_.push_back(Point_3(0.5, 0.5, 0));
	color_.push_back(Point_3(0.5, 0, 0.5));
	color_.push_back(Point_3(0, 0.5, 0.5));
	color_.push_back(Point_3(0.6, 0.6, 0.6));
	color_.push_back(Point_3(0.2, 0.2, 0.2));

	for (int k = 0; k < polys_.size(); k++)
	{
		//glColor3d(color_[k%color_.size()].x(), color_[k%color_.size()].y(), color_[k%color_.size()].z());
		glColor3d(0,0,0.5);
		glBegin(GL_POINTS);
		for (int i = 0; i < polys_[k].size(); i++)
		{
			glVertex2d(polys_[k][i].imagedomain.x(), polys_[k][i].imagedomain.y());
			//glVertex2d(polys_[k][i].image_newp.x(), polys_[k][i].image_newp.y());
		}
		glEnd();

		/*glBegin(GL_LINE_STRIP);
		for (int i = 0; i < polys_[k].size(); i++)
		{
		glVertex2d(polys_[k][i].x(), polys_[k][i].y());
		}
		glEnd();*/
	}
}

void draw_selected_feature_sequence(CMInfo* cm_, vector<int> current_index_, vector<vector<CMVertexInfo>>control_points_seqs)
{
	int feature_line_index = -1;
	int index = 0;
	for (int i = 0; i < cm_->vertices.size(); i++)
	{
		if (cm_->vertices[i].flag == 1 && cm_->vertices[i].do_display)
		{
			if (cm_->vertices[i].is_on_feature)
			{
				vector<int>::iterator res1 = std::find(current_index_.begin(), current_index_.end(), index);
				if (res1 != current_index_.end())
				{
					feature_line_index = cm_->vertices[i].feacp_type;
					break;
				}
				index++;

				vector<int>::iterator res2 = std::find(current_index_.begin(), current_index_.end(), index);
				if (res2 != current_index_.end())
				{
					feature_line_index = cm_->vertices[i].feacp_type;
					break;
				}
				index++;
			}
			else
			{
				index++;
			}
		}
	}

	if (feature_line_index != -1)
	{
		glPointSize(12.5);
		glColor3d(0, 0, 0);
		glBegin(GL_POINTS);
		for (int i = 0; i < control_points_seqs[feature_line_index].size(); i++)
		{
			glVertex3d(control_points_seqs[feature_line_index][i].vertex.x, control_points_seqs[feature_line_index][i].vertex.y, 0);
		}
		glEnd();
	}
	
}

void draw_featureregion_sample_points(vector<Point_2> sample_domain1, vector<RGBPoint> original_values1, 
	vector<Point_2> sample_domain2, vector<RGBPoint> original_values2, 
	vector<Point_2> sample_domain3, vector<RGBPoint> original_values3,Mesh *originalmesh)
{
	glPointSize(1.5);
	glColor3d(1.0, 0.0, 0.0);
	glBegin(GL_POINTS);
	for (int k = 0; k < sample_domain1.size(); k++)
	{
		glColor3d(original_values1[k].r, original_values1[k].g, original_values1[k].b);
		glVertex2d(sample_domain1[k].x(), sample_domain1[k].y());
	}
	glEnd();

	glBegin(GL_POINTS);
	for (int k = 0; k < sample_domain2.size(); k++)
	{
		glColor3d(original_values2[k].r, original_values2[k].g, original_values2[k].b);
		glVertex2d(sample_domain2[k].x(), sample_domain2[k].y());
	}
	glEnd();

	glPointSize(2.0);
	glBegin(GL_POINTS);
	for (int k = 0; k < sample_domain3.size(); k++)
	{
		glColor3d(1.0, 0.0, 0.0);
		glVertex2d(sample_domain3[k].x(), sample_domain3[k].y());
	}
	glEnd();
	/*glBegin(GL_LINES);
	for (int k = 0; k < sample_domain3.size()-1; k++)
	{
	glColor3d(1.0, 0.0, 0.0);
	glVertex2d(sample_domain3[k].x(), sample_domain3[k].y());
	glVertex2d(sample_domain3[k+1].x(), sample_domain3[k+1].y());
	}
	glEnd();*/

	glPointSize(2.5);
	glBegin(GL_POINTS);
	for (auto vit = originalmesh->vertices_begin();vit != originalmesh->vertices_end();vit++)
	{
		if (!vit->is_feature())
		{
			glColor3d(vit->vertex_pixel_color().x(), vit->vertex_pixel_color().y(), vit->vertex_pixel_color().z());
			glVertex2d(vit->get_domain().x(), vit->get_domain().y());
		}
	}
	glEnd();
}

void draw_featureregion_original_points(vector<RGBPoint> original_values1,vector<RGBPoint> original_values2, vector<RGBPoint> original_values3)
{
	glPointSize(1.5);
	glColor3d(1.0, 0.0, 0.0);

	if (!original_values1.empty())
	{
		glBegin(GL_POINTS);
		for (int k = 0; k < original_values1.size(); k++)
		{
			glColor3d(original_values1[k].r, original_values1[k].g, original_values1[k].b);
			glVertex2d(original_values1[k].x, original_values1[k].y);
		}
		glEnd();
	}

	if (!original_values2.empty())
	{
		glBegin(GL_POINTS);
		for (int k = 0; k < original_values2.size(); k++)
		{
			glColor3d(original_values2[k].r, original_values2[k].g, original_values2[k].b);
			glVertex2d(original_values2[k].x, original_values2[k].y);
		}
		glEnd();
	}
	
	if (!original_values3.empty())
	{
		glPointSize(4.0);
		glBegin(GL_POINTS);
		for (int k = 0; k < original_values3.size(); k++)
		{
			glColor3d(1.0, 0.0, 0.0);
			glVertex2d(original_values3[k].x, original_values3[k].y);
		}
		glEnd();

		/*glBegin(GL_LINES);
		for (int k = 0; k < original_values3.size()-1; k++)
		{
			glColor3d(1.0, 0.0, 0.0);
			glVertex2d(original_values3[k].x, original_values3[k].y);
			glVertex2d(original_values3[k+1].x, original_values3[k+1].y);
		}
		glEnd();*/
	}
	
}

void draw_original_surface_points(Mesh *mesh)
{
	Mesh *temp_mesh = mesh;
	if (temp_mesh != NULL && temp_mesh->is_valid() && !temp_mesh->empty())
	{
		glPointSize(2.0);
		glBegin(GL_POINTS);
		glColor3f(0.0, 0.0, 0.5);
		Mesh::Vertex_iterator v_it;
		for (v_it = temp_mesh->vertices_begin(); v_it != temp_mesh->vertices_end(); v_it++)
		{
			glVertex3d(v_it->point().x(), v_it->point().y(), v_it->point().z());
		}
		glEnd();
	}
}

void draw_original_surface_edges(Mesh *mesh)
{
	Mesh *temp_mesh = mesh;

	if (temp_mesh != NULL && temp_mesh->is_valid() && !temp_mesh->empty())
	{
		Mesh::Face_iterator fit = temp_mesh->facets_begin();
		for (; fit != temp_mesh->facets_end(); fit++)
		{
			Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
			glColor3f(0.0, 0.0, 0.2);
			glBegin(GL_LINE_LOOP);
			do
			{
				//glNormal3d(hc->vertex()->normal().x(), hc->vertex()->normal().y(), hc->vertex()->normal().z());
				glVertex3d(hc->vertex()->point().x(), hc->vertex()->point().y(), hc->vertex()->point().z());
			} while (++hc != hs);
			glEnd();
		}
	}
}

void draw_original_surface_faces(Mesh *mesh)
{
	Mesh *temp_mesh = mesh;

	if (temp_mesh != NULL && temp_mesh->is_valid() && !temp_mesh->empty())
	{
		Mesh::Face_iterator fit = temp_mesh->facets_begin();
		for (; fit != temp_mesh->facets_end(); fit++)
		{
			Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
			//glColor3f(0.1, 0.1, 0.5);
			glBegin(GL_TRIANGLES);
			do
			{
				glColor3d(hc->vertex()->vertex_input_color().x(),
					hc->vertex()->vertex_input_color().y(), hc->vertex()->vertex_input_color().z());
				glNormal3d(hc->vertex()->normal().x(), hc->vertex()->normal().y(), hc->vertex()->normal().z());
				glVertex3d(hc->vertex()->point().x(), hc->vertex()->point().y(), hc->vertex()->point().z());
			} while (++hc != hs);
			glEnd();
		}
		/*glBegin(GL_LINES);
		for (auto eit = temp_mesh->edges_begin();eit != temp_mesh->edges_end();eit++)
		{
			glColor3f(0.1, 0.1, 0.5);
			glVertex3d(eit->vertex()->point().x(), eit->vertex()->point().y(), eit->vertex()->point().z());
			glVertex3d(eit->opposite()->vertex()->point().x(), eit->opposite()->vertex()->point().y(), eit->opposite()->vertex()->point().z());
		}
		glEnd();*/
	}
}

void draw_fitted_surface_points(Mesh *mesh)
{
	Mesh *temp_mesh = mesh;
	if (temp_mesh != NULL && temp_mesh->is_valid() && !temp_mesh->empty())
	{
		glPointSize(2.0);
		glBegin(GL_POINTS);
		glColor3f(0.7, 0.7, 0.7);
		Mesh::Vertex_iterator v_it;
		for (v_it = temp_mesh->vertices_begin(); v_it != temp_mesh->vertices_end(); v_it++)
		{
			/*glVertex3d(v_it->point().x(), v_it->point().y(), v_it->vertex_pixel_color().x());*/
			glVertex3d(v_it->point().x(), v_it->point().y(), v_it->point().z());
		}
		glEnd();
	}
}

void draw_fitted_surface_edges(Mesh *mesh)
{
	Mesh *temp_mesh = mesh;

	if (temp_mesh != NULL && temp_mesh->is_valid() && !temp_mesh->empty())
	{
		Mesh::Face_iterator fit = temp_mesh->facets_begin();
		for (; fit != temp_mesh->facets_end(); fit++)
		{
			Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
			glColor3f(0.2, 0.0, 0.0);
			glBegin(GL_LINE_LOOP);
			do
			{
				glNormal3d(hc->vertex()->normal().x(), hc->vertex()->normal().y(), hc->vertex()->normal().z());
				glVertex3d(hc->vertex()->point().x(), hc->vertex()->point().y(), hc->vertex()->point().z());
			} while (++hc != hs);
			glEnd();
		}
	}
}

void draw_fitted_surface_faces(Mesh *mesh)
{
	Mesh *temp_mesh = mesh;

	if (temp_mesh != NULL && temp_mesh->is_valid() && !temp_mesh->empty())
	{
		glBegin(GL_TRIANGLES);
		Mesh::Face_iterator fit = temp_mesh->facets_begin();
		for (; fit != temp_mesh->facets_end(); fit++)
		{
			Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
			//glColor3f(0.5, 0.1, 0.1);
			do
			{
				glColor3f(hc->vertex()->vertex_pixel_color().x(), hc->vertex()->vertex_pixel_color().y(),
					hc->vertex()->vertex_pixel_color().z());
				glNormal3d(hc->vertex()->normal().x(), hc->vertex()->normal().y(), hc->vertex()->normal().z());
				glVertex3d(hc->vertex()->point().x(), hc->vertex()->point().y(), hc->vertex()->point().z());
			} while (++hc != hs);
		}
		glEnd();
	}
}

void draw_control_mesh_points(CMInfo &cmInfo,int type)
{
	//glColor3d(194 / 255.0, 116 / 255.0, 167 / 255.0);
	if (type == 3)
	{
		glPointSize(7.0);
	}
	glPointSize(7.0);
	for (int i = 0; i < cmInfo.vertices.size(); i++)
	{
		if (cmInfo.vertices[i].flag == 1 && cmInfo.vertices[i].do_display)
		{	
			if (type == 3)
			{
				glBegin(GL_POINTS);
				glColor3d(cmInfo.vertices[i].vertex.r, cmInfo.vertices[i].vertex.g, cmInfo.vertices[i].vertex.b);
				glVertex3d(cmInfo.vertices[i].vertex.x, cmInfo.vertices[i].vertex.y, cmInfo.vertices[i].vertex.r);
				glEnd();
			}
			else if (type == 2)
			{
				glBegin(GL_POINTS);
				glColor3d(cmInfo.vertices[i].vertex.r, cmInfo.vertices[i].vertex.g, cmInfo.vertices[i].vertex.b);
				glVertex2d(cmInfo.vertices[i].center.x(), cmInfo.vertices[i].center.y());
				glEnd();
			}
			else if (type == 32)
			{
				if (cmInfo.vertices[i].is_on_feature)
				{
					/////////////////////////////////////////////////one side
					//auxiliary
					glPointSize(10.0);
					glBegin(GL_POINTS);
					glColor3d(cmInfo.vertices[i].cpp_little_trans.first.r,
						cmInfo.vertices[i].cpp_little_trans.first.g, cmInfo.vertices[i].cpp_little_trans.first.b);
					glVertex2d(cmInfo.vertices[i].cpp_little_trans.first.x, cmInfo.vertices[i].cpp_little_trans.first.y);
					glEnd();
					//center
					glPointSize(6.0);
					glBegin(GL_POINTS);
					glColor3d(cmInfo.vertices[i].cpp_little_trans.first.r,
						cmInfo.vertices[i].cpp_little_trans.first.g, cmInfo.vertices[i].cpp_little_trans.first.b);
					glVertex2d(cmInfo.vertices[i].control_point_pair.first.x, cmInfo.vertices[i].control_point_pair.first.y);
					glEnd();

					//black ring
					glPointSize(11);
					glBegin(GL_POINTS);
					glColor3d(0, 0, 0);
					glVertex2d(cmInfo.vertices[i].cpp_little_trans.first.x, cmInfo.vertices[i].cpp_little_trans.first.y);
					glEnd();

					//segment
					glLineWidth(1.0);
					glColor3d(0, 0, 0);
					glBegin(GL_LINES);
					glVertex2d(cmInfo.vertices[i].cpp_little_trans.first.x, cmInfo.vertices[i].cpp_little_trans.first.y);
					glVertex2d(cmInfo.vertices[i].control_point_pair.first.x, cmInfo.vertices[i].control_point_pair.first.y);
					glEnd();

					/////////////////////////////////////////////////another side
					glPointSize(10.0);
					glBegin(GL_POINTS);
					glColor3d(cmInfo.vertices[i].cpp_little_trans.second.r,
						cmInfo.vertices[i].cpp_little_trans.second.g, cmInfo.vertices[i].cpp_little_trans.second.b);
					glVertex2d(cmInfo.vertices[i].cpp_little_trans.second.x, cmInfo.vertices[i].cpp_little_trans.second.y);
					glVertex2d(cmInfo.vertices[i].control_point_pair.second.x, cmInfo.vertices[i].control_point_pair.second.y);
					glEnd();

					//black ring
					glPointSize(11);
					glBegin(GL_POINTS);
					glColor3d(0, 0, 0);
					glVertex2d(cmInfo.vertices[i].cpp_little_trans.second.x, cmInfo.vertices[i].cpp_little_trans.second.y);
					glEnd();

					glLineWidth(1.0);
					glColor3d(0, 0, 0);
					glBegin(GL_LINES);
					glVertex2d(cmInfo.vertices[i].cpp_little_trans.second.x, cmInfo.vertices[i].cpp_little_trans.second.y);
					glVertex2d(cmInfo.vertices[i].control_point_pair.second.x, cmInfo.vertices[i].control_point_pair.second.y);
					glEnd();
				}
				else
				{
					glPointSize(7.0);
					glBegin(GL_POINTS);
					glColor3d(cmInfo.vertices[i].vertex.r, cmInfo.vertices[i].vertex.g, cmInfo.vertices[i].vertex.b);
					glVertex2d(cmInfo.vertices[i].vertex.x, cmInfo.vertices[i].vertex.y);
					glEnd();
				}		
			}
		}	
	}
}

void draw_control_mesh_edges(CMInfo &cmInfo,int type,Point_2 psize)
{
	if (type == 3)
	{
		glLineWidth(2.0f);
	}
	else
	{
		glLineWidth(1.0f);
	}
	glBegin(GL_LINES);
	std::set<CMEdgeInfo>::iterator it = cmInfo.edges.begin();
	for (; it != cmInfo.edges.end(); it++)
	{
		glColor3d(119 / 255.0, 189 / 255.0, 191 / 255.0);
		if (cmInfo.vertices[it->first()].flag==1 && cmInfo.vertices[it->second()].flag==1 &&
			cmInfo.vertices[it->first()].do_display && cmInfo.vertices[it->second()].do_display)
		{
			if (type == 3)
			{
				glVertex3d(cmInfo.vertices[it->first()].vertex.x, cmInfo.vertices[it->first()].vertex.y, cmInfo.vertices[it->first()].vertex.r);
				glVertex3d(cmInfo.vertices[it->second()].vertex.x, cmInfo.vertices[it->second()].vertex.y, cmInfo.vertices[it->second()].vertex.r);
			}
			else if (type == 2)
			{
				glVertex2d(cmInfo.vertices[it->first()].center.x(), cmInfo.vertices[it->first()].center.y());
				glVertex2d(cmInfo.vertices[it->second()].center.x(), cmInfo.vertices[it->second()].center.y());
			}
			else if (type == 23)
			{
				glVertex2d(cmInfo.vertices[it->first()].center.x()*2.0*psize.x()-psize.x(), 
					cmInfo.vertices[it->first()].center.y()*2.0*psize.y() - psize.y());
				glVertex2d(cmInfo.vertices[it->second()].center.x()*2.0*psize.x() - psize.x(),
					cmInfo.vertices[it->second()].center.y()*2.0*psize.y() - psize.y());
			}
			else if (type == 32)
			{
				if (cmInfo.vertices[it->first()].is_on_feature && cmInfo.vertices[it->second()].is_on_feature)
				{
					glColor3d(0, 0, 0);
				}
				glVertex2d(cmInfo.vertices[it->first()].vertex.x, cmInfo.vertices[it->first()].vertex.y);
				glVertex2d(cmInfo.vertices[it->second()].vertex.x, cmInfo.vertices[it->second()].vertex.y);
			}
		}
	}
	glEnd();
}

void draw_control_points_featurepair(CMInfo &cmInfo)
{
	glPointSize(6.0);
	glLineWidth(1.5f);
	for (int i = 0; i < cmInfo.vertices.size(); i++)
	{
		if (cmInfo.vertices[i].flag == 1 && cmInfo.vertices[i].do_display)
		{
			if (cmInfo.vertices[i].is_on_feature)
			{
				/////////////////////////////////////////////////one side
				//auxiliary
				glPointSize(10.0);
				glBegin(GL_POINTS);
				glColor3d(cmInfo.vertices[i].cpp_little_trans.first.r, 
					cmInfo.vertices[i].cpp_little_trans.first.g, cmInfo.vertices[i].cpp_little_trans.first.b);
				glVertex2d(cmInfo.vertices[i].cpp_little_trans.first.x, cmInfo.vertices[i].cpp_little_trans.first.y);
				glEnd();
				//center
				glPointSize(6.0);
				glBegin(GL_POINTS);
				glColor3d(cmInfo.vertices[i].cpp_little_trans.first.r,
					cmInfo.vertices[i].cpp_little_trans.first.g, cmInfo.vertices[i].cpp_little_trans.first.b);
				glVertex2d(cmInfo.vertices[i].control_point_pair.first.x, cmInfo.vertices[i].control_point_pair.first.y);
				glEnd();

				//black ring
				glPointSize(11);
				glBegin(GL_POINTS);
				glColor3d(0, 0, 0);
				glVertex2d(cmInfo.vertices[i].cpp_little_trans.first.x, cmInfo.vertices[i].cpp_little_trans.first.y);
				glEnd();

				//segment
				glLineWidth(1.0);
				glColor3d(0, 0, 0);
				glBegin(GL_LINES);
				glVertex2d(cmInfo.vertices[i].cpp_little_trans.first.x, cmInfo.vertices[i].cpp_little_trans.first.y);
				glVertex2d(cmInfo.vertices[i].control_point_pair.first.x, cmInfo.vertices[i].control_point_pair.first.y);
				glEnd();

				/////////////////////////////////////////////////another side
				glPointSize(10.0);
				glBegin(GL_POINTS);
				glColor3d(cmInfo.vertices[i].cpp_little_trans.second.r,
					cmInfo.vertices[i].cpp_little_trans.second.g, cmInfo.vertices[i].cpp_little_trans.second.b);
				glVertex2d(cmInfo.vertices[i].cpp_little_trans.second.x, cmInfo.vertices[i].cpp_little_trans.second.y);
				glVertex2d(cmInfo.vertices[i].control_point_pair.second.x, cmInfo.vertices[i].control_point_pair.second.y);
				glEnd();

				//black ring
				glPointSize(11);
				glBegin(GL_POINTS);
				glColor3d(0, 0, 0);
				glVertex2d(cmInfo.vertices[i].cpp_little_trans.second.x, cmInfo.vertices[i].cpp_little_trans.second.y);
				glEnd();

				glLineWidth(1.0);
				glColor3d(0, 0, 0);
				glBegin(GL_LINES);
				glVertex2d(cmInfo.vertices[i].cpp_little_trans.second.x, cmInfo.vertices[i].cpp_little_trans.second.y);
				glVertex2d(cmInfo.vertices[i].control_point_pair.second.x, cmInfo.vertices[i].control_point_pair.second.y);
				glEnd();
			}
		}
	}
}

void draw_test_basis(Mesh *temp_mesh)
{
	if (temp_mesh != NULL && temp_mesh->is_valid() && !temp_mesh->empty())
	{
		Mesh::Vertex_iterator v_it;

		double range_min, range_max;

		int cut_number = 0;

		vector<double> meanvalue_cur;
		double max_z = 0.0;
		meanvalue_cur.reserve(temp_mesh->size_of_vertices());
		for (v_it = temp_mesh->vertices_begin(); v_it != temp_mesh->vertices_end(); v_it++)
		{
			meanvalue_cur.push_back(v_it->point().z());
		}
		//qsort(meanvalue_cur, surfaceMesh->size_of_vertices(), sizeof(double), qsort_compare);
		std::sort(meanvalue_cur.begin(), meanvalue_cur.end());
		max_z = meanvalue_cur[temp_mesh->size_of_vertices() - 1 - cut_number];

		range_min = meanvalue_cur[cut_number];
		range_max = meanvalue_cur[temp_mesh->size_of_vertices() - 1 - cut_number];

		CColorRamp		mRamp;
		mRamp.BuildRainbow();
		for (v_it = temp_mesh->vertices_begin(); v_it != temp_mesh->vertices_end(); v_it++)
		{
			double cur_m = v_it->point().z();

			if (cur_m > range_max)
			{
				cur_m = range_max;
			}
			if (cur_m < range_min)
			{
				cur_m = range_min;
			}
			cur_m = (cur_m - range_min) / (range_max - range_min);

			//v_it->point() = Point(v_it->point().x(), v_it->point().y(), 0.6*cur_m);

			Point_3 color_(mRamp.Red(255 * cur_m) / 255.0, mRamp.Green(255 * cur_m) / 255.0,
				mRamp.Blue(255 * cur_m) / 255.0);
			v_it->vertex_color() = color_;
		}

		for (auto fit = temp_mesh->facets_begin(); fit != temp_mesh->facets_end(); fit++)
		{
			glBegin(GL_LINE_LOOP);
			Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
			do
			{
				glColor3d(hc->vertex()->vertex_color().x(), hc->vertex()->vertex_color().y(), hc->vertex()->vertex_color().z());
				glNormal3d(hc->vertex()->normal().x(), hc->vertex()->normal().y(), hc->vertex()->normal().z());
				glVertex3d(hc->vertex()->point().x(), hc->vertex()->point().y(), hc->vertex()->point().z()*0.02/max_z);
			} while (++hc != hs);
			glEnd();
		}
	}
}

void draw_image_error_bymesh(Mesh *temp_mesh,QImage *image_)
{
	if (temp_mesh != NULL && temp_mesh->is_valid() && !temp_mesh->empty())
	{
		Mesh::Vertex_iterator v_it;

#if 0
		double range_min, range_max;

		int cut_number = temp_mesh->size_of_vertices() * 0.05;

		vector<double> meanvalue_cur;
		meanvalue_cur.reserve(temp_mesh->size_of_vertices());
		for (v_it = temp_mesh->vertices_begin(); v_it != temp_mesh->vertices_end(); v_it++)
		{
			meanvalue_cur.push_back(v_it->vertex_error());
		}
		//qsort(meanvalue_cur, surfaceMesh->size_of_vertices(), sizeof(double), qsort_compare);
		std::sort(meanvalue_cur.begin(), meanvalue_cur.end());

		range_min = meanvalue_cur[cut_number];
		range_max = meanvalue_cur[temp_mesh->size_of_vertices() - 1 - cut_number];

		CColorRamp		mRamp;
		mRamp.BuildRainbow();
		for (v_it = temp_mesh->vertices_begin(); v_it != temp_mesh->vertices_end(); v_it++)
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
			cur_m = (cur_m - range_min) / (range_max - range_min);

			Point_3 color_(mRamp.Red(255 * cur_m) / 255.0, mRamp.Green(255 * cur_m) / 255.0,
				mRamp.Blue(255 * cur_m) / 255.0);
			v_it->vertex_color() = color_;
		}
#endif
#if 0
		double range_min = 0.0, range_max = 1.0;

		vector<double> meanvalue_cur;
		meanvalue_cur.reserve(temp_mesh->size_of_vertices());
		for (v_it = temp_mesh->vertices_begin(); v_it != temp_mesh->vertices_end(); v_it++)
		{
			meanvalue_cur.push_back(v_it->vertex_error());
		}
		CColorRamp		mRamp;
		mRamp.BuildRainbow();
		for (v_it = temp_mesh->vertices_begin(); v_it != temp_mesh->vertices_end(); v_it++)
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
			cur_m = (cur_m - range_min) / (range_max - range_min);

			Point_3 color_(mRamp.Red(255 * cur_m) / 255.0, mRamp.Green(255 * cur_m) / 255.0,
				mRamp.Blue(255 * cur_m) / 255.0);
			v_it->vertex_color() = color_;
		}


		int lw = image_->width();
		int lh = image_->height();
		double m_dx = IMAGEWIDTHSIZE;
		double pixel_l = 2.0*IMAGEWIDTHSIZE / lw;
		double m_dy = pixel_l * lh / 2.0;
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		for (auto vit = temp_mesh->vertices_begin(); vit != temp_mesh->vertices_end(); vit++)
		{
			int i = vit->vertex_coordinate().first;
			int j = vit->vertex_coordinate().second;
			double x = -m_dx + (double(i) + 0.5)*pixel_l;
			double y = -m_dy + (double(j) + 0.5)*pixel_l;
			glColor3d(vit->vertex_color().x(), vit->vertex_color().y(), vit->vertex_color().z());
			draw_quad(x, y, pixel_l);
		}
#endif

		double range_min = 0.0, range_max = 1.0;
		for (v_it = temp_mesh->vertices_begin(); v_it != temp_mesh->vertices_end(); v_it++)
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

		int lw = image_->width();
		int lh = image_->height();
		double m_dx = IMAGEWIDTHSIZE;
		double pixel_l = 2.0*IMAGEWIDTHSIZE / lw;
		double m_dy = pixel_l * lh / 2.0;

		cv::Mat cm_in(lh, lw, CV_8UC3);
		for (auto vit = temp_mesh->vertices_begin(); vit != temp_mesh->vertices_end(); vit++)
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
		// Apply the colormap:
		cv::applyColorMap(cm_in, cm_out, cv::COLORMAP_JET);
		// Show the result:
		//cv::imshow("colormap out", cm_out);

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		for (auto vit = temp_mesh->vertices_begin(); vit != temp_mesh->vertices_end(); vit++)
		{
			int i = vit->vertex_coordinate().first;
			int j = vit->vertex_coordinate().second;
			double x = -m_dx + (double(i) + 0.5)*pixel_l;
			double y = -m_dy + (double(j) + 0.5)*pixel_l;

			double r_ = cm_out.at<cv::Vec3b>(lh - 1 - j, i)[2] / 255.0;
			double g_ = cm_out.at<cv::Vec3b>(lh - 1 - j, i)[1] / 255.0;
			double b_ = cm_out.at<cv::Vec3b>(lh - 1 - j, i)[0] / 255.0;

			glColor3d(r_, g_, b_);
			draw_quad(x, y, pixel_l);
		}
	}
}

void draw_image_error_based_onthres(Mesh *temp_mesh, QImage *image_,double thres)
{
	if (temp_mesh != NULL && temp_mesh->is_valid() && !temp_mesh->empty())
	{
		Mesh::Vertex_iterator v_it;

		double range_min = 0.0, range_max = 1.0;
		for (v_it = temp_mesh->vertices_begin(); v_it != temp_mesh->vertices_end(); v_it++)
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
			cur_m = (cur_m - range_min) / (range_max - range_min);
			if (cur_m*255.0 > thres)
			{
				Point_3 color_(0.0,0.0,0.0);
				v_it->vertex_color() = color_;
			}
			else
			{
				Point_3 color_(1.0, 1.0, 1.0);
				v_it->vertex_color() = color_;
			}
			
		}

		int lw = image_->width();
		int lh = image_->height();
		double m_dx = IMAGEWIDTHSIZE;
		double pixel_l = 2.0*IMAGEWIDTHSIZE / lw;
		double m_dy = pixel_l * lh / 2.0;

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		for (auto vit = temp_mesh->vertices_begin(); vit != temp_mesh->vertices_end(); vit++)
		{
			int i = vit->vertex_coordinate().first;
			int j = vit->vertex_coordinate().second;
			double x = -m_dx + (double(i) + 0.5)*pixel_l;
			double y = -m_dy + (double(j) + 0.5)*pixel_l;

			glColor3d(vit->vertex_color().x(), vit->vertex_color().x(), vit->vertex_color().x());
			draw_quad(x, y, pixel_l);
		}
	}
}

void draw_image_error_regions(vector<pair<int, int>>pixels,vector<vector<pair<int, int>>> regions, QImage *image_)
{
	if (!regions.empty())
	{
		int lw = image_->width();
		int lh = image_->height();
		double m_dx = IMAGEWIDTHSIZE;
		double pixel_l = 2.0*IMAGEWIDTHSIZE / lw;
		double m_dy = pixel_l * lh / 2.0;

		vector<Point_3> color_;
		color_.push_back(Point_3(1, 0, 0));
		color_.push_back(Point_3(0, 1, 0));
		color_.push_back(Point_3(0, 0, 1));
		color_.push_back(Point_3(1, 1, 0));
		color_.push_back(Point_3(1, 0, 1));
		color_.push_back(Point_3(0, 1, 1));
		color_.push_back(Point_3(0.5, 0, 0));
		color_.push_back(Point_3(0, 0.5, 0));
		color_.push_back(Point_3(0, 0, 0.5));
		color_.push_back(Point_3(0.5, 0.5, 0));
		color_.push_back(Point_3(0.5, 0, 0.5));
		color_.push_back(Point_3(0, 0.5, 0.5));
			
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		for (int i = 0;i<regions.size();i++)
		{
			glColor3d(0,0,0);
			//glColor3d(color_[(i) % color_.size()].x(), color_[(i) % color_.size()].y(), color_[(i) % color_.size()].z());
			for (int j = 0;j<regions[i].size();j++)
			{
				int it = regions[i][j].first;
				int jt = regions[i][j].second;
				double x = -m_dx + (double(it) + 0.5)*pixel_l;
				double y = -m_dy + (double(jt) + 0.5)*pixel_l;		
				draw_quad(x, y, pixel_l);
			}			
		}

		glPointSize(4.0);
		for (int i = 0;i<pixels.size();i++)
		{
			int it = pixels[i].first;
			int jt = pixels[i].second;
			double x = -m_dx + (double(it) + 0.5)*pixel_l;
			double y = -m_dy + (double(jt) + 0.5)*pixel_l;

			glColor3d(0,1,0);
			draw_quad(x, y, pixel_l);
		}
	}
}

void draw_fitted_image_bymesh(Mesh *mesh_temp)
{
	glBegin(GL_TRIANGLES);
	for (auto fit = mesh_temp->facets_begin(); fit != mesh_temp->facets_end(); fit++)
	{
		Mesh::Halfedge_around_facet_circulator hfr = fit->facet_begin();
		Mesh::Vertex_iterator v1, v2, v3;
		v1 = hfr->vertex(); hfr++;
		v2 = hfr->vertex(); hfr++;
		v3 = hfr->vertex();
		//assert(!(v1->is_feature() && v2->is_feature() && v3->is_feature()));
#if 1
		bool is_on_feature = false;
		if (!v1->is_feature() && (v2->is_feature() || v3->is_feature()))
		{
			is_on_feature = true;
			glColor3d(v1->vertex_pixel_color().x(), v1->vertex_pixel_color().y(), v1->vertex_pixel_color().z());
		}
		else if (!v2->is_feature() && (v1->is_feature() || v3->is_feature()))
		{
			is_on_feature = true;
			glColor3d(v2->vertex_pixel_color().x(), v2->vertex_pixel_color().y(), v2->vertex_pixel_color().z());
		}
		else if (!v3->is_feature() && (v1->is_feature() || v2->is_feature()))
		{
			is_on_feature = true;
			glColor3d(v3->vertex_pixel_color().x(), v3->vertex_pixel_color().y(), v3->vertex_pixel_color().z());
		}
		if (is_on_feature)
		{
			glVertex2d(v1->point().x(), v1->point().y());
			glVertex2d(v2->point().x(), v2->point().y());
			glVertex2d(v3->point().x(), v3->point().y());
		}
		else
#endif
		{
			glColor3d(v1->vertex_pixel_color().x(), v1->vertex_pixel_color().y(), v1->vertex_pixel_color().z());
			glVertex2d(v1->point().x(), v1->point().y());
			glColor3d(v2->vertex_pixel_color().x(), v2->vertex_pixel_color().y(), v2->vertex_pixel_color().z());
			glVertex2d(v2->point().x(), v2->point().y());
			glColor3d(v3->vertex_pixel_color().x(), v3->vertex_pixel_color().y(), v3->vertex_pixel_color().z());
			glVertex2d(v3->point().x(), v3->point().y());
		}
	}
	glEnd();
}

void draw_feature_line_sample(vector<vector<RGBPoint>> flsp)
{
#if 0
	glLineWidth(3.0);
	glColor3d(0.0, 0.0, 0.5);
	for (int i = 0; i < flsp.size(); i++)
	{
		glBegin(GL_LINE_LOOP);
		for (int j = 0; j < flsp[i].size(); j++)
		{
			glVertex3d(flsp[i][j].x(), flsp[i][j].y(), 0.0);
		}
		glEnd();
	}
#endif

	glLineWidth(4.0);
	glColor3d(0.0, 0.8, 0.0);
	for (int i = 0; i < flsp.size(); i++)
	{
		glBegin(GL_LINES);
		for (int j = 0; j < flsp[i].size()-1; j++)
		{
			//double grey_ = (flsp[i][j].z() - IMAGEGRAYMIN) / (IMAGEGRAYMAX - IMAGEGRAYMIN);
			//glColor3d(grey_, grey_, grey_);
			glVertex3d(flsp[i][j].x, flsp[i][j].y, 0.0);
			glVertex3d(flsp[i][j+1].x, flsp[i][j+1].y, 0.0);
		}
		glEnd();
	}
}

void draw_sample_image_byfacets(vector<vector<pair<int, bool>>> facets, vector<RGBPoint> values_)
{
#if 0
	glBegin(GL_TRIANGLES);
	for (auto fit = cdt_.faces_begin(); fit != cdt_.faces_end(); fit++)
	{
		for (int i = 0; i<3; i++)
		{
			Point_3 color_ = values_[fit->vertex(i)->get_associated_index()];
			double grey_ = (color_.z() - IMAGEGRAYMIN) / (IMAGEGRAYMAX - IMAGEGRAYMIN);
			glColor3d(grey_, grey_, grey_);
			glVertex3d(values_[fit->vertex(i)->get_associated_index()].x(),
				values_[fit->vertex(i)->get_associated_index()].y(),
				0.0);
		}
	}
	glEnd();
#endif

#if 0
	for (auto fit = cdt_.faces_begin(); fit != cdt_.faces_end(); fit++)
	{
		glBegin(GL_TRIANGLES);
		/*assert(!(fit->vertex(0)->get_new_status() &&
		fit->vertex(1)->get_new_status() &&
		fit->vertex(2)->get_new_status()));*/
		bool is_on_feature = false;
		if (!fit->vertex(0)->get_new_status() &&
			(fit->vertex(1)->get_new_status() || fit->vertex(2)->get_new_status()))
		{
			is_on_feature = true;
			Point_3 color_ = values_[fit->vertex(0)->get_associated_index()];
			double grey_ = (color_.z() - IMAGEGRAYMIN) / (IMAGEGRAYMAX - IMAGEGRAYMIN);
			glColor3d(grey_, grey_, grey_);
		}
		else if (!fit->vertex(1)->get_new_status() &&
			(fit->vertex(0)->get_new_status() || fit->vertex(2)->get_new_status()))
		{
			is_on_feature = true;
			Point_3 color_ = values_[fit->vertex(1)->get_associated_index()];
			double grey_ = (color_.z() - IMAGEGRAYMIN) / (IMAGEGRAYMAX - IMAGEGRAYMIN);
			glColor3d(grey_, grey_, grey_);
		}
		else if (!fit->vertex(2)->get_new_status() &&
			(fit->vertex(0)->get_new_status() || fit->vertex(1)->get_new_status()))
		{
			is_on_feature = true;
			Point_3 color_ = values_[fit->vertex(2)->get_associated_index()];
			double grey_ = (color_.z() - IMAGEGRAYMIN) / (IMAGEGRAYMAX - IMAGEGRAYMIN);
			glColor3d(grey_, grey_, grey_);
		}
		if (is_on_feature)
		{
			glVertex2d(values_[fit->vertex(0)->get_associated_index()].x(),
				values_[fit->vertex(0)->get_associated_index()].y());
			glVertex2d(values_[fit->vertex(1)->get_associated_index()].x(),
				values_[fit->vertex(1)->get_associated_index()].y());
			glVertex2d(values_[fit->vertex(2)->get_associated_index()].x(),
				values_[fit->vertex(2)->get_associated_index()].y());
		}
		else
		{
			double grey_0 = (values_[fit->vertex(0)->get_associated_index()].z() - IMAGEGRAYMIN) / (IMAGEGRAYMAX - IMAGEGRAYMIN);
			glColor3d(grey_0, grey_0, grey_0);
			glVertex2d(values_[fit->vertex(0)->get_associated_index()].x(),
				values_[fit->vertex(0)->get_associated_index()].y());

			double grey_1 = (values_[fit->vertex(1)->get_associated_index()].z() - IMAGEGRAYMIN) / (IMAGEGRAYMAX - IMAGEGRAYMIN);
			glColor3d(grey_1, grey_1, grey_1);
			glVertex2d(values_[fit->vertex(1)->get_associated_index()].x(),
				values_[fit->vertex(1)->get_associated_index()].y());

			double grey_2 = (values_[fit->vertex(2)->get_associated_index()].z() - IMAGEGRAYMIN) / (IMAGEGRAYMAX - IMAGEGRAYMIN);
			glColor3d(grey_2, grey_2, grey_2);
			glVertex2d(values_[fit->vertex(2)->get_associated_index()].x(),
				values_[fit->vertex(2)->get_associated_index()].y());
		}
		glEnd();
	}
#endif

#if 1
	for (int i = 0;i<facets.size();i++)
	{
		glBegin(GL_TRIANGLES);
		bool is_on_feature = false;
		if (!facets[i][0].second &&(facets[i][1].second || facets[i][2].second))
		{
			is_on_feature = true;
			glColor3d(values_[facets[i][0].first].r, values_[facets[i][0].first].g, values_[facets[i][0].first].b);
		}
		else if (!facets[i][1].second && (facets[i][0].second || facets[i][2].second))
		{
			is_on_feature = true;
			glColor3d(values_[facets[i][1].first].r, values_[facets[i][1].first].g, values_[facets[i][1].first].b);
		}
		else if (!facets[i][2].second && (facets[i][0].second || facets[i][1].second))
		{
			is_on_feature = true;
			glColor3d(values_[facets[i][2].first].r, values_[facets[i][2].first].g, values_[facets[i][2].first].b);
		}
		if (is_on_feature)
		{
			glVertex2d(values_[facets[i][0].first].x,values_[facets[i][0].first].y);
			glVertex2d(values_[facets[i][1].first].x,values_[facets[i][1].first].y);
			glVertex2d(values_[facets[i][2].first].x,values_[facets[i][2].first].y);
		}
		else
		{
			glColor3d(values_[facets[i][0].first].r, values_[facets[i][0].first].g, values_[facets[i][0].first].b);
			glVertex2d(values_[facets[i][0].first].x, values_[facets[i][0].first].y);

			glColor3d(values_[facets[i][1].first].r, values_[facets[i][1].first].g, values_[facets[i][1].first].b);
			glVertex2d(values_[facets[i][1].first].x, values_[facets[i][1].first].y);

			glColor3d(values_[facets[i][2].first].r, values_[facets[i][2].first].g, values_[facets[i][2].first].b);
			glVertex2d(values_[facets[i][2].first].x, values_[facets[i][2].first].y);
		}
		glEnd();
	}
#endif

#if 0
	glColor3f(0.4, 0.4, 0.4);
	glLineWidth(0.5f);
	for (int i = 0; i<facets.size(); i++)
	{
		glBegin(GL_LINE_LOOP);
		glVertex2d(values_[facets[i][0].first].x(), values_[facets[i][0].first].y());
		glVertex2d(values_[facets[i][1].first].x(), values_[facets[i][1].first].y());
		glVertex2d(values_[facets[i][2].first].x(), values_[facets[i][2].first].y());
		glEnd();
}
#endif

}

void draw_sample_image_byfacets_lineloop(vector<vector<pair<int, bool>>> facets, vector<RGBPoint> values_)
{
#if 1
	glColor3f(0.4, 0.4, 0.4);
	glLineWidth(0.5f);
	for (int i = 0; i<facets.size(); i++)
	{
		glBegin(GL_LINE_LOOP);
		glVertex2d(values_[facets[i][0].first].x, values_[facets[i][0].first].y);
		glVertex2d(values_[facets[i][1].first].x, values_[facets[i][1].first].y);
		glVertex2d(values_[facets[i][2].first].x, values_[facets[i][2].first].y);
		glEnd();
	}
#endif

}

void draw_resampled_image_and_knottri(QImage image_,Mesh *mesh_)
{
	int lw = image_.width();
	int lh = image_.height();
	double m_dx = IMAGEWIDTHSIZE;
	double pixel_l = 2.0*IMAGEWIDTHSIZE / lw;
	double m_dy = pixel_l * lh / 2.0;

	if (mesh_ != NULL)
	{
		glColor3f(0.2, 0.0, 0.0);
		Mesh::Face_iterator fit = mesh_->facets_begin();
		for (; fit != mesh_->facets_end(); fit++)
		{
			Mesh::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
			glBegin(GL_LINE_LOOP);
			do
			{
				double x = hc->vertex()->point().x() * 2 * m_dx - m_dx;
				double y = hc->vertex()->point().y() * 2 * m_dy - m_dy;
				glVertex2d(x, y);
			} while (++hc != hs);
			glEnd();
		}
	}

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	for (unsigned int i = 0; i < lw; i++)
	{
		double x = -m_dx + (double(i) + 0.5)*pixel_l;
		for (unsigned int j = 0; j < lh; j++)
		{
			double y = -m_dy + (double(j) + 0.5)*pixel_l;
			QRgb color_ = image_.pixel(i, lh - 1 - j);
			glColor3d(qRed(color_) / 255.0, qGreen(color_) / 255.0, qBlue(color_) / 255.0);
			draw_quad(x, y, pixel_l);
		}
	}
}


void SamplingPoint2dInTriangleUniformly(const vector<Point_2> triangle, double insert_rate, vector<Point_2> &triangle_interior_points)
{
	if (triangle.size() != 3)
	{
		return;
	}
	triangle_interior_points.clear();
	Point_2 p1 = triangle[0], center = triangle[1], p2 = triangle[2];
	double d_j = 1.0;
	for (int j = 1; j < int(insert_rate); j++)
	{
		double lamda0 = d_j / insert_rate;
		double d_k = 1.0;
		for (int k = 1; k < int(insert_rate) - j; k++)
		{
			double lamda1 = d_k / insert_rate;
			double lamda2 = 1.0 - lamda0 - lamda1;
			double x = lamda0 * center.x() + lamda1 * p1.x() + lamda2 * p2.x();
			double y = lamda0 * center.y() + lamda1 * p1.y() + lamda2 * p2.y();
			Point_2 temp(x, y);
			triangle_interior_points.push_back(temp);
			d_k = d_k + 1.0;
		}
		d_j += 1.0;
	}
}

void SamplingPoint2dInTriangleCGAL(const vector<Point_2> triangle, double cgal_insert_rate, vector<Point_2> &triangle_interior_points)
{
	CDT_Refine cdt_temp;
	vector<Point_2> points;
	points.clear();
	points.push_back(triangle[0]);
	points.push_back(triangle[1]);
	points.push_back(triangle[2]);
	for (int i = 0; i < points.size(); i++)
	{
		CDT_Refine::Vertex_handle v_h_1, v_h_2;
		v_h_1 = cdt_temp.insert(points[i]);
		v_h_2 = cdt_temp.insert(points[(i + 1) % points.size()]);
		cdt_temp.insert_constraint(v_h_1, v_h_2);
	}

	CGAL::refine_Delaunay_mesh_2(cdt_temp, Criteria(0.125, cgal_insert_rate));

	CDT_Refine::Vertex_iterator vit;
	for (vit = cdt_temp.vertices_begin(); vit != cdt_temp.vertices_end(); vit++)
	{
		Point_2 p(vit->point().x(), vit->point().y());
		if (CGAL::bounded_side_2(triangle.begin(), triangle.end(), p) == CGAL::ON_BOUNDED_SIDE)
		{
			triangle_interior_points.push_back(p);
		}
	}
}

void SamplingPoint2dInLineUniformly(const Point_2 p1, const Point_2 p2, double insert_rate, vector<Point_2> &sample_interior_points)//without two side
{
	sample_interior_points.clear();
	double d_i = 0.0;
	for (int i = 0; i < int(insert_rate + 0.5)+1; i++)
	{
		double lamda2 = d_i / (insert_rate);
		double lamda1 = 1.0 - lamda2;
		double x = lamda1 * p1.x() + lamda2 * p2.x();
		double y = lamda1 * p1.y() + lamda2 * p2.y();
		Point_2 temp(x, y);
		sample_interior_points.push_back(temp);
		d_i = d_i + 1.0;
	}
}

void SamplingPoint2dInPolygonUniform(vector<Point_2> polygon, double sample_rate, vector<Point_2> &sample_points)//note that points must be in order
{
	sample_points.clear();
	if (polygon.size() < 3)
	{
		return;
	}
	Point_2 center = ComputePolygonCentroid(polygon);
	sample_points.push_back(center);
	for (int i = 0; i < polygon.size(); i++)
	{
		Point_2 p1 = polygon[i];
		Point_2 p2 = polygon[(i + 1) % polygon.size()];
		double d_j = 0.0;
		for (int j = 0; j < int(sample_rate); j++)
		{
			double lamda0 = d_j / sample_rate;
			double d_k = 0.0;
			for (int k = 0; k < int(sample_rate) - j; k++)
			{
				double lamda1 = d_k / sample_rate;
				double lamda2 = 1.0 - lamda0 - lamda1;
				double x = lamda0 * center.x() + lamda1 * p1.x() + lamda2 * p2.x();
				double y = lamda0 * center.y() + lamda1 * p1.y() + lamda2 * p2.y();
				Point_2 temp(x, y);
				sample_points.push_back(temp);
				d_k += 1.0;
			}
			d_j += 1.0;
		}
	}
}

Point_2 ComputePolygonCentroid(vector<Point_2> polygon)
{
	Point_2 center_point;
	double x = 0.0, y = 0.0;
	for (int i = 0; i < polygon.size(); i++)
	{
		x += polygon[i].x();
		y += polygon[i].y();
	}
	center_point = { x / polygon.size(), y / polygon.size() };
	return center_point;
}

Point_2 ComputePolygonCentroid(Polygon_2 polygon)
{
	Point_2 center_point;
	double x = 0.0, y = 0.0;
	for (int i = 0; i < polygon.size(); i++)
	{
		x += polygon[i].x();
		y += polygon[i].y();
	}
	center_point = { x / polygon.size(), y / polygon.size() };
	return center_point;
}

void SamplingPoint2dInPolygonCGAL(vector<Point_2> sample_domain, double basis_cgal_sample_rate, CDT_Refine &cdt_temp)
{
	vector<double> box_2d = return_box2d_from_vector_point(sample_domain);
	for (int it_out = 0; it_out < sample_domain.size(); it_out++)
	{
		CDT_Refine::Vertex_handle v_h_1, v_h_2;
		v_h_1 = cdt_temp.insert(sample_domain[it_out]);
		v_h_2 = cdt_temp.insert(sample_domain[(it_out + 1) % sample_domain.size()]);
		cdt_temp.insert_constraint(v_h_1, v_h_2);
	}
	double length_ratio = (std::max)(box_2d[0] - box_2d[2], box_2d[1] - box_2d[3]) * basis_cgal_sample_rate;
	CGAL::refine_Delaunay_mesh_2(cdt_temp, Criteria(0.125, length_ratio));

	CDT_Refine::Face_iterator f_it;
	for (f_it = cdt_temp.faces_begin(); f_it != cdt_temp.faces_end(); f_it++)
	{
		if (!f_it->is_in_domain())
		{
			cdt_temp.delete_face(f_it);
		}
	}
}

vector<double> return_box2d_from_vector_point(vector<Point_2> points_2d)
{
	vector<Point_2>  polygon = points_2d;
	vector<double>	quaternion;
	double max_x = polygon[0].x(), max_y = polygon[0].y();
	double min_x = polygon[0].x(), min_y = polygon[0].y();
	for (unsigned i = 0; i < polygon.size(); i++)
	{
		if (polygon[i].x() > max_x)
		{
			max_x = polygon[i].x();
		}
		if (polygon[i].x() < min_x)
		{
			min_x = polygon[i].x();
		}
		if (polygon[i].y() > max_y)
		{
			max_y = polygon[i].y();
		}
		if (polygon[i].y() < min_y)
		{
			min_y = polygon[i].y();
		}
	}
	quaternion.push_back(min_x);
	quaternion.push_back(max_x);
	quaternion.push_back(min_y);
	quaternion.push_back(max_y);
	return quaternion;
}
