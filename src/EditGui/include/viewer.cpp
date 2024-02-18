#include "viewer.h"
#include "Assistant/mean_value_coord.h"

namespace EDITGUI {

	using namespace std;
	using namespace qglviewer;

#define MAX_CHAR       128

	Viewer::Viewer(int window_typ)
	{
		window_type = window_typ;

		editing_authoring = NULL;

		do_draw_cp_e = false;
		do_draw_feature = false;
		do_draw_cp_p = false;
		do_draw_cpm = false;
		do_sync_other_viewers = true;

		do_selecting_ = false;		
		do_adding_ = false;
		do_deleting_ = false;

		do_insert_points = false;
		do_insert_edges = false;

		do_select_feapolygon = false;
		do_made_polygon = false;

		//edit		
		do_edit_color_ = false;
		do_edit_block_color_ = false;
		do_edit_slider_color_ = false;
		do_edit_color_channel_ = false;
		do_edit_shape_ = false;
		do_select_edit_objects = false;

		do_cwarp_shape = false;
		cwarp_radius = 180;
		cwarp_strength_ = 0.3;

		do_rotate_scene = false;
		do_translation_scene = true;

		intersaction_status = NONE;
		previous_status = NONE;

		current_position_ofselect = QPoint(0, 0);
		start_position_ofselect = QPoint(0, 0);
		
		brush_size_ = 2;
		pixel_screen_width_ = 20;

		setAutoFillBackground(false);
		bBox = Bbox_3(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5);
	}

	Viewer::~Viewer()
	{
		if (!constraints)
		{
			delete constraints;
			constraints = NULL;
		}
	}

	void Viewer::set_scene(Bbox_3 &box)
	{
		Point_3 p1(box.xmax(), box.ymax(), box.zmax());
		Point_3 p2(box.xmin(), box.ymin(), box.zmin());
		Vector_3 v(p2 - p1);
		v = v / 2;
		Point_3 center = p1 + v;
		radius = sqrt(v.squared_length());
		setSceneCenter(qglviewer::Vec(center.x(), center.y(), center.z()));
		//setSceneCenter(qglviewer::Vec(0,0,0));
		setSceneRadius(0.7*radius);
		showEntireScene();
		//std::cout << "set scene \n";
	}

	void Viewer::paintEvent(QPaintEvent *event)
	{
		Q_UNUSED(event)
			QPainter painter;
		painter.begin(this);
		painter.setRenderHint(QPainter::Antialiasing);
		painter.end();

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		//camera()->getProjectedCoordinatesOf();
		do_rotate_scene = false;
		do_translation_scene = true;

		////brush width
		//double in0[3] = { 0,0,0 };
		//double inp[3] = { 0.5 / 500,0.5 / 500,0 };
		//double out0[3] = { 0,0,0 };
		//double outp[3] = { 0,0,0 };
		//camera()->getProjectedCoordinatesOf(in0, out0);
		//camera()->getProjectedCoordinatesOf(inp, outp);
		//pixel_screen_width_ = abs(outp[0] - out0[0]);

		if (editing_authoring)
		{
			if (window_type == OPERATION_W)
			{
				double q1, q2, q3, q4;
				camera()->frame()->getOrientation(q1, q2, q3, q4);
				vector<double> orient = { q1, q2, q3, q4 };
				double p1, p2, p3;
				camera()->frame()->getPosition(p1, p2, p3);
				vector<double> pos = { p1,p2,p3 };
				vector<vector<double>> pos_orient;
				pos_orient.push_back(pos);
				pos_orient.push_back(orient);
				editing_authoring->SetCameraPosAndOrientation(pos_orient);
			}
			else if (window_type != OPERATION_W && do_sync_other_viewers)
			{
				vector<vector<double>>  pos_orientation;
				editing_authoring->GetCameraPosAndOrientation(pos_orientation);
				if (!pos_orientation.empty())
				{
					camera()->frame()->setPosition(pos_orientation[0][0], pos_orientation[0][1], pos_orientation[0][2]);
					//camera()->frame()->setOrientation(Quaternion(pos_orientation[1][0], pos_orientation[1][1], pos_orientation[1][2], pos_orientation[1][3]));
					do_rotate_scene = false;
					do_translation_scene = false;

					editing_authoring->is_fixed_model_view() = true;
				}
			}
		}

		if (do_rotate_scene)
		{
			constraints->setRotationConstraintType(AxisPlaneConstraint::FREE);
		}
		else
		{
			constraints->setRotationConstraintType(AxisPlaneConstraint::FORBIDDEN);
		}
		if (do_translation_scene)
		{
			constraints->setTranslationConstraintType(AxisPlaneConstraint::FREE);
		}
		else
		{
			constraints->setTranslationConstraintType(AxisPlaneConstraint::FORBIDDEN);
		}

		dir = qglviewer::Vec(0.5, 0.5, 0.5);
		constraints->setRotationConstraintDirection(dir);
		constraints->setTranslationConstraintDirection(dir);

		if (intersaction_status == TRANSLATION)
			setCursor(Qt::ClosedHandCursor);
		else if (intersaction_status == SCALE)
		{
			setCursor(Qt::SizeAllCursor);
		}
		else if (intersaction_status == COLOR_BLOCK ||
			intersaction_status == COLOR_SLIDER)
		{
			setCursor(Qt::PointingHandCursor);
		}
		else if (intersaction_status >=RED_CHANNEL && intersaction_status <= BLUE_CHANNEL)
		{
			setCursor(Qt::SizeVerCursor);
		}
		else if (intersaction_status >= SELFPOLYGON && intersaction_status <= SQUARE ||
			intersaction_status >= CWARP_CWROTATE && intersaction_status <= CWARP_TRANS ||
			intersaction_status >= INSERT_P)
		{
			setCursor(Qt::CrossCursor);
		}
		else
		{
			setCursor(Qt::ArrowCursor);
		}		

		// Classical 3D drawing, usually performed by paintGL().
		preDraw();
		draw();
		postDraw();

		// Restore OpenGL state
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();

		//swapBuffers();
	}


	void Viewer::draw()
	{
		makeCurrent();

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();

		glClearColor(1.0, 1.0, 1.0, 1.0);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);

		glShadeModel(GL_SMOOTH);
		glEnable(GL_LINE_SMOOTH);
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_POLYGON_SMOOTH);


		if (editing_authoring != NULL)
		{
			//test
			if (0)
			{
				QImage ima_;
				QString filename = "E:/Projects/ImageApproxiTCB/ImageApproxiPub/build16/bin/Results/face/face.png";
				ima_.load(filename);
				draw_image(&ima_);
			}
			if (do_draw_cp_e)
			{
				if (editing_authoring->is_editing()) {
					EditCM* cm_ = editing_authoring->get_control_mesh();
					draw_control_mesh_edges(cm_, 32);
				}
				else
				{
					AuthoringCM* cm_ = editing_authoring->get_authoring_mesh();
					draw_control_mesh_edges(cm_, 32);
				}
			}		
			if (do_draw_cpm)
			{
				AuthoringCM* cm_ = editing_authoring->get_authoring_mesh();
				draw_features(cm_);
				draw_control_mesh_edges(&realCm, cm_);
			}
			if (do_draw_feature)
			{
				if (!editing_authoring->is_editing()) {
					AuthoringCM* cm_ = editing_authoring->get_authoring_mesh();
					draw_features(cm_);
				}
			}

			if (0)
			{
				std::vector<Point_2> bound = { Point_2(-0.5,-0.5),Point_2(-0.5,0.5),
				Point_2(0.5,0.5),Point_2(0.5,-0.5) };
				for (int i = 0; i < 4; i++)
				{
					glLineWidth(3.0f);
					glColor3f(0, 0, 0);
					glBegin(GL_LINES);
					glVertex2d(bound[i].x(), bound[i].y());
					glVertex2d(bound[(i + 1) % 4].x(), bound[(i + 1) % 4].y());
					glEnd();
				}
			}
		}

		if (!selected_pts_index_.empty() ||
			!selected_featureploy_index_.empty() || !color_edit_region_.empty())
		{
			if (editing_authoring->is_editing()) {
				EditCM* cm_ = editing_authoring->get_control_mesh();
				CPs	cps = editing_authoring->get_cps();
				draw_selected_elements(intersaction_status, cm_, cps);
			}
			else
			{
				//AuthoringCM* cm_ = editing_->get_authoring_mesh();
				//draw_selected_elements(intersaction_status, cm_);
			}
		}

		if (do_edit_slider_color_)
		{
			QPainter *paint = new QPainter;
			paint->begin(this);
			paint->setPen(QPen(Qt::blue, 4, Qt::DashLine));
			//paint->setBrush(QBrush(Qt::red, Qt::SolidPattern));
			QVector<QPoint> line_;
			line_.push_back(start_position_ofselect);
			line_.push_back(current_position_ofselect);
			paint->drawLines(line_);
			paint->end();
		}

		if (editing_authoring != NULL)
		{
			if (do_draw_cp_p)
			{		
				if (editing_authoring->is_editing()) {
					EditCM* cm_ = editing_authoring->get_control_mesh();
					draw_control_mesh_points(cm_, 32);
				}
				else
				{
					AuthoringCM* cm_ = editing_authoring->get_authoring_mesh();
					draw_control_mesh_points(cm_, 32);
				}
			}		
		}

		if (do_selecting_ || do_select_feapolygon 
			|| do_edit_color_ || do_select_edit_objects)
		{
			PickVert(intersaction_status);
			//BrushVert();			
			drawXORRect(intersaction_status);
		}	

		if (selfSelected_polygon.size() > 0 && intersaction_status == SELFPOLYGON ||
			(intersaction_status == TRANSLATION || intersaction_status == SCALE))
		{
			drawPolygon(intersaction_status);
		}

		if (!inserted_points.empty() && intersaction_status >= INSERT_P && intersaction_status <= INSERT_CREASE_SHAPE)
		{
			drawInsertedElements();
		}

		if (!inserted_edges.empty() && intersaction_status >= INSERT_P && intersaction_status <= INSERT_CREASE_SHAPE)
		{
			drawInsertedEdges();
		}

		if (do_cwarp_shape)
		{
			if (intersaction_status == CWARP_TRANS)
			{
				draw_cycle(current_position_ofselect);
			}
			else
			{
				draw_cycle(start_position_ofselect);
			}	
		}

		glShadeModel(GL_FLAT);

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}

	void Viewer::init()
	{
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		// Restore previous viewer state.
		restoreStateFromFile();

		setMouseBinding(Qt::ControlModifier, Qt::RightButton, NO_CLICK_ACTION);
		setMouseBinding(Qt::ControlModifier, Qt::LeftButton, NO_CLICK_ACTION);
		setMouseBinding(Qt::NoModifier, Qt::LeftButton, NO_CLICK_ACTION);

		//setWheelBinding(Qt::Key_I, Qt::NoModifier, CAMERA, ZOOM);

		constraints = new WorldConstraint();
		camera()->frame()->setConstraint(constraints);
		camera()->setOrientation(Quaternion());
		camera()->setZNearCoefficient(0.00001);

		glClearColor(1.0, 1.0, 1.0, 1.0);
	}

	void Viewer::draw_cycle(QPoint center)
	{
		int radius = cwarp_radius;
		QPainter *paint = new QPainter;
		paint->begin(this);
		paint->setPen(QPen(Qt::black, 4, Qt::DashLine));
		//paint->setBrush(QBrush(Qt::red, Qt::SolidPattern));
		paint->drawEllipse(center.x()- radius/2, center.y()- radius/2, radius, radius);
		paint->end();
	}

	void Viewer::keyPressEvent(QKeyEvent *e)
	{
		const Qt::KeyboardModifiers modifiers = e->modifiers();
		bool handled = false;
		if ((e->key() == Qt::Key_R) && (modifiers == Qt::ControlModifier))
		{
			handled = true;
			do_rotate_scene = !do_rotate_scene;
		}
		else if ((e->key() == Qt::Key_T) && (modifiers == Qt::ControlModifier))
		{
			handled = true;
			do_translation_scene = !do_translation_scene;
		}
		else if ((e->key() == Qt::Key_P) && (modifiers == Qt::ControlModifier))
		{
			handled = true;
			QString fo("current.author");
			editing_authoring->save_authoring_data(&fo);
			Image_Approximation appimage;
			CMInfo realCmn;
			appimage.gene_control_mesh(fo, realCmn);
			realCm = realCmn;

			AuthoringCM* cm_ = editing_authoring->get_authoring_mesh();
			for (auto it = cm_->faces.begin(); it != cm_->faces.end(); it++)
			{
				for (int i = 0; i < 3; i++)
				{
					double r_ = 0.5 * (cm_->vertices[it->face[i]].cp.r + cm_->vertices[it->face[(i + 1) % 3]].cp.r);
					double g_ = 0.5 * (cm_->vertices[it->face[i]].cp.g + cm_->vertices[it->face[(i + 1) % 3]].cp.g);
					double b_ = 0.5 * (cm_->vertices[it->face[i]].cp.b + cm_->vertices[it->face[(i + 1) % 3]].cp.b);
					double x_ = 0.5 * (cm_->vertices[it->face[i]].cp.x + cm_->vertices[it->face[(i + 1) % 3]].cp.x);
					double y_ = 0.5 * (cm_->vertices[it->face[i]].cp.y + cm_->vertices[it->face[(i + 1) % 3]].cp.y);

					for (auto vit = realCm.vertices.begin(); vit != realCm.vertices.end(); vit++)
					{
						if (abs(vit->vertex.x - x_) < 1e-5 && abs(vit->vertex.y - y_) < 1e-5)
						{
							vit->vertex.r = r_;
							vit->vertex.g = g_;
							vit->vertex.b = b_;
						}
					}					
				}
			}
			std::set<int> v_seg;
			for (auto it = cm_->edges.begin(); it != cm_->edges.end(); it++)
			{
				v_seg.insert(it->first);
				v_seg.insert(it->second);
			}
			std::cout << "number of knots on feature: " << v_seg.size() << "\n";
			std::cout << "number of knots: " << cm_->vertices.size()- v_seg.size() /2<< "\n";
			for (auto it = cm_->vertices.begin(); it != cm_->vertices.end(); it++)
			{
				double x_ = it->cp.x;
				double y_ = it->cp.y;
				for (auto vit = realCm.vertices.begin(); vit != realCm.vertices.end(); vit++)
				{
					if (abs(vit->vertex.x - x_) < 1e-5 && abs(vit->vertex.y - y_) < 1e-5)
					{
						vit->vertex.r = it->cp.r;
						vit->vertex.g = it->cp.g;
						vit->vertex.b = it->cp.b;
					}
				}
			}
			std::set<int> vs;
			for (auto eit = realCm.edges.begin(); eit != realCm.edges.end(); eit++) {
				vs.insert(eit->first());
				vs.insert(eit->second());
			}
			std::cout << "number of cp: " << vs.size() - v_seg.size() / 2 <<"\n";
			do_draw_cpm = true;
		}
		else if ((e->key() == Qt::Key_P) && (modifiers == Qt::AltModifier))
		{
			handled = true;
			do_draw_cpm = !do_draw_cpm;
		}
		else if ((e->key() == Qt::Key_I) && (modifiers == Qt::ControlModifier))
		{
			handled = true;
			editing_authoring->set_edit_data(updating_cp_candidates);
			//editing_authoring->interpolated_vecimage(200);
		}
		else if ((e->key() == Qt::Key_Y) && (modifiers == Qt::ControlModifier))
		{
			handled = true;

#if 0
			std::cout << "snapshot\n";
			FILE* fp;
			int state = GL2PS_OVERFLOW, buffsize = 0;

			QString name = OutFileName;
			name.append("inputimage.pdf");
			fp = fopen(name.toLatin1(), "wb");

			GLint viewport[4];
			glGetIntegerv(GL_VIEWPORT, viewport);

			EditCM* cm_ = editing_authoring->get_control_mesh();

			while (state == GL2PS_OVERFLOW) {
				buffsize += 1024 * 1024 * 16;
				gl2psBeginPage("test", "gl2psTestSimple", viewport, GL2PS_PDF, GL2PS_SIMPLE_SORT,
					GL2PS_OCCLUSION_CULL,
					GL_RGBA, 0, NULL, 0, 0, 0, buffsize, fp, name.toLatin1());
				draw_control_mesh_edges(cm_, 32);
				draw_control_mesh_points(cm_, 32);
				state = gl2psEndPage();
			}
			fclose(fp);
#endif // 0
			QString dirnow = OutFileName;
			dirnow.append("inputimage_snap.png");

			setSnapshotFormat("PNG");
			setSnapshotQuality(96);
			saveSnapshot(dirnow);
			/*QPixmap p = this->grab(this->rect());
			if (!p.save(dirnow, "png"))
			{
				cout << "save widget screen failed" << endl;
			}*/
		}
		if (!handled)
		{
			QGLViewer::keyPressEvent(e);
		}

		dir = qglviewer::Vec(0.5, 0.5, 0.5);
		//constraints->setTranslationConstraintDirection(dir);
		constraints->setRotationConstraintDirection(dir);

		update();
	}

	void Viewer::mousePressEvent(QMouseEvent *e)
	{
		float* col_right = editing_authoring->get_authoring_right_color();
		float* col_left = editing_authoring->get_authoring_left_color();

		if (e->buttons() == Qt::LeftButton)
		{
			start_position_ofselect = e->pos();
			current_position_ofselect = start_position_ofselect;

			if (intersaction_status >= INSERT_P && intersaction_status <= INSERT_CREASE_SHAPE)
			{
				if (intersaction_status == INSERT_P)
				{
					if (e->modifiers() == Qt::ControlModifier)
					{
						if (!do_insert_points)
						{
							do_insert_points = true;
						}
						QPoint current_position = e->pos();

						//brush width
						double in0[3] = { 0,0,0 };
						double inp[3] = { 0.01,0.01,0 };
						double out0[3] = { 0,0,0 };
						double outp[3] = { 0,0,0 };
						camera()->getProjectedCoordinatesOf(in0, out0);
						camera()->getProjectedCoordinatesOf(inp, outp);

						double _x = (current_position.x() - out0[0]) / (outp[0] - out0[0]) * 0.01;
						double _y = (current_position.y() - out0[1]) / (outp[1] - out0[1]) * 0.01;

						AuthoringP2 p; p.point2 = Point_2(_x, _y); p.id = inserted_points.size();
						RGBPoint vp; vp.r = col_right[0]; vp.g = col_right[1]; vp.b = col_right[2];
						p.v_positive = vp;
						inserted_points.push_back(p);
					}
				}
				if (intersaction_status == INSERT_CREASE_E || intersaction_status == INSERT_NORMAL_E)
				{
					if (e->modifiers() == Qt::ControlModifier)
					{
						if (!do_insert_edges)
						{
							do_insert_edges = true;
						}
						QPoint current_position = e->pos();

						//brush width
						double in0[3] = { 0,0,0 };
						double inp[3] = { 0.01,0.01,0 };
						double out0[3] = { 0,0,0 };
						double outp[3] = { 0,0,0 };
						camera()->getProjectedCoordinatesOf(in0, out0);
						camera()->getProjectedCoordinatesOf(inp, outp);

						double _x = (current_position.x() - out0[0]) / (outp[0] - out0[0]) * 0.01;
						double _y = (current_position.y() - out0[1]) / (outp[1] - out0[1]) * 0.01;
						AuthoringP2 p; p.point2 = Point_2(_x, _y); p.id = inserted_points.size();
						RGBPoint vp; vp.r = col_right[0]; vp.g = col_right[1]; vp.b = col_right[2];
						RGBPoint vn; vn.r = col_left[0]; vn.g = col_left[1]; vn.b = col_left[2];
						p.v_positive = vp;
						p.v_negative = vn;
						inserted_points.push_back(p);
					}
					else if (e->modifiers() == Qt::NoModifier)
					{
						do_insert_edges = false;
					}
				}
			}
			else if (intersaction_status >= SELFPOLYGON &&intersaction_status <= SQUARE)
			{				
				if (intersaction_status > SELFPOLYGON)
				{
					do_selecting_ = true;
				}
				
				if (intersaction_status == SELFPOLYGON)
				{
					if (e->modifiers() == Qt::ControlModifier)
					{
						if (!do_made_polygon)
						{
							selfSelected_polygon.clear();
							do_made_polygon = true;
						}
						QPoint current_position = e->pos();

						//brush width
						double in0[3] = { 0,0,0 };
						double inp[3] = { 0.01,0.01,0 };
						double out0[3] = { 0,0,0 };
						double outp[3] = { 0,0,0 };
						camera()->getProjectedCoordinatesOf(in0, out0);
						camera()->getProjectedCoordinatesOf(inp, outp);

						double _x = (current_position.x() - out0[0]) / (outp[0] - out0[0])*0.01;
						double _y = (current_position.y() - out0[1]) / (outp[1] - out0[1])*0.01;

						selfSelected_polygon.push_back(Point_2(_x, _y));
						//std::cout << "push back p" << _x << "," << _y << std::endl;

					}					
					else if(e->modifiers() == Qt::AltModifier)
					{
						if (selfSelected_polygon.size()>0)
						{
							selfSelected_polygon.pop_back();
						}					
					}
					else if (e->modifiers() == Qt::NoModifier)
					{
						do_made_polygon = true;
						do_select_feapolygon = true;
					}
				}

				if (intersaction_status == SQUARE || intersaction_status == FEATURE)
				{
					if (e->modifiers() == Qt::ControlModifier)
					{
						do_adding_ = true;
					}
					else if (e->modifiers() == Qt::AltModifier)
					{
						do_deleting_ = true;
					}
				}

				if (intersaction_status == BRUSH)
				{
					if (e->modifiers() == Qt::ControlModifier ||
						e->modifiers() == Qt::NoModifier)
					{
						do_adding_ = true;
					}
					else if (e->modifiers() == Qt::AltModifier)
					{
						do_deleting_ = true;
					}
				}		
			}
			else if (intersaction_status == COLOR)
			{
				do_edit_color_ = true;
			}
			else if (intersaction_status == COLOR_BLOCK)
			{
				do_edit_block_color_ = true;
			}
			else if (intersaction_status == COLOR_SLIDER)
			{
				do_edit_slider_color_ = true;
			}
			else if (intersaction_status >= RED_CHANNEL && intersaction_status <= BLUE_CHANNEL)
			{
				do_edit_color_channel_ = true;
			}
			else if (intersaction_status == TRANSLATION || intersaction_status == SINGLE_CP)
			{
				do_select_edit_objects = true;
				do_edit_shape_ = true;
			}
			else if (intersaction_status == SCALE)
			{
				do_edit_shape_ = true;
			}
			else if (intersaction_status >= CWARP_CWROTATE && intersaction_status <= CWARP_TRANS)
			{
				do_cwarp_shape = true;
			}
			update();
		}
		else if (e->buttons() == Qt::RightButton && e->modifiers() == Qt::ControlModifier)
		{
			if (intersaction_status == INSERT_NORMAL_E || intersaction_status == INSERT_CREASE_E)
			{
				do_insert_points = false;				
				inserted_points.push_back(*inserted_points.begin());
			}
			if (intersaction_status == SELFPOLYGON)
			{
				if (do_made_polygon)
				{
					do_made_polygon = false;
					if (!selfSelected_polygon.empty())
					{
						selfSelected_polygon.push_back(selfSelected_polygon[0]);
					}
				}
				else
				{
					selfSelected_polygon.clear();
				}
				update();
			}
		}
		else
		{
			QGLViewer::mousePressEvent(e);
		}
	}

	void Viewer::mouseMoveEvent(QMouseEvent *e)
	{
		if (e->buttons() == Qt::LeftButton)
		{
			current_position_ofselect = e->pos();
			double in0[3] = { 0,0,0 };
			double inp[3] = { 1.0,1.0,0 };
			double out0[3] = { 0,0,0 };
			double outp[3] = { 0,0,0 };
			camera()->getProjectedCoordinatesOf(in0, out0);
			camera()->getProjectedCoordinatesOf(inp, outp);

			double dx = (current_position_ofselect.x() - start_position_ofselect.x()) / (outp[0] - out0[0]);
			double dy = (current_position_ofselect.y() - start_position_ofselect.y()) / (outp[1] - out0[1]);
			edit_move_ = Vector_2(dx, dy);

			if (intersaction_status >= SELFPOLYGON &&intersaction_status <= SQUARE)
			{			
				if (e->modifiers() == Qt::NoModifier)
				{
					
				}
			}
			else if (intersaction_status >= RED_CHANNEL && intersaction_status <= BLUE_CHANNEL)
			{			
				Point_2 size_ = editing_authoring->get_image_size();
				if (sqrt(dx*dx + dy*dy) > 0.01*IMAGEWIDTHSIZE / size_.x())
				{		
					double dis_ = sqrt(dx*dx + dy*dy) / (IMAGEWIDTHSIZE / size_.x()*size_.y());
					dis_ = dy > 0.0 ? dis_ : -dis_;
					if (intersaction_status == RED_CHANNEL)
					{
						update_channel_operation(RED_CHANNEL, dis_);
					}
					else if (intersaction_status == GREEN_CHANNEL)
					{
						update_channel_operation(GREEN_CHANNEL, dis_);
					}
					else
					{
						update_channel_operation(BLUE_CHANNEL, dis_);
					}
				}
			}
			else if (intersaction_status == TRANSLATION)
			{
				do_select_edit_objects = false;
				if (!deform_polygon.empty())
				{
					for (int i = 0; i < deform_polygon.size(); i++)
					{
						if (deform_polygon[i].bselected)
						{
							deform_polygon[i].deviation = Vector_2(dx, dy);
						}
					}
					Point_2 size_ = editing_authoring->get_image_size();
					if (sqrt(dx*dx + dy*dy) > 0.01*IMAGEWIDTHSIZE / size_.x())
					{
						warp_cps(size_);
					}
				}				
			}
			else if (intersaction_status == SCALE)
			{
				if (!deform_polygon.empty())
				{
					Vector_2 center_(0, 0);
					for (int i = 0;i<deform_polygon.size();i++)
					{
						center_ += deform_polygon[i].origin;
					}
					center_ = center_ / deform_polygon.size();

					for (int i = 0; i < deform_polygon.size(); i++)
					{						
						double xmove_ = (deform_polygon[i].origin - center_).x()*dx;
						double ymove_ = (deform_polygon[i].origin - center_).y()*dy;

						deform_polygon[i].deviation = Vector_2(xmove_, ymove_);
					}
					Point_2 size_ = editing_authoring->get_image_size();
					if (sqrt(dx*dx + dy*dy) > 0.01*IMAGEWIDTHSIZE / size_.x())
					{
						warp_cps(size_);
					}
				}
			}
			else if (intersaction_status == SINGLE_CP)
			{
				do_select_edit_objects = false;
				EditCM *cm_ = editing_authoring->get_control_mesh();
				for (int i = 0; i < cm_->vertices.size(); i++)
				{
					if (!cm_->vertices[i].is_bound && cm_->vertices[i].is_updated)
					{
						if (cm_->vertices[i].is_on_feature)
						{
							cm_->vertices[i].cp_pair.first.x = cm_->vertices[i].previous_pair.first.x + dx;
							cm_->vertices[i].cp_pair.first.y = cm_->vertices[i].previous_pair.first.y + dy;
							cm_->vertices[i].cp.x = cm_->vertices[i].cp_pair.first.x;
							cm_->vertices[i].cp.y = cm_->vertices[i].cp_pair.first.y;
							cm_->vertices[i].cp_pair.second.x = cm_->vertices[i].previous_pair.second.x + dx;
							cm_->vertices[i].cp_pair.second.y = cm_->vertices[i].previous_pair.second.y + dy;
						}
						else
						{
							cm_->vertices[i].cp.x = cm_->vertices[i].previous_cp.x + dx;
							cm_->vertices[i].cp.y = cm_->vertices[i].previous_cp.y + dy;
						}								
					}
				}
			}
			else if (intersaction_status <= CWARP_TRANS && intersaction_status >= CWARP_CWROTATE)
			{
				cwarp_d_radius = (cwarp_radius/2.0) / (outp[0] - out0[0]);
				double cx = (start_position_ofselect.x() - out0[0]) / (outp[0] - out0[0]);
				double cy = (start_position_ofselect.y() - out0[1]) / (outp[1] - out0[1]);

				cwarp_center_ = Vector_2(cx, cy);

				//UWarp operations, ref: [Interactive Image Warping]
				EditCM *cm_ = editing_authoring->get_control_mesh();
				for (int i = 0; i < cm_->vertices.size(); i++)
				{
					if (!cm_->vertices[i].is_bound)
					{
						if (cm_->vertices[i].is_on_feature)
						{
							Vector_2 p_temp(cm_->vertices[i].previous_pair.first.x, cm_->vertices[i].previous_pair.first.y);
							if (sqrt((p_temp - cwarp_center_).squared_length()) <
								cwarp_d_radius)
							{
								Vector_2 dis_(0,0);
								if (intersaction_status == CWARP_TRANS)
								{
									double numerator_ = (pow(cwarp_d_radius, 2) - (p_temp - cwarp_center_).squared_length())
										/ pow(cwarp_d_radius, 2);
									double coeff_ = cwarp_strength_*numerator_;
									dis_ = Vector_2(coeff_*dx, coeff_*dy);
								}
								else if (intersaction_status == CWARP_ROOMIN)
								{
									double in_ = sqrt((p_temp - cwarp_center_).squared_length() / pow(cwarp_d_radius, 2));
									dis_ = cwarp_strength_*((in_ - 1)*(in_ - 1))*
										(p_temp - cwarp_center_)/ sqrt((p_temp - cwarp_center_).squared_length())*
										0.2*sqrt(edit_move_.squared_length());
								}
								else if (intersaction_status == CWARP_ROOMOUT)
								{
									double in_ = sqrt((p_temp - cwarp_center_).squared_length() / pow(cwarp_d_radius, 2));
									dis_ = -cwarp_strength_*((in_ - 1)*(in_ - 1))*
										(p_temp - cwarp_center_) / sqrt((p_temp - cwarp_center_).squared_length())*
										0.2*sqrt(edit_move_.squared_length());
								}
								else if (intersaction_status == CWARP_CWROTATE)
								{
									
								}
								cm_->vertices[i].cp_pair.first.x = cm_->vertices[i].previous_pair.first.x + dis_.x();
								cm_->vertices[i].cp_pair.first.y = cm_->vertices[i].previous_pair.first.y + dis_.y();
								cm_->vertices[i].cp.x = cm_->vertices[i].cp_pair.first.x;
								cm_->vertices[i].cp.y = cm_->vertices[i].cp_pair.first.y;
								cm_->vertices[i].cp_pair.second.x = cm_->vertices[i].previous_pair.second.x + dis_.x();
								cm_->vertices[i].cp_pair.second.y = cm_->vertices[i].previous_pair.second.y + dis_.y();
							}
						}
						else
						{
							Vector_2 p_temp(cm_->vertices[i].previous_cp.x, cm_->vertices[i].previous_cp.y);
							if (sqrt((p_temp - cwarp_center_).squared_length()) <
								cwarp_d_radius)
							{
								Vector_2 dis_;
								if (intersaction_status == CWARP_TRANS)
								{
									double numerator_ = (pow(cwarp_d_radius, 2) - (p_temp - cwarp_center_).squared_length())
										/ pow(cwarp_d_radius, 2);
									double coeff_ = cwarp_strength_*numerator_;
									dis_ = Vector_2(coeff_*dx, coeff_*dy);
								}
								else if (intersaction_status == CWARP_ROOMIN)
								{
									double in_ = sqrt((p_temp - cwarp_center_).squared_length() / pow(cwarp_d_radius, 2));
									dis_ = cwarp_strength_*((in_ - 1)*(in_ - 1))*
										(p_temp - cwarp_center_) / sqrt((p_temp - cwarp_center_).squared_length())*
										0.2*sqrt(edit_move_.squared_length());
								}
								else if (intersaction_status == CWARP_ROOMOUT)
								{
									double in_ = sqrt((p_temp - cwarp_center_).squared_length() / pow(cwarp_d_radius, 2));
									dis_ = -cwarp_strength_*((in_ - 1)*(in_ - 1))*
										(p_temp - cwarp_center_) / sqrt((p_temp - cwarp_center_).squared_length())*
										0.2*sqrt(edit_move_.squared_length());
								}
								cm_->vertices[i].cp.x = cm_->vertices[i].previous_cp.x + dis_.x();
								cm_->vertices[i].cp.y = cm_->vertices[i].previous_cp.y + dis_.y();
							}
						}
					}
				}
			}
			update();
		}		
		else
		{
			QGLViewer::mouseMoveEvent(e);
		}
	}

	void Viewer::mouseReleaseEvent(QMouseEvent *e)
	{
		current_position_ofselect = e->pos();

		if (do_insert_points)
		{
			do_insert_points = false;

			insert_elements(inserted_points);
			inserted_points.clear();
		}
		if (!do_insert_edges && !inserted_points.empty())
		{
			insert_elements(inserted_points);
			if (intersaction_status == INSERT_NORMAL_E)
			{
				inserted_edges.push_back(inserted_points);
			}
			inserted_points.clear();
		}

		if (do_selecting_)
		{
			//test
			/*std::cout << "selected points size:" << current_index_.size() << "consist of ";
			for (auto i = current_index_.begin(); i != current_index_.end(); i++)
			{
				std::cout << *i << "-";
			}
			std::cout << std::endl;*/

			do_selecting_ = false;
		}
		if (do_select_feapolygon)
		{
			do_select_feapolygon = false;
		}
		if (do_adding_)
		{
			do_adding_ = false;
		}
		if (do_edit_color_)
		{
			do_edit_color_ = false;
			colored_selected_pts();
		}
		if (do_edit_block_color_)
		{
			do_edit_block_color_ = false;
			colored_block_pts();
		}
		if (do_edit_slider_color_)
		{
			do_edit_slider_color_ = false;
			colored_slider_pts();
		}
		if (do_edit_color_channel_)
		{
			do_edit_color_channel_ = false;
			channel_edit_selected_pts();
		}
		if (do_select_edit_objects)
		{
			do_select_edit_objects = false;
		}
		if (do_edit_shape_)
		{
			do_edit_shape_ = false;
			if (intersaction_status == SINGLE_CP)
			{
				moved_selected_pts_directly();
			}
			else
			{
				moved_selected_pts();
			}
		}
		if (do_cwarp_shape)
		{
			do_cwarp_shape = false;
			moved_selected_pts_directly();
		}
		if (do_deleting_)
		{
			do_deleting_ = false;
		}
		{
			setCursor(Qt::ArrowCursor);
			QGLViewer::mouseReleaseEvent(e);
		}		
	}

	void Viewer::wheelEvent(QWheelEvent *e)
	{
		if (e->modifiers() == Qt::ControlModifier)
		{
			if (intersaction_status == BRUSH)
			{
				do_selecting_ = true;
				QPoint numDegrees = e->angleDelta() / 8;
				if (!numDegrees.isNull()) {
					QPoint numSteps = numDegrees / 15;
					int brush_size_new = brush_size_ + numSteps.y();
					brush_size_ = std::max(brush_size_new, 1);
					brush_size_ = std::min(brush_size_, 8);
				}
			}
			else if (intersaction_status >= CWARP_CWROTATE &&
				intersaction_status <= CWARP_TRANS)
			{
				do_cwarp_shape = true;
				QPoint numDegrees = e->angleDelta() / 8;
				if (!numDegrees.isNull()) {
					QPoint numSteps = numDegrees / 15;
					int cwarp_radius_new = cwarp_radius + 10 * numSteps.y();
					cwarp_radius = std::max(cwarp_radius_new, 30);
					cwarp_radius = std::min(cwarp_radius, 300);
				}
			}
			update();
			return;
		}
		else
		{
			do_selecting_ = false;
			QGLViewer::wheelEvent(e);
		}		
	}

	void Viewer::colored_slider_pts()
	{
		EditCM *cm_ = editing_authoring->get_control_mesh();
		float *col = editing_authoring->get_edit_color();
		float *col_back = editing_authoring->get_edit_color_backg();

		double in0[3] = { 0,0,0 };
		double inp[3] = { 1.0,1.0,0 };
		double out0[3] = { 0,0,0 };
		double outp[3] = { 0,0,0 };
		camera()->getProjectedCoordinatesOf(in0, out0);
		camera()->getProjectedCoordinatesOf(inp, outp);

		double dx = (current_position_ofselect.x() - out0[0]) / (outp[0] - out0[0]) *(inp[0] - in0[0]) + in0[0];
		double dy = (current_position_ofselect.y() - out0[1]) / (outp[1] - out0[1]) *(inp[1] - in0[1]) + in0[1];
		Point_2 to_(dx, dy);
		dx = (start_position_ofselect.x() - out0[0]) / (outp[0] - out0[0]) *(inp[0] - in0[0]) + in0[0];
		dy = (start_position_ofselect.y() - out0[1]) / (outp[1] - out0[1]) *(inp[1] - in0[1]) + in0[1];
		Point_2 from_(dx, dy);

		Point_2 cen_((from_.x()+to_.x())/2.0, (from_.y() + to_.y()) / 2.0);
		double r_sq = (from_ - to_).squared_length()/4.0;
		set<int> colored_pts;
		for (int i = 0; i < cm_->vertices.size(); i++)
		{
			if (cm_->vertices[i].is_on_feature)
			{
				Point_2 pt(cm_->vertices[i].cp_pair.second.x,
					cm_->vertices[i].cp_pair.second.y);
				if ((pt-cen_).squared_length() < r_sq)
				{
					colored_pts.insert(i);
				}
			}
			else
			{
				Point_2 pt1(cm_->vertices[i].cp.x,
					cm_->vertices[i].cp.y);
				if ((pt1 - cen_).squared_length() < r_sq)
				{
					colored_pts.insert(i);
				}
			}
		}

		for (auto it = colored_pts.begin(); it != colored_pts.end(); it++)
		{
			double rd_, gd_, bd_;			
			Point_2 pt(cm_->vertices[*it].cp.x, cm_->vertices[*it].cp.y);
			double r_ = (pt - from_)*(to_ - from_) / (to_ - from_).squared_length()*(col_back[0] - col[0]) + col[0];
			double g_ = (pt - from_)*(to_ - from_) / (to_ - from_).squared_length()*(col_back[1] - col[1]) + col[1];
			double b_ = (pt - from_)*(to_ - from_) / (to_ - from_).squared_length()*(col_back[2] - col[2]) + col[2];
			double colnow[3] = { r_,g_,b_};

			if (cm_->vertices[*it].is_on_feature)
			{
				rd_ = colnow[0] - cm_->vertices[*it].previous_pair.first.r;
				gd_ = colnow[1] - cm_->vertices[*it].previous_pair.first.g;
				bd_ = colnow[2] - cm_->vertices[*it].previous_pair.first.b;
				cm_->vertices[*it].previous_pair.first.r = colnow[0];
				cm_->vertices[*it].previous_pair.first.g = colnow[1];
				cm_->vertices[*it].previous_pair.first.b = colnow[2];
			}
			else
			{
				rd_ = colnow[0] - cm_->vertices[*it].previous_cp.r;
				gd_ = colnow[1] - cm_->vertices[*it].previous_cp.g;
				bd_ = colnow[2] - cm_->vertices[*it].previous_cp.b;
				cm_->vertices[*it].previous_cp.r = colnow[0];
				cm_->vertices[*it].previous_cp.g = colnow[1];
				cm_->vertices[*it].previous_cp.b = colnow[2];
			}
			cm_->vertices[*it].cp_pair.first.r = colnow[0];
			cm_->vertices[*it].cp_pair.first.g = colnow[1];
			cm_->vertices[*it].cp_pair.first.b = colnow[2];
			cm_->vertices[*it].cp_pair.second.r = colnow[0];
			cm_->vertices[*it].cp_pair.second.g = colnow[1];
			cm_->vertices[*it].cp_pair.second.b = colnow[2];
			cm_->vertices[*it].cp.r = colnow[0];
			cm_->vertices[*it].cp.g = colnow[1];
			cm_->vertices[*it].cp.b = colnow[2];

			if (bAnimated)
			{
				cm_->vertices[*it].deviation.x += 0;
				cm_->vertices[*it].deviation.y += 0;
				cm_->vertices[*it].deviation.r += rd_;
				cm_->vertices[*it].deviation.g += gd_;
				cm_->vertices[*it].deviation.b += bd_;
			}
			else
			{
				cm_->vertices[*it].deviation.x = 0;
				cm_->vertices[*it].deviation.y = 0;
				cm_->vertices[*it].deviation.r = rd_;
				cm_->vertices[*it].deviation.g = gd_;
				cm_->vertices[*it].deviation.b = bd_;
			}		
			updating_cp_candidates.insert(*it);
		}
		if (!bAnimated) {
			editing_authoring->update_edited_vecimage(updating_cp_candidates);
			updating_cp_candidates.clear();
		}
	}

	void Viewer::colored_block_pts()
	{
		EditCM* cm_ = editing_authoring->get_control_mesh();
		float* col = editing_authoring->get_edit_color();

		double in0[3] = { 0,0,0 };
		double inp[3] = { 1.0,1.0,0 };
		double out0[3] = { 0,0,0 };
		double outp[3] = { 0,0,0 };
		camera()->getProjectedCoordinatesOf(in0, out0);
		camera()->getProjectedCoordinatesOf(inp, outp);

		double dx = (current_position_ofselect.x() - out0[0]) / (outp[0] - out0[0]) * (inp[0] - in0[0]) + in0[0];
		double dy = (current_position_ofselect.y() - out0[1]) / (outp[1] - out0[1]) * (inp[1] - in0[1]) + in0[1];
		Point_2 cen_(dx, dy);

		CPs	cps = editing_authoring->get_cps();
		set<int> colored_pts;
		vector<vector<Point_2>> boundarys_;
		for (int i = 0; i < cps.size(); i++)
		{
			vector<Point_2> polygon_origin_;
			bool is_find = false;
			for (int j = 0; j < cps[i].size(); j++)
			{
				polygon_origin_.push_back(cps[i][j].point2);
			}
			if (CGAL::bounded_side_2(polygon_origin_.begin(), polygon_origin_.end(),
				cen_) != CGAL::ON_UNBOUNDED_SIDE)
			{
				is_find = true;
			}
			if (is_find)
			{
				boundarys_.push_back(polygon_origin_);
				screen_outside_points(cm_, boundarys_, colored_pts, BY_CP);
				break;
			}
		}

		for (auto it = colored_pts.begin(); it != colored_pts.end(); it++)
		{
			double rd_, gd_, bd_;
			if (cm_->vertices[*it].is_on_feature)
			{
				rd_ = col[0] - cm_->vertices[*it].previous_pair.first.r;
				gd_ = col[1] - cm_->vertices[*it].previous_pair.first.g;
				bd_ = col[2] - cm_->vertices[*it].previous_pair.first.b;
				cm_->vertices[*it].previous_pair.first.r = col[0];
				cm_->vertices[*it].previous_pair.first.g = col[1];
				cm_->vertices[*it].previous_pair.first.b = col[2];
			}
			else
			{
				rd_ = col[0] - cm_->vertices[*it].previous_cp.r;
				gd_ = col[1] - cm_->vertices[*it].previous_cp.g;
				bd_ = col[2] - cm_->vertices[*it].previous_cp.b;
				cm_->vertices[*it].previous_cp.r = col[0];
				cm_->vertices[*it].previous_cp.g = col[1];
				cm_->vertices[*it].previous_cp.b = col[2];
			}
			cm_->vertices[*it].cp_pair.first.r = col[0];
			cm_->vertices[*it].cp_pair.first.g = col[1];
			cm_->vertices[*it].cp_pair.first.b = col[2];
			cm_->vertices[*it].cp_pair.second.r = col[0];
			cm_->vertices[*it].cp_pair.second.g = col[1];
			cm_->vertices[*it].cp_pair.second.b = col[2];
			cm_->vertices[*it].cp.r = col[0];
			cm_->vertices[*it].cp.g = col[1];
			cm_->vertices[*it].cp.b = col[2];

			if (bAnimated)
			{
				cm_->vertices[*it].deviation.x += 0;
				cm_->vertices[*it].deviation.y += 0;
				cm_->vertices[*it].deviation.r += rd_;
				cm_->vertices[*it].deviation.g += gd_;
				cm_->vertices[*it].deviation.b += bd_;
			}
			else
			{
				cm_->vertices[*it].deviation.x = 0;
				cm_->vertices[*it].deviation.y = 0;
				cm_->vertices[*it].deviation.r = rd_;
				cm_->vertices[*it].deviation.g = gd_;
				cm_->vertices[*it].deviation.b = bd_;
			}
			updating_cp_candidates.insert(*it);
		}
		if (!bAnimated) {
			editing_authoring->update_edited_vecimage(updating_cp_candidates);
			updating_cp_candidates.clear();
		}
	}

	void Viewer::colored_selected_pts()
	{
		EditCM *cm_ = editing_authoring->get_control_mesh();
		float *col = editing_authoring->get_edit_color();
		for (auto it = selected_colored_pts.begin();it!= selected_colored_pts.end();it++)
		{
			double rd_, gd_, bd_;
			if (cm_->vertices[*it].is_on_feature)
			{
				rd_ = col[0] - cm_->vertices[*it].previous_pair.first.r;
				gd_ = col[1] - cm_->vertices[*it].previous_pair.first.g;
				bd_ = col[2] - cm_->vertices[*it].previous_pair.first.b;
				cm_->vertices[*it].previous_pair.first.r = col[0];
				cm_->vertices[*it].previous_pair.first.g = col[1];
				cm_->vertices[*it].previous_pair.first.b = col[2];
			}
			else
			{
				rd_ = col[0] - cm_->vertices[*it].previous_cp.r;
				gd_ = col[1] - cm_->vertices[*it].previous_cp.g;
				bd_ = col[2] - cm_->vertices[*it].previous_cp.b;
				cm_->vertices[*it].previous_cp.r = col[0];
				cm_->vertices[*it].previous_cp.g = col[1];
				cm_->vertices[*it].previous_cp.b = col[2];
			}

			if (bAnimated)
			{
				cm_->vertices[*it].deviation.x += 0;
				cm_->vertices[*it].deviation.y += 0;
				cm_->vertices[*it].deviation.r += rd_;
				cm_->vertices[*it].deviation.g += gd_;
				cm_->vertices[*it].deviation.b += bd_;
			}
			else
			{
				cm_->vertices[*it].deviation.x = 0;
				cm_->vertices[*it].deviation.y = 0;
				cm_->vertices[*it].deviation.r = rd_;
				cm_->vertices[*it].deviation.g = gd_;
				cm_->vertices[*it].deviation.b = bd_;
			}
			updating_cp_candidates.insert(*it);
		}
		if (!bAnimated) {
			editing_authoring->update_edited_vecimage(updating_cp_candidates);
			selected_colored_pts.clear();
			updating_cp_candidates.clear();
		}
	}

	void Viewer::update_channel_operation(int type_, double dis_)
	{
		EditCM *cm_ = editing_authoring->get_control_mesh();
		for (auto it = color_edit_region_.begin(); it != color_edit_region_.end(); it++)
		{
			if (cm_->vertices[*it].is_on_feature)
			{
				if (type_ == RED_CHANNEL)
				{
					double color_ = cm_->vertices[*it].previous_pair.first.r + dis_;
					color_ = std::max(color_, 0.0);
					color_ = std::min(color_, 1.0);
					cm_->vertices[*it].cp_pair.first.r = color_;
					cm_->vertices[*it].cp_pair.second.r = color_;
				}
				else if (type_ == GREEN_CHANNEL)
				{
					double color_ = cm_->vertices[*it].previous_pair.first.g + dis_;
					color_ = std::max(color_, 0.0);
					color_ = std::min(color_, 1.0);
					cm_->vertices[*it].cp_pair.first.g = color_;
					cm_->vertices[*it].cp_pair.second.g = color_;
				}
				else
				{
					double color_ = cm_->vertices[*it].previous_pair.first.b + dis_;
					color_ = std::max(color_, 0.0);
					color_ = std::min(color_, 1.0);
					cm_->vertices[*it].cp_pair.first.b = color_;
					cm_->vertices[*it].cp_pair.second.b = color_;
				}					
			}
			else
			{
				if (type_ == RED_CHANNEL)
				{
					double color_ = cm_->vertices[*it].previous_cp.r + dis_;
					color_ = std::max(color_, 0.0);
					color_ = std::min(color_, 1.0);
					cm_->vertices[*it].cp.r = color_;
				}
				else if (type_ == GREEN_CHANNEL)
				{
					double color_ = cm_->vertices[*it].previous_cp.g + dis_;
					color_ = std::max(color_, 0.0);
					color_ = std::min(color_, 1.0);
					cm_->vertices[*it].cp.g = color_;
				}
				else
				{
					double color_ = cm_->vertices[*it].previous_cp.b + dis_;
					color_ = std::max(color_, 0.0);
					color_ = std::min(color_, 1.0);
					cm_->vertices[*it].cp.b = color_;
				}		
			}
		}
	}

	void Viewer::channel_edit_selected_pts()
	{
		EditCM *cm_ = editing_authoring->get_control_mesh();
		for (auto it = color_edit_region_.begin(); it != color_edit_region_.end(); it++)
		{
			double rd_, gd_, bd_;
			if (cm_->vertices[*it].is_on_feature)
			{
				rd_ = cm_->vertices[*it].cp_pair.first.r - cm_->vertices[*it].previous_pair.first.r;
				gd_ = cm_->vertices[*it].cp_pair.first.g - cm_->vertices[*it].previous_pair.first.g;
				bd_ = cm_->vertices[*it].cp_pair.first.b - cm_->vertices[*it].previous_pair.first.b;
				cm_->vertices[*it].previous_pair.first.r = cm_->vertices[*it].cp_pair.first.r;
				cm_->vertices[*it].previous_pair.first.g = cm_->vertices[*it].cp_pair.first.g;
				cm_->vertices[*it].previous_pair.first.b = cm_->vertices[*it].cp_pair.first.b;
			}
			else
			{
				rd_ = cm_->vertices[*it].cp.r - cm_->vertices[*it].previous_cp.r;
				gd_ = cm_->vertices[*it].cp.g - cm_->vertices[*it].previous_cp.g;
				bd_ = cm_->vertices[*it].cp.b - cm_->vertices[*it].previous_cp.b;
				cm_->vertices[*it].previous_cp.r = cm_->vertices[*it].cp.r;
				cm_->vertices[*it].previous_cp.g = cm_->vertices[*it].cp.g;
				cm_->vertices[*it].previous_cp.b = cm_->vertices[*it].cp.b;
			}
			if (bAnimated)
			{
				cm_->vertices[*it].deviation.x += 0;
				cm_->vertices[*it].deviation.y += 0;
				cm_->vertices[*it].deviation.r += rd_;
				cm_->vertices[*it].deviation.g += gd_;
				cm_->vertices[*it].deviation.b += bd_;
			}
			else
			{
				cm_->vertices[*it].deviation.x = 0;
				cm_->vertices[*it].deviation.y = 0;
				cm_->vertices[*it].deviation.r = rd_;
				cm_->vertices[*it].deviation.g = gd_;
				cm_->vertices[*it].deviation.b = bd_;
			}
			updating_cp_candidates.insert(*it);
		}
		if (!bAnimated) {
			editing_authoring->update_edited_vecimage(updating_cp_candidates);
			updating_cp_candidates.clear();
		}
	}

	void Viewer::moved_selected_pts()
	{
		if (deform_polygon.empty())
		{
			return;
		}

		vector<Point_2> polygon_origin_, polygon_moved;
		for (int i = 0; i < deform_polygon.size(); i++)
		{
			polygon_origin_.push_back(Point_2(deform_polygon[i].origin.x(), deform_polygon[i].origin.y()));
			Vector_2 pm = deform_polygon[i].origin + deform_polygon[i].deviation;
			polygon_moved.push_back(Point_2(pm.x(), pm.y()));
		}

		vector<Point_2> input_data, output_data;
		CPs* cps = editing_authoring->get_cpsP();
		for (auto i = cps->begin(); i != cps->end(); i++)
		{
			for (auto j = i->begin(); j != i->end(); j++)
			{
				j->is_updated = false;

				Vector_2 temp(j->point2.x(), j->point2.y());
				bool bdis_condition = false;
				for (int i = 0; i < deform_polygon.size(); i++)
				{
					if ((deform_polygon[i].origin - temp).squared_length()
						< deform_polygon[i].deviation.squared_length())
					{
						bdis_condition = true;
						break;
					}
				}
				if (CGAL::bounded_side_2(polygon_origin_.begin(), polygon_origin_.end(), j->point2)
					!= CGAL::ON_UNBOUNDED_SIDE || bdis_condition)
				{
					j->is_updated = true;
					input_data.push_back(j->point2);
				}
			}
		}

		mean_value_warping warp_;
		warp_.do_warp(polygon_origin_, polygon_moved, input_data, output_data);

		int it = 0;
		for (auto i = cps->begin(); i != cps->end(); i++)
		{
			for (auto j = i->begin(); j != i->end(); j++)
			{
				if (j->is_updated)
				{
					j->point2 = output_data[it];
					it++;
				}
			}
		}

		for (int i = 0; i < deform_polygon.size(); i++)
		{
			deform_polygon[i].origin += deform_polygon[i].deviation;
			deform_polygon[i].deviation = Vector_2(0, 0);
		}

		EditCM* cm_ = editing_authoring->get_control_mesh();
		for (int i = 0; i < cm_->vertices.size(); i++)
		{
			if (!cm_->vertices[i].is_bound)
			{
				double xd_, yd_;
				if (cm_->vertices[i].is_on_feature)
				{
					xd_ = cm_->vertices[i].cp_pair.first.x - cm_->vertices[i].previous_pair.first.x;
					yd_ = cm_->vertices[i].cp_pair.first.y - cm_->vertices[i].previous_pair.first.y;
					if (sqrt(xd_ * xd_ + yd_ * yd_) > 1e-6)
					{
						cm_->vertices[i].previous_pair = cm_->vertices[i].cp_pair;
						updating_cp_candidates.insert(i);
					}
				}
				else
				{
					xd_ = cm_->vertices[i].cp.x - cm_->vertices[i].previous_cp.x;
					yd_ = cm_->vertices[i].cp.y - cm_->vertices[i].previous_cp.y;
					if (sqrt(xd_ * xd_ + yd_ * yd_) > 1e-6)
					{
						cm_->vertices[i].previous_cp = cm_->vertices[i].cp;
						updating_cp_candidates.insert(i);
					}
				}

				if (bAnimated)
				{
					cm_->vertices[i].deviation.x += xd_;
					cm_->vertices[i].deviation.y += yd_;
					cm_->vertices[i].deviation.r = 0;
					cm_->vertices[i].deviation.g = 0;
					cm_->vertices[i].deviation.b = 0;
				}
				else
				{
					cm_->vertices[i].deviation.x = xd_;
					cm_->vertices[i].deviation.y = yd_;
					cm_->vertices[i].deviation.r = 0;
					cm_->vertices[i].deviation.g = 0;
					cm_->vertices[i].deviation.b = 0;
				}
			}
		}
		if (!bAnimated) {
			editing_authoring->update_edited_vecimage(updating_cp_candidates);
			updating_cp_candidates.clear();
		}
	}

	void Viewer::moved_selected_pts_directly()
	{
		EditCM *cm_ = editing_authoring->get_control_mesh();
		for (int i = 0; i < cm_->vertices.size(); i++)
		{
			if (!cm_->vertices[i].is_bound)
			{	
				double xd_ = 0, yd_ = 0;
				if (cm_->vertices[i].is_on_feature)
				{
					xd_ = cm_->vertices[i].cp_pair.first.x - cm_->vertices[i].previous_pair.first.x;
					yd_ = cm_->vertices[i].cp_pair.first.y - cm_->vertices[i].previous_pair.first.y;
					if (sqrt(xd_*xd_ + yd_*yd_) > 1e-6)
					{
						cm_->vertices[i].previous_pair = cm_->vertices[i].cp_pair;
						updating_cp_candidates.insert(i);
					}
				}
				else
				{
					xd_ = cm_->vertices[i].cp.x - cm_->vertices[i].previous_cp.x;
					yd_ = cm_->vertices[i].cp.y - cm_->vertices[i].previous_cp.y;
					if (sqrt(xd_*xd_ + yd_*yd_) > 1e-6)
					{
						cm_->vertices[i].previous_cp = cm_->vertices[i].cp;
						updating_cp_candidates.insert(i);
					}
				}
				if (bAnimated)
				{
					cm_->vertices[i].deviation.x += xd_;
					cm_->vertices[i].deviation.y += yd_;
					cm_->vertices[i].deviation.r = 0;
					cm_->vertices[i].deviation.g = 0;
					cm_->vertices[i].deviation.b = 0;
				}
				else
				{
					cm_->vertices[i].deviation.x = xd_;
					cm_->vertices[i].deviation.y = yd_;
					cm_->vertices[i].deviation.r = 0;
					cm_->vertices[i].deviation.g = 0;
					cm_->vertices[i].deviation.b = 0;
				}
			}
		}

		CPs	*cpsp = editing_authoring->get_cpsP();
		for (int i = 0; i < (*cpsp).size(); i++)
		{
			for (int j = 0; j < (*cpsp)[i].size(); j++)
			{
				if (intersaction_status == SINGLE_CP)
				{
					if ((*cpsp)[i][j].is_updated)
					{
						Point_2 p_temp = (*cpsp)[i][j].point2;
						(*cpsp)[i][j].point2 = Point_2(p_temp.x() + edit_move_.x(), p_temp.y() + edit_move_.y());
					}
				}
				else if (intersaction_status == CWARP_TRANS)
				{
					Vector_2 p_temp = (*cpsp)[i][j].point2 - Point_2(0, 0);
					if (sqrt((p_temp - cwarp_center_).squared_length()) < cwarp_d_radius)
					{
						double numerator_ = (pow(cwarp_d_radius, 2) - (p_temp - cwarp_center_).squared_length())
							/ pow(cwarp_d_radius, 2);
						double coeff_ = cwarp_strength_*numerator_;
						(*cpsp)[i][j].point2 = Point_2(p_temp.x() + coeff_*edit_move_.x(), p_temp.y() + coeff_*edit_move_.y());
					}
				}
				else if (intersaction_status == CWARP_ROOMIN)
				{
					Vector_2 p_temp = (*cpsp)[i][j].point2 - Point_2(0, 0);
					if (sqrt((p_temp - cwarp_center_).squared_length()) < cwarp_d_radius)
					{
						double in_ = sqrt((p_temp - cwarp_center_).squared_length() / pow(cwarp_d_radius, 2));
						Vector_2 dis_ = cwarp_strength_*((in_ - 1)*(in_ - 1))*
							(p_temp - cwarp_center_) / sqrt((p_temp - cwarp_center_).squared_length())*
							0.2*sqrt(edit_move_.squared_length());
						(*cpsp)[i][j].point2 = Point_2(p_temp.x() + dis_.x(), p_temp.y() + dis_.y());
					}		
				}
				else if (intersaction_status == CWARP_ROOMOUT)
				{
					Vector_2 p_temp = (*cpsp)[i][j].point2 - Point_2(0, 0);
					if (sqrt((p_temp - cwarp_center_).squared_length()) < cwarp_d_radius)
					{
						double in_ = sqrt((p_temp - cwarp_center_).squared_length() / pow(cwarp_d_radius, 2));
						Vector_2 dis_ = -cwarp_strength_*((in_ - 1)*(in_ - 1))*
							(p_temp - cwarp_center_) / sqrt((p_temp - cwarp_center_).squared_length())*
							0.2*sqrt(edit_move_.squared_length());
						(*cpsp)[i][j].point2 = Point_2(p_temp.x() + dis_.x(), p_temp.y() + dis_.y());
					}
				}
			}
		}
		
		if (!bAnimated) {
			editing_authoring->update_edited_vecimage(updating_cp_candidates);
			updating_cp_candidates.clear();
		}
	}

	void Viewer::warp_cps(Point_2 image_size)
	{
		if (deform_polygon.empty())
		{
			return;
		}

		vector<Point_2> polygon_origin_, polygon_moved;
		for (int i = 0; i < deform_polygon.size(); i++)
		{
			polygon_origin_.push_back(Point_2(deform_polygon[i].origin.x(), deform_polygon[i].origin.y()));
			Vector_2 pm = deform_polygon[i].origin + deform_polygon[i].deviation;
			polygon_moved.push_back(Point_2(pm.x(), pm.y()));
		}

		vector<Point_2> input_data, output_data;
		EditCM *cm_ = editing_authoring->get_control_mesh();
		for (int i = 0; i < cm_->vertices.size(); i++)
		{
			cm_->vertices[i].is_updated = false;
			if (!cm_->vertices[i].is_bound)
			{
				if (cm_->vertices[i].is_on_feature)
				{
					Vector_2 temp(cm_->vertices[i].previous_pair.first.x,
						cm_->vertices[i].previous_pair.first.y);
					bool bdis_condition = false;
					for (int i = 0; i < deform_polygon.size(); i++)
					{
						if ((deform_polygon[i].origin - temp).squared_length()
							< deform_polygon[i].deviation.squared_length())
						{
							bdis_condition = true;
							break;
						}
					}
					if (CGAL::bounded_side_2(polygon_origin_.begin(), polygon_origin_.end(),Point_2(temp.x(),temp.y()))
						!= CGAL::ON_UNBOUNDED_SIDE || bdis_condition)
					{
						cm_->vertices[i].is_updated = true;
						input_data.push_back(Point_2(cm_->vertices[i].previous_pair.first.x,
							cm_->vertices[i].previous_pair.first.y));
						input_data.push_back(Point_2(cm_->vertices[i].previous_pair.second.x,
							cm_->vertices[i].previous_pair.second.y));
					}
					
				}
				else
				{
					Vector_2 temp(cm_->vertices[i].previous_cp.x,
						cm_->vertices[i].previous_cp.y);
					bool bdis_condition = false;
					for (int i = 0; i < deform_polygon.size(); i++)
					{
						if ((deform_polygon[i].origin - temp).squared_length()
							< deform_polygon[i].deviation.squared_length())
						{
							bdis_condition = true;
							break;
						}
					}
					if (CGAL::bounded_side_2(polygon_origin_.begin(), polygon_origin_.end(), Point_2(temp.x(), temp.y()))
						!= CGAL::ON_UNBOUNDED_SIDE || bdis_condition)
					{
						cm_->vertices[i].is_updated = true;
						input_data.push_back(Point_2(cm_->vertices[i].previous_cp.x,
							cm_->vertices[i].previous_cp.y));
					}
					
				}
			}	
		}

		mean_value_warping warp_;
		warp_.do_warp(polygon_origin_, polygon_moved, input_data, output_data);

		if (output_data.empty())
		{
			return;
		}

		int it = 0;
		for (int i = 0; i < cm_->vertices.size(); i++)
		{
			if (cm_->vertices[i].is_updated)
			{
				if (cm_->vertices[i].is_on_feature)
				{
					//Point_2 pnow(cm_->vertices[i].previous_pair.first.x, cm_->vertices[i].previous_pair.first.y);
					//if ((output_data[it]-pnow).squared_length()>1e-10)
					{
						cm_->vertices[i].cp.x = output_data[it].x();
						cm_->vertices[i].cp.y = output_data[it].y();
						cm_->vertices[i].cp_pair.first.x = output_data[it].x();
						cm_->vertices[i].cp_pair.first.y = output_data[it].y();
						cm_->vertices[i].cp_pair.second.x = output_data[it + 1].x();
						cm_->vertices[i].cp_pair.second.y = output_data[it + 1].y();
					}
					it++; it++;
				}
				else
				{
					//Point_2 pnow(cm_->vertices[i].previous_cp.x, cm_->vertices[i].previous_cp.y);
					//if ((output_data[it]-pnow).squared_length()>1e-10)
					{
						cm_->vertices[i].cp.x = output_data[it].x();
						cm_->vertices[i].cp.y = output_data[it].y();
					}
					it++;
				}
			}		
		}
	}

	void Viewer::set_editing_source(VecEditing *data)
	{
		editing_authoring = data;

		bBox = Bbox_3(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5);
		set_scene(bBox);
		update();
	}

	void Viewer::drawPolygon(int type_) {
		/*glEnable(GL_COLOR_LOGIC_OP);
		glLogicOp(GL_XOR);*/

		glDisable(GL_LIGHTING);
		glLineWidth(6.0);

		if (type_ == TRANSLATION || type_ == SCALE)
		{
			/*glEnable(GL_LINE_STIPPLE);
			glLineStipple(2, 0x0f0f);

			if (deform_polygon.size() > 1)
			{
				glBegin(GL_LINES);
				for (int i = 0; i < deform_polygon.size(); i++)
				{
					glVertex2d(deform_polygon[i].origin.x()+deform_polygon[i].deviation.x(),
						deform_polygon[i].origin.y() + deform_polygon[i].deviation.y());
					glVertex2d(deform_polygon[(i + 1)%deform_polygon.size()].origin.x()
						+ deform_polygon[(i + 1) % deform_polygon.size()].deviation.x(),
						deform_polygon[(i + 1) % deform_polygon.size()].origin.y()+
						deform_polygon[(i + 1) % deform_polygon.size()].deviation.y());
				}
				glEnd();
			}
			glDisable(GL_LINE_STIPPLE);*/

			glPointSize(12.0);
			glBegin(GL_POINTS);
			for (int i = 0; i < deform_polygon.size(); i++)
			{
				if (deform_polygon[i].bselected == 0)
				{
					glColor3d(0.5, 0.0, 0.0);
				}
				else
				{
					glColor3d(1.0, 0.0, 0.0);
				}
				glVertex2d(deform_polygon[i].origin.x() + deform_polygon[i].deviation.x(),
					deform_polygon[i].origin.y() + deform_polygon[i].deviation.y());
			}
			glEnd();	

			glPointSize(4.0);
			glColor3d(0.0, 0.0, 0.0);
			glBegin(GL_POINTS);
			for (int i = 0; i < deform_polygon.size(); i++)
			{
				
				glVertex2d(deform_polygon[i].origin.x() + deform_polygon[i].deviation.x(),
					deform_polygon[i].origin.y() + deform_polygon[i].deviation.y());
			}
			glEnd();
		}
		else
		{
			/*if (selfSelected_polygon.size() > 1)
			{
				glBegin(GL_LINES);
				for (int i = 0; i < selfSelected_polygon.size() - 1; i++)
				{
					glVertex3d(selfSelected_polygon[i].x(), selfSelected_polygon[i].y(), 0.0);
					glVertex3d(selfSelected_polygon[i + 1].x(), selfSelected_polygon[i + 1].y(), 0.0);
				}
				glEnd();
			}*/
			glColor3d(0.5, 0.0, 0.0);
			glPointSize(12.0);
			glBegin(GL_POINTS);
			for (int i = 0; i < selfSelected_polygon.size(); i++)
			{
				glVertex2d(selfSelected_polygon[i].x(), selfSelected_polygon[i].y());
			}
			glEnd();
		}
		//glDisable(GL_COLOR_LOGIC_OP);
	}

	void Viewer::drawInsertedElements()
	{	
		glLineWidth(4.5);
		if (intersaction_status == INSERT_P)
		{
			glPointSize(11.0);
			glBegin(GL_POINTS);
			for (int i = 0; i < inserted_points.size(); i++)
			{		
				glColor3d(0,0,0);
				glVertex2d(inserted_points[i].point2.x(), inserted_points[i].point2.y());	
			}
			glEnd();
			glPointSize(8.0);
			glBegin(GL_POINTS);
			for (int i = 0; i < inserted_points.size(); i++)
			{			
				glColor3d(inserted_points[i].v_positive.r, inserted_points[i].v_positive.g, inserted_points[i].v_positive.b);
				glVertex2d(inserted_points[i].point2.x(), inserted_points[i].point2.y());
			}
			glEnd();
		}
		else if (intersaction_status == INSERT_NORMAL_E || intersaction_status == INSERT_CREASE_E)
		{
			if (intersaction_status == INSERT_NORMAL_E)
			{
				glEnable(GL_LINE_STIPPLE);
				glLineStipple(1, 0x0101);
			}				
			glLineWidth(6.0);
			glBegin(GL_LINES);
			glColor3d(0, 0, 0);
			for (int i = 0; i < inserted_points.size() - 1; i++)
			{
				glVertex2d(inserted_points[i].point2.x(), inserted_points[i].point2.y());
				glVertex2d(inserted_points[i + 1].point2.x(), inserted_points[i + 1].point2.y());
			}
			glEnd();

			glLineWidth(3.0);			
			glBegin(GL_LINES);
			for (int i = 0; i < inserted_points.size()-1; i++)
			{			
				glColor3d(inserted_points[i].v_positive.r, inserted_points[i].v_positive.g, inserted_points[i].v_positive.b);
				glVertex2d(inserted_points[i].point2.x(), inserted_points[i].point2.y());
				glColor3d(inserted_points[i+1].v_positive.r, inserted_points[i+1].v_positive.g, inserted_points[i+1].v_positive.b);
				glVertex2d(inserted_points[i+1].point2.x(), inserted_points[i+1].point2.y());
			}
			glEnd();

			glPointSize(11.0);
			glBegin(GL_POINTS);
			glColor3d(0, 0, 0);
			for (int i = 0; i < inserted_points.size(); i++)
			{
				glVertex2d(inserted_points[i].point2.x(), inserted_points[i].point2.y());
			}
			glEnd();
			glPointSize(8.0);
			glBegin(GL_POINTS);
			for (int i = 0; i < inserted_points.size(); i++)
			{
				glColor3d(inserted_points[i].v_positive.r, inserted_points[i].v_positive.g, inserted_points[i].v_positive.b);
				glVertex2d(inserted_points[i].point2.x(), inserted_points[i].point2.y());
			}
			glEnd();
		}
		else if (intersaction_status == INSERT_CREASE_SHAPE)
		{

		}

		glDisable(GL_LINE_STIPPLE);
	}

	void Viewer::drawInsertedEdges()
	{
		glEnable(GL_LINE_STIPPLE);

		glPointSize(11.0);
		glBegin(GL_POINTS);
		for (int i = 0; i < inserted_edges.size(); i++)
		{
			for (int j = 0; j < inserted_edges[i].size(); j++)
			{
				glColor3d(0, 0, 0);
				glVertex2d(inserted_edges[i][j].point2.x(), inserted_edges[i][j].point2.y());
			}
		}
		glEnd();
		glPointSize(8.0);
		glBegin(GL_POINTS);
		for (int i = 0; i < inserted_edges.size(); i++)
		{
			for (int j = 0; j < inserted_edges[i].size(); j++)
			{
				glColor3d(inserted_edges[i][j].v_positive.r, inserted_edges[i][j].v_positive.g, inserted_edges[i][j].v_positive.b);
				glVertex2d(inserted_edges[i][j].point2.x(), inserted_edges[i][j].point2.y());
			}
		}
		glEnd();

		glLineWidth(6.0);
		glLineStipple(1, 0x0101);
		glBegin(GL_LINES);
		glColor3d(0, 0, 0);
		for (int i = 0; i < inserted_edges.size(); i++)
		{
			for (int j = 0; j < inserted_edges[i].size() - 1; j++)
			{
				glVertex2d(inserted_edges[i][j].point2.x(), inserted_edges[i][j].point2.y());
				glVertex2d(inserted_edges[i][j + 1].point2.x(), inserted_edges[i][j + 1].point2.y());
			}
		}
		glEnd();

		glLineWidth(3.0);
		glLineStipple(1, 0x0101);
		glBegin(GL_LINES);
		for (int i = 0; i < inserted_edges.size(); i++)
		{
			for (int j = 0; j < inserted_edges[i].size() - 1; j++)
			{
				glColor3d(inserted_edges[i][j].v_positive.r, inserted_edges[i][j].v_positive.g, inserted_edges[i][j].v_positive.b);
				glVertex2d(inserted_edges[i][j].point2.x(), inserted_edges[i][j].point2.y());
				glColor3d(inserted_edges[i][j+1].v_positive.r, inserted_edges[i][j+1].v_positive.g, inserted_edges[i][j+1].v_positive.b);
				glVertex2d(inserted_edges[i][j+1].point2.x(), inserted_edges[i][j+1].point2.y());
			}
		}
		glEnd();

		glDisable(GL_LINE_STIPPLE);
	}


	void Viewer::insert_elements(vector<AuthoringP2> inserted_points)
	{
		CDT* domain_mesh = editing_authoring->get_domain_mesh();
		int size_ = domain_mesh->number_of_vertices();
		float* col = editing_authoring->get_authoring_right_color();
		if (intersaction_status == INSERT_P)
		{
			for (int i = 0; i < inserted_points.size(); i++)
			{
				CDT::Vertex_handle vh;
				vh = domain_mesh->insert(inserted_points[i].point2);
				vh->set_associated_index(size_);
				size_++;
				RGBPoint p; p.x = inserted_points[i].point2.x();
				p.y = inserted_points[i].point2.y();
				p.r = col[0]; p.g = col[1]; p.b = col[2];
				p.is_on_bound = false;
				p.is_on_feature = false;
				vh->set_cp(p);
			}
		}
		else if (intersaction_status == INSERT_NORMAL_E)
		{
			map<int, CDT::Vertex_handle> cdt_vertex_handle;
			for (int i = 0; i < inserted_points.size(); i++)
			{
				if (inserted_points.rbegin()->id == inserted_points.begin()->id &&
					i == inserted_points.size() - 1)
				{
					continue;
				}
				CDT::Vertex_handle vh;
				vh = domain_mesh->insert(inserted_points[i].point2);
				vh->set_associated_index(size_);
				
				RGBPoint p; p.x = inserted_points[i].point2.x();
				p.y = inserted_points[i].point2.y();
				p.r = col[0]; p.g = col[1]; p.b = col[2];
				p.is_on_bound = false;
				p.is_on_feature = false;
				vh->set_cp(p);
				cdt_vertex_handle[i] = vh;
				size_++;
			}
			for (int i = 0; i < inserted_points.size()-1; i++)
			{
				if (inserted_points.rbegin()->id == inserted_points.begin()->id &&
					i == inserted_points.size() - 2)
				{
					domain_mesh->insert_constraint(cdt_vertex_handle.at(i), cdt_vertex_handle.at(0));
					continue;
				}
				domain_mesh->insert_constraint(cdt_vertex_handle.at(i),cdt_vertex_handle.at(i + 1));
			}
		}
		else if (intersaction_status == INSERT_CREASE_E)
		{
			ACPs* cps = editing_authoring->get_authoring_cps();
			int size_cps = cps->size();
			map<int, CDT::Vertex_handle> cdt_vertex_handle;

			bool bcycle = inserted_points.rbegin()->id == inserted_points.begin()->id;
			if (bcycle)
				inserted_points.pop_back();

			for (int i = 0; i < inserted_points.size(); i++)
			{
				CDT::Vertex_handle vh;
				vh = domain_mesh->insert(inserted_points[i].point2);
				vh->set_associated_index(size_);

				RGBPoint p; p.x = inserted_points[i].point2.x();
				p.y = inserted_points[i].point2.y();
				p.r = col[0]; p.g = col[1]; p.b = col[2];
				p.is_on_bound = false;
				p.is_on_feature = true;
				vh->set_cp(p);
				cdt_vertex_handle[i] = vh;
				inserted_points[i].id = size_;
				vh->set_cseq_index(Vpair(size_cps, i));
				size_++;
			}
			for (int i = 0; i < inserted_points.size() - 1; i++)
			{
				domain_mesh->insert_constraint(cdt_vertex_handle.at(i), cdt_vertex_handle.at(i + 1));
				if (bcycle && i == inserted_points.size() - 2)
				{
					domain_mesh->insert_constraint(cdt_vertex_handle.at(inserted_points.size() - 1), cdt_vertex_handle.at(0));
				}
			}

			//cal direction
			for (int j = 0; j < inserted_points.size(); j++)
			{			
				inserted_points[j].is_cycle = bcycle;

				Vector_2 vecV;
				Point_2 pfir, psed;
				Vector_2 vecV1, vecV2;
				
				if (j == 0 && !bcycle)
				{
					pfir = inserted_points[j].point2; psed = inserted_points[j + 1].point2;
					vecV = inserted_points[j + 1].point2 - inserted_points[j].point2;
					vecV = Vector_2(-vecV.y(), vecV.x()) / sqrt(vecV.squared_length());
				}
				else if (j == inserted_points.size() - 1 && !bcycle)
				{
					pfir = inserted_points[j - 1].point2; psed = inserted_points[j].point2;
					vecV = inserted_points[j].point2 - inserted_points[j - 1].point2;
					vecV = Vector_2(-vecV.y(), vecV.x()) / sqrt(vecV.squared_length());
				}
				else
				{
					Point_2 ppre, pcen, pnext;
					ppre = inserted_points[(j - 1 + inserted_points.size()) % inserted_points.size()].point2;
					pcen = inserted_points[j].point2;
					pnext = inserted_points[(j + 1) % inserted_points.size()].point2;

					pfir = pcen; psed = pnext;

					vecV1 = pcen - ppre;
					vecV1 = Vector_2(-vecV1.y(), vecV1.x()) / sqrt(vecV1.squared_length());
					vecV2 = pnext - pcen;
					vecV2 = Vector_2(-vecV2.y(), vecV2.x()) / sqrt(vecV2.squared_length());

					vecV = vecV1 + vecV2;
				}
			
				double dis = 0.002;
				Point_2 pcenter = inserted_points[j].point2;
				inserted_points[j].v_negative.x = pcenter.x() - vecV.x() * dis;
				inserted_points[j].v_negative.y = pcenter.y() - vecV.y() * dis;
				inserted_points[j].v_positive.x = pcenter.x() + vecV.x() * dis;
				inserted_points[j].v_positive.y = pcenter.y() + vecV.y() * dis;		
			}
			cps->push_back(inserted_points);
			
		}
		else if (intersaction_status == INSERT_CREASE_SHAPE)
		{

		}

		editing_authoring->update_authoring_data();
	}

	void Viewer::drawXORRect(int type_)
	{
		if (type_ == NONE)
		{
			return;
		}

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0, width(), height(), 0, -1, 1);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glPushAttrib(GL_ENABLE_BIT);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
		glEnable(GL_COLOR_LOGIC_OP);
		glLogicOp(GL_XOR);
		glColor3f(1.0, 1.0, 1.0);

		if (type_ == SQUARE)
		{
			glBegin(GL_LINE_LOOP);
			glVertex2f(start_position_ofselect.x(), start_position_ofselect.y());
			glVertex2f(current_position_ofselect.x(), start_position_ofselect.y());
			glVertex2f(current_position_ofselect.x(), current_position_ofselect.y());
			glVertex2f(start_position_ofselect.x(), current_position_ofselect.y());
			glEnd();
			glDisable(GL_LOGIC_OP);
		}		
		else
		{
			if (type_ == COLOR)
			{
				glDisable(GL_COLOR_LOGIC_OP);
				float *col = editing_authoring->get_edit_color();
				glLineWidth(5.0);
				glColor3f(col[0],col[1],col[2]);
			}
			else
			{
				glLineWidth(2.0);
			}
			/*glPointSize(4.0);
			glBegin(GL_POINTS);
			glVertex2f(current_position_ofselect.x(), current_position_ofselect.y());
			glEnd();*/

			if (type_ != SINGLE_CP && type_ != TRANSLATION)
			{
				glBegin(GL_LINE_LOOP);
				glVertex2f(current_position_ofselect.x() - brush_size_*pixel_screen_width_ / 2,
					current_position_ofselect.y() - brush_size_*pixel_screen_width_ / 2);
				glVertex2f(current_position_ofselect.x() + brush_size_*pixel_screen_width_ / 2,
					current_position_ofselect.y() - brush_size_*pixel_screen_width_ / 2);
				glVertex2f(current_position_ofselect.x() + brush_size_*pixel_screen_width_ / 2,
					current_position_ofselect.y() + brush_size_*pixel_screen_width_ / 2);
				glVertex2f(current_position_ofselect.x() - brush_size_*pixel_screen_width_ / 2,
					current_position_ofselect.y() + brush_size_*pixel_screen_width_ / 2);
				glEnd();
			}		
		}
	
		glDisable(GL_LOGIC_OP);

		glPopAttrib();
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
	}

	void Viewer::PickVert(int type_)
	{
		if (type_ == NONE)
		{
			return;
		}

		if (!do_adding_ && !do_deleting_)
		{
			if (type_ == FEATURE)
			{
				selected_featureploy_index_.clear();
			}
			if (type_ == SQUARE || type_ == BRUSH)
			{
				selected_pts_index_.clear();
			}
		}

		//select control points
		EditCM *cm_ = editing_authoring->get_control_mesh();
		CPs	cps;
		if (type_ == FEATURE || type_ == SELFPOLYGON)
		{
			cps = editing_authoring->get_cps();
		}
		CPs	*cpsp;
		if (type_ == SINGLE_CP)
		{
			cpsp = editing_authoring->get_cpsP();
		}
		QPoint mid, wid;

		if (type_ == SQUARE)
		{
			mid = (start_position_ofselect + current_position_ofselect) / 2;
			mid.setY(height() - mid.y());
			wid = (start_position_ofselect - current_position_ofselect);
		}
		else if (type_ == SINGLE_CP)
		{
			mid = current_position_ofselect;
			mid.setY(height() - mid.y());
			wid = QPoint(brush_size_/3.0*pixel_screen_width_, brush_size_/3.0*pixel_screen_width_);
			//wid = QPoint(5,5);
		}
		else
		{
			mid = current_position_ofselect;
			mid.setY(height() - mid.y());
			wid = QPoint(brush_size_*pixel_screen_width_, brush_size_*pixel_screen_width_);
		}	

		if (wid.x() < 0) wid.setX(-wid.x());
		if (wid.y() < 0) wid.setY(-wid.y());
		if (wid.x() == 0 || wid.y() == 0) return;
		int sz = 10 * cm_->vertices.size();
		GLuint *selectBuf = new GLuint[sz];
		glSelectBuffer(sz, selectBuf);
		glRenderMode(GL_SELECT);
		glInitNames();
		glPushName(-1);
		double mp[16];
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		glMatrixMode(GL_PROJECTION);
		glGetDoublev(GL_PROJECTION_MATRIX, mp);
		glPushMatrix();
		glLoadIdentity();
		gluPickMatrix(mid.x(), mid.y(), wid.x(), wid.y(), viewport);
		glMultMatrixd(mp);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();

		if (type_ == FEATURE)
		{
			for (int i= 0;i<cps.size();i++)
			{
				glLoadName(i);
				for (int j = 0;j<cps[i].size()-1;j++)
				{
					glBegin(GL_LINES);
					glVertex2d(cps[i][j].point2.x(), cps[i][j].point2.y());
					glVertex2d(cps[i][j+1].point2.x(), cps[i][j+1].point2.y());
					glEnd();					
				}
			}
		}
		else if (type_ == SELFPOLYGON)
		{		
			for (int i = 0; i < cps.size(); i++)
			{
				if (cps[i].size()>1&& cps[i][0].index != cps[i][cps[i].size()-1].index)
				{
					glLoadName(i);
					for (int j = 0; j < cps[i].size() - 1; j++)
					{
						glBegin(GL_LINES);
						glVertex2d(cps[i][j].point2.x(), cps[i][j].point2.y());
						glVertex2d(cps[i][j + 1].point2.x(), cps[i][j + 1].point2.y());
						glEnd();
					}
				}			
			}
		}
		else if (type_ == COLOR && !color_edit_region_.empty())
		{
			for (auto k = color_edit_region_.begin(); k != color_edit_region_.end(); k++)
			{
				glLoadName(*k);
				glBegin(GL_POINTS);
				if (cm_->vertices[*k].is_on_feature)
				{
					glVertex2d(cm_->vertices[*k].cp_pair.second.x,
						cm_->vertices[*k].cp_pair.second.y);
				}
				else
				{
					glVertex2d(cm_->vertices[*k].cp.x, cm_->vertices[*k].cp.y);
				}
				glEnd();
			}
		}
		else if (type_ == TRANSLATION)
		{
			if (!deform_polygon.empty())
			{
				for (int i = 0; i < deform_polygon.size(); i++)
				{
					deform_polygon[i].bselected = false;
					glLoadName(i);
					glBegin(GL_POINTS);
					glVertex2d(deform_polygon[i].origin.x() + deform_polygon[i].deviation.x(),
						deform_polygon[i].origin.y() + deform_polygon[i].deviation.y());
					glEnd();
				}
			}
		}
		else if (type_ == SINGLE_CP)
		{
			int k = 0;
			for (; k < cm_->vertices.size(); k++)
			{
				cm_->vertices[k].is_updated = false;
				glLoadName(k);
				glBegin(GL_POINTS);
				if (cm_->vertices[k].is_on_feature)
				{
					glVertex2d(cm_->vertices[k].cp_pair.first.x,
						cm_->vertices[k].cp_pair.first.y);
				}
				else
				{
					glVertex2d(cm_->vertices[k].cp.x, cm_->vertices[k].cp.y);
				}
				glEnd();
			}
			for (int i = 0; i<cpsp->size(); i++)
			{				
				for (int j = 0; j<(*cpsp)[i].size(); j++)
				{
					(*cpsp)[i][j].is_updated = false;
					glLoadName(k);
					glBegin(GL_POINTS);
					glVertex2d((*cpsp)[i][j].point2.x(), (*cpsp)[i][j].point2.y());
					glEnd();
					k++;
				}
			}
		}
		else
		{
			for (int k = 0; k < cm_->vertices.size(); k++)
			{
				cm_->vertices[k].is_updated = false;
				glLoadName(k);
				glBegin(GL_POINTS);
				if (cm_->vertices[k].is_on_feature)
				{
					glVertex2d(cm_->vertices[k].cp_pair.second.x,
						cm_->vertices[k].cp_pair.second.y);
				}
				else
				{
					glVertex2d(cm_->vertices[k].cp.x, cm_->vertices[k].cp.y);
				}
				glEnd();
			}
		}		
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		long hits = glRenderMode(GL_RENDER);
		vector<pair<double, unsigned int>> H;
		for (long i = 0; i < hits; i++)
		{
			H.push_back(pair<double, unsigned int>(selectBuf[i * 4 + 1] / 4294967295.0, selectBuf[i * 4 + 3]));
		}
		if (type_ == SELFPOLYGON)
		{			
			if (hits > 0)
			{
				selfSelected_polygon.clear();
				for (int j = 0; j < cps[H[0].second].size(); j++)
				{
					selfSelected_polygon.push_back(cps[H[0].second][j].point2);
				}
			}
		}
		else if (type_ == COLOR)
		{
			float *col = editing_authoring->get_edit_color();
			for (int i = 0; i != hits; i++)
			{
				cm_->vertices[H[i].second].cp_pair.first.r = col[0];
				cm_->vertices[H[i].second].cp_pair.first.g = col[1];
				cm_->vertices[H[i].second].cp_pair.first.b = col[2];
				cm_->vertices[H[i].second].cp_pair.second.r = col[0];
				cm_->vertices[H[i].second].cp_pair.second.g = col[1];
				cm_->vertices[H[i].second].cp_pair.second.b = col[2];
				cm_->vertices[H[i].second].cp.r = col[0];
				cm_->vertices[H[i].second].cp.g = col[1];
				cm_->vertices[H[i].second].cp.b = col[2];
				selected_colored_pts.insert(H[i].second);
			}	
		}
		else if (type_ == TRANSLATION)
		{
			for (int i = 0; i != hits; i++)
			{
				deform_polygon[H[i].second].bselected = 1;				
			}
		}
		else if (type_ == SINGLE_CP)
		{
			for (int i = 0; i != hits; i++)
			{
				if (H[i].second < cm_->vertices.size())
				{
					cm_->vertices[H[i].second].is_updated = true;
				}			
			}
			int it = cm_->vertices.size();
			for (int i = 0; i < (*cpsp).size(); i++)
			{
				for (int j = 0; j < (*cpsp)[i].size(); j++)
				{
					for (int t = 0; t != hits; t++)
					{
						if (H[t].second == it)
						{
							(*cpsp)[i][j].is_updated = true;
						}
					}
					it++;
				}
			}
		}
		else
		{
			if (do_deleting_)
			{
				for (int i = 0; i != hits; i++)
				{
					if (type_ == FEATURE)
					{
						selected_featureploy_index_.erase(H[i].second);
					}
					if (type_ == SQUARE || type_ == BRUSH)
					{
						selected_pts_index_.erase(H[i].second);
					}
				}
			}
			else
			{
				for (int i = 0; i != hits; i++)
				{
					if (type_ == FEATURE)
					{
						selected_featureploy_index_.insert(H[i].second);
					}
					if (type_ == SQUARE || type_ == BRUSH)
					{
						selected_pts_index_.insert(H[i].second);
					}
				}
			}
		}	
		delete[] selectBuf;
	}

	void Viewer::draw_selected_elements(int type_, EditCM* cm_,
		 CPs cps) {
		if (type_ == SQUARE || type_ == BRUSH)
		{
			glPointSize(13.5);
			glColor3d(1, 0, 0);
			glBegin(GL_POINTS);
			for (auto k = selected_pts_index_.begin(); k != selected_pts_index_.end(); k++)
			{
				if (cm_->vertices[*k].is_on_feature)
				{
					glVertex2d(cm_->vertices[*k].cp_pair.second.x,
						cm_->vertices[*k].cp_pair.second.y);
				}
				else
				{
					glVertex2d(cm_->vertices[*k].cp.x, cm_->vertices[*k].cp.y);
				}
			}
			glEnd();
		}
		else if(type_ == FEATURE)
		{		
			glColor3d(1, 0, 0);

			glLineWidth(2.5);
			glBegin(GL_LINES);
			for (auto it = selected_featureploy_index_.begin(); it != selected_featureploy_index_.end(); it++)
			{
				for (int i = 0; i < cps[*it].size()-1; i++)
				{
					glVertex2d(cps[*it][i].point2.x(), cps[*it][i].point2.y());
					glVertex2d(cps[*it][i+1].point2.x(), cps[*it][i+1].point2.y());
				}
			}
			glEnd();

			glPointSize(12.5);			
			glBegin(GL_POINTS);
			for (auto it = selected_featureploy_index_.begin(); it != selected_featureploy_index_.end(); it++)
			{
				for (int i = 0; i < cps[*it].size(); i++)
				{
					glVertex2d(cps[*it][i].point2.x(), cps[*it][i].point2.y());
				}
			}
			glEnd();
		}
		if (type_ == BACK_UP)
		{
			int index = 0;
			glPointSize(12.5);
			glColor3d(0.2, 0.2, 0.2);
			glBegin(GL_POINTS);
			for (auto k = operation_pts.begin(); k != operation_pts.end(); k++)
			{
				if (cm_->vertices[*k].is_on_feature)
				{
					glVertex2d(cm_->vertices[*k].cp_pair.second.x,
						cm_->vertices[*k].cp_pair.second.y);
				}
				else
				{
					glVertex2d(cm_->vertices[*k].cp.x, cm_->vertices[*k].cp.y);
				}
			}
			glEnd();
		}
		if (type_ == COLOR || type_ == RED_CHANNEL ||
			type_ == GREEN_CHANNEL || type_ == BLUE_CHANNEL)
		{
			int index = 0;
			glPointSize(12.5);
			glColor3d(0.2, 0.2, 0.2);
			glBegin(GL_POINTS);
			for (auto k = color_edit_region_.begin(); k != color_edit_region_.end(); k++)
			{
				if (cm_->vertices[*k].is_on_feature)
				{
					glVertex2d(cm_->vertices[*k].cp_pair.second.x,
						cm_->vertices[*k].cp_pair.second.y);
				}
				else
				{
					glVertex2d(cm_->vertices[*k].cp.x, cm_->vertices[*k].cp.y);
				}
			}
			glEnd();
		}
	}


	void Viewer::update_whole_viewer()
	{
		if (!editing_authoring)
		{
			return;
		}
		repaint();
	}

	void Viewer::check_cppts_view(bool bv)
	{
		if (!editing_authoring)
		{
			return;
		}
		do_draw_cp_p = bv;
		update();
	}

	void Viewer::check_cpedge_view(bool bv)
	{
		if (!editing_authoring)
		{
			return;
		}
		do_draw_cp_e = bv;
		update();
	}

	void Viewer::check_sync_view(bool bv)
	{
		if (!editing_authoring)
		{
			return;
		}
		do_sync_other_viewers = bv;
		update();
	}

	void Viewer::check_feature_view(bool v)
	{
		if (!editing_authoring)
		{
			return;
		}
		do_draw_feature = v;
		update();
	}

	void Viewer::set_intersaction_status(int v) {
		intersaction_status = v;
		if (intersaction_status >= INSERT_P)
		{

		}
		else {
			if (intersaction_status >= SELFPOLYGON && intersaction_status <= SQUARE &&
				(previous_status >= COLOR && previous_status <= BLUE_CHANNEL))
			{
				//color_edit_region_.clear();
			}
			if (previous_status >= SELFPOLYGON && previous_status <= SQUARE &&
				(intersaction_status >= COLOR && intersaction_status <= BLUE_CHANNEL))
			{
				color_edit_region_.clear();
				setSelectedInvert();
			}
			if (previous_status >= SELFPOLYGON && previous_status <= SQUARE &&
				intersaction_status == BACK_UP)
			{
				operation_pts.clear();
				setSelectedBackUp();
			}
			if ((previous_status >= SELFPOLYGON && previous_status <= SQUARE)
				&& (intersaction_status == TRANSLATION || intersaction_status == SCALE))
			{
				if (previous_status == SELFPOLYGON)
				{
					for (int i = 0; i < selfSelected_polygon.size() - 1; i++)
					{
						EditPoint pt; pt.origin = selfSelected_polygon[i] - Point_2(0, 0); pt.bselected = 0;
						pt.deviation = Vector_2(0, 0);
						deform_polygon.push_back(pt);
					}
				}
				else
				{
					CPs	cps = editing_authoring->get_cps();
					if (previous_status == FEATURE && selected_featureploy_index_.size() <= cps.size())
					{
						for (auto it = selected_featureploy_index_.begin(); it != selected_featureploy_index_.end(); it++)
						{
							if (*it < cps.size() && cps[(*it)][0].index == cps[(*it)][cps[(*it)].size() - 1].index)
							{
								for (int j = 0; j < cps[(*it)].size() - 1; j++)
								{
									EditPoint pt; pt.origin = cps[(*it)][j].point2 - Point_2(0, 0);
									pt.bselected = 0;
									pt.deviation = Vector_2(0, 0);
									deform_polygon.push_back(pt);
								}
							}
							break;
						}
					}
				}
			}
			if ((previous_status == TRANSLATION || previous_status == SCALE) &&
				intersaction_status >= SELFPOLYGON && intersaction_status <= SQUARE)
			{
				deform_polygon.clear();
			}
		}
		previous_status = intersaction_status;
		update();
	}

	void Viewer::set_brush_size(int a)
	{
		brush_size_ = a;
		update();
	}

	void Viewer::screen_outside_points(EditCM *cm_, vector<vector<Point_2>>boundarys_,
		set<int> &inside_region, int screen_mode) {
		inside_region.clear();
		for (int it = 0; it < boundarys_.size(); it++)
		{
			Polygon_2 polygon_;
			for (int i = 0; i < boundarys_[it].size()-1; i++)
			{
				polygon_.push_back(boundarys_[it][i]);
			}
			//if (polygon_.is_simple())
			{
				for (int i = 0; i < cm_->vertices.size(); i++)
				{
					if (cm_->vertices[i].is_on_feature)
					{
						Point_2 pt(cm_->vertices[i].cp_pair.second.x,
							cm_->vertices[i].cp_pair.second.y);
						if (screen_mode == BY_DOMIAN)
						{
							pt = cm_->vertices[i].domain_midEdge;
							if (polygon_.bounded_side(pt) != CGAL::ON_UNBOUNDED_SIDE &&
								!cm_->vertices[i].bCCWorientation || 
								polygon_.bounded_side(pt) == CGAL::ON_BOUNDED_SIDE)
							{
								inside_region.insert(i);
							}
						}
						else
						{
							if (polygon_.bounded_side(pt) == CGAL::ON_BOUNDED_SIDE)
							{
								inside_region.insert(i);
							}
						}
						
					}
					else
					{
						Point_2 pt1(cm_->vertices[i].cp.x,
							cm_->vertices[i].cp.y);
						if (screen_mode == BY_DOMIAN)
						{
							pt1 = cm_->vertices[i].domain_midEdge;
						}
						if (polygon_.bounded_side(pt1) == CGAL::ON_BOUNDED_SIDE)
						{
							inside_region.insert(i);
						}
					}
				}
			}
		}
	}

	void Viewer::setSelectedInvert() {
		EditCM *cm_ = editing_authoring->get_control_mesh();
		if (color_edit_region_.empty())
		{						
			if (previous_status == SQUARE ||
				previous_status == BRUSH)
			{
				color_edit_region_ = selected_pts_index_;
			}
			else if (previous_status == FEATURE ||
				previous_status == SELFPOLYGON)
			{				
				CPs	cps = editing_authoring->get_cps();
				vector<vector<Point_2>> boundarys_;
				if (previous_status == FEATURE && selected_featureploy_index_.size()<=cps.size())
				{
					for (auto it = selected_featureploy_index_.begin();it!= selected_featureploy_index_.end();it++)
					{
						if (*it<cps.size()&& cps[(*it)][0].index == cps[(*it)][cps[(*it)].size()-1].index)
						{
							vector<Point_2> bound_;
							for (int j = 0;j<cps[(*it)].size();j++)
							{
								if (SCREEN_FEATURE_PLOY_MODE == BY_DOMIAN)
								{
									bound_.push_back(cm_->vertices[cps[(*it)][j].index].domain_midEdge);
								}
								else
								{
									bound_.push_back(cps[(*it)][j].point2);
								}
							}
							boundarys_.push_back(bound_);
						}
					}
				}
				if (previous_status == SELFPOLYGON)
				{
					if (selfSelected_polygon.empty())
					{
						return;
					}
					boundarys_.push_back(selfSelected_polygon);
				}
				if (SCREEN_FEATURE_PLOY_MODE == BY_DOMIAN && previous_status == FEATURE)
				{
					screen_outside_points(cm_, boundarys_, color_edit_region_,BY_DOMIAN);
				}
				else
				{
					screen_outside_points(cm_, boundarys_, color_edit_region_,BY_CP);
				}	
			}
		}
		else
		{
			set<int> inverted_region;
			for (int i = 0; i < cm_->vertices.size(); i++)
			{
				set<int>::iterator res = std::find(color_edit_region_.begin(), color_edit_region_.end(), i);
				if (res == color_edit_region_.end())
				{
					inverted_region.insert(i);
				}
			}
			color_edit_region_ = inverted_region;
		}

		update();
	}

	void Viewer::setSelectedBackUp()
	{
		EditCM *cm_ = editing_authoring->get_control_mesh();
		if (previous_status == SQUARE ||
			previous_status == BRUSH)
		{
			operation_pts = selected_pts_index_;
		}
		else if (previous_status == FEATURE ||
			previous_status == SELFPOLYGON)
		{
			CPs	cps = editing_authoring->get_cps();
			vector<vector<Point_2>> boundarys_;
			if (previous_status == FEATURE && selected_featureploy_index_.size() <= cps.size())
			{
				for (auto it = selected_featureploy_index_.begin(); it != selected_featureploy_index_.end(); it++)
				{
					if (*it < cps.size() && cps[(*it)][0].index == cps[(*it)][cps[(*it)].size() - 1].index)
					{
						vector<Point_2> bound_;
						for (int j = 0; j < cps[(*it)].size(); j++)
						{
							if (SCREEN_FEATURE_PLOY_MODE == BY_DOMIAN)
							{
								bound_.push_back(cm_->vertices[cps[(*it)][j].index].domain_midEdge);
							}
							else
							{
								bound_.push_back(cps[(*it)][j].point2);
							}
						}
						boundarys_.push_back(bound_);
					}
				}
			}
			if (previous_status == SELFPOLYGON)
			{
				if (selfSelected_polygon.empty())
				{
					return;
				}
				boundarys_.push_back(selfSelected_polygon);
			}
			if (SCREEN_FEATURE_PLOY_MODE == BY_DOMIAN && previous_status == FEATURE)
			{
				screen_outside_points(cm_, boundarys_, operation_pts, BY_DOMIAN);
			}
			else
			{
				screen_outside_points(cm_, boundarys_, operation_pts, BY_CP);
			}
		}
		update();
	}

	void Viewer::setSelectedDiff()
	{
		if (!operation_pts.empty())
		{
			set<int> new_region;
			for (auto it = operation_pts.begin();it != operation_pts.end();it++)
			{
				new_region.insert(*it);
			}
			int size_pre = 0;
			set<int> new_region1;
			for (auto it = color_edit_region_.begin(); it != color_edit_region_.end(); it++)
			{
				size_pre = new_region.size();
				new_region.insert(*it);
				if (new_region.size() > size_pre)
				{
					new_region1.insert(*it);
				}
			}
			color_edit_region_ = new_region1;		
		}
		update();
	}
}
