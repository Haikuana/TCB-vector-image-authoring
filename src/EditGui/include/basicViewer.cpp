#include "basicViewer.h"

namespace EDITGUI {

	using namespace std;
	using namespace qglviewer;

#define MAX_CHAR       128

	VectorFrames::VectorFrames(VecEditing* authoring) {
		editing_authoring = authoring;
		init();
	}

	void VectorFrames::animate() {
		if (++age_ >= ageMax_)
			return;
		editing_authoring->interpolated_vecimage(ageMax_);
	}

	void VectorFrames::draw() {
		glDisable(GL_POLYGON_SMOOTH);
		RGBPoint** veci_ = new RGBPoint*;
		int size_;
		editing_authoring->get_vec_image(veci_, size_);
		if (size_ > 0)
		{
			draw_vecImage(*veci_, size_);
		}
	}

	void VectorFrames::init() {
		age_ = 0;
		ageMax_ = 200;
	}

	BasicViewer::BasicViewer(int window_typ)
	{
		window_type = window_typ;

		editing_authoring = NULL;

		do_draw_vecimage = false;
		do_sync_other_viewers = true;

		do_rotate_scene = false;
		do_translation_scene = true;

		setAutoFillBackground(false);
		bBox = Bbox_3(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5);
	}

	BasicViewer::~BasicViewer()
	{
		if (!constraints)
		{
			delete constraints;
			constraints = NULL;
		}
	}

	void BasicViewer::set_scene(Bbox_3& box)
	{
		Point_3 p1(box.xmax(), box.ymax(), box.zmax());
		Point_3 p2(box.xmin(), box.ymin(), box.zmin());
		Vector_3 v(p2 - p1);
		v = v / 2;
		Point_3 center = p1 + v;
		radius = sqrt(v.squared_length());
		setSceneCenter(qglviewer::Vec(center.x(), center.y(), center.z()));
		//setSceneCenter(qglviewer::Vec(0,0,0));
		setSceneRadius(0.7 * radius);
		showEntireScene();
		//std::cout << "set scene \n";
	}

	void BasicViewer::paintEvent(QPaintEvent* event)
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


	void BasicViewer::draw()
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

		if (banimated)
		{
			frame_animation->draw();
		}
		else
		{
			//mesh
			if (editing_authoring != NULL)
			{
				if (do_draw_vecimage)
				{
#if 1
					glDisable(GL_POLYGON_SMOOTH);
					RGBPoint** veci_ = new RGBPoint*;
					int size_;
					editing_authoring->get_vec_image(veci_, size_);
					if (size_ > 0)
					{
						draw_vecImage(*veci_, size_);
					}
				}
#endif // 0


#if 0
				EditCM* cm_ = editing_authoring->get_control_mesh();

				draw_control_mesh_edges(cm_, 32);
				//draw_control_mesh_points(cm_, 32);
#endif
			}
		}

		glShadeModel(GL_FLAT);

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}

	void BasicViewer::init()
	{
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		// Restore previous viewer state.
		restoreStateFromFile();

		setMouseBinding(Qt::ControlModifier, Qt::RightButton, NO_CLICK_ACTION);
		setMouseBinding(Qt::ControlModifier, Qt::LeftButton, NO_CLICK_ACTION);
		setMouseBinding(Qt::NoModifier, Qt::LeftButton, NO_CLICK_ACTION);

		constraints = new WorldConstraint();
		camera()->frame()->setConstraint(constraints);
		camera()->setOrientation(Quaternion());
		camera()->setZNearCoefficient(0.0001);

		glClearColor(1.0, 1.0, 1.0, 1.0);
	}

	void BasicViewer::animate()
	{
		frame_animation->animate();
	}

	void BasicViewer::keyPressEvent(QKeyEvent *e)
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
		else if ((e->key() == Qt::Key_I) && (modifiers == Qt::ControlModifier))
		{
			handled = true;
			banimated = true;
			frame_animation = new VectorFrames(editing_authoring);
			startAnimation();
		}		
		else if ((e->key() == Qt::Key_Y) && (modifiers == Qt::ControlModifier))
		{
			handled = true;

#if 1
			std::cout << "snapshot\n";
			FILE* fp;
			int state = GL2PS_OVERFLOW, buffsize = 0;

			QString name = OutFileName;
			name.append("inputimage.pdf");
			fp = fopen(name.toLatin1(), "wb");

			GLint viewport[4];
			glGetIntegerv(GL_VIEWPORT, viewport);

			RGBPoint** veci_ = new RGBPoint*;
			int size_;
			editing_authoring->get_vec_image(veci_, size_);

			while (state == GL2PS_OVERFLOW) {
				buffsize += 1024 * 1024 * 16;
				gl2psBeginPage("test", "gl2psTestSimple", viewport, GL2PS_PDF, GL2PS_SIMPLE_SORT,
					GL2PS_OCCLUSION_CULL,
					GL_RGBA, 0, NULL, 0, 0, 0, buffsize, fp, name.toLatin1());

				if (size_ > 0)
				{
					draw_vecImage(*veci_, size_);
		}
				state = gl2psEndPage();
			}
			fclose(fp);
#endif // 0
			QString dirnow = OutFileName;
			dirnow.append("inputimage_snap.png");

			setSnapshotFormat("PNG");
			setSnapshotQuality(100);
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

	void BasicViewer::mousePressEvent(QMouseEvent *e)
	{	
		QGLViewer::mousePressEvent(e);

	}

	void BasicViewer::mouseMoveEvent(QMouseEvent *e)
	{
		
		QGLViewer::mouseMoveEvent(e);

	}

	void BasicViewer::mouseReleaseEvent(QMouseEvent *e)
	{
		setCursor(Qt::ArrowCursor);
		QGLViewer::mouseReleaseEvent(e);
	}

	void BasicViewer::set_editing_source(VecEditing *data)
	{
		editing_authoring = data;

		bBox = Bbox_3(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5);
		set_scene(bBox);
		update();
	}


	void BasicViewer::update_whole_viewer()
	{
		if (!editing_authoring)
		{
			return;
		}
		repaint();
	}

	void BasicViewer::check_vecim_view(bool bv)
	{
		if (!editing_authoring)
		{
			return;
		}
		do_draw_vecimage = bv;
		update();
	}

	void BasicViewer::check_sync_view(bool bv)
	{
		if (!editing_authoring)
		{
			return;
		}
		do_sync_other_viewers = bv;
		update();
	}

}
