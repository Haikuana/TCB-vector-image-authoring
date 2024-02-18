#pragma once
#ifndef BASICIMAGEVIEWER_H
#define BASICIMAGEVIEWER_H

#include "editing.h"
#include "draw_aux.h"
#include <QGLWidget>
#include <QPaintEvent>
#include <QVector3D>
#include <qglviewer.h>
#include <manipulatedCameraFrame.h>

namespace EDITGUI {

	class VectorFrames {
	public:
		VectorFrames(VecEditing* authoring);

		void init();
		void draw();
		void animate();

	private:
		int age_, ageMax_;
		VecEditing* editing_authoring;
	};

	class BasicViewer :public QGLViewer
	{
		Q_OBJECT
	public:
		BasicViewer(int);
		~BasicViewer();

		void set_editing_source(VecEditing *data);

	protected:
		virtual void draw();
		virtual void init();
		virtual void animate();
		virtual void paintEvent(QPaintEvent *event);
		void mousePressEvent(QMouseEvent *event);
		void mouseMoveEvent(QMouseEvent *event);
		void mouseReleaseEvent(QMouseEvent *event);
		virtual void keyPressEvent(QKeyEvent *);

	private:
		void set_scene(Bbox_3 &box);
		

		public slots:
		void update_whole_viewer();
		void check_vecim_view(bool v);
		void check_sync_view(bool v);

	private:
		VecEditing						*editing_authoring;
		QString							OutFileName;

		int								image_wid;
		int								image_hei;
		bool							do_draw_vecimage;
		bool							do_sync_other_viewers;

		//viewer
		int								window_type;
		bool							do_rotate_scene;
		bool							do_translation_scene;
		qglviewer::AxisPlaneConstraint	*constraints;
		qglviewer::Vec					dir;
		double							radius;
		Bbox_3							bBox;

		bool							banimated = false;
		VectorFrames*					frame_animation;
	};


}
#endif