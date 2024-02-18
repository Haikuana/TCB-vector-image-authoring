#pragma once
#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

#include "editing.h"
#include "draw_aux.h"
#include <QGLWidget>
#include <QPaintEvent>
#include <QVector3D>
#include <qglviewer.h>
#include <manipulatedCameraFrame.h>

namespace EDITGUI {

	class Viewer :public QGLViewer
	{
		Q_OBJECT
	public:
		Viewer(int);
		~Viewer();

		void set_editing_source(VecEditing *data);
		void set_intersaction_status(int v);
		void set_brush_size(int);

		void setSelectedInvert();
		void setSelectedBackUp();
		void setSelectedDiff();//color_edit_region_ but operation_pts

	protected:
		void draw() override;
		void init()override;
		void paintEvent(QPaintEvent *event)override;
		void mousePressEvent(QMouseEvent *event)override;
		void mouseMoveEvent(QMouseEvent *event)override;
		void mouseReleaseEvent(QMouseEvent *event)override;
		void wheelEvent(QWheelEvent *event)override;
		void keyPressEvent(QKeyEvent *)override;

	private:
		void set_scene(Bbox_3 &box);
		void PickVert(int type);
		void drawXORRect(int type);

		void draw_cycle(QPoint);
		void drawPolygon(int type);

		void drawInsertedElements();
		void drawInsertedEdges();
		void insert_elements(vector<AuthoringP2> inserted_points);

		void draw_selected_elements(int type_, EditCM* cm_,
			 CPs control_points_seqs);

		void screen_outside_points(EditCM *cm_,
			vector<vector<Point_2>>boundarys_, set<int> &inside_region,int);

		void colored_selected_pts();
		void colored_block_pts();
		void colored_slider_pts();

		void update_channel_operation(int,double);
		void channel_edit_selected_pts();

		void warp_cps(Point_2);
		void moved_selected_pts();
		void moved_selected_pts_directly();

		public slots:
		void update_whole_viewer();
		void check_cppts_view(bool v);
		void check_cpedge_view(bool v);
		void check_sync_view(bool v);
		void check_feature_view(bool v);

	private:
		VecEditing						*editing_authoring;
		QString							OutFileName;

		bool							do_draw_cp_p;
		bool							do_draw_cp_e;
		bool							do_draw_feature;
		bool							do_sync_other_viewers;

		//viewer
		int								window_type;
		bool							do_rotate_scene;
		bool							do_translation_scene;
		qglviewer::AxisPlaneConstraint	*constraints;
		qglviewer::Vec					dir;
		double							radius;
		Bbox_3							bBox;

		int								intersaction_status;
		int								previous_status;

		//authoring
		vector<vector<AuthoringP2>>		inserted_edges;
		vector<AuthoringP2>				inserted_points;
		bool							do_insert_edges;
		bool							do_insert_points;
		CMInfo							realCm;
		bool							do_draw_cpm;

		//select
		set<int>						selected_featureploy_index_;
		set<int>						selected_pts_index_;
		QPoint							current_position_ofselect;
		QPoint							start_position_ofselect;
		bool							do_selecting_;
		bool							do_adding_;
		bool							do_deleting_;
		int								brush_size_;
		int								pixel_screen_width_;

		vector<Point_2>					selfSelected_polygon;
		bool							do_made_polygon;
		bool							do_select_feapolygon;

		//boolean
		set<int>						operation_pts;

		//edit
		set<int> 						color_edit_region_;
		bool							do_edit_color_;
		bool							do_edit_block_color_;
		bool							do_edit_slider_color_;
		bool							do_edit_color_channel_;
		set<int>						selected_colored_pts;

		bool							do_edit_shape_;
		bool							do_select_edit_objects;
		vector<EditPoint>				deform_polygon;
		set<int>						selected_moved_pts;

		int								cwarp_radius;
		bool							do_cwarp_shape;
		Vector_2						edit_move_;
		double							cwarp_d_radius;
		Vector_2						cwarp_center_;
		double							cwarp_strength_;

		bool							bAnimated = false;
		set<int>						updating_cp_candidates;
	};


}
#endif