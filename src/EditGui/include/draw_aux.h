#pragma once
#ifndef DRAWAUX_H
#define DRAWAUX_H

#include "editing.h"
#include <QGLWidget>
#include <QPaintEvent>
#include <QVector3D>
#include <qglviewer.h>
#include <manipulatedCameraFrame.h>

namespace EDITGUI {

	void draw_vecImage(RGBPoint* veci, int size_);

	//control mesh
	void draw_control_mesh_points(EditCM *cm_, int);
	void draw_control_mesh_edges(EditCM *cm_, int, Point_2 psize = Point_2(0.5,0.5));

	void draw_control_mesh_points(AuthoringCM* cm_, int);
	void draw_control_mesh_edges(AuthoringCM* cm_, int, Point_2 psize = Point_2(0.5, 0.5));
	void draw_control_mesh_edges(CMInfo* cmInfo, AuthoringCM* cm_);
	void draw_features(AuthoringCM* cmInfo);
}
#endif