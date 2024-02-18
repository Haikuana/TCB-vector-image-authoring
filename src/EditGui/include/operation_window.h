#pragma once
#ifndef OPERATIONWINDOW_H
#define OPERATIONWINDOW_H

#include "viewer.h"
#include "editing.h"

#include <QString>
#include <QStringList>
#include <QtWidgets>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QAction>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QBitmap>

namespace EDITGUI {

	class OperationWindow : public QMainWindow
	{
		Q_OBJECT

	public:
		OperationWindow();
		void set_editing_source(VecEditing *data);
		Viewer* viewer() { return viewer_; }

	signals:
		void window_close();

		public slots:
		void check_cp_pts_view(bool v) { viewer()->check_cppts_view(v); }
		void check_cp_edge_view(bool v) { viewer()->check_cpedge_view(v); }
		void check_feature_view(bool v) { viewer()->check_feature_view(v); }

	protected:
		void closeEvent(QCloseEvent *event);


	private:
		VecEditing						*editing_authoring;
		Viewer							*viewer_;

		QLabel							*ImagewLabel;
		QLabel							*ImagehLabel;

		QGroupBox						*image_groupBox;
		QGroupBox						*brush_groupBox;
		QLineEdit						*lineEdit;
		QSlider							*slider;

	};

}
#endif
