#pragma once
#ifndef VECIMAGEWINDOW_H
#define VECIMAGEWINDOW_H

#include "basicViewer.h"
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

	class VecImageWindow : public QMainWindow
	{
		Q_OBJECT

	public:
		VecImageWindow();
		void set_editing_source(VecEditing *data);

		BasicViewer* viewer() { return viewer_; }

	signals:
		void window_close();

		public slots:
		void check_vecimage_view(bool v) { viewer()->check_vecim_view(v); }
		void check_sync_view(bool v) { viewer()->check_sync_view(v); }

	protected:
		void closeEvent(QCloseEvent *event);

	private:
		VecEditing						*editing_authoring;
		BasicViewer							*viewer_;
	};


}
#endif
