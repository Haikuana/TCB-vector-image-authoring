#pragma once
#ifndef REFERENCEWINDOW_H
#define REFERENCEWINDOW_H

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

	class ReferenceWindow : public QMainWindow
	{
		Q_OBJECT

	public:
		ReferenceWindow();
		void set_editing_source(VecEditing *data);

	signals:
		void window_close();


	protected:
		void closeEvent(QCloseEvent *event);

	private:
		VecEditing						*editing_authoring;
		BasicViewer						*viewer_;

		QLabel							*ImagewLabel;
		QLabel							*ImagehLabel;

		QGroupBox						*image_groupBox;
	};


}
#endif
