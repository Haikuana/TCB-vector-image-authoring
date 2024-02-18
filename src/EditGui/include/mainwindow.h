#pragma once

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>

#include "ui_mainwindow.h"

#include "operation_window.h"
#include "reference_window.h"
#include "vecImage_window.h"
#include "viewer.h"
#include "action_list.h"

#include <QtWidgets/QMdiArea>
#include <QtWidgets/QMdiSubWindow>
#include <QtWidgets/QAction>
#include <QtWidgets/QActionGroup>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QStatusBar>

namespace EDITGUI {

	class MainWindow : public QMainWindow
	{
		Q_OBJECT

	public:
		
		explicit MainWindow();
		~MainWindow();

		OperationWindow	*operationWindow() { return operation_window; }
		ReferenceWindow	*referenceWindow() { return reference_window; }
		VecImageWindow	*veciamgeWindow() { return veciamge_window; }
		VecEditing		*vecEditing() { return editing; }
		QString			sourceFilesPath() { return source_files_path; }

		void read_edit_data();
		void read_authoring_data();
		void set_interaction_status(int v) { actionList->uncheckedOtherMode(v); }

	private:
		void CreateMenus();
		void CreateMainActions();

		private slots:
		void color_selected();
		void color_selected_back();
		void right_color_selected();
		void left_color_selected();

		void set_window_visible(bool bVisible);
		void operation_window_close();
		void reference_window_close();
		void vecimage_window_close();


	private:
		Ui::mainwindowClass ui;

		//! List of actions.
		ActionList						*actionList;

		QString							source_files_path;

		int								app_width;
		int								app_height;

		QMenu							*fileMenu;
		QMenu							*viewMenu;
	
		QAction							*operation_window_Action;
		QAction							*reference_window_Action;
		QAction							*vectimage_window_Action;

		//for authoring
		QPushButton						*rightDir_button;//activated for normal edges
		QPushButton						*leftDir_button;

		//for editing
		QPushButton						*color_button;
		QPushButton						*backcolor_button;

		QMdiArea						*mdi;
		OperationWindow					*operation_window;
		ReferenceWindow					*reference_window;
		VecImageWindow					*veciamge_window;

		VecEditing						*editing;
	};

}
#endif // MAINWINDOW_H

