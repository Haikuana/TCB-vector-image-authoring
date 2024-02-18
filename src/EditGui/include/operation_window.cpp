#include "operation_window.h"

namespace EDITGUI {

	OperationWindow::OperationWindow()
	{
		editing_authoring = NULL;

		QWidget *widget = new QWidget;
		this->setWindowIcon(QIcon("Resources"));
		widget->setStyleSheet("QWidget{background-color: black}");
		setCentralWidget(widget);

		viewer_ = new Viewer(OPERATION_W);
		viewer_->setAttribute(Qt::WA_OpaquePaintEvent);
		viewer_->setAttribute(Qt::WA_NoSystemBackground);

		QGridLayout *layout_main = new QGridLayout;
		layout_main->setContentsMargins(0, 0, 0, 0);
		layout_main->addWidget(viewer_, 0, 0, -1, -1);

		widget->setLayout(layout_main);

		QTimer *timer = new QTimer(this);
		connect(timer, SIGNAL(timeout()), viewer_, SLOT(update_whole_viewer()));
		timer->start(1);

		setWindowTitle("Editing Operation");

	}

	void OperationWindow::set_editing_source(VecEditing *data)
	{
		editing_authoring = data;
		//updateStatusBar();
		viewer_->set_editing_source(data);
		viewer_->check_cppts_view(true);
		viewer_->check_cpedge_view(true);
	}

	void OperationWindow::closeEvent(QCloseEvent *event)
	{
		event->accept();
		emit window_close();
	}

}