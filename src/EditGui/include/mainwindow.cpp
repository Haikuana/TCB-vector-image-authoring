#include "mainwindow.h"
#include <QtGui>
#include <QtWidgets/QFileDialog>

namespace EDITGUI {

	MainWindow::MainWindow()
	{
		ui.setupUi(this);

		this->setStyleSheet(
			//"QWidget{background-image:url(Resources/3.png)}"
			"QToolButton:hover {background-color:lightgray}"
			"QPushButton {background-color: rgb(127, 0, 0)}"
			"QToolButton:checked {border-width: 2px;border-style: solid;border-color: rgb(30, 30, 150);}"
			//"QToolButton:!hover { background-color: white}"
		);

		mdi = new QMdiArea;
		setCentralWidget(mdi);
		mdi->cascadeSubWindows();

		char rootPath[MAX_PATH];
		GetModuleFileName(NULL, rootPath, MAX_PATH);
		source_files_path = QString(rootPath);
		QStringList dataColumns = source_files_path.split("\\");
		dataColumns.pop_back();
		dataColumns.pop_back();
		dataColumns.push_back("Results");
		source_files_path = dataColumns.join('/');

		QScreen *screen = QGuiApplication::primaryScreen();
		QRect mm = screen->availableGeometry();
		app_width = mm.width()*0.8;
		app_height = mm.height()*0.95;

		this->setGeometry(0.02*mm.width(), 0.04*mm.height(), app_width, app_height);

		//showFullScreen();

		editing = NULL;

		veciamge_window = new VecImageWindow();
		veciamge_window->setAttribute(Qt::WA_DeleteOnClose);
		mdi->addSubWindow(veciamge_window);

		reference_window = new ReferenceWindow();
		reference_window->setAttribute(Qt::WA_DeleteOnClose);
		//mdi->addSubWindow(reference_window);

		operation_window = new OperationWindow();
		operation_window->setAttribute(Qt::WA_DeleteOnClose);
		mdi->addSubWindow(operation_window);

		//mdi->tileSubWindows();

		QGridLayout *window_lay = new QGridLayout;
		window_lay->setContentsMargins(0, 0, 0, 0);
		window_lay->addWidget(operation_window, 0, 0);
		window_lay->addWidget(veciamge_window, 0, 1);
		window_lay->setColumnStretch(0, 1);
		window_lay->setColumnStretch(1, 1);
		mdi->setLayout(window_lay);

		connect(operation_window, SIGNAL(window_close()), this, SLOT(operation_window_close()));
		connect(reference_window, SIGNAL(window_close()), this, SLOT(reference_window_close()));
		connect(veciamge_window, SIGNAL(window_close()), this, SLOT(vecimage_window_close()));

		this->actionList = new EDITGUI::ActionList(this);
		CreateMenus();
		CreateMainActions();

		setWindowTitle("Vector Image Editing and Authoring Framework");
		setWindowIcon(QIcon("Resources/windowicon.png"));
		//setStyleSheet("QWidget{background-image:url(Resources/3.png)}");
	}

	MainWindow::~MainWindow()
	{
		if (editing)
			delete editing;
	}

	void MainWindow::CreateMenus()
	{
		fileMenu = menuBar()->addMenu(tr("Read"));
		fileMenu->addAction(this->actionList->getAction(EDITGUI::ACTION_IMAGE_OPEN));
		fileMenu->addAction(this->actionList->getAction(EDITGUI::ACTION_AUTHORING_OPEN));

		fileMenu = menuBar()->addMenu(tr("Write"));
		fileMenu->addAction(this->actionList->getAction(EDITGUI::ACTION_IMAGE_SAVE));
		fileMenu->addAction(this->actionList->getAction(EDITGUI::ACTION_AUTHORING_SAVE));
		fileMenu->addAction(this->actionList->getAction(EDITGUI::ACTION_IMAGE_SAVE_AS));
	}

	void MainWindow::CreateMainActions()
	{
		QToolBar *mainToolBar = new QToolBar(this);
		mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
		mainToolBar->setWindowTitle(tr("Main toolbar"));
		this->addToolBar(Qt::TopToolBarArea, mainToolBar);
		mainToolBar->addAction(this->actionList->getAction(EDITGUI::ACTION_IMAGE_OPEN));
		mainToolBar->addAction(this->actionList->getAction(EDITGUI::ACTION_IMAGE_SAVE));
		mainToolBar->addAction(this->actionList->getAction(EDITGUI::ACTION_AUTHORING_OPEN));
		mainToolBar->addAction(this->actionList->getAction(EDITGUI::ACTION_AUTHORING_SAVE));
		mainToolBar->addAction(this->actionList->getAction(EDITGUI::ACTION_IMAGE_SAVE_AS));
		mainToolBar->addAction(this->actionList->getAction(EDITGUI::ACTION_IMAGE_CLOSE));

		QToolBar *viewTool = new QToolBar(this);
		viewTool->setObjectName(QString::fromUtf8("ViewBar"));
		viewTool->setWindowTitle(tr("View toolbar"));
		this->addToolBar(Qt::TopToolBarArea, viewTool);
		viewTool->addAction(this->actionList->getAction(EDITGUI::ACTION_VIEW_CM_POINTS));
		viewTool->addAction(this->actionList->getAction(EDITGUI::ACTION_VIEW_CM_EDGES));
		viewTool->addAction(this->actionList->getAction(EDITGUI::ACTION_VIEW_CM_FEATURES));
		viewTool->addAction(this->actionList->getAction(EDITGUI::ACTION_VIEW_SYNC));
		viewTool->setFloatable(true);

		QToolBar *selectTool = new QToolBar(this);
		selectTool->setObjectName(QString::fromUtf8("SelcetBar"));
		selectTool->setWindowTitle(tr("Selection toolbar"));
		this->addToolBar(Qt::TopToolBarArea, selectTool);
		selectTool->addAction(this->actionList->getAction(EDITGUI::ACTION_SELECT_SQUARE));
		selectTool->addAction(this->actionList->getAction(EDITGUI::ACTION_SELECT_BRUSH));
		selectTool->addAction(this->actionList->getAction(EDITGUI::ACTION_SELECT_FEATURE));
		selectTool->addAction(this->actionList->getAction(EDITGUI::ACTION_SELECT_SELFPOLY));		
		selectTool->setFloatable(true);		

		QToolBar* processBar = new QToolBar(this);
		processBar->setObjectName(QString::fromUtf8("ProcessBar"));
		processBar->setWindowTitle(tr("process"));
		this->addToolBar(Qt::TopToolBarArea, processBar);
		processBar->addAction(this->actionList->getAction(EDITGUI::ACTION_EDITING_REDO));
		processBar->addAction(this->actionList->getAction(EDITGUI::ACTION_EDITING_UNDO));
		processBar->setFloatable(true);

		rightDir_button = new QPushButton(this);
		rightDir_button->setCursor(QCursor(Qt::PointingHandCursor));
		rightDir_button->setIcon(QIcon("Resources/colorbar.png"));
		rightDir_button->setCheckable(false);
		rightDir_button->setToolTip("Select a color for right");
		rightDir_button->setContentsMargins(0, 0, 0, 0);
		rightDir_button->setEnabled(false);
		connect(rightDir_button, SIGNAL(clicked()), this, SLOT(right_color_selected()));

		leftDir_button = new QPushButton(this);
		leftDir_button->setCursor(QCursor(Qt::PointingHandCursor));
		leftDir_button->setIcon(QIcon("Resources/colorbar.png"));
		leftDir_button->setCheckable(false);
		leftDir_button->setToolTip("Select a color for left");
		leftDir_button->setContentsMargins(0, 0, 0, 0);
		leftDir_button->setEnabled(false);
		connect(leftDir_button, SIGNAL(clicked()), this, SLOT(left_color_selected()));

		QToolBar* authoringTool = new QToolBar(this);
		authoringTool->setObjectName(QString::fromUtf8("AuthoringBar"));
		authoringTool->setWindowTitle(tr("Authoring toolbar"));
		this->addToolBar(Qt::TopToolBarArea, authoringTool);
		authoringTool->addWidget(rightDir_button);
		authoringTool->addWidget(leftDir_button);
		authoringTool->addAction(this->actionList->getAction(EDITGUI::ACTION_INSERT_POINTS));
		authoringTool->addAction(this->actionList->getAction(EDITGUI::ACTION_INSERT_NORMAL_EDGES));
		authoringTool->addAction(this->actionList->getAction(EDITGUI::ACTION_INSERT_CREASE_EDGES));
		authoringTool->addAction(this->actionList->getAction(EDITGUI::ACTION_INSERT_CREASE_SHAPE));
		authoringTool->setFloatable(true);

		this->insertToolBarBreak(authoringTool);

		color_button = new QPushButton(this);
		color_button->setCursor(QCursor(Qt::PointingHandCursor));
		color_button->setIcon(QIcon("Resources/colorbar.png"));
		color_button->setCheckable(false);
		color_button->setToolTip("Select a color");
		color_button->setContentsMargins(0, 0, 0, 0);
		color_button->setEnabled(false);
		connect(color_button, SIGNAL(clicked()), this, SLOT(color_selected()));

		backcolor_button = new QPushButton(this);
		backcolor_button->setCursor(QCursor(Qt::PointingHandCursor));
		backcolor_button->setIcon(QIcon("Resources/colorbar.png"));
		backcolor_button->setCheckable(false);
		backcolor_button->setToolTip("Select a back color");
		backcolor_button->setContentsMargins(0, 0, 0, 0);
		backcolor_button->setEnabled(false);
		connect(backcolor_button, SIGNAL(clicked()), this, SLOT(color_selected_back()));

		QToolBar *editTool = new QToolBar(this);
		editTool->setObjectName(QString::fromUtf8("EditBar"));
		editTool->setWindowTitle(tr("Edition toolbar"));
		this->addToolBar(Qt::TopToolBarArea, editTool);
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_SELECT_BACKUP));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_SELECT_INVERT));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_DIFF_SELECTION));
		editTool->insertSeparator(NULL);
		editTool->addWidget(color_button);
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_COLOR));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_COLOR_RED_CHANNEL));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_COLOR_GREEN_CHANNEL));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_COLOR_BLUE_CHANNEL));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_COLOR_BLOCK));
		editTool->addWidget(backcolor_button);
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_COLOR_SLIDER));
		editTool->insertSeparator(NULL);
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_ROTATE));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_SCALE));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_TRANS));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_CPS_TRANS));
		editTool->insertSeparator(NULL);
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_CYCLE_CW_ROTATE));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_CYCLE_CCW_ROTATE));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_CYCLE_ROOM_IN));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_CYCLE_ROOM_OUT));
		editTool->addAction(this->actionList->getAction(EDITGUI::ACTION_EIDT_CYCLE_TRANS));
		editTool->setFloatable(true);
	}

	void MainWindow::read_edit_data()
	{
		if (!editing)
			editing = new VecEditing;
		else
		{
			delete editing;
			editing = new VecEditing;
		}
		QString fileName = QFileDialog::getOpenFileName(this,
			tr("Select File..."), source_files_path, tr("Model Files(*.edit ...)"));
		if (!fileName.isEmpty())
		{
			//if(editing->convert2concise(fileName))
			if (editing->load_editing_data(fileName))
			{
				printf("%s: read data done \n", __FUNCTION__);
				actionList->enable();
				operation_window->set_editing_source(editing);
				veciamge_window->set_editing_source(editing);
				
				this->actionList->getAction(EDITGUI::ACTION_VIEW_CM_POINTS)->setChecked(true);
				this->actionList->getAction(EDITGUI::ACTION_VIEW_CM_EDGES)->setChecked(true);
				this->actionList->getAction(EDITGUI::ACTION_VIEW_SYNC)->setChecked(true);
				this->actionList->getAction(EDITGUI::ACTION_SELECT_SQUARE)->setChecked(true);
				operation_window->viewer()->set_intersaction_status(SQUARE);
				color_button->setEnabled(true);
				backcolor_button->setEnabled(true);
				return;
			}
		}
		return;
	}

	void MainWindow::read_authoring_data()
	{
		if (!editing)
			editing = new VecEditing;
		else
		{
			delete editing;
			editing = new VecEditing;
		}
		QString fileName = QFileDialog::getOpenFileName(this,
			tr("Select File..."), source_files_path, tr("Model Files(*.author ...)"));
		if (!fileName.isEmpty())
		{
			//if (editing->convert2concise(fileName))
			if (editing->load_authoring_data(fileName))			
			{
				printf("%s: read data done \n", __FUNCTION__);
				actionList->enable();
				operation_window->set_editing_source(editing);
				veciamge_window->set_editing_source(editing);

				this->actionList->getAction(EDITGUI::ACTION_VIEW_CM_POINTS)->setChecked(true);
				this->actionList->getAction(EDITGUI::ACTION_VIEW_CM_EDGES)->setChecked(true);
				this->actionList->getAction(EDITGUI::ACTION_VIEW_CM_FEATURES)->setChecked(true);
				this->actionList->getAction(EDITGUI::ACTION_VIEW_SYNC)->setChecked(true);
				this->actionList->getAction(EDITGUI::ACTION_INSERT_POINTS)->setChecked(true);
				operation_window->viewer()->set_intersaction_status(INSERT_P);
				rightDir_button->setEnabled(true);
				leftDir_button->setEnabled(true);
				return;
			}
		}
		return;
	}

	void MainWindow::color_selected() {
		QColor color = QColorDialog::getColor();
		color_button->setStyleSheet(QString("QPushButton {"
			"background-color: %1"
			"}")
			.arg(color.name()));
		editing->get_edit_color()[0] = color.redF();
		editing->get_edit_color()[1] = color.greenF();
		editing->get_edit_color()[2] = color.blueF();
	}

	void MainWindow::color_selected_back() {
		QColor color = QColorDialog::getColor();
		backcolor_button->setStyleSheet(QString("QPushButton {"
			"background-color: %1"
			"}")
			.arg(color.name()));
		editing->get_edit_color_backg()[0] = color.redF();
		editing->get_edit_color_backg()[1] = color.greenF();
		editing->get_edit_color_backg()[2] = color.blueF();
	}

	void MainWindow::right_color_selected()
	{
		QColor color = QColorDialog::getColor();
		rightDir_button->setStyleSheet(QString("QPushButton {"
			"background-color: %1"
			"}")
			.arg(color.name()));
		editing->get_authoring_right_color()[0] = color.redF();
		editing->get_authoring_right_color()[1] = color.greenF();
		editing->get_authoring_right_color()[2] = color.blueF();
	}

	void MainWindow::left_color_selected()
	{
		QColor color = QColorDialog::getColor();
		leftDir_button->setStyleSheet(QString("QPushButton {"
			"background-color: %1"
			"}")
			.arg(color.name()));
		editing->get_authoring_left_color()[0] = color.redF();
		editing->get_authoring_left_color()[1] = color.greenF();
		editing->get_authoring_left_color()[2] = color.blueF();
	}

	void MainWindow::set_window_visible(bool bVisible)
	{
		QObject *send = sender();
		if (send == operation_window_Action)
		{
			mdi->setActiveSubWindow(qobject_cast<QMdiSubWindow *>(operation_window));
		}
		else if (send == reference_window_Action)
		{
			mdi->setActiveSubWindow(qobject_cast<QMdiSubWindow *>(reference_window));
		}
	}

	void MainWindow::operation_window_close()
	{
		delete operation_window;
		operation_window = NULL;
	}

	void MainWindow::reference_window_close()
	{
		delete reference_window;
		reference_window = NULL;
	}

	void MainWindow::vecimage_window_close()
	{
		delete veciamge_window;
		veciamge_window = NULL;
	}




}