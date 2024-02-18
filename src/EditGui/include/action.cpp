/*********************************************************************
 *  AUTHOR: Tomas Soltys                                             *
 *  FILE:   action.cpp                                               *
 *  GROUP:  Range                                                    *
 *  TYPE:   source file (*.cpp)                                      *
 *  DATE:   8-th August 2013                                         *
 *                                                                   *
 *  DESCRIPTION: Action class definition                             *
 *********************************************************************/

#include <QString>
#include <QApplication>
#include <QMainWindow>
#include <QMessageBox>
#include <QFileDialog>

#include "action.h"
#include "mainwindow.h"

namespace EDITGUI {
	Action::Action(ActionType type, MainWindow *mainWindow, QObject *parent) :
		QAction(parent),
		type(type),
		mainWindow(mainWindow)
	{
		const ActionDefinition actionDefinition;
		this->setCheckable(false);

		if (this->type == ACTION_NONE)
		{
			return;
		}
		else if (this->type == ACTION_SEPARATOR)
		{
			this->setSeparator(true);
			return;
		}

		this->setText(tr(actionDefinition.getText(this->type).toUtf8().constData()));
		this->setToolTip(tr(actionDefinition.getDescription(this->type).toUtf8().constData()));
		this->setShortcut(tr(actionDefinition.getShortCut(this->type).toUtf8().constData()));
		if (!actionDefinition.getIcon(this->type).isEmpty())
		{
			this->setIcon(QIcon(actionDefinition.getIcon(this->type).toUtf8().constData()));
			this->setIconVisibleInMenu(true);
		}

		if (type < ACTION_VIEW_CM_POINTS || type == ACTION_SELECT_INVERT ||
			type == ACTION_DIFF_SELECTION)
		{
			QObject::connect(this, &Action::triggered, this, actionDefinition.getSlot(this->type));
		}
		else
		{
			this->setCheckable(true);
			QObject::connect(this, &Action::toggled, this, actionDefinition.getSlot(this->type));
		}
	}

	void Action::enable()
	{
		this->setEnabled(true);
	}

	void Action::disable()
	{
		this->setDisabled(true);
	}

	void Action::onImageOpen()
	{
		mainWindow->read_edit_data();
	}

	void Action::onImageSave()
	{
		mainWindow->vecEditing()->save_editing_data();
	}

	void Action::onAuthoringImageOpen()
	{
		mainWindow->read_authoring_data();
	}

	void Action::onAuthoringImageSave()
	{
		mainWindow->vecEditing()->save_authoring_data();
	}

	void Action::onImageSaveAs()
	{
		QString path_ = mainWindow->sourceFilesPath();
		QString fileName = QFileDialog::getSaveFileName(mainWindow,
			tr("Select File..."), path_, tr("Model Files(*.edit *.author ...)"));
		if (!fileName.isEmpty())
		{
			if (mainWindow->vecEditing()->is_editing())
				mainWindow->vecEditing()->save_editing_data(&fileName);
			else
				mainWindow->vecEditing()->save_authoring_data(&fileName);
		}	
	}

	void Action::onImageClose()
	{
	}

	void Action::onInsertPoint()
	{
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(INSERT_P);
			mainWindow->set_interaction_status(INSERT_P);
		}
	}

	void Action::onInsertNormalEdge()
	{
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(INSERT_NORMAL_E);
			mainWindow->set_interaction_status(INSERT_NORMAL_E);
		}
	}

	void Action::onInsertCreaseEdge()
	{
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(INSERT_CREASE_E);
			mainWindow->set_interaction_status(INSERT_CREASE_E);
		}
	}

	void Action::onInsertCreaseShape()
	{
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(INSERT_CREASE_SHAPE);
			mainWindow->set_interaction_status(INSERT_CREASE_SHAPE);
		}
	}

	void Action::onEditingUndo() {

	}

	void Action::onEditingRedo() {

	}

	void Action::onShowCMpoints() {
		if (isChecked())
		{
			mainWindow->operationWindow()->check_cp_pts_view(true);
		}
		else
		{
			mainWindow->operationWindow()->check_cp_pts_view(false);
		}
	}

	void Action::onShowCMedges() {
		if (isChecked())
		{
			mainWindow->operationWindow()->check_cp_edge_view(true);
		}
		else
		{
			mainWindow->operationWindow()->check_cp_edge_view(false);
		}
	}

	void Action::onShowCMfeatures()
	{
		if (isChecked())
		{
			mainWindow->operationWindow()->check_feature_view(true);
		}
		else
		{
			mainWindow->operationWindow()->check_feature_view(false);
		}
	}

	void Action::onSyncViewers() {
		if (isChecked())
		{
			mainWindow->veciamgeWindow()->check_sync_view(true);
		}
		else
		{
			mainWindow->veciamgeWindow()->check_sync_view(false);
		}
	}

	void Action::onFeatureSelect() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(FEATURE);
			mainWindow->set_interaction_status(FEATURE);
		}	
	}

	void Action::onSquareSelect() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(SQUARE);
			mainWindow->set_interaction_status(SQUARE);
		}	
	}

	void Action::onPolygonSelect() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(SELFPOLYGON);
			mainWindow->set_interaction_status(SELFPOLYGON);
		}	
	}

	void Action::onBrushSelect() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(BRUSH);
			mainWindow->set_interaction_status(BRUSH);
		}	
	}

	void Action::onSelectInvert() {
		mainWindow->operationWindow()->viewer()->setSelectedInvert();
	}

	void Action::onSelectBackup()
	{
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(BACK_UP);
			mainWindow->set_interaction_status(BACK_UP);
		}
	}

	void Action::onDiffSelection()
	{
		mainWindow->operationWindow()->viewer()->setSelectedDiff();
	}

	void Action::onColorEdit() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(COLOR);
			mainWindow->set_interaction_status(COLOR);
		}		
	}

	void Action::onColorBlockEdit() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(COLOR_BLOCK);
			mainWindow->set_interaction_status(COLOR_BLOCK);
		}
	}

	void Action::onColorSliderEdit() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(COLOR_SLIDER);
			mainWindow->set_interaction_status(COLOR_SLIDER);
		}
	}	

	void Action::onColorRedEdit() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(RED_CHANNEL);
			mainWindow->set_interaction_status(RED_CHANNEL);
		}
	}

	void Action::onColorGreenEdit() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(GREEN_CHANNEL);
			mainWindow->set_interaction_status(GREEN_CHANNEL);
		}
	}

	void Action::onColorBlueEdit() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(BLUE_CHANNEL);
			mainWindow->set_interaction_status(BLUE_CHANNEL);
		}
	}

	void Action::onRotateEdit() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(ROTATE);
			mainWindow->set_interaction_status(ROTATE);
		}		
	}

	void Action::onScaleEdit() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(SCALE);
			mainWindow->set_interaction_status(SCALE);
		}
	}

	void Action::onTransEdit() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(TRANSLATION);
			mainWindow->set_interaction_status(TRANSLATION);
		}	
	}


	void Action::onSingleCPTransEdit()
	{
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(SINGLE_CP);
			mainWindow->set_interaction_status(SINGLE_CP);
		}
	}

	void Action::onCWarpCWRotateEdit() {
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(CWARP_CWROTATE);
			mainWindow->set_interaction_status(CWARP_CWROTATE);
		}
	}

	void Action::onCWarpCCWRotateEdit()
	{
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(CWARP_CCWROTATE);
			mainWindow->set_interaction_status(CWARP_CCWROTATE);
		}
	}

	void Action::onCWarpRoomInEdit()
	{
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(CWARP_ROOMIN);
			mainWindow->set_interaction_status(CWARP_ROOMIN);
		}
	}

	void Action::onCWarpRoomOutEdit()
	{
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(CWARP_ROOMOUT);
			mainWindow->set_interaction_status(CWARP_ROOMOUT);
		}
	}

	void Action::onCWarpTransEdit()
	{
		if (isChecked())
		{
			mainWindow->operationWindow()->viewer()->set_intersaction_status(CWARP_TRANS);
			mainWindow->set_interaction_status(CWARP_TRANS);
		}
	}

}
