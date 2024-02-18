/*********************************************************************
 *  AUTHOR: Tomas Soltys                                             *
 *  FILE:   action_list.cpp                                          *
 *  GROUP:  Range                                                    *
 *  TYPE:   source file (*.cpp)                                      *
 *  DATE:   16-th January 2012                                       *
 *                                                                   *
 *  DESCRIPTION: Action list class definition                        *
 *********************************************************************/

#include <QKeySequence>

#include "action_list.h"
#include "mainwindow.h"

namespace EDITGUI {
	ActionList::ActionList(MainWindow *mainw,QObject *parent)
		: QObject(parent),mainWindow(mainw)
	{
		this->actions.resize(ACTION_N_TYPES);

		for (uint i = 0; i < ACTION_N_TYPES; i++)
		{
			if (ActionType(i) != ACTION_SEPARATOR)
			{
				this->actions[i] = new Action(ActionType(i), mainWindow, this);
			}
		}

		this->processAvailability();
	}

	uint ActionList::getNActions(void) const
	{
		return this->actions.size();
	}

	Action * ActionList::getAction(ActionType type)
	{
		if (type == ACTION_SEPARATOR)
		{
			return new Action(ACTION_SEPARATOR, mainWindow, this);
		}
		else
		{
			return this->actions[type];
		}
	}

	Action * ActionList::getAction(const QString &name)
	{
		for (QVector<Action*>::size_type i = 0; i != this->actions.size(); i++)
		{
			if (this->actions[i] == 0)
			{
				continue;
			}
			if (this->actions[i]->objectName() == name)
			{
				return this->getAction(ActionType(i));
			}
		}
		return 0;
	}

	void ActionList::processAvailability(void)
	{
		this->setDisabled(false);

		//this->getAction(ACTION_GEOMETRY_UNDO)->setDisabled(!Session::getInstance().isUndoAvailable());
		//this->getAction(ACTION_GEOMETRY_UNDO)->setToolTip(Session::getInstance().getUndoTooltip());
		//this->getAction(ACTION_GEOMETRY_REDO)->setDisabled(!Session::getInstance().isRedoAvailable());
		//this->getAction(ACTION_GEOMETRY_REDO)->setToolTip(Session::getInstance().getRedoTooltip());

		this->getAction(ACTION_IMAGE_OPEN)->setEnabled(true);
		this->getAction(ACTION_AUTHORING_OPEN)->setEnabled(true);

		//if exists image
		if (mainWindow->vecEditing() != NULL)
		{
			this->getAction(ACTION_IMAGE_SAVE)->setEnabled(true);
			this->getAction(ACTION_AUTHORING_SAVE)->setEnabled(true);
			this->getAction(ACTION_IMAGE_SAVE_AS)->setEnabled(true);
			this->getAction(ACTION_IMAGE_CLOSE)->setEnabled(true);
			this->getAction(ACTION_VIEW_CM_POINTS)->setEnabled(true);
			this->getAction(ACTION_VIEW_CM_EDGES)->setEnabled(true);
			this->getAction(ACTION_VIEW_CM_FEATURES)->setEnabled(true);
			this->getAction(ACTION_VIEW_SYNC)->setEnabled(true);

			this->getAction(ACTION_EDITING_UNDO)->setEnabled(true);
			this->getAction(ACTION_EDITING_REDO)->setEnabled(true);

			this->getAction(ACTION_INSERT_POINTS)->setEnabled(true);
			this->getAction(ACTION_INSERT_NORMAL_EDGES)->setEnabled(true);
			this->getAction(ACTION_INSERT_CREASE_EDGES)->setEnabled(true);
			this->getAction(ACTION_INSERT_CREASE_SHAPE)->setEnabled(true);

			this->getAction(ACTION_SELECT_BRUSH)->setEnabled(true);
			this->getAction(ACTION_SELECT_FEATURE)->setEnabled(true);
			this->getAction(ACTION_SELECT_SELFPOLY)->setEnabled(true);
			this->getAction(ACTION_SELECT_SQUARE)->setEnabled(true);
			this->getAction(ACTION_SELECT_INVERT)->setEnabled(true);
			this->getAction(ACTION_SELECT_BACKUP)->setEnabled(true);
			this->getAction(ACTION_DIFF_SELECTION)->setEnabled(true);

			this->getAction(ACTION_EIDT_COLOR)->setEnabled(true);
			this->getAction(ACTION_EIDT_COLOR_GREEN_CHANNEL)->setEnabled(true);
			this->getAction(ACTION_EIDT_COLOR_RED_CHANNEL)->setEnabled(true);
			this->getAction(ACTION_EIDT_COLOR_BLUE_CHANNEL)->setEnabled(true);
			this->getAction(ACTION_EIDT_COLOR_BLOCK)->setEnabled(true);
			this->getAction(ACTION_EIDT_COLOR_SLIDER)->setEnabled(true);

			this->getAction(ACTION_EIDT_ROTATE)->setEnabled(true);
			this->getAction(ACTION_EIDT_SCALE)->setEnabled(true);
			this->getAction(ACTION_EIDT_TRANS)->setEnabled(true);

			this->getAction(ACTION_EIDT_CPS_TRANS)->setEnabled(true);
			this->getAction(ACTION_EIDT_CYCLE_CW_ROTATE)->setEnabled(true);
			this->getAction(ACTION_EIDT_CYCLE_CCW_ROTATE)->setEnabled(true);
			this->getAction(ACTION_EIDT_CYCLE_ROOM_IN)->setEnabled(true);
			this->getAction(ACTION_EIDT_CYCLE_ROOM_OUT)->setEnabled(true);
			this->getAction(ACTION_EIDT_CYCLE_TRANS)->setEnabled(true);
		}
	}

	void ActionList::enable(void)
	{
		this->processAvailability();
	}

	void ActionList::disable(void)
	{
		this->setDisabled(true);
	}

	void ActionList::uncheckedOtherMode(int v)
	{
		bool enbale_ = v == BRUSH ? true : false;
		this->getAction(ACTION_SELECT_BRUSH)->setChecked(enbale_);

		enbale_ = v == FEATURE ? true : false;
		this->getAction(ACTION_SELECT_FEATURE)->setChecked(enbale_);

		enbale_ = v == SELFPOLYGON ? true : false;
		this->getAction(ACTION_SELECT_SELFPOLY)->setChecked(enbale_);

		enbale_ = v == SQUARE ? true : false;
		this->getAction(ACTION_SELECT_SQUARE)->setChecked(enbale_);

		enbale_ = v == BACK_UP ? true : false;
		this->getAction(ACTION_SELECT_BACKUP)->setChecked(enbale_);

		enbale_ = v == COLOR ? true : false;
		this->getAction(ACTION_EIDT_COLOR)->setChecked(enbale_);

		enbale_ = v == INSERT_P ? true : false;
		this->getAction(ACTION_INSERT_POINTS)->setChecked(enbale_);

		enbale_ = v == INSERT_NORMAL_E ? true : false;
		this->getAction(ACTION_INSERT_NORMAL_EDGES)->setChecked(enbale_);

		enbale_ = v == INSERT_CREASE_E ? true : false;
		this->getAction(ACTION_INSERT_CREASE_EDGES)->setChecked(enbale_);

		enbale_ = v == INSERT_CREASE_SHAPE ? true : false;
		this->getAction(ACTION_INSERT_CREASE_SHAPE)->setChecked(enbale_);

		enbale_ = v == COLOR_BLOCK ? true : false;
		this->getAction(ACTION_EIDT_COLOR_BLOCK)->setChecked(enbale_);

		enbale_ = v == COLOR_SLIDER ? true : false;
		this->getAction(ACTION_EIDT_COLOR_SLIDER)->setChecked(enbale_);

		enbale_ = v == ROTATE ? true : false;
		this->getAction(ACTION_EIDT_ROTATE)->setChecked(enbale_);

		enbale_ = v == SCALE ? true : false;
		this->getAction(ACTION_EIDT_SCALE)->setChecked(enbale_);

		enbale_ = v == TRANSLATION ? true : false;
		this->getAction(ACTION_EIDT_TRANS)->setChecked(enbale_);

		enbale_ = v == RED_CHANNEL ? true : false;
		this->getAction(ACTION_EIDT_COLOR_RED_CHANNEL)->setChecked(enbale_);

		enbale_ = v == GREEN_CHANNEL ? true : false;
		this->getAction(ACTION_EIDT_COLOR_GREEN_CHANNEL)->setChecked(enbale_);

		enbale_ = v == BLUE_CHANNEL ? true : false;
		this->getAction(ACTION_EIDT_COLOR_BLUE_CHANNEL)->setChecked(enbale_);

		enbale_ = v == SINGLE_CP ? true : false;
		this->getAction(ACTION_EIDT_CPS_TRANS)->setChecked(enbale_);

		enbale_ = v == CWARP_CWROTATE ? true : false;
		this->getAction(ACTION_EIDT_CYCLE_CW_ROTATE)->setChecked(enbale_);

		enbale_ = v == CWARP_CCWROTATE ? true : false;
		this->getAction(ACTION_EIDT_CYCLE_CCW_ROTATE)->setChecked(enbale_);

		enbale_ = v == CWARP_ROOMIN ? true : false;
		this->getAction(ACTION_EIDT_CYCLE_ROOM_IN)->setChecked(enbale_);

		enbale_ = v == CWARP_ROOMOUT ? true : false;
		this->getAction(ACTION_EIDT_CYCLE_ROOM_OUT)->setChecked(enbale_);

		enbale_ = v == CWARP_TRANS ? true : false;
		this->getAction(ACTION_EIDT_CYCLE_TRANS)->setChecked(enbale_);
	}

	void ActionList::setDisabled(bool allActions)
	{
		bool enabled = false;

		if (allActions)
		{
			this->getAction(ACTION_IMAGE_OPEN)->setEnabled(enabled);
			this->getAction(ACTION_AUTHORING_OPEN)->setEnabled(enabled);
		}
		this->getAction(ACTION_IMAGE_SAVE)->setEnabled(enabled);
		this->getAction(ACTION_AUTHORING_SAVE)->setEnabled(enabled);
		this->getAction(ACTION_IMAGE_SAVE_AS)->setEnabled(enabled);
		this->getAction(ACTION_IMAGE_CLOSE)->setEnabled(enabled);
		this->getAction(ACTION_EDITING_UNDO)->setEnabled(enabled);
		this->getAction(ACTION_EDITING_REDO)->setEnabled(enabled);
		this->getAction(ACTION_VIEW_CM_POINTS)->setEnabled(enabled);
		this->getAction(ACTION_VIEW_CM_EDGES)->setEnabled(enabled);
		this->getAction(ACTION_VIEW_CM_FEATURES)->setEnabled(enabled);
		this->getAction(ACTION_VIEW_SYNC)->setEnabled(enabled);

		this->getAction(ACTION_INSERT_POINTS)->setEnabled(enabled);
		this->getAction(ACTION_INSERT_NORMAL_EDGES)->setEnabled(enabled);
		this->getAction(ACTION_INSERT_CREASE_EDGES)->setEnabled(enabled);
		this->getAction(ACTION_INSERT_CREASE_SHAPE)->setEnabled(enabled);

		this->getAction(ACTION_SELECT_BRUSH)->setEnabled(enabled);
		this->getAction(ACTION_SELECT_FEATURE)->setEnabled(enabled);
		this->getAction(ACTION_SELECT_SELFPOLY)->setEnabled(enabled);
		this->getAction(ACTION_SELECT_SQUARE)->setEnabled(enabled);
		this->getAction(ACTION_SELECT_INVERT)->setEnabled(enabled);

		this->getAction(ACTION_SELECT_BACKUP)->setEnabled(enabled);
		this->getAction(ACTION_DIFF_SELECTION)->setEnabled(enabled);

		this->getAction(ACTION_EIDT_COLOR)->setEnabled(enabled);
		this->getAction(ACTION_EIDT_COLOR_GREEN_CHANNEL)->setEnabled(enabled);
		this->getAction(ACTION_EIDT_COLOR_RED_CHANNEL)->setEnabled(enabled);
		this->getAction(ACTION_EIDT_COLOR_BLUE_CHANNEL)->setEnabled(enabled);
		this->getAction(ACTION_EIDT_COLOR_BLOCK)->setEnabled(enabled);
		this->getAction(ACTION_EIDT_COLOR_SLIDER)->setEnabled(enabled);

		this->getAction(ACTION_EIDT_ROTATE)->setEnabled(enabled);
		this->getAction(ACTION_EIDT_SCALE)->setEnabled(enabled);
		this->getAction(ACTION_EIDT_TRANS)->setEnabled(enabled);

		this->getAction(ACTION_EIDT_CPS_TRANS)->setEnabled(enabled);
		this->getAction(ACTION_EIDT_CYCLE_CW_ROTATE)->setEnabled(enabled);
		this->getAction(ACTION_EIDT_CYCLE_CCW_ROTATE)->setEnabled(enabled);
		this->getAction(ACTION_EIDT_CYCLE_ROOM_IN)->setEnabled(enabled);
		this->getAction(ACTION_EIDT_CYCLE_ROOM_OUT)->setEnabled(enabled);
		this->getAction(ACTION_EIDT_CYCLE_TRANS)->setEnabled(enabled);
	}

	void ActionList::changeShortcut(ActionType actionType, const QString &shortcut)
	{
		this->getAction(actionType)->setShortcut(QKeySequence(shortcut));
	}

}
