/*********************************************************************
 *  AUTHOR: Tomas Soltys                                             *
 *  FILE:   action.h                                                 *
 *  GROUP:  Range                                                    *
 *  TYPE:   header file (*.h)                                        *
 *  DATE:   8-th August 2013                                         *
 *                                                                   *
 *  DESCRIPTION: Action class declaration                            *
 *********************************************************************/

#ifndef ACTION_H
#define ACTION_H

#include <QAction>

#include "action_definition.h"
namespace EDITGUI {
	class MainWindow;

	class Action : public QAction
	{

		Q_OBJECT

	protected:

		//! Action type.
		ActionType type;
		//! Pointer to main window.
		MainWindow *mainWindow;

	public:

		//! Constructor.
		explicit Action(ActionType type, MainWindow *mainWindow, QObject *parent = nullptr);

		protected slots:

		//! Set action enabled.
		void enable();

		//! Set action disabled.
		void disable();

		public slots:

		void onImageOpen();
		void onImageSave();
		void onAuthoringImageOpen();
		void onAuthoringImageSave();
		void onImageSaveAs();
		void onImageClose();

		void onInsertPoint();
		void onInsertNormalEdge();
		void onInsertCreaseEdge();
		void onInsertCreaseShape();

		void onEditingUndo();
		void onEditingRedo();

		void onShowCMpoints();
		void onShowCMedges();
		void onShowCMfeatures();
		void onSyncViewers();

		void onFeatureSelect();
		void onSquareSelect();
		void onPolygonSelect();
		void onBrushSelect();

		void onSelectInvert();
		void onSelectBackup();
		void onDiffSelection();

		void onColorEdit();
		void onColorRedEdit();
		void onColorGreenEdit();
		void onColorBlueEdit();
		void onColorBlockEdit();
		void onColorSliderEdit();

		void onRotateEdit();
		void onScaleEdit();
		void onTransEdit();

		void onSingleCPTransEdit();
		void onCWarpCWRotateEdit();
		void onCWarpCCWRotateEdit();
		void onCWarpRoomInEdit();
		void onCWarpRoomOutEdit();
		void onCWarpTransEdit();
	};
}
#endif // ACTION_H


