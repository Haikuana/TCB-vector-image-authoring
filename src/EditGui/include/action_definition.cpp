/*********************************************************************
 *  AUTHOR: Tomas Soltys                                             *
 *  FILE:   action_definition.cpp                                    *
 *  GROUP:  Range                                                    *
 *  TYPE:   source file (*.cpp)                                      *
 *  DATE:   9-th September 2014                                      *
 *                                                                   *
 *  DESCRIPTION: Action definition class definition                  *
 *********************************************************************/

#include "action_definition.h"
#include "action.h"
namespace EDITGUI {
	ActionDefinition::ActionDefinition(QObject *parent)
		: QObject(parent)
	{
		ActionDefinition::generateActionDescList(this->actionDesc);
	}

	ActionType ActionDefinition::findShortcutActionType(const QString &shortCut) const
	{
		for (int i = 0; i<int(ACTION_N_TYPES); i++)
		{
			if (this->actionDesc[i].shortCut == shortCut)
			{
				return ActionType(i);
			}
		}
		return ACTION_NONE;
	}

	const QString &ActionDefinition::getText(ActionType type) const
	{
		if (!ACTION_TYPE_IS_VALID(type))
		{
			//throw RError(R_ERROR_APPLICATION, R_ERROR_REF, "Invalid action type: &d\n", int(type));
		}
		return this->actionDesc[type].text;
	}

	const QString &ActionDefinition::getDescription(ActionType type) const
	{
		if (!ACTION_TYPE_IS_VALID(type))
		{
			//throw RError(R_ERROR_APPLICATION, R_ERROR_REF, "Invalid action type: &d\n", int(type));
		}
		return this->actionDesc[type].desc;
	}

	const QString &ActionDefinition::getShortCut(ActionType type) const
	{
		if (!ACTION_TYPE_IS_VALID(type))
		{
			//throw RError(R_ERROR_APPLICATION, R_ERROR_REF, "Invalid action type: &d\n", int(type));
		}
		return this->actionDesc[type].shortCut;
	}

	void ActionDefinition::setShortcut(ActionType type, const QString &shortCut)
	{
		if (!ACTION_TYPE_IS_VALID(type))
		{
			//throw RError(R_ERROR_APPLICATION, R_ERROR_REF, "Invalid action type: &d\n", int(type));
		}
		this->actionDesc[type].shortCut = shortCut;
		emit this->shortcutChanged(type, shortCut);
	}

	const QString &ActionDefinition::getIcon(ActionType type) const
	{
		if (!ACTION_TYPE_IS_VALID(type))
		{
			//throw RError(R_ERROR_APPLICATION, R_ERROR_REF, "Invalid action type: &d\n", int(type));
		}
		return this->actionDesc[type].icon;
	}

	PointerToMemberFunction ActionDefinition::getSlot(ActionType type) const
	{
		if (!ACTION_TYPE_IS_VALID(type))
		{
			//throw RError(R_ERROR_APPLICATION, R_ERROR_REF, "Invalid action type: &d\n", int(type));
		}
		return this->actionDesc[type].slot;
	}

	void ActionDefinition::generateActionDescList(QList<ActionDefinitionItem> &actionDesc)
	{
		actionDesc.clear();
		actionDesc.push_back(ActionDefinitionItem(ACTION_NONE, "", "", "", "", nullptr));
		actionDesc.push_back(ActionDefinitionItem(ACTION_IMAGE_OPEN, "Open Editable Image", "Open Editable Image.", "Ctrl+O", "Resources/image_open.png", &Action::onImageOpen));
		actionDesc.push_back(ActionDefinitionItem(ACTION_IMAGE_SAVE, "Save Editable Data", "Save Editable Data. ", "Ctrl+S", "Resources/image_save.png", &Action::onImageSave));
		actionDesc.push_back(ActionDefinitionItem(ACTION_AUTHORING_OPEN, "Open Authoring Image", "Open Authoring Image.", "Ctrl+O", "Resources/image_opena.png", &Action::onAuthoringImageOpen));
		actionDesc.push_back(ActionDefinitionItem(ACTION_AUTHORING_SAVE, "Save Authoring Data", "Save Authoring Data. ", "Ctrl+S", "Resources/image_savea.png", &Action::onAuthoringImageSave));
		actionDesc.push_back(ActionDefinitionItem(ACTION_IMAGE_SAVE_AS, "Save Data As", "Save selected model under a different filename.", "Ctrl+Shift+S", "Resources/saveas.png", &Action::onImageSaveAs));
		actionDesc.push_back(ActionDefinitionItem(ACTION_IMAGE_CLOSE, "Close Image", "", "Ctrl+W", "Resources/image_close.png", &Action::onImageClose));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EDITING_UNDO, "Undo", "", "Ctrl+Z", "Resources/undo.png", &Action::onEditingUndo));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EDITING_REDO, "Redo", "", "Ctrl+Shift+Z", "Resources/redo.png", &Action::onEditingRedo));
		actionDesc.push_back(ActionDefinitionItem(ACTION_VIEW_CM_POINTS, "Show Control Points", "", "Ctrl+1", "Resources/pointicon.png", &Action::onShowCMpoints));
		actionDesc.push_back(ActionDefinitionItem(ACTION_VIEW_CM_EDGES, "Show Control Mesh", "", "Ctrl+2", "Resources/triicon.png", &Action::onShowCMedges));
		actionDesc.push_back(ActionDefinitionItem(ACTION_VIEW_CM_FEATURES, "Show Features", "", "Ctrl+3", "Resources/tri_fea.png", &Action::onShowCMfeatures));
		actionDesc.push_back(ActionDefinitionItem(ACTION_VIEW_SYNC, "Sync Viewers", "", "Ctrl+3", "Resources/sync-icon.png", &Action::onSyncViewers));
		//SELECT
		actionDesc.push_back(ActionDefinitionItem(ACTION_SELECT_FEATURE, "Select Feature Polylines", "", "", "Resources/feaicon.png", &Action::onFeatureSelect));
		actionDesc.push_back(ActionDefinitionItem(ACTION_SELECT_SQUARE, "Select Square Region", "", "", "Resources/squareicon.png", &Action::onSquareSelect));
		actionDesc.push_back(ActionDefinitionItem(ACTION_SELECT_SELFPOLY, "Select Polygon Region", "", "", "Resources/addpolyicon.png", &Action::onPolygonSelect));
		actionDesc.push_back(ActionDefinitionItem(ACTION_SELECT_BRUSH, "Brush Mode", "", "", "Resources/brushicon.png", &Action::onBrushSelect));
		actionDesc.push_back(ActionDefinitionItem(ACTION_SELECT_INVERT, "Selected Invert", "", "", "Resources/invert.png", &Action::onSelectInvert));
		actionDesc.push_back(ActionDefinitionItem(ACTION_SELECT_BACKUP, "Selected Back Up", "", "", "Resources/backup.png", &Action::onSelectBackup));
		actionDesc.push_back(ActionDefinitionItem(ACTION_DIFF_SELECTION, "Selected Diff", "", "", "Resources/bool.png", &Action::onDiffSelection));

		//authoring
		actionDesc.push_back(ActionDefinitionItem(ACTION_INSERT_POINTS, "Insert Points", "", "", "Resources/insert_point.png", &Action::onInsertPoint));
		actionDesc.push_back(ActionDefinitionItem(ACTION_INSERT_NORMAL_EDGES, "Insert Normal Edges", "", "", "Resources/insert_normal_edge.png", &Action::onInsertNormalEdge));
		actionDesc.push_back(ActionDefinitionItem(ACTION_INSERT_CREASE_EDGES, "Insert Crease Edges", "", "", "Resources/insert_crease_edge.png", &Action::onInsertCreaseEdge));
		actionDesc.push_back(ActionDefinitionItem(ACTION_INSERT_CREASE_SHAPE, "Insert Crease Shape", "", "", "Resources/insert_crease_shape.png", &Action::onInsertCreaseShape));

		//edit
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_COLOR, "Edit Color", "", "", "Resources/drawcolor.png", &Action::onColorEdit));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_COLOR_RED_CHANNEL, "Edit Red Channel", "", "", "Resources/red.png", &Action::onColorRedEdit));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_COLOR_GREEN_CHANNEL, "Edit Green Channel", "", "", "Resources/green.png", &Action::onColorGreenEdit));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_COLOR_BLUE_CHANNEL, "Edit Blue Channel", "", "", "Resources/blue.png", &Action::onColorBlueEdit));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_COLOR_BLOCK, "Edit Color Block", "", "", "Resources/drawcolorblock.png", &Action::onColorBlockEdit));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_COLOR_SLIDER, "Edit Color Slider", "", "", "Resources/drawcolorslider.png", &Action::onColorSliderEdit));

		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_ROTATE, "Edit Rotate Shape", "", "", "Resources/rotate.png", &Action::onRotateEdit));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_SCALE, "Edit Scale Shape", "", "", "Resources/scale.png", &Action::onScaleEdit));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_TRANS, "Edit Translate Shape", "", "", "Resources/translation.png", &Action::onTransEdit));

		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_CPS_TRANS, "Move Single CP", "", "", "Resources/cps_move.png", &Action::onSingleCPTransEdit));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_CYCLE_CW_ROTATE, "CWarp CW-Rotate", "", "", "Resources/c_cw_rotate.png", &Action::onCWarpCWRotateEdit));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_CYCLE_CCW_ROTATE, "CWarp CCW-Rotate", "", "", "Resources/c_ccw_rotate.png", &Action::onCWarpCCWRotateEdit));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_CYCLE_ROOM_IN, "CWarp RoomIn", "", "", "Resources/c_scale_out.png", &Action::onCWarpRoomInEdit));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_CYCLE_ROOM_OUT, "CWarp RoomOut", "", "", "Resources/c_scale_in.png", &Action::onCWarpRoomOutEdit));
		actionDesc.push_back(ActionDefinitionItem(ACTION_EIDT_CYCLE_TRANS, "CWarp Translate", "", "", "Resources/c_trans.png", &Action::onCWarpTransEdit));

		actionDesc.push_back(ActionDefinitionItem(ACTION_SEPARATOR, "", "", "", "", nullptr));
	}

}