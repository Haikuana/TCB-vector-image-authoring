/*********************************************************************
 *  AUTHOR: Tomas Soltys                                             *
 *  FILE:   action_definition_item.h                                 *
 *  GROUP:  Range                                                    *
 *  TYPE:   header file (*.h)                                        *
 *  DATE:   10-th September 2014                                     *
 *                                                                   *
 *  DESCRIPTION: Action definition item class declaration            *
 *********************************************************************/


#ifndef ACTION_DEFINITION_ITEM_H
#define ACTION_DEFINITION_ITEM_H

#include <QString>

namespace EDITGUI {

#define ACTION_TYPE_IS_VALID(_type) \
            ( _type >= ACTION_NONE && _type < ACTION_N_TYPES )

#define ACTION_TYPE_IS_ACTION(_type) \
            ( _type > ACTION_NONE && _type < ACTION_N_TYPES && _type != ACTION_SEPARATOR )

	typedef enum _ActionType
	{
		ACTION_NONE = 0,
		ACTION_IMAGE_OPEN,
		ACTION_IMAGE_SAVE,
		ACTION_AUTHORING_OPEN,
		ACTION_AUTHORING_SAVE,
		ACTION_IMAGE_SAVE_AS,
		ACTION_IMAGE_CLOSE,
		ACTION_EDITING_UNDO,
		ACTION_EDITING_REDO,
		ACTION_VIEW_CM_POINTS,//view
		ACTION_VIEW_CM_EDGES,
		ACTION_VIEW_CM_FEATURES,
		ACTION_VIEW_SYNC,
		ACTION_SELECT_FEATURE,//select
		ACTION_SELECT_SQUARE,
		ACTION_SELECT_SELFPOLY,
		ACTION_SELECT_BRUSH,
		ACTION_SELECT_INVERT,
		ACTION_SELECT_BACKUP,
		ACTION_DIFF_SELECTION,
		ACTION_INSERT_POINTS,//authoring
		ACTION_INSERT_NORMAL_EDGES,
		ACTION_INSERT_CREASE_EDGES,
		ACTION_INSERT_CREASE_SHAPE,
		ACTION_EIDT_COLOR,//EDIT
		ACTION_EIDT_COLOR_RED_CHANNEL,
		ACTION_EIDT_COLOR_GREEN_CHANNEL,
		ACTION_EIDT_COLOR_BLUE_CHANNEL,
		ACTION_EIDT_COLOR_BLOCK,
		ACTION_EIDT_COLOR_SLIDER,
		ACTION_EIDT_ROTATE,
		ACTION_EIDT_SCALE,
		ACTION_EIDT_TRANS,
		ACTION_EIDT_CPS_TRANS,
		ACTION_EIDT_CYCLE_CW_ROTATE,
		ACTION_EIDT_CYCLE_CCW_ROTATE,
		ACTION_EIDT_CYCLE_ROOM_IN,
		ACTION_EIDT_CYCLE_ROOM_OUT,
		ACTION_EIDT_CYCLE_TRANS,
		ACTION_SEPARATOR,//END
		ACTION_N_TYPES
	} ActionType;

	class Action;

	typedef void (Action::*PointerToMemberFunction)();

	class ActionDefinitionItem
	{

	protected:

		ActionType type;
		QString text;
		QString desc;
		QString shortCut;
		QString icon;
		PointerToMemberFunction slot;

	private:

		//! Internal initialization function.
		void _init(const ActionDefinitionItem *pActionDefinitionItem = nullptr);

	public:

		//! Constructor.
		ActionDefinitionItem(ActionType type = ACTION_NONE,
			const QString &text = QString(),
			const QString &desc = QString(),
			const QString &shortCut = QString(),
			const QString &icon = QString(),
			PointerToMemberFunction slot = nullptr);

		//! Copy constructor.
		ActionDefinitionItem(const ActionDefinitionItem &actionDefinitionItem);

		//! Destructor.
		~ActionDefinitionItem();

		//! Assignment operator.
		ActionDefinitionItem &operator =(const ActionDefinitionItem &actionDefinitionItem);

		//! Return action type.
		ActionType getType() const;

		//! Return const reference to shortcut.
		const QString &getShortcut() const;

		//! ActionDefinition is a container class and need to have an access to protected members.
		friend class ActionDefinition;

	};
}
#endif // ACTION_DEFINITION_ITEM_H
