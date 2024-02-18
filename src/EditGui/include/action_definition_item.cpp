/*********************************************************************
 *  AUTHOR: Tomas Soltys                                             *
 *  FILE:   action_definition_item.cpp                               *
 *  GROUP:  Range                                                    *
 *  TYPE:   source file (*.cpp)                                      *
 *  DATE:   10-th September 2014                                     *
 *                                                                   *
 *  DESCRIPTION: Action definition item class definition             *
 *********************************************************************/

#include <QObject>

#include "action_definition_item.h"
namespace EDITGUI {

	void ActionDefinitionItem::_init(const ActionDefinitionItem *pActionDefinitionItem)
	{
		if (pActionDefinitionItem)
		{
			this->type = pActionDefinitionItem->type;
			this->text = pActionDefinitionItem->text;
			this->desc = pActionDefinitionItem->desc;
			this->shortCut = pActionDefinitionItem->shortCut;
			this->icon = pActionDefinitionItem->icon;
			this->slot = pActionDefinitionItem->slot;
		}
	}

	ActionDefinitionItem::ActionDefinitionItem(ActionType type,  const QString &text, const QString &desc, const QString &shortCut, const QString &icon, PointerToMemberFunction slot)
		: type(type)
		, text(text)
		, desc(desc)
		, shortCut(shortCut)
		, icon(icon)
		, slot(slot)
	{
		this->_init();
	}

	ActionDefinitionItem::ActionDefinitionItem(const ActionDefinitionItem &actionDefinitionItem)
	{
		this->_init(&actionDefinitionItem);
	}

	ActionDefinitionItem::~ActionDefinitionItem()
	{

	}

	ActionDefinitionItem &ActionDefinitionItem::operator =(const ActionDefinitionItem &actionDefinitionItem)
	{
		this->_init(&actionDefinitionItem);
		return (*this);
	}

	ActionType ActionDefinitionItem::getType() const
	{
		return this->type;
	}

	const QString &ActionDefinitionItem::getShortcut() const
	{
		return this->shortCut;
	}

}