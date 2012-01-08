/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#ifndef XCSOAR_FULL_BLACKBOARD_HPP
#define XCSOAR_FULL_BLACKBOARD_HPP

#include "BaseBlackboard.hpp"
#include "ComputerSettingsBlackboard.hpp"
#include "UISettings.hpp"

/**
 * A blackboard which contains all existing blackboards.  This is the
 * base class for InterfaceBlackboard, and may be used to pass
 * everything we have in one pointer.
 */
class FullBlackboard
  : public BaseBlackboard,
    public ComputerSettingsBlackboard
{
protected:
  UISettings ui_settings;

public:
  const UISettings &GetUISettings() const {
    return ui_settings;
  }

  const MapSettings &GetMapSettings() const {
    return ui_settings.map;
  }
};

#endif
