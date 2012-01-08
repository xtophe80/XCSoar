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

#include "WaypointDisplayConfigPanel.hpp"
#include "Profile/ProfileKeys.hpp"
#include "DataField/Enum.hpp"
#include "Interface.hpp"
#include "Language/Language.hpp"
#include "Form/Form.hpp"
#include "DataField/Base.hpp"
#include "Form/RowFormWidget.hpp"
#include "Screen/Layout.hpp"
#include "UIGlobals.hpp"

enum ControlIndex {
  WaypointLabels,
  WaypointArrivalHeightDisplay,
  WaypointLabelStyle,
  WaypointLabelSelection,
  AppIndLandable,
  AppUseSWLandablesRendering,
  AppLandableRenderingScale,
  AppScaleRunwayLength
};

class WaypointDisplayConfigPanel : public RowFormWidget {
public:
  WaypointDisplayConfigPanel()
    :RowFormWidget(UIGlobals::GetDialogLook(), Layout::Scale(100)) {}

public:
  virtual void Prepare(ContainerWindow &parent, const PixelRect &rc);
  virtual bool Save(bool &changed, bool &require_restart);
  void UpdateVisibilities();
};

/** XXX this hack is needed because the form callbacks don't get a
    context pointer - please refactor! */
static WaypointDisplayConfigPanel *instance;

void
WaypointDisplayConfigPanel::UpdateVisibilities()
{
  bool visible;
  SaveValue(AppUseSWLandablesRendering, visible);

  GetControl(AppLandableRenderingScale).set_visible(visible);
  GetControl(AppScaleRunwayLength).set_visible(visible);
}

static void
OnRenderingTypeData(gcc_unused DataField *Sender,
                    DataField::DataAccessKind_t Mode)
{
  switch (Mode) {
  case DataField::daChange: {
    instance->UpdateVisibilities();
    break;
  }
  case DataField::daSpecial:
    return;
  }
}


void
WaypointDisplayConfigPanel::Prepare(ContainerWindow &parent, const PixelRect &rc)
{
  const WaypointRendererSettings &settings = CommonInterface::GetMapSettings().waypoint;

  instance = this;
  RowFormWidget::Prepare(parent, rc);

  static gcc_constexpr_data StaticEnumChoice wp_labels_list[] = {
    { DISPLAYNAME, N_("Full name"),
      N_("The full name of each waypoint is displayed.") },
    { DISPLAYUNTILSPACE, N_("First word of name"),
      N_("The first word of the waypoint name is displayed.") },
    { DISPLAYFIRSTTHREE, N_("First 3 letters"),
      N_("The first 3 letters of the waypoint name are displayed.") },
    { DISPLAYFIRSTFIVE, N_("First 5 letters"),
      N_("The first 5 letters of the waypoint name are displayed.") },
    { DISPLAYNONE, N_("None"), N_("No waypoint name is displayed.") },
    { 0 }
  };
  AddEnum(_("Label format"), _("Determines how labels are displayed with each waypoint"),
          wp_labels_list, settings.display_text_type);

  static gcc_constexpr_data StaticEnumChoice wp_arrival_list[] = {
    { WP_ARRIVAL_HEIGHT_NONE, N_("None"),
      N_("No arrival height is displayed.") },
    { WP_ARRIVAL_HEIGHT_GLIDE, N_("Straight glide"),
      N_("Straight glide arrival height (no terrain is considered).") },
    { WP_ARRIVAL_HEIGHT_TERRAIN, N_("Terrain avoidance glide"),
      N_("Arrival height considering terrain avoidance") },
    { WP_ARRIVAL_HEIGHT_GLIDE_AND_TERRAIN, N_("Straight & terrain glide"),
      N_("Both arrival heights are displayed.") },
    { 0 }
  };
  // Expert item (TODO)
  AddEnum(_("Arrival height"), _("Determines how arrival height is displayed in waypoint labels"),
          wp_arrival_list, settings.arrival_height_display);

  const TCHAR *wp_label_help = N_("Select a label shape.");
  static const StaticEnumChoice wp_label_list[] = {
    { RM_ROUNDED_BLACK, N_("Rounded rectangle"), wp_label_help },
    { RM_OUTLINED_INVERTED, N_("Outlined"), wp_label_help },
    { 0 }
  };
  // Expert item
  AddEnum(_("Label style"), _T(""), wp_label_list, settings.landable_render_mode);

  static gcc_constexpr_data StaticEnumChoice wp_selection_list[] = {
    { wlsAllWaypoints, N_("All"), N_("All waypoint labels will be displayed.") },
    { wlsTaskAndLandableWaypoints, N_("Task waypoints & landables"),
      N_("All waypoints part of a task and all landables will be displayed.") },
    { wlsTaskWaypoints, N_("Task waypoints"),
      N_("All waypoints part of a task will be displayed.") },
    { wlsNoWaypoints, N_("None"), N_("No waypoint labels will be displayed.") },
    { 0 }
  };
  // Expert item
  AddEnum(_("Label visibility"),
          _("Determines what waypoint labels are displayed for each waypoint (space permitting)."),
          wp_selection_list, settings.label_selection);

  static gcc_constexpr_data StaticEnumChoice wp_style_list[] = {
    { wpLandableWinPilot, N_("Purple circle"),
      N_("Airports and outlanding fields are displayed as purple circles. If the waypoint is "
          "reachable a bigger green circle is added behind the purple one. If the waypoint is "
          "blocked by a mountain the green circle will be red instead.") },
    { wpLandableAltA, N_("B/W"),
      N_("Airports and outlanding fields are displayed in white/grey. If the waypoint is "
          "reachable the color is changed to green. If the waypoint is blocked by a mountain "
          "the color is changed to red instead.") },
    { wpLandableAltB, N_("Traffic lights"),
      N_("Airports and outlanding fields are displayed in the colors of a traffic light. "
          "Green if reachable, Orange if blocked by mountain and red if not reachable at all.") },
    { 0 }
  };
  AddEnum(_("Landable symbols"),
          _("Three styles are available: Purple circles (WinPilot style), a high "
              "contrast (monochrome) style, or orange. The rendering differs for landable "
              "field and airport. All styles mark the waypoints within reach green."),
          wp_style_list, settings.landable_style);

  // Expert item
  AddBoolean(_("Detailed landables"),
             _("[Off] Display fixed icons for landables.\n"
                 "[On] Show landables with variable information like runway length and heading."),
             settings.vector_landable_rendering, OnRenderingTypeData);

  // Expert item
  AddInteger(_("Landable size"),
             _("A percentage to select the size landables are displayed on the map."),
             _T("%u %%"), _T("%u"), 50, 200, 10, settings.landable_rendering_scale);

  // Expert item
  AddBoolean(_("Scale runway length"),
             _("[Off] Display fixed length for runways.\n"
                 "[On] Scale displayed runway length based on real length."),
             settings.scale_runway_length);

  UpdateVisibilities();
}

bool
WaypointDisplayConfigPanel::Save(bool &_changed, bool &_require_restart)
{
  bool changed = false, require_restart = false;

  WaypointRendererSettings &settings = CommonInterface::SetMapSettings().waypoint;

  changed |= SaveValueEnum(WaypointLabels, szProfileDisplayText, settings.display_text_type);

  changed |= SaveValueEnum(WaypointArrivalHeightDisplay, szProfileWaypointArrivalHeightDisplay,
                           settings.arrival_height_display);

  changed |= SaveValueEnum(WaypointLabelStyle, szProfileWaypointLabelStyle,
                           settings.landable_render_mode);

  changed |= SaveValueEnum(WaypointLabelSelection, szProfileWaypointLabelSelection,
                           settings.label_selection);

  changed |= SaveValueEnum(AppIndLandable, szProfileAppIndLandable, settings.landable_style);

  changed |= SaveValue(AppUseSWLandablesRendering, szProfileAppUseSWLandablesRendering,
                       settings.vector_landable_rendering);

  changed |= SaveValue(AppLandableRenderingScale, szProfileAppLandableRenderingScale,
                       settings.landable_rendering_scale);

  changed |= SaveValue(AppScaleRunwayLength, szProfileAppScaleRunwayLength,
                       settings.scale_runway_length);

  _changed |= changed;
  _require_restart |= require_restart;

  return true;
}

Widget *
CreateWaypointDisplayConfigPanel()
{
  return new WaypointDisplayConfigPanel();
}
