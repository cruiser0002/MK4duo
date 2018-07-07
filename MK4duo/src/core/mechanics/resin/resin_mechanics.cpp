/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * resin_mechanics.cpp
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#include "../../../../MK4duo.h"
#include "resin_mechanics.h"

#if IS_RESIN

  Resin_Mechanics mechanics;

  /** Public Parameters */
  const float Resin_Mechanics::base_max_pos[XYZ]  = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS },
              Resin_Mechanics::base_min_pos[XYZ]  = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
              Resin_Mechanics::base_home_pos[XYZ] = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS },
              Resin_Mechanics::max_length[XYZ]    = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH };

  /** Public Function */
  void Resin_Mechanics::init() { }

  void Resin_Mechanics::sync_plan_position_mech_specific() {
    sync_plan_position();
  }

  /**
   * Home Resin
   */
  void Resin_Mechanics::home(const bool homeX/*=false*/, const bool homeY/*=false*/, const bool homeZ/*=false*/) {

    if (printer.debugSimulation()) {
      LOOP_XYZ(axis) set_axis_is_at_home((AxisEnum)axis);
      return;
    }

    // Wait for planner moves to finish!
    planner.synchronize();

    printer.setup_for_endstop_or_probe_move();

    endstops.setEnabled(true); // Enable endstops for next homing move

    bool come_back = parser.boolval('B');
    float lastpos[NUM_AXIS];
    float old_feedrate_mm_s;
    if (come_back) {
      old_feedrate_mm_s = feedrate_mm_s;
      COPY_ARRAY(lastpos, current_position);
    }

    const bool home_all = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

    set_destination_to_current();

    const float z_homing_height = printer.isZHomed() ? MIN_Z_HEIGHT_FOR_HOMING : 0;

    if (z_homing_height && (home_all || homeX || homeY)) {
      // Raise Z before homing any other axes and z is not already high enough (never lower z)
      destination[Z_AXIS] = z_homing_height;
      if (destination[Z_AXIS] > current_position[Z_AXIS]) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (printer.debugLeveling())
            SERIAL_EMV("Raise Z (before homing) to ", destination[Z_AXIS]);
        #endif
        do_blocking_move_to_z(destination[Z_AXIS]);
      }
    }

    // Home X
    if (home_all || homeX) {
        homeaxis(X_AXIS);
    }

    // Home Z last if homing towards the bed

    if (home_all || homeZ) {
        homeaxis(Z_AXIS);
    } // home_all || homeZ


    sync_plan_position();
    endstops.setNotHoming();

    if (come_back) {
      feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
      COPY_ARRAY(destination, lastpos);
      prepare_move_to_destination();
      feedrate_mm_s = old_feedrate_mm_s;
    }

    printer.clean_up_after_endstop_or_probe_move();

    planner.synchronize();


    lcd_refresh();

    mechanics.report_current_position();


  }

  /**
   * Prepare a linear move in a Resin setup.
   *
   * When a mesh-based leveling system is active, moves are segmented
   * according to the configuration of the leveling system.
   *
   * Returns true if current_position[] was set to destination[]
   */
  bool Resin_Mechanics::prepare_move_to_destination_mech_specific() {
    line_to_destination(MMS_SCALED(feedrate_mm_s));
    return false;
  }

  void Resin_Mechanics::homeaxis(const AxisEnum axis) {

    #define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;


    const int axis_home_dir = (
      home_dir[axis]
    );



    // Fast move towards endstop until triggered
    mechanics.do_homing_move(axis, 1.5 * max_length[axis] * axis_home_dir);

    // When homing Z with probe respect probe clearance
    const float bump = axis_home_dir * (
      home_bump_mm[axis]
    );

    // If a second homing move is configured...
    if (bump) {
      // Move away from the endstop by the axis HOME_BUMP_MM

      mechanics.do_homing_move(axis, -bump);

      // Slow move towards endstop until triggered
      mechanics.do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
    }


    // For resin machines,
    // set the axis to its home position
    set_axis_is_at_home(axis);
    sync_plan_position();

    destination[axis] = current_position[axis];
  }



  /**
   * Set an axis' current position to its home position (after homing).
   *
   * For Resin this applies one-to-one when an
   * individual axis has been homed.
   *
   * Callers must sync the planner position after calling this!
   */
  void Resin_Mechanics::set_axis_is_at_home(const AxisEnum axis) {
    printer.setAxisHomed(axis, true);
    current_position[axis] = base_home_pos[axis];
  }

#endif // IS_RESIN
