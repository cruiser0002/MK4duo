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
              Resin_Mechanics::max_length[XYZ]    = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH },

              Resin_Mechanics::resin_segments_per_second      = RESIN_SEGMENTS_PER_SECOND,
              
              Resin_Mechanics::resin_z0                       = RESIN_Z0,
              Resin_Mechanics::resin_z0_squared               = RESIN_Z0*RESIN_Z0,
              Resin_Mechanics::resin_r                        = RESIN_R,
              Resin_Mechanics::resin_size_2_deg               = RESIN_SIZE_2_ANGLE*RESIN_RAD_2_DEG;
  
  float Resin_Mechanics::resin[XYZ]                    = { 0.0 };

  /** Public Function */
  void Resin_Mechanics::init() {

    pinMode(45, INPUT);
    pinMode(CASE_OPEN_PIN, INPUT);
    //digitalWrite(CASE_OPEN_PIN, HIGH);
    pinMode(CASE_OPEN2_PIN, INPUT);
    //digitalWrite(CASE_OPEN2_PIN, HIGH);
    pinMode(LASER_FIRING_PIN, OUTPUT);
    digitalWrite(LASER_FIRING_PIN, LOW);
    pinMode(GALVO_SS_PIN, OUTPUT);
    WRITE(GALVO_SS_PIN, HIGH);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV2); // Run at 8 MHz 
    // start the SPI library:
    SPI.begin();

    #if HAS_MICROSTEPS
      //stepper.microstep_mode(2, 4);
    #endif
  }

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

    if (current_position[E_AXIS] < destination[E_AXIS] && ((current_position[X_AXIS] != destination [X_AXIS]) || (current_position[Y_AXIS] != destination [Y_AXIS])))
      laser.status = LASER_ON;
    else
      laser.status = LASER_OFF;

    line_to_destination(MMS_SCALED(feedrate_mm_s));
    return false;

    /*
      void Mechanics::line_to_destination(float fr_mm_s) {
        planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, tools.active_extruder);
      }
    */

    // Get the top feedrate of the move in the XY plane
    const float _feedrate_mm_s = MMS_SCALED(feedrate_mm_s);

    // Get the cartesian distances moved in XYZE
    const float difference[XYZE] = {
      destination[X_AXIS] - current_position[X_AXIS],
      destination[Y_AXIS] - current_position[Y_AXIS],
      destination[Z_AXIS] - current_position[Z_AXIS],
      destination[E_AXIS] - current_position[E_AXIS]
    };

    // If the move is only in Z/E don't split up the move
    if (!difference[X_AXIS] && !difference[Y_AXIS]) {
      planner.buffer_line_kinematic(destination, _feedrate_mm_s, tools.active_extruder);
      return false; // caller will update current_position
    }

    // Fail if attempting move outside printable radius
    if (endstops.isSoftEndstop() && !mechanics.position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) return true;

    // Get the linear distance in XYZ
    float cartesian_mm = SQRT(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));

    // If the move is very short, check the E move distance
    if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = ABS(difference[E_AXIS]);

    // No E move either? Game over.
    if (UNEAR_ZERO(cartesian_mm)) return true;

    // Minimum number of seconds to move the given distance
    const float seconds = cartesian_mm / _feedrate_mm_s;

    // The number of segments-per-second times the duration
    // gives the number of segments we should produce
    uint16_t segments = resin_segments_per_second * seconds;

    // At least one segment is required
    NOLESS(segments, 1U);

    // The approximate length of each segment
    const float inv_segments = RECIPROCAL(segments),
                cartesian_segment_mm = cartesian_mm * inv_segments,
                segment_distance[XYZE] = {
                  difference[X_AXIS] * inv_segments,
                  difference[Y_AXIS] * inv_segments,
                  difference[Z_AXIS] * inv_segments,
                  difference[E_AXIS] * inv_segments
                };


    // Get the current position as starting point
    float raw[XYZ];
    COPY_ARRAY(raw, current_position);

    // Calculate and execute the segments
    while (--segments) {

      printer.check_periodical_actions();

      LOOP_XYZE(i) raw[i] += segment_distance[i];
      calculate_resin(raw);


      if (!planner.buffer_line(resin[X_AXIS], resin[Y_AXIS], resin[Z_AXIS], raw[E_AXIS], _feedrate_mm_s, tools.active_extruder, cartesian_segment_mm))
        break;

    }

    planner.buffer_line_kinematic(destination, _feedrate_mm_s, tools.active_extruder, cartesian_segment_mm);

    return false; // caller will update current_position
  }


  float Resin_Mechanics::fast_atan(const float x) {
    float abs_x = fabs(x);
    return 3.14159265/4*x - x*(abs_x-1)*(0.2447 + 0.0663*abs_x);
  }

  #if ENABLED(__AVR__)

    /**
     * Fast inverse SQRT from Quake III Arena
     * See: https://en.wikipedia.org/wiki/Fast_inverse_square_root
     
    float Resin_Mechanics::Q_rsqrt(float number) {
      long i;
      float x2, y;
      const float threehalfs = 1.5f;
      x2 = number * 0.5f;
      y  = number;
      i  = * ( long * ) &y;                         // evil floating point bit level hacking
      i  = 0x5F3759DF - ( i >> 1 );
      y  = * ( float * ) &i;
      y  = y * ( threehalfs - ( x2 * y * y ) );     // 1st iteration
      // y  = y * ( threehalfs - ( x2 * y * y ) );  // 2nd iteration, this can be removed
      return y;
    }
    */


    float Resin_Mechanics::fast_sqrt(const float x) {
      float xhalf = 0.5*x;
      union {
          float x;
          long i;
      } u;
      
      u.x = x;
      u.i = 0x5F3759DF - (u.i>>1);
      return x*u.x*(1.5-xhalf*sq(u.x));
    }

  #endif

  void Resin_Mechanics::calculate_resin(const float logical[XYZ]){
    
    float beta_y = fast_atan((logical[Y_AXIS])/resin_z0);
    float beta_x = fast_atan((logical[X_AXIS])/(resin_r+fast_sqrt(logical[Y_AXIS]*logical[Y_AXIS] + resin_z0_squared)));
    
      
    resin[X_AXIS] = beta_x*resin_size_2_deg;
    resin[Y_AXIS] = beta_y*resin_size_2_deg;
    resin[Z_AXIS] = logical[Z_AXIS];
    
    /*
    resin[X_AXIS] = logical[X_AXIS];
    resin[Y_AXIS] = logical[Y_AXIS];
    resin[Z_AXIS] = logical[Z_AXIS];
    */
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
