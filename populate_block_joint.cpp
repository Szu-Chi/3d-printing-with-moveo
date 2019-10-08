bool Planner::_populate_block_joint(block_t * const block, bool split_move,
  const int32_t (&target)[NUM_AXIS], const int32_t (&joint)[Joint_All]
  #if HAS_POSITION_FLOAT
    , const float (&target_float)[NUM_AXIS]
  #endif
  , float fr_mm_s, const uint8_t extruder, const float &millimeters/*=0.0*/
  #if ENABLED(UNREGISTERED_MOVE_SUPPORT)
    , const bool count_it/*=true*/
  #endif
) {

  const int32_t da = target[A_AXIS] - position[A_AXIS],
                db = target[B_AXIS] - position[B_AXIS],
                dc = target[C_AXIS] - position[C_AXIS]
                #if ENABLED(HANGPRINTER)
                  , dd = target[D_AXIS] - position[D_AXIS]
                #endif
              ;

  const int32_t d0 = joint[Joint1_AXIS] - position_joint[Joint1_AXIS],
                d1 = joint[Joint2_AXIS] - position_joint[Joint2_AXIS],
                d2 = joint[Joint3_AXIS] - position_joint[Joint3_AXIS],
                d3 = joint[Joint4_AXIS] - position_joint[Joint4_AXIS],
                d4 = joint[Joint5_AXIS] - position_joint[Joint5_AXIS];
  
  int32_t de = target[E_AXIS] - position[E_AXIS];

  position_joint[Joint1_AXIS]=joint[Joint1_AXIS];
  position_joint[Joint2_AXIS]=joint[Joint2_AXIS];
  position_joint[Joint3_AXIS]=joint[Joint3_AXIS];
  position_joint[Joint4_AXIS]=joint[Joint4_AXIS];
  position_joint[Joint5_AXIS]=joint[Joint5_AXIS];

  /* <-- add a slash to enable
    SERIAL_ECHOPAIR("  _populate_block FR:", fr_mm_s);
    SERIAL_ECHOPAIR(" A:", target[A_AXIS]);
    SERIAL_ECHOPAIR(" (", da);
    SERIAL_ECHOPAIR(" steps) B:", target[B_AXIS]);
    SERIAL_ECHOPAIR(" (", db);
    SERIAL_ECHOPAIR(" steps) C:", target[C_AXIS]);
    SERIAL_ECHOPAIR(" (", dc);
    SERIAL_ECHOPAIR(" steps) E:", target[E_AXIS]);
    SERIAL_ECHOPAIR(" (", de);
    SERIAL_ECHOPGM(" steps)");  

    SERIAL_ECHOPAIR(" J1:", joint[Joint1_AXIS]);
    SERIAL_ECHOPAIR(" (", d0);
    SERIAL_ECHOPGM(" steps)");

    SERIAL_ECHOPAIR(" J2:", joint[Joint2_AXIS]);
    SERIAL_ECHOPAIR(" (", d1);
    SERIAL_ECHOPGM(" steps)");

    SERIAL_ECHOPAIR(" J3:", joint[Joint3_AXIS]);
    SERIAL_ECHOPAIR(" (", d2);
    SERIAL_ECHOPGM(" steps)");

    SERIAL_ECHOPAIR(" J4:", joint[Joint4_AXIS]);
    SERIAL_ECHOPAIR(" (", d3);
    SERIAL_ECHOPGM(" steps)");

    SERIAL_ECHOPAIR(" J5:", joint[Joint5_AXIS]);
    SERIAL_ECHOPAIR(" (", d4);
    SERIAL_ECHOLNPGM(" steps)");   
  //*/

  #if ENABLED(PREVENT_COLD_EXTRUSION) || ENABLED(PREVENT_LENGTHY_EXTRUDE)
    if (de) {
      #if ENABLED(PREVENT_COLD_EXTRUSION)
        if (thermalManager.tooColdToExtrude(extruder)) {
          if (COUNT_MOVE) {
            position[E_AXIS] = target[E_AXIS]; // Behave as if the move really took place, but ignore E part
            #if HAS_POSITION_FLOAT
              position_float[E_AXIS] = target_float[E_AXIS];
            #endif
          }
          de = 0; // no difference
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
        }
      #endif // PREVENT_COLD_EXTRUSION
      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        if (ABS(de * e_factor[extruder]) > (int32_t)axis_steps_per_mm[E_AXIS_N] * (EXTRUDE_MAXLENGTH)) { // It's not important to get max. extrusion length in a precision < 1mm, so save some cycles and cast to int
          if (COUNT_MOVE) {
            position[E_AXIS] = target[E_AXIS]; // Behave as if the move really took place, but ignore E part
            #if HAS_POSITION_FLOAT
              position_float[E_AXIS] = target_float[E_AXIS];
            #endif
          }
          de = 0; // no difference
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
        }
      #endif // PREVENT_LENGTHY_EXTRUDE
    }
  #endif // PREVENT_COLD_EXTRUSION || PREVENT_LENGTHY_EXTRUDE

  // Compute direction bit-mask for this block
  uint8_t dm = 0, djm = 0;
    
  if (da < 0) SBI(dm, X_AXIS);
  if (db < 0) SBI(dm, Y_AXIS);
  if (dc < 0) SBI(dm, Z_AXIS);
  if (d0 < 0) SBI(djm, Joint1_AXIS);
  if (d1 < 0) SBI(djm, Joint2_AXIS);
  if (d2 < 0) SBI(djm, Joint3_AXIS);
  if (d3 < 0) SBI(djm, Joint4_AXIS);
  if (d4 < 0) SBI(djm, Joint5_AXIS);
  
  if (de < 0) SBI(dm, E_AXIS);

  const float esteps_float = de * e_factor[extruder];
  const uint32_t esteps = ABS(esteps_float) + 0.5f;

  // Clear all flags, including the "busy" bit
  block->flag = 0x00;

  // Set direction bits
  block->direction_bits = dm;
  block->direction_bits_joint = djm;
  // Specify if block is to be counted or not
  #if ENABLED(UNREGISTERED_MOVE_SUPPORT)
    block->count_it = count_it;
  #endif

  
  // default non-h-bot planning
  block->steps[A_AXIS] = ABS(da);
  block->steps[B_AXIS] = ABS(db);
  block->steps[C_AXIS] = ABS(dc);
  block->step_Joint[Joint1_AXIS] = ABS(d0);
  block->step_Joint[Joint2_AXIS] = ABS(d1);
  block->step_Joint[Joint3_AXIS] = ABS(d2);
  block->step_Joint[Joint4_AXIS] = ABS(d3);
  block->step_Joint[Joint5_AXIS] = ABS(d4);
  

  block->steps[E_AXIS] = esteps;

  block->step_event_count = (
      //MAX4(block->steps[A_AXIS], block->steps[B_AXIS], block->steps[C_AXIS], esteps)
      MAX6(block->step_Joint[Joint1_AXIS], block->step_Joint[Joint2_AXIS], block->step_Joint[Joint3_AXIS],
           block->step_Joint[Joint4_AXIS], block->step_Joint[Joint5_AXIS], esteps)
  );

  // Bail if this is a zero-length block
  if (block->step_event_count < MIN_STEPS_PER_SEGMENT) return false;

  // For a mixing extruder, get a magnified esteps for each
  #if ENABLED(MIXING_EXTRUDER)
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
      block->mix_steps[i] = mixing_factor[i] * esteps;
  #endif

  #if FAN_COUNT > 0
    for (uint8_t i = 0; i < FAN_COUNT; i++) block->fan_speed[i] = fanSpeeds[i];
  #endif

  #if ENABLED(BARICUDA)
    block->valve_pressure = baricuda_valve_pressure;
    block->e_to_p_pressure = baricuda_e_to_p_pressure;
  #endif

  block->active_extruder = extruder;

  #if ENABLED(AUTO_POWER_CONTROL)
    if (block->steps[X_AXIS] || block->steps[Y_AXIS] || block->steps[Z_AXIS])
      powerManager.power_on();
  #endif

  enable_Joint1();
  enable_Joint2();
  enable_Joint3();
  enable_Joint4();
  enable_Joint5();

  // Enable extruder(s)
  if (esteps) {
    #if ENABLED(AUTO_POWER_CONTROL)
      powerManager.power_on();
    #endif

    #if ENABLED(DISABLE_INACTIVE_EXTRUDER) // Enable only the selected extruder

      #define DISABLE_IDLE_E(N) if (!g_uc_extruder_last_move[N]) disable_E##N();

      for (uint8_t i = 0; i < EXTRUDERS; i++)
        if (g_uc_extruder_last_move[i] > 0) g_uc_extruder_last_move[i]--;

      switch (extruder) {
        case 0:
          #if EXTRUDERS > 1
            DISABLE_IDLE_E(1);
            #if EXTRUDERS > 2
              DISABLE_IDLE_E(2);
              #if EXTRUDERS > 3
                DISABLE_IDLE_E(3);
                #if EXTRUDERS > 4
                  DISABLE_IDLE_E(4);
                #endif // EXTRUDERS > 4
              #endif // EXTRUDERS > 3
            #endif // EXTRUDERS > 2
          #endif // EXTRUDERS > 1
          enable_E0();
          g_uc_extruder_last_move[0] = (BLOCK_BUFFER_SIZE) * 2;
          #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
            if (extruder_duplication_enabled) {
              enable_E1();
              g_uc_extruder_last_move[1] = (BLOCK_BUFFER_SIZE) * 2;
            }
          #endif
        break;
        #if EXTRUDERS > 1
          case 1:
            DISABLE_IDLE_E(0);
            #if EXTRUDERS > 2
              DISABLE_IDLE_E(2);
              #if EXTRUDERS > 3
                DISABLE_IDLE_E(3);
                #if EXTRUDERS > 4
                  DISABLE_IDLE_E(4);
                #endif // EXTRUDERS > 4
              #endif // EXTRUDERS > 3
            #endif // EXTRUDERS > 2
            enable_E1();
            g_uc_extruder_last_move[1] = (BLOCK_BUFFER_SIZE) * 2;
          break;
          #if EXTRUDERS > 2
            case 2:
              DISABLE_IDLE_E(0);
              DISABLE_IDLE_E(1);
              #if EXTRUDERS > 3
                DISABLE_IDLE_E(3);
                #if EXTRUDERS > 4
                  DISABLE_IDLE_E(4);
                #endif
              #endif
              enable_E2();
              g_uc_extruder_last_move[2] = (BLOCK_BUFFER_SIZE) * 2;
            break;
            #if EXTRUDERS > 3
              case 3:
                DISABLE_IDLE_E(0);
                DISABLE_IDLE_E(1);
                DISABLE_IDLE_E(2);
                #if EXTRUDERS > 4
                  DISABLE_IDLE_E(4);
                #endif
                enable_E3();
                g_uc_extruder_last_move[3] = (BLOCK_BUFFER_SIZE) * 2;
              break;
              #if EXTRUDERS > 4
                case 4:
                  DISABLE_IDLE_E(0);
                  DISABLE_IDLE_E(1);
                  DISABLE_IDLE_E(2);
                  DISABLE_IDLE_E(3);
                  enable_E4();
                  g_uc_extruder_last_move[4] = (BLOCK_BUFFER_SIZE) * 2;
                break;
              #endif // EXTRUDERS > 4
            #endif // EXTRUDERS > 3
          #endif // EXTRUDERS > 2
        #endif // EXTRUDERS > 1
      }
    #else
      enable_E0();
      enable_E1();
      enable_E2();
      enable_E3();
      enable_E4();
    #endif
  }

  if (esteps)
    NOLESS(fr_mm_s, min_feedrate_mm_s);
  else
    NOLESS(fr_mm_s, min_travel_feedrate_mm_s);

    //  SERIAL_ECHOPAIR("feedrate_mm_s:",feedrate_mm_s);
    

  /**
   * This part of the code calculates the total length of the movement.
   * For cartesian bots, the X_AXIS is the real X movement and same for Y_AXIS.
   * But for corexy bots, that is not true. The "X_AXIS" and "Y_AXIS" motors (that should be named to A_AXIS
   * and B_AXIS) cannot be used for X and Y length, because A=X+Y and B=X-Y.
   * So we need to create other 2 "AXIS", named X_HEAD and Y_HEAD, meaning the real displacement of the Head.
   * Having the real displacement of the head, we can calculate the total movement length and apply the desired speed.
   */
  
  float delta_mm[NUM_AXIS];
  delta_mm[A_AXIS] = da * steps_to_mm[A_AXIS];
  delta_mm[B_AXIS] = db * steps_to_mm[B_AXIS];
  delta_mm[C_AXIS] = dc * steps_to_mm[C_AXIS];
  delta_mm[E_AXIS] = esteps_float * steps_to_mm[E_AXIS_N];
  
  long delta_joint_step[Joint_All];
  delta_joint_step[Joint1_AXIS] = d0;
  delta_joint_step[Joint2_AXIS] = d1;
  delta_joint_step[Joint3_AXIS] = d2;
  delta_joint_step[Joint4_AXIS] = d3;
  delta_joint_step[Joint5_AXIS] = d4;

  if (block->steps[A_AXIS] < MIN_STEPS_PER_SEGMENT && block->steps[B_AXIS] < MIN_STEPS_PER_SEGMENT && block->steps[C_AXIS] < MIN_STEPS_PER_SEGMENT) {
    block->millimeters = ABS(delta_mm[E_AXIS]);
  }
  else if (!millimeters) {
    block->millimeters = SQRT(
      sq(delta_mm[X_AXIS]) + sq(delta_mm[Y_AXIS]) + sq(delta_mm[Z_AXIS])
    );
  }
  else {
    block->millimeters = millimeters;
  }
    

  const float inverse_millimeters = 1.0f / block->millimeters;  // Inverse millimeters to remove multiple divides

  // Calculate inverse time for this move. No divide by zero due to previous checks.
  // Example: At 120mm/s a 60mm move takes 0.5s. So this will give 2.0.
  float inverse_secs = fr_mm_s * inverse_millimeters;

  // Get the number of non busy movements in queue (non busy means that they can be altered)
  const uint8_t moves_queued = nonbusy_movesplanned();

  // Slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill
  #if ENABLED(SLOWDOWN) || ENABLED(ULTRA_LCD) || defined(XY_FREQUENCY_LIMIT)
    // Segment time im micro seconds
    uint32_t segment_time_us = LROUND(1000000.0f / inverse_secs);
  #endif

  #if ENABLED(SLOWDOWN)
    if (WITHIN(moves_queued, 2, (BLOCK_BUFFER_SIZE) / 2 - 1)) {
      if (segment_time_us < min_segment_time_us) {
        // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
        const uint32_t nst = segment_time_us + LROUND(2 * (min_segment_time_us - segment_time_us) / moves_queued);
        inverse_secs = 1000000.0f / nst;
        #if defined(XY_FREQUENCY_LIMIT) || ENABLED(ULTRA_LCD)
          segment_time_us = nst;
        #endif
      }
    }
  #endif

  #if ENABLED(ULTRA_LCD)
    // Protect the access to the position.
    const bool was_enabled = STEPPER_ISR_ENABLED();
    if (was_enabled) DISABLE_STEPPER_DRIVER_INTERRUPT();

    block_buffer_runtime_us += segment_time_us;

    if (was_enabled) ENABLE_STEPPER_DRIVER_INTERRUPT();
  #endif

  block->nominal_speed_sqr = sq(block->millimeters * inverse_secs);   //   (mm/sec)^2 Always > 0
  block->nominal_rate = CEIL(block->step_event_count * inverse_secs); // (step/sec) Always > 0

  

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[NUM_AXIS], speed_factor = 1.0f; // factor <1 decreases speed
  LOOP_NUM_AXIS(i) {
    const float cs = ABS((current_speed[i] = delta_mm[i] * inverse_secs));
    #if ENABLED(DISTINCT_E_FACTORS)
      if (i == E_AXIS) i += extruder;
    #endif
    if (cs > max_feedrate_mm_s[i]) NOMORE(speed_factor, max_feedrate_mm_s[i] / cs);
  }

  
  // Correct the speed
  if (speed_factor < 1.0f) {
    LOOP_NUM_AXIS(i) current_speed[i] *= speed_factor;
    block->nominal_rate *= speed_factor;
    block->nominal_speed_sqr = block->nominal_speed_sqr * sq(speed_factor);
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  const float steps_per_mm = block->step_event_count * inverse_millimeters;
  uint32_t accel;
  if (!block->steps[A_AXIS] && !block->steps[B_AXIS] && !block->steps[C_AXIS]) {
    // convert to: acceleration steps/sec^2
    accel = CEIL(retract_acceleration * steps_per_mm);
    #if ENABLED(LIN_ADVANCE)
      block->use_advance_lead = false;
    #endif
  }
  else {
    #define LIMIT_ACCEL_LONG(AXIS,INDX) do{ \
      if (block->steps[AXIS] && max_acceleration_steps_per_s2[AXIS+INDX] < accel) { \
        const uint32_t comp = max_acceleration_steps_per_s2[AXIS+INDX] * block->step_event_count; \
        if (accel * block->steps[AXIS] > comp) accel = comp / block->steps[AXIS]; \
      } \
    }while(0)

    #define LIMIT_ACCEL_FLOAT(AXIS,INDX) do{ \
      if (block->steps[AXIS] && max_acceleration_steps_per_s2[AXIS+INDX] < accel) { \
        const float comp = (float)max_acceleration_steps_per_s2[AXIS+INDX] * (float)block->step_event_count; \
        if ((float)accel * (float)block->steps[AXIS] > comp) accel = comp / (float)block->steps[AXIS]; \
      } \
    }while(0)

    // Start with print or travel acceleration
    accel = CEIL((esteps ? acceleration : travel_acceleration) * steps_per_mm);

    #if ENABLED(LIN_ADVANCE)

      #if ENABLED(JUNCTION_DEVIATION)
        #if ENABLED(DISTINCT_E_FACTORS)
          #define MAX_E_JERK max_e_jerk[extruder]
        #else
          #define MAX_E_JERK max_e_jerk
        #endif
      #else
        #define MAX_E_JERK max_jerk[E_AXIS]
      #endif

      /**
       *
       * Use LIN_ADVANCE for blocks if all these are true:
       *
       * esteps             : This is a print move, because we checked for A, B, C steps before.
       *
       * extruder_advance_K : There is an advance factor set.
       *
       * de > 0             : Extruder is running forward (e.g., for "Wipe while retracting" (Slic3r) or "Combing" (Cura) moves)
       */
      block->use_advance_lead =  esteps
                              && extruder_advance_K
                              && de > 0;

      if (block->use_advance_lead) {
        block->e_D_ratio = (target_float[E_AXIS] - position_float[E_AXIS]) /
          #if IS_KINEMATIC
            block->millimeters
          #else
            SQRT(sq(target_float[X_AXIS] - position_float[X_AXIS])
               + sq(target_float[Y_AXIS] - position_float[Y_AXIS])
               + sq(target_float[Z_AXIS] - position_float[Z_AXIS]))
          #endif
        ;

        // Check for unusual high e_D ratio to detect if a retract move was combined with the last print move due to min. steps per segment. Never execute this with advance!
        // This assumes no one will use a retract length of 0mm < retr_length < ~0.2mm and no one will print 100mm wide lines using 3mm filament or 35mm wide lines using 1.75mm filament.
        if (block->e_D_ratio > 3.0f)
          block->use_advance_lead = false;
        else {
          const uint32_t max_accel_steps_per_s2 = MAX_E_JERK / (extruder_advance_K * block->e_D_ratio) * steps_per_mm;
          #if ENABLED(LA_DEBUG)
            if (accel > max_accel_steps_per_s2) SERIAL_ECHOLNPGM("Acceleration limited.");
          #endif
          NOMORE(accel, max_accel_steps_per_s2);
        }
      }
    #endif

    #if ENABLED(DISTINCT_E_FACTORS)
      #define ACCEL_IDX extruder
    #else
      #define ACCEL_IDX 0
    #endif

    // Limit acceleration per axis
    if (block->step_event_count <= cutoff_long) {
      LIMIT_ACCEL_LONG(A_AXIS, 0);
      LIMIT_ACCEL_LONG(B_AXIS, 0);
      LIMIT_ACCEL_LONG(C_AXIS, 0);
      #if ENABLED(HANGPRINTER)
        LIMIT_ACCEL_LONG(D_AXIS, 0);
      #endif
      LIMIT_ACCEL_LONG(E_AXIS, ACCEL_IDX);
    }
    else {
      LIMIT_ACCEL_FLOAT(A_AXIS, 0);
      LIMIT_ACCEL_FLOAT(B_AXIS, 0);
      LIMIT_ACCEL_FLOAT(C_AXIS, 0);
      #if ENABLED(HANGPRINTER)
        LIMIT_ACCEL_FLOAT(D_AXIS, 0);
      #endif
      LIMIT_ACCEL_FLOAT(E_AXIS, ACCEL_IDX);
    }
  }
  block->acceleration_steps_per_s2 = accel;
  block->acceleration = accel / steps_per_mm;
  #if DISABLED(S_CURVE_ACCELERATION)
    block->acceleration_rate = (uint32_t)(accel * (4096.0f * 4096.0f / (STEPPER_TIMER_RATE)));
  #endif
  #if ENABLED(LIN_ADVANCE)
    if (block->use_advance_lead) {
      block->advance_speed = (STEPPER_TIMER_RATE) / (extruder_advance_K * block->e_D_ratio * block->acceleration * axis_steps_per_mm[E_AXIS_N]);
      #if ENABLED(LA_DEBUG)
        if (extruder_advance_K * block->e_D_ratio * block->acceleration * 2 < SQRT(block->nominal_speed_sqr) * block->e_D_ratio)
          SERIAL_ECHOLNPGM("More than 2 steps per eISR loop executed.");
        if (block->advance_speed < 200)
          SERIAL_ECHOLNPGM("eISR running at > 10kHz.");
      #endif
    }
  #endif

  float vmax_junction_sqr; // Initial limit on the segment entry velocity (mm/s)^2

  #if ENABLED(JUNCTION_DEVIATION)

    /**
     * Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
     * Let a circle be tangent to both previous and current path line segments, where the junction
     * deviation is defined as the distance from the junction to the closest edge of the circle,
     * colinear with the circle center. The circular segment joining the two paths represents the
     * path of centripetal acceleration. Solve for max velocity based on max acceleration about the
     * radius of the circle, defined indirectly by junction deviation. This may be also viewed as
     * path width or max_jerk in the previous Grbl version. This approach does not actually deviate
     * from path, but used as a robust way to compute cornering speeds, as it takes into account the
     * nonlinearities of both the junction angle and junction velocity.
     *
     * NOTE: If the junction deviation value is finite, Grbl executes the motions in an exact path
     * mode (G61). If the junction deviation value is zero, Grbl will execute the motion in an exact
     * stop mode (G61.1) manner. In the future, if continuous mode (G64) is desired, the math here
     * is exactly the same. Instead of motioning all the way to junction point, the machine will
     * just follow the arc circle defined here. The Arduino doesn't have the CPU cycles to perform
     * a continuous mode path, but ARM-based microcontrollers most certainly do.
     *
     * NOTE: The max junction speed is a fixed value, since machine acceleration limits cannot be
     * changed dynamically during operation nor can the line move geometry. This must be kept in
     * memory in the event of a feedrate override changing the nominal speeds of blocks, which can
     * change the overall maximum entry speed conditions of all blocks.
     *
     * #######
     * https://github.com/MarlinFirmware/Marlin/issues/10341#issuecomment-388191754
     *
     * hoffbaked: on May 10 2018 tuned and improved the GRBL algorithm for Marlin:
          Okay! It seems to be working good. I somewhat arbitrarily cut it off at 1mm
          on then on anything with less sides than an octagon. With this, and the
          reverse pass actually recalculating things, a corner acceleration value
          of 1000 junction deviation of .05 are pretty reasonable. If the cycles
          can be spared, a better acos could be used. For all I know, it may be
          already calculated in a different place. */

    // Unit vector of previous path line segment
    static float previous_unit_vec[XYZE];

    float unit_vec[] = {
      delta_mm[A_AXIS] * inverse_millimeters,
      delta_mm[B_AXIS] * inverse_millimeters,
      delta_mm[C_AXIS] * inverse_millimeters,
      delta_mm[E_AXIS] * inverse_millimeters
    };

    // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
    if (moves_queued && !UNEAR_ZERO(previous_nominal_speed_sqr)) {
      // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
      // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
      float junction_cos_theta = -previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
                                 -previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
                                 -previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS]
                                 -previous_unit_vec[E_AXIS] * unit_vec[E_AXIS]
                                ;

      // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
      if (junction_cos_theta > 0.999999f) {
        // For a 0 degree acute junction, just set minimum junction speed.
        vmax_junction_sqr = sq(float(MINIMUM_PLANNER_SPEED));
      }
      else {
        NOLESS(junction_cos_theta, -0.999999f); // Check for numerical round-off to avoid divide by zero.

        // Convert delta vector to unit vector
        float junction_unit_vec[XYZE] = {
          unit_vec[X_AXIS] - previous_unit_vec[X_AXIS],
          unit_vec[Y_AXIS] - previous_unit_vec[Y_AXIS],
          unit_vec[Z_AXIS] - previous_unit_vec[Z_AXIS],
          unit_vec[E_AXIS] - previous_unit_vec[E_AXIS]
        };
        normalize_junction_vector(junction_unit_vec);

        const float junction_acceleration = limit_value_by_axis_maximum(block->acceleration, junction_unit_vec),
                    sin_theta_d2 = SQRT(0.5f * (1.0f - junction_cos_theta)); // Trig half angle identity. Always positive.

        vmax_junction_sqr = (junction_acceleration * junction_deviation_mm * sin_theta_d2) / (1.0f - sin_theta_d2);
        if (block->millimeters < 1) {

          // Fast acos approximation, minus the error bar to be safe
          const float junction_theta = (RADIANS(-40) * sq(junction_cos_theta) - RADIANS(50)) * junction_cos_theta + RADIANS(90) - 0.18f;

          // If angle is greater than 135 degrees (octagon), find speed for approximate arc
          if (junction_theta > RADIANS(135)) {
            const float limit_sqr = block->millimeters / (RADIANS(180) - junction_theta) * junction_acceleration;
            NOMORE(vmax_junction_sqr, limit_sqr);
          }
        }
      }

      // Get the lowest speed
      vmax_junction_sqr = MIN3(vmax_junction_sqr, block->nominal_speed_sqr, previous_nominal_speed_sqr);
    }
    else // Init entry speed to zero. Assume it starts from rest. Planner will correct this later.
      vmax_junction_sqr = 0;

    COPY(previous_unit_vec, unit_vec);

  #else // Classic Jerk Limiting

    /**
     * Adapted from Průša MKS firmware
     * https://github.com/prusa3d/Prusa-Firmware
     */
    const float nominal_speed = SQRT(block->nominal_speed_sqr);

    // Exit speed limited by a jerk to full halt of a previous last segment
    static float previous_safe_speed;

    // Start with a safe speed (from which the machine may halt to stop immediately).
    float safe_speed = nominal_speed;

    uint8_t limited = 0;
    LOOP_NUM_AXIS(i) {
      const float jerk = ABS(current_speed[i]),   // cs : Starting from zero, change in speed for this axis
                  maxj = max_jerk[i];             // mj : The max jerk setting for this axis
      if (jerk > maxj) {                          // cs > mj : New current speed too fast?
        if (limited) {                            // limited already?
          const float mjerk = nominal_speed * maxj; // ns*mj
          if (jerk * safe_speed > mjerk) safe_speed = mjerk / jerk; // ns*mj/cs
        }
        else {
          safe_speed *= maxj / jerk;              // Initial limit: ns*mj/cs
          ++limited;                              // Initially limited
        }
      }
    }

    float vmax_junction;
    if (moves_queued && !UNEAR_ZERO(previous_nominal_speed_sqr)) {
      // Estimate a maximum velocity allowed at a joint of two successive segments.
      // If this maximum velocity allowed is lower than the minimum of the entry / exit safe velocities,
      // then the machine is not coasting anymore and the safe entry / exit velocities shall be used.

      // Factor to multiply the previous / current nominal velocities to get componentwise limited velocities.
      float v_factor = 1;
      limited = 0;

      // The junction velocity will be shared between successive segments. Limit the junction velocity to their minimum.
      // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
      const float previous_nominal_speed = SQRT(previous_nominal_speed_sqr);
      vmax_junction = MIN(nominal_speed, previous_nominal_speed);

      // Now limit the jerk in all axes.
      const float smaller_speed_factor = vmax_junction / previous_nominal_speed;
      LOOP_NUM_AXIS(axis) {
        // Limit an axis. We have to differentiate: coasting, reversal of an axis, full stop.
        float v_exit = previous_speed[axis] * smaller_speed_factor,
              v_entry = current_speed[axis];
        if (limited) {
          v_exit *= v_factor;
          v_entry *= v_factor;
        }

        // Calculate jerk depending on whether the axis is coasting in the same direction or reversing.
        const float jerk = (v_exit > v_entry)
            ? //                                  coasting             axis reversal
              ( (v_entry > 0 || v_exit < 0) ? (v_exit - v_entry) : MAX(v_exit, -v_entry) )
            : // v_exit <= v_entry                coasting             axis reversal
              ( (v_entry < 0 || v_exit > 0) ? (v_entry - v_exit) : MAX(-v_exit, v_entry) );

        if (jerk > max_jerk[axis]) {
          v_factor *= max_jerk[axis] / jerk;
          ++limited;
        }
      }
      if (limited) vmax_junction *= v_factor;
      // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
      // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve faster prints.
      const float vmax_junction_threshold = vmax_junction * 0.99f;
      if (previous_safe_speed > vmax_junction_threshold && safe_speed > vmax_junction_threshold)
        vmax_junction = safe_speed;
    }
    else
      vmax_junction = safe_speed;

    previous_safe_speed = safe_speed;
    vmax_junction_sqr = sq(vmax_junction);

  #endif // Classic Jerk Limiting

  // Max entry speed of this block equals the max exit speed of the previous block.
  block->max_entry_speed_sqr = vmax_junction_sqr;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  const float v_allowable_sqr = max_allowable_speed_sqr(-block->acceleration, sq(float(MINIMUM_PLANNER_SPEED)), block->millimeters);

  // If we are trying to add a split block, start with the
  // max. allowed speed to avoid an interrupted first move.
  block->entry_speed_sqr = !split_move ? sq(float(MINIMUM_PLANNER_SPEED)) : MIN(vmax_junction_sqr, v_allowable_sqr);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  block->flag |= block->nominal_speed_sqr <= v_allowable_sqr ? BLOCK_FLAG_RECALCULATE | BLOCK_FLAG_NOMINAL_LENGTH : BLOCK_FLAG_RECALCULATE;

  // Update previous path unit_vector and nominal speed
  COPY(previous_speed, current_speed);
  previous_nominal_speed_sqr = block->nominal_speed_sqr;

  // Update the position (only when a move was queued)
  static_assert(COUNT(target) > 1, "Parameter to _populate_block must be (&target)["
    #if ENABLED(HANGPRINTER)
      "ABCD"
    #else
      "XYZ"
    #endif
    "E]!"
  );

  if (COUNT_MOVE) {
    COPY(position, target);
    #if HAS_POSITION_FLOAT
      COPY(position_float, target_float);
    #endif
  }
  
  // Movement was accepted
  return true;
} // _populate_block_joint()
