/*
 * Copyright (C) 2011-2012 Team ATMOS
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file firmwares/rotorcraft/guidance/force_allocation.c
 *  Distribute Outerloop Acceleration Commands To Lifting Surfaces
 *
 */


#include "std.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"

#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/ahrs.h"
#include "subsystems/ins.h"
#include "math/pprz_algebra_int.h"
#include "firmwares/rotorcraft/force_allocation_laws.h"
/*#include "modules/ATMOS/multiGain.h"
#include "modules/ATMOS/newTransition.h"*/

uint8_t transition_percentage;
uint8_t transition_percentage_nav = FAILSAVE_TRANSITION_PCT;

// FIXME change to BFPs
float force_allocation_fixedwing_max_climb         = FORCE_ALLOCATION_MAX_CLIMB;           // m/s
float force_allocation_fixedwing_pitch_of_vz       = FORCE_ALLOCATION_PITCH_OF_VZ;
float force_allocation_fixedwing_throttle_of_vz    = FORCE_ALLOCATION_THROTTLE_OF_VZ;
float force_allocation_fixedwing_pitch_trim        = FORCE_ALLOCATION_PITCH_TRIM;          // radians
float force_allocation_fixedwing_throttle_of_xdd   = FORCE_ALLOCATION_THROTTLE_OF_XDD;
float force_allocation_fixedwing_yawrate_of_ydd    = FORCE_ALLOCATION_YAWRATE_OF_YDD;

int32_t dbg1 = 0;
uint32_t dbg2 = 0;
uint32_t dbg3 = 0;
int32_t dbg4 = 0;


int32_t outerloop_throttle_command = 0;

struct PprzLiftDevice lift_devices[LIFT_DEVICES_NB] = LIFT_DEVICES;

/**   force_allocation_laws_run
 *    Distribute outerloop acceleration commands over the available lift devices.
 *    @param input1 = stabilization_cmd[COMMAND_THRUST]:  updated every loop in guidance_v.
 *    @param input2 = stab_att_sp_euler: in attitude mode only updated on RC-frame = 1 out of 10 times.
 *
 *    @param output = stab_att_sp_quat
 */
void force_allocation_laws_run(void)
{
  int32_t cmd_thrust = 0;
  struct Int32Eulers command_euler; /**< Blended output commands in euler (with force allocation laws applied). */

  INT_EULERS_ZERO(command_euler);

  float orientation_rotation   = 0;

  //transition_percentage=percent_from_rc(RADIO_EXTRA1);

  if (autopilot_mode == AP_MODE_NAV) {
    transition_percentage = transition_percentage_nav;
  }
  else {
#if FORCE_ALLOCATION_RADIO_TRANSITION_CHANNEL
    if (radio_control.status == RC_OK) {
      transition_percentage = percent_from_rc(FORCE_ALLOCATION_RADIO_TRANSITION_CHANNEL);
    }
    else {
      transition_percentage = FORCE_ALLOCATION_FAILSAVE_TRANSITION_PCT;
    }
#else
    transition_percentage = FORCE_ALLOCATION_FAILSAVE_TRANSITION_PCT;
#endif
  }

  lift_devices[0].activation = transition_percentage;
  lift_devices[1].activation = 100-transition_percentage;

  lift_devices[0].lift_type = ROTOR_LIFTING_DEVICE;
  lift_devices[1].lift_type = WING_LIFTING_DEVICE;

  lift_devices[0].orientation_pitch = 0;
  lift_devices[1].orientation_pitch = -90;
  /////////////////////////////////////////////////////


  if (transition_percentage < 30) {
    //fixedwing
    SetGainSetC();
  }
  else if (transition_percentage >= 30 && transition_percentage < 60) {
    //fixedwing+rotor
    SetGainSetB();
  }
  else if (transition_percentage >= 60) {
    //rotor
    SetGainSetA();
  }

  for (int i=0; i < LIFT_DEVICES_NB; i++)
  {

    struct PprzLiftDevice *wing = &(lift_devices[i]);
    float percent = ((float)wing->activation) / 100.0f;

    if (wing->lift_type == ROTOR_LIFTING_DEVICE)
    {
      // Rotorcraft Mode
      // ---------------
      // lift command (vertical acceleration/) -> thrust
      // forward acceleration (command) -> pitch
      // lateral acceleration (command) -> roll
      // heading ANGLE -> yaw

      /* Rotate to body frame */
      int32_t s_psi, c_psi;
      PPRZ_ITRIG_SIN(s_psi, ahrs.ltp_to_lift_euler.psi);
      PPRZ_ITRIG_COS(c_psi, ahrs.ltp_to_lift_euler.psi);

      if (autopilot_mode < AP_MODE_HOVER_DIRECT) {
        wing->commands[COMMAND_ROLL]    = stab_att_sp_euler.phi;
        wing->commands[COMMAND_PITCH]    = stab_att_sp_euler.theta;
      }
      else
      {
        // Restore angle ref resolution after rotation
        wing->commands[COMMAND_ROLL]   = ( - s_psi * stab_att_sp_euler.phi + c_psi * stab_att_sp_euler.theta) >> INT32_TRIG_FRAC;
        wing->commands[COMMAND_PITCH]  = - ( c_psi * stab_att_sp_euler.phi + s_psi * stab_att_sp_euler.theta) >> INT32_TRIG_FRAC;;
      }

      wing->commands[COMMAND_THRUST] = outerloop_throttle_command;
      wing->commands[COMMAND_YAW]    = stab_att_sp_euler.psi;
    }
    else
    {
      // Plane Mode
      // ----------
      // lift command (verical acceleration) -> pitch + thrust
      // forward acceleration (neglected)
      // lateral acceleration (command) -> roll
      // heading ANGLE -> integrated

      float climb_speed = ((outerloop_throttle_command - (MAX_PPRZ / 2)) * 2 * force_allocation_fixedwing_max_climb);  // MAX_PPRZ

      // Lateral Plane Motion
      int32_t destination = 0;
      if (autopilot_mode < AP_MODE_HOVER_DIRECT) {
        wing->commands[COMMAND_ROLL]    = stab_att_sp_euler.phi;
        destination = stab_att_sp_euler.psi;
      }
      else
      {
        INT32_ATAN2(destination, stab_att_sp_euler.theta, stab_att_sp_euler.phi);
        destination -= ahrs.ltp_to_lift_euler.psi;
        INT32_ANGLE_NORMALIZE(destination);

        int32_t TRAJ_MAX_BANK = BFP_OF_REAL(RadOfDeg(max_bank_auto), INT32_ANGLE_FRAC);
        if (destination > TRAJ_MAX_BANK)
          destination = TRAJ_MAX_BANK;
        if (destination < (-TRAJ_MAX_BANK))
          destination = -TRAJ_MAX_BANK;

        wing->commands[COMMAND_ROLL] = destination;

      }

      // Longitudinal Plane Motion
      wing->commands[COMMAND_THRUST]  = (guidance_v_nominal_throttle)
                                      + climb_speed * force_allocation_fixedwing_throttle_of_vz
                                      + (-(stab_att_sp_euler.theta * MAX_PPRZ / 2) >> INT32_ANGLE_FRAC ) * force_allocation_fixedwing_throttle_of_xdd; // MAX_PPRZ
                                      //+ ((stab_att_sp_euler.theta * MAX_PPRZ) >> INT32_ANGLE_FRAC ); // MAX_PPRZ

      wing->commands[COMMAND_PITCH]   = ANGLE_BFP_OF_REAL(force_allocation_fixedwing_pitch_trim + climb_speed * force_allocation_fixedwing_pitch_of_vz / MAX_PPRZ);

      // Coordinated Turn
#if !defined(FREE_FLOATING_HEADING)
#pragma message "COORDINATED TURN"
      const int loop_rate = 512;
      wing->commands[COMMAND_YAW]    = stab_att_sp_euler.psi + wing->commands[COMMAND_ROLL] * force_allocation_fixedwing_yawrate_of_ydd / loop_rate;
#elif defined(QUADROTOR_HEADING)
      wing->commands[COMMAND_YAW]    =  stab_att_sp_euler.psi;

      wing->commands[COMMAND_YAW]    = ahrs.ltp_to_lift_euler.psi;
#endif

    }

    cmd_thrust           += wing->commands[COMMAND_THRUST]     * percent;
    command_euler.phi    += wing->commands[COMMAND_ROLL]       * percent;
    command_euler.theta  += wing->commands[COMMAND_PITCH]      * percent;
    command_euler.psi    += wing->commands[COMMAND_YAW]        * percent;     // Hmmm this would benefit from some more thinking...
    orientation_rotation += RadOfDeg((float)wing->orientation_pitch) * percent;
  }

  INT32_ANGLE_NORMALIZE(command_euler.psi);

  nav_heading = command_euler.psi;
  stab_att_sp_euler.psi = command_euler.psi;

  //guidance_h_psi_sp = command_euler.psi;

  stabilization_cmd[COMMAND_THRUST] = cmd_thrust;

  struct Int32Quat command_att;
  INT32_QUAT_OF_EULERS(command_att, command_euler);

  // Post Multiply with the pitch trim...
  QUAT_ASSIGN(ahrs.lift_to_body_quat,
      QUAT1_BFP_OF_REAL(1),
      QUAT1_BFP_OF_REAL(0),
      QUAT1_BFP_OF_REAL(orientation_rotation) / 2,
      QUAT1_BFP_OF_REAL(0) );

  INT32_QUAT_NORMALIZE(ahrs.lift_to_body_quat);
  INT32_QUAT_WRAP_SHORTEST(ahrs.lift_to_body_quat);

  INT32_QUAT_COMP(stab_att_sp_quat, command_att, ahrs.lift_to_body_quat);
//  INT32_QUAT_WRAP_SHORTEST(stab_att_sp_quat);
}

