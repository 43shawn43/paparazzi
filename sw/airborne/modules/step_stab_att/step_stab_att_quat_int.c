/*
 * Copyright (C) 2021 Dennis van Wijngaarden
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
 *
 */

/** @file step_stab_att_quat_int.c
 *  @brief Step stab att quat int
 */

#include <stdio.h>
#include "modules/step_stab_att/step_stab_att_quat_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "generated/settings.h"
#include "autopilot.h"
#include "state.h"

// Declare variables which can be set using settings (as indicated in module xml file)
struct step_yaw_variables step_yaw = {
  .add_yaw_step = false,
  .yaw_step_size = 0.     // degrees
};

void step_stab_att_quat_int_event(void) 
{
  if (step_yaw.add_yaw_step) {
    // Add step to reference and set add_yaw_step variable to false

    struct FloatQuat q_rp_cmd;
    stabilization_attitude_read_rc_roll_pitch_quat_f(&q_rp_cmd);

    /* get current heading */
    const struct FloatVect3 zaxis = {0., 0., 1.};
    struct FloatQuat q_yaw;

    float_quat_of_axis_angle(&q_yaw, &zaxis, stateGetNedToBodyEulers_f()->psi);

    /* roll/pitch commands applied to to current heading */
    struct FloatQuat q_rp_sp;
    float_quat_comp(&q_rp_sp, &q_yaw, &q_rp_cmd);
    float_quat_normalize(&q_rp_sp);

    /* get current heading setpoint */
    struct FloatQuat q_yaw_sp;
    // add psi to stab_att_sp_euler
    

    #if defined STABILIZATION_ATTITUDE_TYPE_INT
      stab_att_sp_euler.psi += ANGLE_BFP_OF_REAL(step_yaw.yaw_step_size / 180. * 3.141592); // To radians
      float_quat_of_axis_angle(&q_yaw_sp, &zaxis, ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.psi));
    #else
      stab_att_sp_euler.psi += step_yaw.yaw_step_size / 180. * 3.141592; // To radians
      float_quat_of_axis_angle(&q_yaw_sp, &zaxis, stab_att_sp_euler.psi);
    #endif

    /* rotation between current yaw and yaw setpoint */
    struct FloatQuat q_yaw_diff;
    float_quat_comp_inv(&q_yaw_diff, &q_yaw_sp, &q_yaw);

    /* compute final setpoint with yaw */
    struct FloatQuat q_sp;
    float_quat_comp_norm_shortest(&q_sp, &q_rp_sp, &q_yaw_diff);

    QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp)

    step_yaw.add_yaw_step = false;
  };
  return;
};