/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * position_estimator_pid.c: PID-based implementation of the position controller
 */

#include <math.h>
#include "num.h"

#include "commander.h"
#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"

struct pidInit_s {
  float kp;
  float ki;
  float kd;
};

struct pidAxis_s {
  PidObject pid;

  struct pidInit_s init;
    stab_mode_t previousMode;
  float setpoint;

  float output;
};

struct this_s {
  struct pidAxis_s pidVX;
  struct pidAxis_s pidVY;
  struct pidAxis_s pidVZ;

  struct pidAxis_s pidX;
  struct pidAxis_s pidY;
  struct pidAxis_s pidZ;

  uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
  uint16_t thrustMin;  // Minimum thrust value to output
};

// Maximum roll/pitch angle permited
static float rpLimit  = 20;
static float rpLimitOverhead = 1.10f;
// Velocity maximums
static float xyVelMax = 1.0f;
static float zVelMax  = 1.0f;
static float velMaxOverhead = 1.10f;
static const float thrustScale = 1000.0f;

#define DT (float)(1.0f/POSITION_RATE)
#define POSITION_LPF_CUTOFF_FREQ 20.0f
#define POSITION_LPF_ENABLE true

#ifndef UNIT_TEST
static struct this_s this = {
  .pidVX = {
    .init = {
      .kp = 25.0f,
      .ki = 1.0f,
      .kd = 0.0f,
    },
    .pid.dt = DT,
  },

  .pidVY = {
    .init = {
      .kp = 25.0f,
      .ki = 1.0f,
      .kd = 0.0f,
    },
    .pid.dt = DT,
  },

  .pidVZ = {
    .init = {
      .kp = 25,
      .ki = 15,
      .kd = 0,
    },
    .pid.dt = DT,
  },

  .pidX = {
    .init = {
      .kp = 2.0f,
      .ki = 0,
      .kd = 0,
    },
    .pid.dt = DT,
  },

  .pidY = {
    .init = {
      .kp = 2.0f,
      .ki = 0,
      .kd = 0,
    },
    .pid.dt = DT,
  },

  .pidZ = {
    .init = {
      .kp = 2.0f,
      .ki = 0.5,
      .kd = 0,
    },
    .pid.dt = DT,
  },

  .thrustBase = 36000,
  .thrustMin  = 20000,
};
#endif

void positionControllerInit()
{
  pidInit(&this.pidX.pid, this.pidX.setpoint, this.pidX.init.kp, this.pidX.init.ki, this.pidX.init.kd,
      this.pidX.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&this.pidY.pid, this.pidY.setpoint, this.pidY.init.kp, this.pidY.init.ki, this.pidY.init.kd,
      this.pidY.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&this.pidZ.pid, this.pidZ.setpoint, this.pidZ.init.kp, this.pidZ.init.ki, this.pidZ.init.kd,
      this.pidZ.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);

  pidInit(&this.pidVX.pid, this.pidVX.setpoint, this.pidVX.init.kp, this.pidVX.init.ki, this.pidVX.init.kd,
      this.pidVX.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&this.pidVY.pid, this.pidVY.setpoint, this.pidVY.init.kp, this.pidVY.init.ki, this.pidVY.init.kd,
      this.pidVY.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&this.pidVZ.pid, this.pidVZ.setpoint, this.pidVZ.init.kp, this.pidVZ.init.ki, this.pidVZ.init.kd,
      this.pidVZ.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
}

static float runPid(float input, struct pidAxis_s *axis, float setpoint, float dt) {
  axis->setpoint = setpoint;

  pidSetDesired(&axis->pid, axis->setpoint);
  return pidUpdate(&axis->pid, input, true);
}

void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  this.pidX.pid.outputLimit = xyVelMax * velMaxOverhead;
  this.pidY.pid.outputLimit = xyVelMax * velMaxOverhead;
  // The ROS landing detector will prematurely trip if
  // this value is below 0.5
  this.pidZ.pid.outputLimit = fmaxf(zVelMax, 0.5f)  * velMaxOverhead;

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  float bodyvx = setpoint->velocity.x;
  float bodyvy = setpoint->velocity.y;

  // X, Y
  if (setpoint->mode.x == modeAbs) {
    setpoint->velocity.x = runPid(state->position.x, &this.pidX, setpoint->position.x, DT);
  } else if (setpoint->velocity_body) {
    setpoint->velocity.x = bodyvx * cosyaw - bodyvy * sinyaw;
  }
  if (setpoint->mode.y == modeAbs) {
    setpoint->velocity.y = runPid(state->position.y, &this.pidY, setpoint->position.y, DT);
  } else if (setpoint->velocity_body) {
    setpoint->velocity.y = bodyvy * cosyaw + bodyvx * sinyaw;
  }
  if (setpoint->mode.z == modeAbs) {
    setpoint->velocity.z = runPid(state->position.z, &this.pidZ, setpoint->position.z, DT);
  }

  velocityController(thrust, attitude, setpoint, state);
}

void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  this.pidVX.pid.outputLimit = rpLimit * rpLimitOverhead;
  this.pidVY.pid.outputLimit = rpLimit * rpLimitOverhead;
  // Set the output limit to the maximum thrust range
  this.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);
  //this.pidVZ.pid.outputLimit = (this.thrustBase - this.thrustMin) / thrustScale;

  // Roll and Pitch
  float rollRaw  = runPid(state->velocity.x, &this.pidVX, setpoint->velocity.x, DT);
  float pitchRaw = runPid(state->velocity.y, &this.pidVY, setpoint->velocity.y, DT);

  float yawRad = state->attitude.yaw * (float)M_PI / 180;
  attitude->pitch = +(rollRaw  * cosf(yawRad)) + (pitchRaw * sinf(yawRad));
  attitude->roll  = -(pitchRaw * cosf(yawRad)) + (rollRaw  * sinf(yawRad));

  attitude->roll  = constrain(attitude->roll,  -rpLimit, rpLimit);
  attitude->pitch = constrain(attitude->pitch, -rpLimit, rpLimit);

  // Thrust
  float thrustRaw = runPid(state->velocity.z, &this.pidVZ, setpoint->velocity.z, DT);
  // Scale the thrust and add feed forward term
  *thrust = thrustRaw*thrustScale + this.thrustBase;
  // Check for minimum thrust
  if (*thrust < this.thrustMin) {
    *thrust = this.thrustMin;
  }
}

void positionControllerResetAllPID()
{
  pidReset(&this.pidX.pid);
  pidReset(&this.pidY.pid);
  pidReset(&this.pidZ.pid);
  pidReset(&this.pidVX.pid);
  pidReset(&this.pidVY.pid);
  pidReset(&this.pidVZ.pid);
}

/**
 * Log variables of the PID position controller
 * 
 * Note: rename to posCtrlPID ?
 */
LOG_GROUP_START(posCtl)

/**
 * @brief PID controller target desired velocity x [m/s]
 * 
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVX, &this.pidVX.pid.desired)
/**
 * @brief PID controller target desired velocity y [m/s]
 * 
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVY, &this.pidVY.pid.desired)
/**
 * @brief PID controller target desired velocity z [m/s]
 * 
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVZ, &this.pidVZ.pid.desired)
/**
 * @brief PID controller target desired position x [m]
 * 
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetX, &this.pidX.pid.desired)
/**
 * @brief PID controller target desired position y [m]
 * 
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetY, &this.pidY.pid.desired)
/**
 * @brief PID controller target desired position z [m]
 * 
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetZ, &this.pidZ.pid.desired)

/**
 * @brief PID proportional output position x
 */
LOG_ADD(LOG_FLOAT, Xp, &this.pidX.pid.outP)
/**
 * @brief PID Integral output position x
 */
LOG_ADD(LOG_FLOAT, Xi, &this.pidX.pid.outI)
/**
 * @brief PID Derivative output position x
 */
LOG_ADD(LOG_FLOAT, Xd, &this.pidX.pid.outD)

/**
 * @brief PID proportional output position y
 */
LOG_ADD(LOG_FLOAT, Yp, &this.pidY.pid.outP)
/**
 * @brief PID Integral output position y
 */
LOG_ADD(LOG_FLOAT, Yi, &this.pidY.pid.outI)
/**
 * @brief PID Derivative output position y
 */
LOG_ADD(LOG_FLOAT, Yd, &this.pidY.pid.outD)

/**
 * @brief PID proportional output position z
 */
LOG_ADD(LOG_FLOAT, Zp, &this.pidZ.pid.outP)
/**
 * @brief PID Integral output position z
 */
LOG_ADD(LOG_FLOAT, Zi, &this.pidZ.pid.outI)
/**
 * @brief PID derivative output position z
 */
LOG_ADD(LOG_FLOAT, Zd, &this.pidZ.pid.outD)

/**
 * @brief PID proportional output velocity x
 */
LOG_ADD(LOG_FLOAT, VXp, &this.pidVX.pid.outP)
/**
 * @brief PID integral output velocity x
 */
LOG_ADD(LOG_FLOAT, VXi, &this.pidVX.pid.outI)
/**
 * @brief PID derivative output velocity x
 */
LOG_ADD(LOG_FLOAT, VXd, &this.pidVX.pid.outD)

/**
 * @brief PID proportional output velocity z
 */
LOG_ADD(LOG_FLOAT, VZp, &this.pidVZ.pid.outP)
/**
 * @brief PID integral output velocity z
 */
LOG_ADD(LOG_FLOAT, VZi, &this.pidVZ.pid.outI)
/**
 * @brief PID intrgral output velocity z
 */
LOG_ADD(LOG_FLOAT, VZd, &this.pidVZ.pid.outD)

LOG_GROUP_STOP(posCtl)

/**
 * Tuning settings for the gains of the PID
 * controller for the velocity of the Crazyflie ¨
 * in the X, Y and Z direction in the body fixed
 * coordinate system.
 */
PARAM_GROUP_START(velCtlPid)
/**
 * @brief Proportional gain for the velocity PID in the body X direction
 */
PARAM_ADD(PARAM_FLOAT, vxKp, &this.pidVX.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the body X direction
 */
PARAM_ADD(PARAM_FLOAT, vxKi, &this.pidVX.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the body X direction
 */
PARAM_ADD(PARAM_FLOAT, vxKd, &this.pidVX.pid.kd)

/**
 * @brief Proportional gain for the velocity PID in the body Y direction
 */
PARAM_ADD(PARAM_FLOAT, vyKp, &this.pidVY.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the body Y direction
 */
PARAM_ADD(PARAM_FLOAT, vyKi, &this.pidVY.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the body Y direction
 */
PARAM_ADD(PARAM_FLOAT, vyKd, &this.pidVY.pid.kd)

/**
 * @brief Proportional gain for the velocity PID in the body Z direction
 */
PARAM_ADD(PARAM_FLOAT, vzKp, &this.pidVZ.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the body Z direction
 */
PARAM_ADD(PARAM_FLOAT, vzKi, &this.pidVZ.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the body Z direction
 */
PARAM_ADD(PARAM_FLOAT, vzKd, &this.pidVZ.pid.kd)

PARAM_GROUP_STOP(velCtlPid)

/**
 * Tuning settings for the gains of the PID
 * controller for the position of the Crazyflie ¨
 * in the X, Y and Z direction in the global
 * coordinate system.
 */
PARAM_GROUP_START(posCtlPid)
/**
 * @brief Proportional gain for the position PID in the global X direction
 */
PARAM_ADD(PARAM_FLOAT, xKp, &this.pidX.pid.kp)
/**
 * @brief Proportional gain for the position PID in the global X direction
 */
PARAM_ADD(PARAM_FLOAT, xKi, &this.pidX.pid.ki)
/**
 * @brief Derivative gain for the position PID in the global X direction
 */
PARAM_ADD(PARAM_FLOAT, xKd, &this.pidX.pid.kd)

/**
 * @brief Proportional gain for the position PID in the global Y direction
 */
PARAM_ADD(PARAM_FLOAT, yKp, &this.pidY.pid.kp)
/**
 * @brief Integral gain for the position PID in the global Y direction
 */
PARAM_ADD(PARAM_FLOAT, yKi, &this.pidY.pid.ki)
/**
 * @brief Derivative gain for the position PID in the global Y direction
 */
PARAM_ADD(PARAM_FLOAT, yKd, &this.pidY.pid.kd)

/**
 * @brief Proportional gain for the position PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT, zKp, &this.pidZ.pid.kp)
/**
 * @brief Integral gain for the position PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT, zKi, &this.pidZ.pid.ki)
/**
 * @brief Derivative gain for the position PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT, zKd, &this.pidZ.pid.kd)

/**
 * @brief Approx. thrust needed for hover
 */
PARAM_ADD(PARAM_UINT16, thrustBase, &this.thrustBase)
/**
 * @brief Min. thrust value to output
 */
PARAM_ADD(PARAM_UINT16, thrustMin, &this.thrustMin)

/**
 * @brief Roll/Pitch absolute limit
 */
PARAM_ADD(PARAM_FLOAT, rpLimit,  &rpLimit)
/**
 * @brief Maximum X/Y velocity
 */
PARAM_ADD(PARAM_FLOAT, xyVelMax, &xyVelMax)
/**
 * @brief Maximum Z Velocity
 */
PARAM_ADD(PARAM_FLOAT, zVelMax,  &zVelMax)

PARAM_GROUP_STOP(posCtlPid)
