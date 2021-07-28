/**
 *   Authored by Victor Freire (https://xu.me.wisc.edu/), May 2021
 *
 *
 */

#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "pid.h"
#include "sensfusion6.h"
#include "controller_lqr.h"

#include "log.h"
#include "param.h"
#include "math3d.h"
#include "cf_math.h"
#include "debug.h"

#include "crtp_commander_sdlqr.h"

#if defined CBF_TYPE_EUL || defined CBF_TYPE_POS
#include "aideck.h"
#endif

#ifndef CF_MASS
#define CF_MASS            0.032f
#endif

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

#ifdef LQR_ALT_PID
#define Z_PID_UPDATE_DT    (float)(1.0f/Z_PID_RATE)
#define PID_Z_KI           1.0f
#define Z_LPF_CUTOFF_FREQ  20.0f
#endif

#define DEG2RAD               (float)M_PI/180.0f
#define RAD2DEG               180.0f/(float)M_PI


// The mode is a parameter that can be changed from the client-side
static lqr_mode_t mode = D9LQR; // Default mode

static state_t err;
static attitude_t rateDesired;
static float u[4]; // Control input u = [T p q r]
static float u_D6[4]; // Control input u = [T phi theta psi]
static float actuatorThrust;
static float KD9[4][9]; // Kalman Gain (9D)
static float KD6[4][6]; // Kalman Gain (6D)
#ifdef LQR_ALT_PID
PidObject pidT; // PID object for altitude (integral)
#endif
static bool flying = false; // Set thrust to 0 when false

#if defined CBF_TYPE_POS || defined CBF_TYPE_EUL
static cbf_qpdata_t qp_data;
#endif

// Logging parameters
#ifdef LQR_ALT_PID
static float pid_T;
#endif
static float u_T;
static float u_p;
static float u_q;
static float u_r;

// Private function: Takes normalized thrust (m/s^2) and returns pwm units
static int to_pwm(float T){
  // 0 = a*(rpm)^2 -b*rpm + c - mass_in_gram
  float a = 109e-9f;
  float b = 210.6e-6f;
  float c = 0.154f;
  // RPM -> PWM conversion
  // rpm = d*pwm + e
  float d = 0.2685f;
  float e = 4070.3f;
  // Convert T to grams
  float g = (CF_MASS*1000.0f*T)/9.81f; // Mass in grams

  float r = (b+sqrtf(powf(b,2)-4*a*(c-g)))/(2*a); // Quadratic formula (+)
  int pwm = (int) ((r-e)/d);
  pwm -= 9000; // Offset calibration
  return pwm;
}

#ifdef CBF_TYPE_EUL
/**
 * Private function. Solve a QP-CBF to limit roll and pitch [phi theta]
 * The OSQP problem is solved in the AI Deck
 */
static void apply_cbf_eul(const state_t *state){
  // Populate cbf_qpdata_t
  qp_data.phi = state->attitude.roll*DEG2RAD;
  qp_data.theta = -state->attitude.pitch*DEG2RAD;
  qp_data.u.T = u[0];
  qp_data.u.p = u[1];
  qp_data.u.q = u[2];
  qp_data.u.r = u[3];
  // Send to AI Deck
  aideck_send_cbf_data(&qp_data);
  // Get the most recent safe control received from AI Deck
  aideck_get_safe_u(u);
}
#endif // CBF_TYPE_EUL


#ifdef CBF_TYPE_POS
/**
 * Private function. Solve a QP-CBF to limit position [x y z]
 * The OSQP problem is solved in the AI Deck
 */
static void apply_cbf_pos(const state_t *state){
  // Populate cbf_qpdata_t
  qp_data.x = state->position.x;
  qp_data.y = state->position.y;
  qp_data.z = state->position.z;
  qp_data.x_dot = state->velocity.x;
  qp_data.y_dot = state->velocity.y;
  qp_data.z_dot = state->velocity.z;
  qp_data.u.T = u_D6[0];
  qp_data.u.phi = u_D6[1];
  qp_data.u.theta = u_D6[2];
  qp_data.u.psi = u_D6[3];
  // Send to AI Deck
  aideck_send_cbf_data(&qp_data);
  // Get the most recent safe control received from AI Deck
  aideck_get_safe_u(u_D6);
}
#endif // CBF_TYPE_POS


// Private function: D9LQR Policy update
static void lqr_D9(setpoint_t *setpoint, const state_t *state, const uint32_t tick){
  if (RATE_DO_EXECUTE(D9LQR_RATE, tick)) {
    // Compute errors in state err = \hat{x} - x_r
    err.position.x = state->position.x - setpoint->position.x;
    err.position.y = state->position.y - setpoint->position.y;
    err.position.z = state->position.z - setpoint->position.z;
    err.attitude.roll = state->attitude.roll*DEG2RAD - setpoint->attitude.roll;
    err.attitude.pitch = -state->attitude.pitch*DEG2RAD - setpoint->attitude.pitch;
    err.attitude.yaw = state->attitude.yaw*DEG2RAD - setpoint->attitude.yaw;
    err.velocity.x = state->velocity.x - setpoint->velocity.x;
    err.velocity.y = state->velocity.y - setpoint->velocity.y;
    err.velocity.z = state->velocity.z - setpoint->velocity.z;

    // Negative state feedback: \deltau = -K*err
    u[0] = -(KD9[0][0]*err.position.x    + KD9[0][1]*err.position.y     + KD9[0][2]*err.position.z
           + KD9[0][3]*err.attitude.roll + KD9[0][4]*err.attitude.pitch + KD9[0][5]*err.attitude.yaw
           + KD9[0][6]*err.velocity.x    + KD9[0][7]*err.velocity.y     + KD9[0][8]*err.velocity.z);
    u[1] = -(KD9[1][0]*err.position.x    + KD9[1][1]*err.position.y     + KD9[1][2]*err.position.z
           + KD9[1][3]*err.attitude.roll + KD9[1][4]*err.attitude.pitch + KD9[1][5]*err.attitude.yaw
           + KD9[1][6]*err.velocity.x    + KD9[1][7]*err.velocity.y     + KD9[1][8]*err.velocity.z);
    u[2] = -(KD9[2][0]*err.position.x    + KD9[2][1]*err.position.y     + KD9[2][2]*err.position.z
           + KD9[2][3]*err.attitude.roll + KD9[2][4]*err.attitude.pitch + KD9[2][5]*err.attitude.yaw
           + KD9[2][6]*err.velocity.x    + KD9[2][7]*err.velocity.y     + KD9[2][8]*err.velocity.z);
    u[3] = -(KD9[3][0]*err.position.x    + KD9[3][1]*err.position.y     + KD9[3][2]*err.position.z
           + KD9[3][3]*err.attitude.roll + KD9[3][4]*err.attitude.pitch + KD9[3][5]*err.attitude.yaw
           + KD9[3][6]*err.velocity.x    + KD9[3][7]*err.velocity.y     + KD9[3][8]*err.velocity.z);

    // Add nominal control:  u = u_r + \deltau
    u[0] += setpoint->thrust;
    u[1] += setpoint->attitudeRate.roll;
    u[2] += setpoint->attitudeRate.pitch;
    u[3] += setpoint->attitudeRate.yaw;

#ifdef CBF_TYPE_EUL
    // Apply CBF_EUL
    apply_cbf_eul(state);  // updates u
#endif
  } // if RATE_DO_EXECUTE

} // lqr_D9()

// Private function: D6LQR Policy update
static void lqr_D6(setpoint_t *setpoint, const state_t *state, const uint32_t tick){
  if (RATE_DO_EXECUTE(D6LQR_RATE, tick)) {
    // Compute errors in state err = \hat{x} - x_r
    err.position.x = state->position.x - setpoint->position.x;
    err.position.y = state->position.y - setpoint->position.y;
    err.position.z = state->position.z - setpoint->position.z;
    err.velocity.x = state->velocity.x - setpoint->velocity.x;
    err.velocity.y = state->velocity.y - setpoint->velocity.y;
    err.velocity.z = state->velocity.z - setpoint->velocity.z;

    // Negative state feedback: \deltau = -K*err
    u_D6[0] = -(KD6[0][0]*err.position.x   + KD6[0][1]*err.position.y   + KD6[0][2]*err.position.z
              + KD6[0][3]*err.velocity.x   + KD6[0][4]*err.velocity.y   + KD6[0][5]*err.velocity.z);
    u_D6[1] = -(KD6[1][0]*err.position.x   + KD6[1][1]*err.position.y   + KD6[1][2]*err.position.z
              + KD6[1][3]*err.velocity.x   + KD6[1][4]*err.velocity.y   + KD6[1][5]*err.velocity.z);
    u_D6[2] = -(KD6[2][0]*err.position.x   + KD6[2][1]*err.position.y   + KD6[2][2]*err.position.z
              + KD6[2][3]*err.velocity.x   + KD6[2][4]*err.velocity.y   + KD6[2][5]*err.velocity.z);
    u_D6[3] = -(KD6[3][0]*err.position.x   + KD6[3][1]*err.position.y   + KD6[3][2]*err.position.z
              + KD6[3][3]*err.velocity.x   + KD6[3][4]*err.velocity.y   + KD6[3][5]*err.velocity.z);

    // Add nominal control:  u = u_r + \deltau
    u_D6[0] += setpoint->thrust;
    u_D6[1] += setpoint->attitude.roll;
    u_D6[2] += setpoint->attitude.pitch;
    u_D6[3] += setpoint->attitude.yaw;

#ifdef CBF_TYPE_POS
    // Apply CBF_POS
    apply_cbf_pos(state);  // updates u_D6
#endif

  } // if RATE_DO_EXECUTE

} // lqr_D6()




void controllerLqrInit(void){
  // Initialize 9-Dim Kalman gain with default Linearization
  //KD9[0][2] = 31.6228f;
  //KD9[0][8] = 12.7768f;
  //KD9[1][1] = -4.4732f;
  //KD9[1][3] = 7.6869f;
  //KD9[1][7] = -3.0014f;
  //KD9[2][0] = 4.4732f;
  //KD9[2][4] = 7.6869f;
  //KD9[2][6] = 3.0014f;
  //KD9[3][5] = 1.0f;
  // KD9 rho=1 Richard Murray Method
  KD9[0][2] = 4.0f;
  KD9[0][8] = 3.4641f;
  KD9[1][1] = -3.4907f;
  KD9[1][3] = 7.8518f;
  KD9[1][7] = -2.9384f;
  KD9[2][0] = 3.4907f;
  KD9[2][4] = 7.8518f;
  KD9[2][6] = 2.9384f;
  KD9[3][5] = 2.0f;

  // Initialize 6-Dim Kalman gain with default Linearization
  //KD6[0][2] = 31.6228f;
  //KD6[0][5] =  8.5584f;
  //KD6[1][1] = -2.2361f;
  //KD6[1][4] = -0.7112f;
  //KD6[2][0] =  2.2361f;
  //KD6[2][3] =  0.7112f;
#ifdef CBF_TYPE_POS
  // KD6 Q = [20 20 100 1 1 1]| R = [0.1 20 20 40]
  KD6[0][2] = 31.6228f;
  KD6[0][5] = 8.5584f;
  KD6[1][1] = -1.0f;
  KD6[1][4] = -0.5039f;
  KD6[2][0] =  1.0f;
  KD6[2][3] =  0.5039f;
#else
  // KD6 rho=0.5 Richard Murray Method
  KD6[0][2] = 5.6569f;
  KD6[0][5] = 4.3947f;
  KD6[1][1] = -2.4683f;
  KD6[1][4] = -1.4235f;
  KD6[2][0] = 2.4683f;
  KD6[2][3] = 1.4235f;
#endif

#ifdef LQR_ALT_PID
  // Initialize altitude pid (T)
  pidInit(&pidT, 0, 0, PID_Z_KI, 0, Z_PID_UPDATE_DT, Z_PID_RATE,
           Z_LPF_CUTOFF_FREQ, false);
  pidSetIntegralLimit(&pidT, 0.5); // [m] integral limit
  pidT.outputLimit = 0.5; // [m/s^2] output limit
#endif

  // Initialize attitude controller
  attitudeControllerInit(ATTITUDE_UPDATE_DT);

}

bool controllerLqrTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}


void controllerLqr(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors,
                   const state_t *state, const uint32_t tick){
  // Flying safety check at max Hz
  if (setpoint->position.z > 0)
    flying = true;
  else
    flying = false;

  // Perform the LQR Update at D9LQR_RATE or D6LQR_RATE
  if (mode==D9LQR)
    lqr_D9(setpoint, state, tick); // update u
  else if (mode==D6LQR){
    lqr_D6(setpoint, state, tick); // update u_D6
  }

  // Perform the Attitude PID Update at ATTITUDE_RATE
  if (mode==D6LQR){ // update u
    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)){
      u[0] = u_D6[0]; // T is the same
      attitudeControllerCorrectAttitudePID(state->attitude.roll, -state->attitude.pitch, state->attitude.yaw,
                               u_D6[1]*RAD2DEG, u_D6[2]*RAD2DEG, u_D6[3]*RAD2DEG,
                               &(u[1]), &(u[2]), &(u[3]) );
      // Convert to rad/s
      u[1] = u[1]*DEG2RAD;
      u[2] = u[2]*DEG2RAD;
      u[3] = u[3]*DEG2RAD;
    }
  } // If mode==D6LQR

#ifdef LQR_ALT_PID
  // Add altitude integral action at Z_PID_RATE FIXME: Acts faster than CBFs
  if(RATE_DO_EXECUTE(Z_PID_RATE, tick)){
    pidSetDesired(&pidT, setpoint->position.z);
    pid_T = pidUpdate(&pidT, state->position.z, true);
    u[0] += pid_T;
  }
#endif

  // Saturations, logging and conversions at ATTITUDE_RATE
  if(RATE_DO_EXECUTE(ATTITUDE_RATE, tick)){
    // Saturate thrust and pqr
    u[0] = fmaxf(fminf(u[0],18.0f), 0.0f); // [0,18] m/s^2
    u[1] = fmaxf(fminf(u[1],3.5f),-3.5f);  // +-200 deg/s
    u[2] = fmaxf(fminf(u[2],3.5f),-3.5f);  // +-200 deg/s
    u[3] = fmaxf(fminf(u[3],3.5f),-3.5f);  // +-200 deg/s

    // Parameter logging
    u_T = u[0];
    u_p = u[1];
    u_q = u[2];
    u_r = u[3];

    // convert [T, p, q, r] to 0xFFFF and deg/s
    actuatorThrust = to_pwm(u[0]);
    rateDesired.roll = u[1]*RAD2DEG;
    rateDesired.pitch = u[2]*RAD2DEG;
    rateDesired.yaw = u[3]*RAD2DEG;
  }

  // Set thrust to 0 if z_desired is 0 and we are close to target
  if((err.position.x + err.position.y + err.position.y) < 0.075f && setpoint->position.z == 0)
      flying = false;
  if(!flying){
      actuatorThrust = 0;
  }

  // Attitude Rate Controller 500 Hz
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Attitude Rate PID update
    attitudeControllerCorrectRatePID(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);
  }

  // Power Distribution variable update
  control->thrust = actuatorThrust;

  // Safety for power distribution
  if (control->thrust == 0)
  {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    attitudeControllerResetAllPID();
#ifdef LQR_ALT_PID
    pidReset(&pidT); // Reset the altitude pid
#endif
  }
  else{
    // Power Distribution variables update
    attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw);
  }
}

// Public function to update entries of KD9 (see crtp_commander_sdlqr)
void update_K_entry(const uint8_t i, const uint8_t j, float value){
    KD9[i][j] = value;
}


/**
 * Parameters to set the LQR Controller mode (D6LQR or D9LQR)
 */
PARAM_GROUP_START(controller_lqr)
/**
 * @brief LQR Controller mode D9LQR(0), D6LQR(1), (Default: 0)
 */
PARAM_ADD_CORE(PARAM_UINT8, mode, &mode)
PARAM_GROUP_STOP(controller_lqr)

/**
 * Logging variables
 */
LOG_GROUP_START(controller_lqr)
LOG_ADD(LOG_FLOAT, u_T, &u_T)
LOG_ADD(LOG_FLOAT, u_p, &u_p)
LOG_ADD(LOG_FLOAT, u_q, &u_q)
LOG_ADD(LOG_FLOAT, u_r, &u_r)
#ifdef LQR_ALT_PID
LOG_ADD(LOG_FLOAT, pid_T, &pid_T)
#endif
LOG_GROUP_STOP(controller_lqr)
