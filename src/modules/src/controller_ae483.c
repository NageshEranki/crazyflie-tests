#include "controller_ae483.h"
#include "stabilizer_types.h"
#include "power_distribution.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

// Sensor measurements
// - tof (from the z ranger on the flow deck)
static uint16_t tof_count = 0;
static float tof_distance = 0.0f;
// - flow (from the optical flow sensor on the flow deck)
static uint16_t flow_count = 0;
static float flow_dpixelx = 0.0f;
static float flow_dpixely = 0.0f;

// // Parameters
static bool use_observer = false;
static bool reset_observer = false;

// State
//------------------------------
// Position
//------------------------------
static float o_x = 0.0f;
static float o_y = 0.0f;
static float o_z = 0.0f;
//------------------------------
// Position from Qualisys
//------------------------------
// static float o_x_qtm = 0.0f;
// static float o_y_qtm = 0.0f;
// static float o_z_qtm = 0.0f;
// static float o_x_qtm_old = 0.0f;
// static float o_y_qtm_old = 0.0f;
// static float o_z_qtm_old = 0.0f;
//------------------------------
// Attitude
//------------------------------
static float psi = 0.0f;
static float theta = 0.0f;
static float phi = 0.0f;
//------------------------------
// Quaternion from Qualisys
//------------------------------
// static float qx_qtm = 0.0f;
// static float qy_qtm = 0.0f;
// static float qz_qtm = 0.0f;
// static float qw_qtm = 0.0f;
//------------------------------
// Velocity
//------------------------------
static float v_x = 0.0f;
static float v_y = 0.0f;
static float v_z = 0.0f;
//------------------------------
// Angular velocity
//------------------------------
static float w_x = 0.0f;
static float w_y = 0.0f;
static float w_z = 0.0f;
//------------------------------
// Angular acc (numerical diff.)
//------------------------------
static float w_x_dot;
static float w_x_old;
static float w_y_dot;
static float w_y_old;
//------------------------------
// Set-point
//------------------------------
static float o_x_des = 0.0f;
static float o_y_des = 0.0f;
static float o_z_des = 0.0f;
//------------------------------
// Input
//------------------------------
static float tau_x = 0.0f;
static float tau_y = 0.0f;
static float tau_z = 0.0f;
static float f_z = 0.0f;
//------------------------------
// Motor power command
//------------------------------
static uint16_t m_1 = 0;
static uint16_t m_2 = 0;
static uint16_t m_3 = 0;
static uint16_t m_4 = 0;
//------------------------------
// Measurements
//------------------------------
static float n_x = 0.0f;
static float n_y = 0.0f;
static float r = 0.0f;
static float a_z = 0.0f;
//------------------------------
// Constants
//------------------------------
// static float k_flow = 4.09255568f;
static float g = 9.81f;
static float dt = 0.002f;
// static float o_z_eq = 0.5f;
//------------------------------
// Measurement errors
//------------------------------
static float n_x_err = 0.0f;
static float n_y_err = 0.0f;
static float r_err = 0.0f;
//------------------------------

void ae483UpdateWithTOF(tofMeasurement_t *tof)
{
  tof_distance = tof->distance;
  tof_count++;
}

void ae483UpdateWithFlow(flowMeasurement_t *flow)
{
  flow_dpixelx = flow->dpixelx;
  flow_dpixely = flow->dpixely;
  flow_count++;
}

void ae483UpdateWithDistance(distanceMeasurement_t *meas)
{
  // If you have a loco positioning deck, this function will be called
  // each time a distance measurement is available. You will have to write
  // code to handle these measurements. These data are available:
  //
  //  meas->anchorId  uint8_t   id of anchor with respect to which distance was measured
  //  meas->x         float     x position of this anchor
  //  meas->y         float     y position of this anchor
  //  meas->z         float     z position of this anchor
  //  meas->distance  float     the measured distance
}

void ae483UpdateWithPosition(positionMeasurement_t *meas)
{
  // This function will be called each time you send an external position
  // measurement (x, y, z) from the client, e.g., from a motion capture system.
  // You will have to write code to handle these measurements. These data are
  // available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
}

void ae483UpdateWithPose(poseMeasurement_t *meas)
{
  // This function will be called each time you send an external "pose" measurement
  // (position as x, y, z and orientation as quaternion) from the client, e.g., from
  // a motion capture system. You will have to write code to handle these measurements.
  // These data are available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
  //  meas->quat.x    float     x component of quaternion from external orientation measurement
  //  meas->quat.y    float     y component of quaternion from external orientation measurement
  //  meas->quat.z    float     z component of quaternion from external orientation measurement
  //  meas->quat.w    float     w component of quaternion from external orientation measurement
}

void ae483UpdateWithData(const struct AE483Data* data)
{
  // This function will be called each time AE483-specific data are sent
  // from the client to the drone. You will have to write code to handle
  // these data. For the example AE483Data struct, these data are:
  //
  //  data->x         float
  //  data->y         float
  //  data->z         float
  //
  // Exactly what "x", "y", and "z" mean in this context is up to you.
}


void controllerAE483Init(void)
{
  // Do nothing
}

bool controllerAE483Test(void)
{
  // Do nothing (test is always passed)
  return true;
}

void controllerAE483(control_t *control,
                     setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    //Whatever is in here executes at 500 Hz

    // Desired position
    o_x_des = setpoint->position.x;
    o_y_des = setpoint->position.y;
    o_z_des = setpoint->position.z;

    // Measurements
    w_x = radians(sensors->gyro.x);
    w_y = radians(sensors->gyro.y);
    w_z = radians(sensors->gyro.z);
    a_z = g * sensors->acc.z;
    n_x = flow_dpixelx;
    n_y = flow_dpixely;
    r = tof_distance;

    if (reset_observer) {
      o_x = 0.0f;
      o_y = 0.0f;
      o_z = 0.0f;
      psi = 0.0f;
      theta = 0.0f;
      phi = 0.0f;
      v_x = 0.0f;
      v_y = 0.0f;
      v_z = 0.0f;
      reset_observer = false;
    }

    // State estimates
    if (use_observer) {
    

      // FIXME: your code goes here
      n_x_err = -n_x + 8.18511136F*v_x - 4.09255568F*w_y;
      n_y_err = -n_y + 8.18511136F*v_y + 4.09255568F*w_x;
      r_err   = 1.0F*o_z - r;

      o_x   += dt * (v_x);
      o_y   += dt * (v_y);
      o_z   += dt * (-22.3215143F*r_err + 1.0F*v_z);
      psi   += dt * (w_z);
      theta += dt * (-0.0137195122F*n_x_err + 1.0F*w_y);
      phi   += dt * (0.0128755365F*n_y_err + 1.0F*w_x);
      v_x   += dt * (-0.22526723F*n_x_err + 9.81F*theta);
      v_y   += dt * (-0.218658417F*n_y_err - 9.81F*phi);
      v_z   += dt * (1.0F*a_z - 1.0F*g - 96.0F*r_err);

      // Update estimates
      // FIXME: your code goes here
      
    } else {
      o_x = state->position.x;
      o_y = state->position.y;
      o_z = state->position.z;
      psi = radians(state->attitude.yaw);
      theta = - radians(state->attitude.pitch);
      phi = radians(state->attitude.roll);
      v_x = state->velocity.x*cosf(psi)*cosf(theta) + state->velocity.y*sinf(psi)*cosf(theta) - state->velocity.z*sinf(theta);
      v_y = state->velocity.x*(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)) + state->velocity.y*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)) + state->velocity.z*sinf(phi)*cosf(theta);
      v_z = state->velocity.x*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)) + state->velocity.y*(-sinf(phi)*cosf(psi) + sinf(psi)*sinf(theta)*cosf(phi)) + state->velocity.z*cosf(phi)*cosf(theta);
    }

    if(setpoint->mode.z ==  modeDisable)
    {
      powerSet(0, 0, 0, 0);
    } else
    {
      // Compute derivative of angular velocities
      // thru finite differencing
      w_x_dot = (w_x - w_x_old)*500.0f;
      w_y_dot = (w_y - w_y_old)*500.0f;

      // For thesis: Improved (as of March 28, 2024)
      tau_x = 0.00383056f * (o_y - o_y_des) -0.01292385f * phi + 0.00354096f * v_y -0.00230436f * w_x;
      tau_y = -0.00383056f * (o_x - o_x_des) -0.01292648f * theta -0.00354125f * v_x -0.00230533f * w_y;
      tau_z = -0.00130574f * psi -0.00097012f * w_z;
      f_z = -0.76830954f * (o_z - o_z_des) -0.58114773f * v_z + 0.32569200f;

      // Additional damping
      tau_x -= 0.00006f*w_x_dot;
      tau_y -= 0.00006f*w_y_dot;
      // ------------------------------------------------------------------------------------------------------
      // Mapping b/w inputs and motor commands
      m_1 = limitUint16( -4311242.0f * tau_x -4311242.0f * tau_y -28468908.0f * tau_z + 142271.0f * f_z );
      m_2 = limitUint16( -4311242.0f * tau_x + 4311242.0f * tau_y + 28468908.0f * tau_z + 142271.0f * f_z );
      m_3 = limitUint16( 4311242.0f * tau_x + 4311242.0f * tau_y -28468908.0f * tau_z + 142271.0f * f_z );
      m_4 = limitUint16( 4311242.0f * tau_x -4311242.0f * tau_y + 28468908.0f * tau_z + 142271.0f * f_z );

      // Update states for derivative calcs.
      w_x_old = w_x;
      w_y_old = w_y;

      powerSet(m_1, m_2, m_3, m_4);

    }    
  }
}

//              1234567890123456789012345678 <-- max total length
//              group   .name
LOG_GROUP_START(ae483log)
LOG_ADD(LOG_UINT16,         num_tof,                &tof_count)
LOG_ADD(LOG_UINT16,         num_flow,               &flow_count)
LOG_ADD(LOG_FLOAT,          o_x,                    &o_x)
LOG_ADD(LOG_FLOAT,          o_y,                    &o_y)
LOG_ADD(LOG_FLOAT,          o_z,                    &o_z)
// LOG_ADD(LOG_FLOAT,          o_x_qtm,                &o_x_qtm)
// LOG_ADD(LOG_FLOAT,          o_y_qtm,                &o_y_qtm)
// LOG_ADD(LOG_FLOAT,          o_z_qtm,                &o_z_qtm)
// LOG_ADD(LOG_FLOAT,          qx_qtm,                 &qx_qtm)
// LOG_ADD(LOG_FLOAT,          qy_qtm,                 &qy_qtm)
// LOG_ADD(LOG_FLOAT,          qz_qtm,                 &qz_qtm)
// LOG_ADD(LOG_FLOAT,          qw_qtm,                 &qw_qtm)
LOG_ADD(LOG_FLOAT,          psi,                    &psi)
LOG_ADD(LOG_FLOAT,          theta,                  &theta)
LOG_ADD(LOG_FLOAT,          phi,                    &phi)
LOG_ADD(LOG_FLOAT,          v_x,                    &v_x)
LOG_ADD(LOG_FLOAT,          v_y,                    &v_y)
LOG_ADD(LOG_FLOAT,          v_z,                    &v_z)
// LOG_ADD(LOG_FLOAT,          vxf,                    &vxf)
// LOG_ADD(LOG_FLOAT,          vyf,                    &vyf)
// LOG_ADD(LOG_FLOAT,          vzf,                    &vzf)
LOG_ADD(LOG_FLOAT,          w_x,                    &w_x)
LOG_ADD(LOG_FLOAT,          w_y,                    &w_y)
LOG_ADD(LOG_FLOAT,          w_z,                    &w_z)
LOG_ADD(LOG_FLOAT,          o_x_des,                &o_x_des)
LOG_ADD(LOG_FLOAT,          o_y_des,                &o_y_des)
LOG_ADD(LOG_FLOAT,          o_z_des,                &o_z_des)
LOG_ADD(LOG_FLOAT,          tau_x,                  &tau_x)
LOG_ADD(LOG_FLOAT,          tau_y,                  &tau_y)
LOG_ADD(LOG_FLOAT,          tau_z,                  &tau_z)
LOG_ADD(LOG_FLOAT,          f_z,                    &f_z)
LOG_ADD(LOG_UINT16,         m_1,                    &m_1)
LOG_ADD(LOG_UINT16,         m_2,                    &m_2)
LOG_ADD(LOG_UINT16,         m_3,                    &m_3)
LOG_ADD(LOG_UINT16,         m_4,                    &m_4)
LOG_ADD(LOG_FLOAT,          n_x,                    &n_x)
LOG_ADD(LOG_FLOAT,          n_y,                    &n_y)
LOG_ADD(LOG_FLOAT,          r,                      &r)
LOG_ADD(LOG_FLOAT,          a_z,                    &a_z)
LOG_GROUP_STOP(ae483log)

//                1234567890123456789012345678 <-- max total length
//                group   .name
PARAM_GROUP_START(ae483par)
PARAM_ADD(PARAM_UINT8,     use_observer,            &use_observer)
PARAM_ADD(PARAM_UINT8,     reset_observer,          &reset_observer)
PARAM_GROUP_STOP(ae483par)