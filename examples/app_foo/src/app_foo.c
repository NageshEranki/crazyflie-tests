/**
 * app_foo.c: Demonstrate the workflow of using the app layer
 */

#include "app.h"
#include "app_channel.h"
#include "log.h"
#include "debug.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "HELLOWORLD"

// Loop counter
static uint64_t counter = 0;


// Lorenz system params
static const float s = 10.0f;
static const float r = 28.0f;
static const float b = 8.0f/3.0f;

static float xn1 = -1.0f;
static float xn2 = 0.0f;
static float xn3 = 0.0f;

// time step size
static float h = 0.001f;

static float k1 = 0.0f;
static float k2 = 0.0f;
static float k3 = 0.0f;
static float k4 = 0.0f;

static float l1 = 0.0f;
static float l2 = 0.0f;
static float l3 = 0.0f;
static float l4 = 0.0f;

static float m1 = 0.0f;
static float m2 = 0.0f;
static float m3 = 0.0f;
static float m4 = 0.0f;

// struct solverStartRX {
//   uint8_t start;
// } __attribute__((packed));

// struct testPacketTX {
//   uint8_t started;
// } __attribute__((packed));

// // RHS 'f'
// float f(float x1, float x2)
// {
//   return x2;
// }

// // RHS 'g'
// float g(float x1, float x2)
// {
//   // return -2*x1+3*x2;
//   return -2*x1;
// }

// RHS 'f1'
float f1(float x1, float x2, float x3)
{
  return s * (x2 - x1);
}

// RHS 'f2'
float f2(float x1, float x2, float x3)
{
  return x1 * (r - x3) - x2;
}

// RHS 'f3'
float f3(float x1, float x2, float x3)
{
  return x1 * x2 - b * x3;
}

void appMain()
{

  /*
      The 'main loop' that runs forever. This function should NEVER RETURN!

      This function iterates to solve systems of ODEs using the Runge-Kutta 4 method.

      x1dot = x2;
      x2dot = -2x1;
  */
  DEBUG_PRINT("Started appTask! ...\n");

  // struct solverStartRX ssPacket;
  // struct testPacketTX txPacket;


  vTaskDelay(M2T(30000));

  // Iterate!
  while(1) {

    if (1)
    {
      // k1 = h * f(xn1, xn2);
      // l1 = h * g(xn1, xn2);
      // k2 = h * f(xn1+k1/2.0f, xn2+l1/2.0f);
      // l2 = h * g(xn1+k1/2.0f, xn2+l1/2.0f);
      // k3 = h * f(xn1+k2/2.0f, xn2+l2/2.0f);
      // l3 = h * g(xn1+k2/2.0f, xn2+l2/2.0f);
      // k4 = h * f(xn1+k3, xn2+l3);
      // l4 = h * g(xn1+k3, xn2+l3);

      // xn1 = xn1 + 1.0f/6.0f * (k1 + 2*k2 + 2*k3 + k4);
      // xn2 = xn2 + 1.0f/6.0f * (l1 + 2*l2 + 2*l3 + l4);

      k1 = h * f1(xn1, xn2, xn3);
      l1 = h * f2(xn1, xn2, xn3);
      m1 = h * f3(xn1, xn2, xn3);

      k2 = h * f1(xn1+k1/2.0f, xn2+l1/2.0f, xn3+m1/2.0f);
      l2 = h * f2(xn1+k1/2.0f, xn2+l1/2.0f, xn3+m1/2.0f);
      m2 = h * f3(xn1+k1/2.0f, xn2+l1/2.0f, xn3+m1/2.0f);

      k3 = h * f1(xn1+k2/2.0f, xn2+l2/2.0f, xn3+m2/2.0f);
      l3 = h * f2(xn1+k2/2.0f, xn2+l2/2.0f, xn3+m2/2.0f);
      m3 = h * f3(xn1+k2/2.0f, xn2+l2/2.0f, xn3+m2/2.0f);

      k4 = h * f1(xn1+k3, xn2+l3, xn3+m3);
      l4 = h * f2(xn1+k3, xn2+l3, xn3+m3);
      m4 = h * f3(xn1+k3, xn2+l3, xn3+m3);

      xn1 = xn1 + 1.0f/6.0f * (k1 + 2*k2 + 2*k3 + k4);
      xn2 = xn2 + 1.0f/6.0f * (l1 + 2*l2 + 2*l3 + l4);
      xn3 = xn3 + 1.0f/6.0f * (m1 + 2*m2 + 2*m3 + m4);

    }
    

    // IMPORTANT: Must include this artificial delay.
    // this loop must yield to the FreeRTOS scheduler
    // compulsorily.
    vTaskDelay(M2T(1));


    // Increment loop counter
    counter++;
  }
}

LOG_GROUP_START(rk4solve)
LOG_ADD(LOG_FLOAT, xn1, &xn1)
LOG_ADD(LOG_FLOAT, xn2, &xn2)
LOG_ADD(LOG_FLOAT, xn3, &xn3)
LOG_GROUP_STOP(rk4solve)
