# Appchannel test app for Crazyflie 2.x

This application solves a system of ODEs using the 4th order Runge-Kutta method. 

This demo defines a protocol where the Crazyflie waits for 3 floas (x, y, z) and sends back the sum as one float.

To run this example, compile and flash the app with ```make && make cload```.

After flashing, turn on the Crazyflie and the app will start if the system is properly initialised. The ODE solver should start right away. The solution x(t) is
available as logging variables under the logging group 'rk4solve'

The solution can be visualised in real time using the 'Plotter' tab in the GUI client.

run ```python tools/flight.py``` to log the ODE solution and optionally run a flight :)