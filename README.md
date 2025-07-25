# cpp-pid

Practical header-only C++ library for deploying a PID on embedded systems / general control systems projects.

Features:
* Different implementations of the PID controller:
  * Parallel form PID
  * Parallel form PID with the derivative on measurement ([Derivative Kick](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-derivative-kick/))
  * Standard form PID with the derivative on measurement
* Integral windup clamping
* Output clamping
* Low-pass filter on the derivative term and output term
* Pre-multiplication of the time step with the integral and derivative gains

As used in: \
[A cascaded TVC rocket project](https://github.com/CAPTR-Project/CAPTR-V1-AVI) \
[A cascaded retractable drone slung payload project](https://github.com/UTAT-UAS/ARCHIVE_hardware_integration/tree/main/packages/ros-payload-control)


## Implementations

### Parallel Form PID
The parallel form is a common PID implementation where the three terms (Proportional, Integral, Derivative) are calculated independently and summed.

<p align="center">
$$\large u(t) = K_p \cdot e(t) + K_i \cdot \int e(t) \, dt + K_d \cdot \frac{de(t)}{dt}$$
</p>

Where:
* `u(t)` is the controller output
* `Kp` is the proportional gain
* `Ki` is the integral gain
* `Kd` is the derivative gain
* `e(t)` is the error (`setpoint - measurement`) (or just -measurement with the derivative on the measurement implementations)

This implementation is available in `pid.hpp` (with derivative on error) and `pid_dom.hpp` (with derivative on measurement).

### Standard Form PID
The standard form (sometimes called the ideal form) includes an overall controller gain that acts on all three terms.

<p align="center">
$$\large u(t) = K \left( K_p e(t) + K_i \int e(t) \, dt - K_d \frac{d(\text{pv})}{dt} \right)$$
</p>

Where:
* `K` is the overall system gain
* `Kp` in this context allows for flexibility in choosing

This implementation is available in `pid_sf.hpp`.

^ Note on above: will likely combine the files in another pull request since with K and Kp, you can switch between parallel and standard easily by setting one or the other = 1.
