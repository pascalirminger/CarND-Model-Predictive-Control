# Model Predictive Control (MPC) Project

# [Rubric](https://review.udacity.com/#!/rubrics/896/view) points

## Compilation

### Your code should compile

The code compiles without errors or warnings using the following command line:

    cmake .. && make

## Implementation

### The Model

The model state contains:

|                |                                                  |
|:---------------|:-------------------------------------------------|
| ```px```       | X-position of the vehicle (in forware direction) |
| ```py```       | Y-position of the vehicle (in lateral direction) |
| ```psi```      | Orientation of the vehicle                       |
| ```v```        | Velocity of the vehicle                          |
| ```cte```      | Cross-track error                                |
| ```epsi```     | Orientation error                                |

The actuators of the vehicle are:

|                |                                                  |
|:---------------|:-------------------------------------------------|
| ```deltaPsi``` | Steering angle                                   |
| ```a```        | Acceleration                                     |

The update equations used to predict future states are:

    px[t+1]  = px[t] + v[t] * cos(psi[t]) * dt
    py[t+1]  = py[t] + v[t] * sin(psi[t]) * dt
    psi[t+1] = psi[t] + v[t] / Lf * deltaPsi * dt
    v[t+1]   = v[t] + a * dt
    psi[t+1] = f(px[t]) - py[t] + v[t] * sin(epsi[t]) * dt
    v[t+1]   = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

Where ```dt``` is the timestep between predictions and ```Lf``` is the distance between the front wheel and the center of gravity of the vehicle, which determines its turning radius.

### Timestep Length and Elapsed Duration (N & dt)

```N``` (number of points) and ```dt``` (time interval) define the prediction horizon. The number of points immediately impacts the controllers performance. With too many points the controller starts to run slower. After playing around with higher values, I decided to stay with ```N=10``` and ```dt=0.1``` (100 ms).

### Polynomial Fitting and MPC Preprocessing

First, the waypoints provided by the simulator are transformed to vehicle coordinates (see [```main.cpp, lines 96-105```](./src/main.cpp#L96-105)):

    const size_t n_waypoints = ptsx.size();
    const double minus_psi = 0.0 - psi;
    auto ptsx_transformed = Eigen::VectorXd(n_waypoints);
    auto ptsy_transformed = Eigen::VectorXd(n_waypoints);
    for (int i = 0; i < n_waypoints; i++) {  // For each waypoint
        const double dX = ptsx[i] - px;
        const double dY = ptsy[i] - py;
        ptsx_transformed[i] = dX * cos(minus_psi) - dY * sin(minus_psi);
        ptsy_transformed[i] = dX * sin(minus_psi) + dY * cos(minus_psi);
    }

Then, a 3rd-degree polynomial is fitted to the transformed waypoints using the ```polyfit()``` function (see [```main.cpp, line 110```](./src/main.cpp#L110)):

    const auto coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);

Finally, the polynomial coefficients are used to calculate ```cte``` (cross-track error) and ```epsi``` (orientation error) (see [```main.cpp, lines 119-120```](./src/main.cpp#L119-120)):

    const double cte = polyeval(coeffs, 0);
    const double epsi = -atan(coeffs[1]);

### Model Predictive Control with Latency

In order to simulate a system that is closer to real-life, latency of 100 milliseconds between a cycle of the MPC controller and the actual actuation was artifically introduced. Predictably, this had multiple effects:

* It affected driving at higher speeds. It did so when the vehicle was driving towards a sharp turn: the vehicle would react too late and run off the road.
* Any existing oscillation was amplified and got even worse.

Regarding this issue, the model has been tuned to drive more conservatively.
