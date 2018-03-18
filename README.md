# Model Predictive Control (MPC) Project

# [Rubric](https://review.udacity.com/#!/rubrics/896/view) points

## Compilation

### Your code should compile

The code compiles without errors or warnings using the following command line:

    cmake .. && make

## Implementation

### The Model

The model state contains:

| Property       | Description                                      |
|:---------------|:-------------------------------------------------|
| ```px```       | X-position of the vehicle (in forware direction) |
| ```py```       | Y-position of the vehicle (in lateral direction) |
| ```psi```      | Orientation of the vehicle                       |
| ```v```        | Velocity of the vehicle                          |
| ```cte```      | Cross-track error                                |
| ```epsi```     | Orientation error                                |

The actuators of the vehicle are:

| Property       | Description                                      |
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

First, the waypoints provided by the simulator are transformed to vehicle coordinates (see [```main.cpp```, lines 95-104](./src/main.cpp#L95-L104)):

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

Then, a 3rd-degree polynomial is fitted to the transformed waypoints using the ```polyfit()``` function (see [```main.cpp```, line 109](./src/main.cpp#L109)):

    const auto coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);

Finally, the polynomial coefficients are used to calculate ```cte``` (cross-track error) and ```epsi``` (orientation error) (see [```main.cpp```, lines 118-119](./src/main.cpp#L118-L119)):

    const double cte0 = polyeval(coeffs, 0);
    const double epsi0 = -atan(coeffs[1]);

### Model Predictive Control with Latency

In order to simulate a system that is closer to real-life, latency of 100 milliseconds between a cycle of the MPC controller and the actual actuation was artifically introduced.

    const double latency = 0.1;  // 100 ms

The calculated initial state at ```t=0``` (see above) and the corresponding latency are used to predict the state at ```t=latency``` (see [```main.cpp```, lines 132-137](./src/main.cpp#L132-L137)):

    const double xLat = x0 + v * cos(psi0) * latency;
    const double yLat = y0 + v * sin(psi0) * latency;
    const double cteLat = cte0 + v * sin(epsi0) * latency;
    const double epsiLat = epsi0 + v * delta * latency / Lf;
    const double psiLat = psi0 + v * delta * latency / Lf;
    const double vLat = v + a * latency;
