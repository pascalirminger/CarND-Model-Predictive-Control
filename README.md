# Model Predictive Control (MPC) Project

# [Rubric](https://review.udacity.com/#!/rubrics/896/view) points

## Compilation

### Your code should compile

The code compiles without errors or warnings using the following command line:

    cmake .. && make

## Implementation

### The Model

**TODO: Student describes their model in detail. This includes the state, actuators and update equations.**

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


