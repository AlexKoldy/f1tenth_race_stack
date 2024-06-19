## Planning
### Path generation
The global raceline planner consists of a custom-built minimum curvature optimization problem. We assume that we have some original set of waypoints, i.e., waypoints along the centerline or manually created waypoints. Let's call the $i\text{th}$ waypoint $p_i$. In order to change the path's shape, we shift the position of $p_i$ along the vector normal to the path:

$$
s_i = p_i + \alpha_i n_i,
$$

where $s_i$ is the shifted position of the waypoint, $\alpha_i$ is the shift factor (a decision variable) and $n_i$ is the unit normal vector at the $i\text{th}$ waypoint. We constrain $\alpha_i$ between some tuneable $\alpha_{min}$ and $\alpha_{max}$.

We now parameterize $N$ splines between each shifted point. We define the $i\text{th}$ spline as a set of cubic polynomials parameterized by $t \in [0, 1]$:

$$
x_i(t) = a_{i, x}t^3 + b_{i, x}t^2 + c_{i, x}t + d_{i, x}
$$

$$
y_i(t) = a_{i, y}t^3 + b_{i, y}t^2 + c_{i, y}t + d_{i, y}
$$

We apply continuity constraints, i.e.,

$$
x_i(0) = s_{i, x}
$$

$$
y_i(0) = s_{i, y}
$$

$$
x_i(1) = s_{i+1, x}
$$

$$
y_i(1) = s_{i+1, y}
$$

This implies that $x_i(1) = x_{i+1}(0)$ and $y_i(1) = y_{i+1}(0)$. Finally, we consider the smoothness constraints, i.e., 

$$
\dot{x}_i(1) = \dot{x}\_{i+1}(0)
$$

$$
\dot{y}_i(1) = \dot{y}\_{i+1}(0)
$$

$$
\ddot{x}_i(1) = \ddot{x}\_{i+1}(0)
$$

$$
\ddot{y}_i(1) = \ddot{y}\_{i+1}(0)
$$

These constraints can be organized into a linear system of equations $A\symbf{x} = b$, where $\symbf{x}$ is the vector of decision variables. 

Finally, we can set up the minimum curvature cost function. The curvature at the $i\text{th}$ shifted waypoint is approximated as

$$
\kappa_i = \frac{\dot{x}_i(0)\ddot{y}_i(0) - \dot{y}_i(0)\ddot{x}_i(0)}{\left(\dot{x}(0)^2 + \dot{y}(0)^2\right)^\frac{2}{3}}
$$

The optimization problem is therefore

$$
\begin{align} 
\min_{\alpha, a, b, c, d} \quad & \sum_{i=1}^N \kappa_i^2\\ 
\textrm{s.t.} \quad & A\symbf{x}=b\\
  & \alpha_{min} \leq \alpha \leq \alpha_{max}   \\
\end{align}
$$

Due to approximation issues, in practice we choose a relatively small range for $\alpha$, and reoptimize $k$ times. The figure below represents three reoptimizations, with the red line being the final raceline.
![min_kappa](https://github.com/AlexKoldy/f1tenth_race_stack/assets/52255127/c15fd79f-dc7f-414d-8b52-c7586e15e75b)


### Velocity profiling
TODO


## Control
### Nonlinear Model Predictive Control
#### Dynamic Model
Since the vehicle moves in $\mathbb{R}^2$, we chose to use a variation of the single-track dynamic model described [here](https://www.ivanshaoportfolio.com/project-page/mpc-drifting-parking-simulation). The figure below showcases the system model with $\beta$ being the slip angle at the vehicles center of gravity. 

![image](https://github.com/AlexKoldy/f1tenth_race_stack/assets/52255127/8fa369ea-25f1-42c8-a9b4-f1a151e5132a)


We define the vehicle's state as the concatenation of it's pose and it's pose derivative:

$$
\symbf{x} = \begin{bmatrix}
    \symbf{q}^\top & \dot{\symbf{q}}^\top
\end{bmatrix}^\top = \begin{bmatrix}
    p\_x & p\_y & \theta & v\_x & v\_y & \omega
\end{bmatrix}^\top
$$

where $p\_x$ and $p\_y$ represent the global position of the vehicle in $\mathbb{R}^2$, $\theta$ represents the vehicle heading (yaw), $v\_x$ and $v\_y$ represent the longitudinal and lateral velocity of the vehicle (vehicle frame), respectively, and $\omega$ represents the yaw rate of the vehicle. We define our input vector as

$$
\symbf{u} = \begin{bmatrix}
    \delta & a\_x
\end{bmatrix}^\top
$$

where $\delta$ is the steering angle and $a\_x$ is the longitudinal acceleration. We chose to use the steering angle as an input rather than the steering rate to reduce the state-space from $\mathbb{R}^7$ to $\mathbb{R}^6$. In order to define the vehicle dynamics, we calculate the slip angle of each tire similarly:

$$
\begin{align*}
    \alpha\_{f} &= \arctan\left(\frac{v\_y + \omega \ell\_{f}}{v\_x}\right) - \delta\\
    \alpha\_{r} &= \arctan\left(\frac{v\_y + \omega \ell\_{r}}{v\_x}\right)
\end{align*}
$$

Moreover, we calculate the load on each axle $F\_z$ as

$$
\begin{align*}
    F\_{z\_f} &= \frac{mg\ell\_r - ma\_xh\_{cog}}{\ell\_f + \ell\_r}\\
     F\_{z\_r} &= \frac{mg\ell\_f - ma\_xh\_{cog}}{\ell\_f + \ell\_r}
\end{align*}
$$

Using these parameters, we can compute the lateral force on each set of tires via the Pacejka Magic Formula, i.e.,

$$
F\_{y\_i} = \tilde{D}\_i \sin \Bigl( C\_i\arctan \bigl( B\_i\alpha\_i - E\_i(B\_i\alpha\_i - \arctan (B\_i\alpha\_i ) \bigr) \Bigr), 
$$

where, $i \in \{f, r\}$ and $\tilde{D} = \mu F\_z D$ and

$$
\begin{align*}
     B &:= \text{Stiffness factor}, \\
     C &:= \text{Shape factor}, \\
     \tilde{D} &:= \text{Peak value}, \\
     E &:= \text{Curvature factor}.
\end{align*}
$$

The system dynamics are therefore defined as

$$
\dot{\symbf{x}} = f(\symbf{x}, \symbf{u}) = \begin{bmatrix}
    v\_x \cos(\theta) - v\_y \sin(\theta) \\
    v\_x \sin(\theta) + v\_y \cos(\theta) \\
    \omega \\
    a\_x - \frac{F\_{y\_{f}}}{m}\sin(\delta) + v\_y \omega \\
    \frac{1}{m}(F\_{y\_{r}}+F\_{y\_{f}}\cos(\delta)-mv\_x\omega \\
    \frac{1}{I\_z}(F\_{y\_{f}}\ell\_{f}\cos{\delta}-F\_{y\_{r}}\ell\_{r}
\end{bmatrix}.
$$

We roll out and discretize our dynamics via fourth-order Runge-Kutta integration (with a refinement factor of $4$) such that

$$
\symbf{x}\_{k + 1} = \symbf{f}(\symbf{x}\_k, \symbf{u}\_k, \Delta t).
$$

#### Nonlinear Programming Problem

We define our decision variables as a set of states and inputs

$$
\mathcal{X} = \\{\symbf{x}\_0, \symbf{u}\_0, \symbf{x}\_1, \symbf{u}\_1, \cdots, \symbf{x}\_{N-1},\symbf{u}\_{N-1},\symbf{x}_{N}\\}
$$


Using this, we set up our nonlinear programming problem as

$$
\begin{align}
\min_{\symbf{x}, \symbf{u} \, \in \, \mathcal{X}} & \quad  \sum_{k=0}^N (\symbf{q}\_k-\symbf{q}\_{goal})^\top Q_{track} (\symbf{q}\_k-\symbf{q}\_{goal}) \\ 
&+ \sum\_{k=0}^{N-1}\symbf{u}\_k^\top R\_{input} \symbf{u}\_k \\ 
&+ \sum\_{k=0}^{N-2}(\symbf{u}\_{k+1} - \symbf{u}\_k)^\top R_{rate} (\symbf{u}\_{k+1} - \symbf{u}\_k) \\ 
&+ Q\_{drift}v\_{y\_k}^2\\
\textrm{s.t.} \quad & \symbf{x}\_{k + 1} = \symbf{f}(\symbf{x}\_k, \symbf{u}\_k, \Delta t), \\, \forall k \in \{0, \cdots, N-1\}\\
&\symbf{x}\_0 = \hat{\symbf{x}} \\
&\symbf{x}\_{min} \leq \symbf{x}\_k \leq  \symbf{x}\_{max}, \, \forall k \in \{1, \cdots, N\} \\
&\symbf{u}\_{min} \leq \symbf{u}\_k \leq  \symbf{u}\_{max}, \, \forall k \in \{1, \cdots, N\}  
\end{align}
$$

where the vehicle's estimated state $\hat{\symbf{x}}$ and goal pose $\symbf{q}_{goal}$ are passed through as parameters

$$
\mathcal{P} = \\{\hat{\symbf{x}}, \symbf{q}_{goal}\\}
$$

The cost includes: a weight on tracking the $x$, $y$ and yaw of the goal pose, where $\theta_{goal}$ is defined as the angle from the current state to the goal point, an experimental cost/reward on drifting, a weight on the inputs and a rate on the input rates for smooth control.

Due to the non-convex nature of our MPC, we use CasADi and IPOPT to solve the nonlinear programming problem. To increase solve frequency, we use CasADi functions for $f(\symbf{x}, \symbf{u})$ and $\symbf{f}(\symbf{x}, \symbf{u}, \Delta t)$. We attempt to make use of CasADi's multi-threading mapping for the function associated with $\symbf{f}$. Moreover, we build CasADi functions for each of the costs. Finally, we generate and compile C code for the entire nonlinear programming problem including the IPOPT solver. 

At each iteration, we pass an initial decision variable guess $\mathcal{X}\_{guess}$, the MPC parameters $\mathcal{P}$ and the constraints on the decision variables $\mathcal{X}\_{min}$ and $\mathcal{X}\_{max}$. The initial guess is warm-started with the solution to the previous iteration of the MPC. Once the optimal control problem is solved, $\symbf{u}_0^\star$ is applied to the vehicle.


