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

### Velocity
