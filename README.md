## Planning
### Path generation
The global raceline planner consists of a custom-built minimum curvature optimization problem. We assume that we have some original set of waypoints, i.e., waypoints along the centerline or manually created waypoints. Let's call the $i\text{th}$ waypoint $p_i$. In order to change the path's shape, we shift the position of $p_i$ along the vector normal to the path:

$$
s_i = p_i + \alpha_i n_i,
$$

where $s_i$ is the shifted position of the waypoint, $\alpha_i$ is the shift factor (a decision variable) and $n_i$ is the unit normal vector at the $i\text{th}$ waypoint.

We now parameterize $N$ splines between each shifted point. 


### Velocity
