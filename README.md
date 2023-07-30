# cloth_sim
This repo contains implementation of a cloth simulator using the mass-spring model. The cloth is represented by *n x n* mass particles on a grid and the  edges connecting these particles act as springs. There are 2 kinds of force acting on the cloth at given time; external and internal forces. External forces can be gravity, wind, collision etc while internal forces include stretch, shear, bend forces, compression etc. Each particle has 2 states, positions $x_i=[x_{i_x}, x_{i_y}, x_{i_z}]$ and velocity $v_i=[v_{i_x}, v_{i_y}, v_{i_z}]$. 

The goal is to achieve the minimal energy state of the system. If $E(x)$ is the total energy acting on the particle, then negative of the gradient of this energy points us towards the steepest decent. Hence the force on the particle is given as: $$f(x)=-\frac{\partial E(x)}{\partial x}$$ For gravitational force, assuming the gravity to act along y direction and the acceleration due to gravity *g* is roughly equal to $9.8$, then the resulting force is given as:
$$f_i(x) = -\frac{\partial E_g(x)}{\partial x_i}$$
$$=- [ \frac{\partial E_g}{\partial x_{i_{x}}}, \frac{\partial E_g}{\partial x_{i_{y}}}, \frac{\partial E_g}{\partial x_{i_{z}}}  ]  = [0,-9.8,0] $$


For each particle $i$ on the grid, the forces exerted by particle $j \in \mathcal{N}$ where $\mathcal{N}$ are the neighbourhood particles of $i$ is given as:
$$ f_i(x) = k(||x_i-x_j|| -L ) \frac{(x_i-x_j) } {||x_i-x_j||}$$

$L$ is the rest length of the spring connecting particles $i$ and $j$ , $k$ is the spring constant and  $||.||$ is the Euclidean distance. We also have a damping force acting on each particle, that helps achieve stability of the system which is given as:
$$d_i(x) = -k_d(v_i-v_j)$$ where $k_d$ is the damping coefficient. 

At each given time step $\delta t$, the update in positions $\Delta x$ and velocities $\Delta v$ is calculated as:
$$\Delta x = \delta t * v_n  $$
$$\Delta v = \delta t * (M^{-1} f(x_n,v_n))$$
Here, $f(x_n,v_n)$ is the sum of all acting forces (gravity, spring and damping)  calculated as mentioned above. Finally the positions and velocity at current time step are updated as follows:
$$x_{n+1} = x_n + \Delta x $$
$$v_{n+1} = v_n + \Delta v $$
Here is the demo of our working mass-spring system for cloth simulation:
![](output.mp4) 