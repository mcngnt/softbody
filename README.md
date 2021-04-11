# softbody
 Softbody simulation using a spring model.

## How does it work ?
This simulation use the euler integration for the moving each dot of the softbody. The springs in the soft body use the Hooke's law for applying foorces. Finally the convex shapes collisions are handled using a raycasting algorithm. The simulation also provides a way to create new shapes.

![](res/demo.gif)

## Controls
- Space : Pause the simulation
- Up/Down : Chnage the simulation precision
- R : restart the simulation
- Left click : move the softbody toward the mouse
- Right click : add a new vertex to the current shape
- T : add the current shape to the simulation
- Middle click : delete the shape closest to the mouse
