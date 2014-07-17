Corridor Map
============
*work in progress*...
![square](/example/square.png?raw=true)

## Construction
Voronoi diagram is rendered on GPU.
OpenCL kernel detects edges and vertices of the medial axis:

![polygons voronoi](/example/polys_1_voronoi.png?raw=true)

CPU traces connections and assembles the final graph:

![polygons map](/example/polys_1.png?raw=true)

This graph can be used to extract corridor, represented as a set of disks:
![polygons corridor](/example/polys_1_corridor.png?raw=true)

Shortest path in the corridor:
![polygons path](/example/polys_1_path.png?raw=true)

## Reference
Roland Geraerts. Planning Short Paths with Clearance using Explicit Corridors.
In IEEE International Conference on Robotics and Automation, pp. 1997-2004, 2010.
