Corridor Map Library
====================
![square](/example/square.png?raw=true)

work in progress...

## Building Library and Examples
Corridor Map project uses premake5 for builds. *At the moment only Windows build is supported*

To generate Visual Studio solution:  ```premake5 vs2013```

premake5 executable can be found in the root of this repository.

## Construction Process Illustrated
Voronoi diagram is rendered on GPU.
OpenCL kernel detects edges and vertices of the medial axis:

![polygons voronoi](/example/polys_1_voronoi.png?raw=true)

CPU traces connections and assembles the final graph:

![polygons map](/example/polys_1.png?raw=true)

This graph can be used to extract corridor, represented as a set of disks:

![polygons corridor](/example/polys_1_corridor.png?raw=true)

Shortest path in the corridor, fast version for triangulated corridor:

![polygons path](/example/polys_1_path.png?raw=true)

Continuous shortest path (arcs & line segments), slower but doesn't require triangulation:

![polygons path continuous](/example/polys_1_continuous_path.png)

## Reference
Roland Geraerts. Planning Short Paths with Clearance using Explicit Corridors.
In IEEE International Conference on Robotics and Automation, pp. 1997-2004, 2010.
[link](http://www.staff.science.uu.nl/~gerae101/motion_planning/ecm.html)
