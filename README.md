# TvContact

TvContact is a deformer that emulates a collision between two meshes. 
The deformer caches the previous evaluation and tests the collision with this mesh first instead of testing with the original mesh. This allows to compute more stable intersections, especially when the collider mesh is almost entirely included in the collided geometry. The whole deformation is then smoothed by applying a laplacian smooth on the deltas.

A small demo of the deformer in action :
![](demos/tvcontact_01.gif)

Next step :
- [ ] Volume preservation
