# cwru_opencv_common

These libraries are meant to fill in some of the functionality that is missing from the standard opencv libraries.
In particular projective geometry algorithms and transformations are included.

'''Note that some features may be common with the ViSP visual servoing library. If this is the case, ViSP may be included instead but ViSP already reinvents many parts of opencv (and other libraries) so it is very bloated.'''

## Example usage

The file projective_geometry.h includes camera projection and camera deprojection that also generates the appropriate jacobians of the object points in image space.

The file opencv_geometry_3d.h creates the 3d namespace. The purpose is to project common geometric object from 3d space into the camera image space (RP^2). Some example shapes include cylinders and spheres. (Other shapes can be implemented here). This file implicitly depends on projective_geometry.h

## Running tests/demos


    
