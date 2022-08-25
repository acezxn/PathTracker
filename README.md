# VEX spin up pure pursuit simulator

modified from [arimb/PurePursuit](https://github.com/arimb/PurePursuit) to be compatible with VEX spin up competition.

![image](https://github.com/acezxn/PathTracker/blob/main/images/movie.gif)

## Additions

- Added VEX spinup field
- Used Catmull rom spline curve instead of smoothing
- Used to-scale units (length is in cm)
- Allowed overlapped paths

## Modifications

- For paths, (0,0) represents the left top corner (going right increases x, and going down increases y)
