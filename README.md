# VEX spin up pure pursuit simulator

modified from [arimb/PurePursuit](https://github.com/arimb/PurePursuit) to be compatible with VEX spin up competition.

![image](https://github.com/acezxn/PathTracker/blob/main/images/movie.gif)

## Additions

- Added VEX spinup field
- Used Catmull rom spline curve instead of smoothing
- Used to-scale units (length is in cm)
- Allowed overlapped paths

### Path generator

- Because it used Catmull rom spline curve, the first and the last points would not count in the generated path. (You can set the first point to be the same location as the second point, and the last point to be the same location as its previous point.) 
- Press enter key to confirm path, and other keys to re-plan the path
- Most functions are largely identicall to [arimb/PurePursuit](https://github.com/arimb/PurePursuit)

### Robot simulator

- Simultes pure pursuit algorithm
- Customized lookahead search algorithm to allow overlapped paths
- Record the scene and export to png and gif
