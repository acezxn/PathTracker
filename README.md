![PathTracker](https://user-images.githubusercontent.com/14313049/187714439-a0fe8056-106f-4645-9821-fbf6778bcde1.png)


# VEX spin up path tracking simulator

modified from [arimb/PurePursuit](https://github.com/arimb/PurePursuit) to be compatible with VEX spin up competition.


## Additions

- Added VEX spinup field
- Used Catmull rom spline curve instead of smoothing
- Used to-scale units (length is in cm)
- Allowed overlapped paths
- Added ramsete controller

## Functionalities
<p align="center">
<img src="https://user-images.githubusercontent.com/14313049/187715396-4e303fdd-b57b-4b49-b3ef-799f9d6185fa.png" width="500" height="500"/>
</p>

### Path generator

- Because it used Catmull rom spline curve, the first and the last points would not count in the generated path. (You can set the first point to be the same location as the second point, and the last point to be the same location as its previous point.) 
- Press enter key to confirm path, and other keys to re-plan the path
- Most functions are largely identicall to [arimb/PurePursuit](https://github.com/arimb/PurePursuit)

### Robot simulator

- Simultes pure pursuit and ramsete algorithm
- Customized lookahead search algorithm to allow overlapped paths
- Record the scene and export to png and gif

### Strategy maker

Extends the path generator and allow multiple paths with different tracking algorithms to be planned and combined into one "strategy." The strategies generated would be stored in the strats directory, and simulated with the strategy maker. 

**Structures of a strategy folder**

- actions.csv: A csv file that programs the order and the way each path is followed.
​For example, RAMSETE,1,0 means to use RAMSETE to follow path 1 (the third value has no meaning).
- ​control_points: A folder that contains user inputted control points of each path, whereas the paths would be ordered numerically.
- paths: A folder that contains cutmull rom generated paths, which would be numerically ordered.
​
**paths**

- Yellow straight lines indicates that the robot would follow the path using simply drive forward and turn.
- Megenta lines indicates that the robot would follow the path with pure pursuit.
- Blue lines indicates that the robot would follow the path with ramsete.