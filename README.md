![PathTracker](https://user-images.githubusercontent.com/14313049/187714439-a0fe8056-106f-4645-9821-fbf6778bcde1.png)


# VRC path tracking simulator

modified from [arimb/PurePursuit](https://github.com/arimb/PurePursuit) to be compatible with VEX spin up competition.


## Additions

- Added VEX spinup field
- Used Catmull rom spline curve instead of smoothing
- Used to-scale units (length is in cm)
- Allowed overlapped paths
- Added ramsete controller
- Added simple move and rotate function
- Added strategy planning system
- Added get coordinate functionality

## Functionalities
<p align="center">
<img src="https://user-images.githubusercontent.com/14313049/187715396-4e303fdd-b57b-4b49-b3ef-799f9d6185fa.png" width="500" height="500"/>
</p>



Allows the user to code the robot to run multiple paths with different tracking algorithms and combine into one "strategy." The strategies generated would be stored in the ```strats/``` directory, and simulated with the strategy maker. 

#### Structures of a strategy folder

- actions.csv: A csv file that programs the order and the way each path is followed.
​For example, RAMSETE,1,0 means to use RAMSETE to follow path 1 (the third value has no meaning).
- ​control_points: A folder that contains user inputted control points of each path, whereas the paths would be ordered numerically.
- paths: A folder that contains cutmull rom generated paths, which would be numerically ordered.
- coordinates: A folder containing coordinates of control points in percentage format (0,0 is the top left corder, and 100,100 is the bottom right corner) for the convenient use of robot programs.
​
​
#### Paths

- Yellow straight lines indicates that the robot would follow the path using simply drive forward and turn.
- Megenta lines indicates that the robot would follow the path with pure pursuit.
- Blue lines indicates that the robot would follow the path with ramsete.

### Single simulations

#### PathGenerator

- Because it used Catmull rom spline curve, the first and the last points would not count in the generated path. (You can set the first point to be the same location as the second point, and the last point to be the same location as its previous point.) 
- Press enter key to confirm path, and other keys to re-plan the path
- Most functions are largely identicall to [arimb/PurePursuit](https://github.com/arimb/PurePursuit)

#### Simulator

- Simultes pure pursuit and ramsete algorithm
- Customized lookahead search algorithm to allow overlapped paths
- Record the scene and export to png and gif

## How to use

### Create strategy

- Click on the field image to insert control points
    - SIMPLE:
        - the path starts at the first control point, ends at the previous point of the last point. The last point determines the direction of the facing.
    - PURE_PURSUIT and RAMSETE:
        - the path starts at the second control point, but the robot would still start at the first point. The path ends at the previous point of the last point.
- Press ENTER to cutmul rom spline curve. Press ENTER again to confirm, or else press any buton except ESC to re-draw.
- Select how to run the path through the selection window
- Press BACKSPACE to delete the last drawn path
- press ESC to stop drawing

The strategy would be stored in strats/[NAME].

### Run strategy

- Insert strategy name (folder name of strats/[NAME])
- Press ESC to stop simulation

### Find coordinates

- The box appeared on the cursor indicates the robot's size, and the circle around shows all possible collision area at the point.
- Click on the field image to show the coordinates on the specific point. The coordinates would be automatically copied to the clipboard.
