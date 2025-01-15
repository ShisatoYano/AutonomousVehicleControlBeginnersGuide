# User's comments

## Pure Persuit Algorithm inspired by Shisato Yano's Python Example
[Link to demo video](https://vimeo.com/1046921330)  

This is an AGV path tracking demo by [Philippe Piatkiewitz](https://nl.linkedin.com/in/philippepiatkiewitz) at [Vectioneer](https://vectioneer.com).  

He implemented this by using [Pure pursuit sample code](/src/components/control/pure_pursuit/pure_pursuit_controller.py) with the following modifications.  

* The selection of the next waypoint was improved and avoids finding waypoints that already have been passed.
* Added the option that the AGV automatically stops at the last waypoint and then also re-orients itself in the desired direction.
* Allow the AGV to drive backwards if that path is shorter.
* Lastly adjust the speed in the corners based on the desired steering angle.