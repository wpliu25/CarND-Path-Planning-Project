# Model Documentation
Reflection
   
## Classes

1. Map
2. Car
3. Navigation
4. Path

### Map
This class stores the waypoints from data read through data/highway_map.txt. An instance is instantiated in main.cpp prior to message loop (map.h, main.cpp line 48).


### Car
This class stores the main car's uniquely identifying localization data. An instance is updated with latest x, y, s, d in frenet, and yaw main.cpp on every message loop (car.h, main.cpp line 79).

### Navigation
This class is used with sensor fusion to update the navigation plan. An instance is called to update the navigation plan by using information from sensors regarding other cars detected nearby (navigation.h, main.cpp line 99). In the UpdateNavigation function (navigation.h lines 24-108) all detected cars are processed. A 'change_lanes' flag is updated if a car directly in the main car's path requires a decrease in velocity. To safely change lanes, a gap of 30m is enforced, and a left lane change is attempted before trying right. A reference velocity 'ref_vel' is updated to adjust the velocity of the car.

### Path
This class calculates the points along the desired path. An instance is called to calculate an udpated on every message loop (path.h, main.cpp line 110). Based on current referenced car localization and desired location (output from navigation), this object uses the project's recommended spline class to compute points evenly spaced 30m, 60m, and 90m out ahead. These desired path points are then combined with previous points to create an updated path currently hardcoded to a length of 50.

## Improvements
* This is a simple model that can be improved with the integration of a more sophisticated controller. We currently change navigation based on desired location, but a better model can account for desired velocity and also adjust the path length accordingly.
* In addition, a state machine can more elegantly track the model of the vehicle. Currently this solution uses simple flags to indicate desired lane change, which carries over across telemetry events. A more sophisticated state machine can consider states that prioritize changing lanes back to the center or to be more aggresive in lane change (shorter safety gap) if we encounter the same prohibiting vehicle more than once.
* Also currently this model does not account for aggresive driving from other cars. We only slow for due to proximity to vehicles in front but we can add behavior more defensive driving that avoids or brakes if detection lane encroachment.

