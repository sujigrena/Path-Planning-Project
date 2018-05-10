# Model Documentation

Generating three key future points of the car, which is done by using the spline library 

```
vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

We fill in the gaps using spline library like below:

```
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
Finally to fill in the paths points using spline, we do the following:
// Fill up the rest of our path planner after filling it with previous points, here we
// will always output 50 points.
for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

    double N = (target_dist/(0.02*ref_vel/2.24)); // 0.5 mph
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after rotating earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;
}
```

Finally the logic to change lanes if our car is not too close to cars in the desired lane can be done as follows:

```
if(too_close){
	ref_vel -= 0.224;
	if (( lane == 1) && !left_too_close){
	lane = 0;
}
else if (( lane == 1) && !right_too_close) {
	lane = 2;
}
else if (( lane == 0) && !right_too_close){
	lane = 1;
}
else if (( lane == 2) && !left_too_close){
	lane = 1;
}
}
else if (ref_vel < 49.5){
           ref_vel += 0.224;
}
```
