# Path-Planning-Project

## Goal
This project aims at safely navigating around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
We are provided the car's localization and sensor fusion data along with a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars and drive inside the marked road lanes except when going from one lane to another. The car should be able to make one complete loop around the 6946m highway. The car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Model for Path Generation
Path generation model consists of two parts:
* Deciding lane to drive in
* Creating a trajectory

### Deciding Lane to Drive in

If no vehicle is present in current driving lane, the car accelerates to maximum velocity, which is close to 50MPH. The acceleration is kept low and constant to minimise jerk. If a vehicle is detected within 30m range in front of the car, `setTargetLane` function is called to change the lane if possible, otherwise the car slows down in the current lane itself.

Following is the code for the same:
```c

            if(too_close){
                cout<< "Too Close\n";
                // slow down the vehicle
                ref_v -= ref_a;
                //set the lane where vehicle must move, if same lane then only decelerate
                setTargetLane(sensor_fusion, car_s, prev_size);
            } else {
                // accelerate to maximum acceptable velocity if no obstacle is present
                ref_v += (ref_v > max_v ? 0.0 : ref_a);
            }

```
The lane is changed only if no other car is present 30m ahead and 15m behind in the adjacent lane. Also preference is given to a lane where there is no other vehicle present in case when lanes on both sides are fine for lane change. 

Following is the definition of `setTargetLane` function which is responsible for lane change:
```c
void setTargetLane(vector<vector<double>> sensor_fusion, double car_s, int prev_size){
    // choose one of the three manuevers, change lane left, change_lane right, move straight with reduced speed
    
    bool LEFT = true;
    bool RIGHT = true;
    bool cars_in_left_lane = false;
    bool cars_in_right_lane = false;
    int ci = 0, li = 3;
    if(lane == 0) {
        LEFT = false;
        li = 2;
    }
    if(lane == 2) {
        RIGHT = false;
        ci = 1;
    }
                
    for(; ci < li; ci++){
        for(int j = 0; j < sensor_fusion.size(); j++){
            //car in j th lane
            float d = sensor_fusion[j][6];
            if(d < (2 + 4*ci + 2) && d > (2 + 4*ci-2)){
                double vx = sensor_fusion[j][3];
                double vy = sensor_fusion[j][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[j][5];
                //if using previous points can project s value out
                check_car_s += (double)prev_size * 0.02 * check_speed;
                //check s values greater that current and s gap
                if((check_car_s >= car_s) && (check_car_s - car_s < 30) && (check_speed < ref_v)){
                    if(ci < lane) LEFT = false;
                    if(ci > lane) RIGHT = false;
                } else {
                    if(ci < lane) cars_in_left_lane = true;
                    if(ci > lane) cars_in_right_lane = true;
                }
                if((check_car_s <= car_s) && (car_s - check_car_s < 15)){
                    if(ci < lane) LEFT = false;
                    if(ci > lane) RIGHT = false;
                }
            }
        }
    }
    if(LEFT) lane--;
    if(RIGHT) lane++;
    if( LEFT && RIGHT ){
        if(cars_in_left_lane)
            lane++;
                    
        if(cars_in_right_lane)
            lane--;
    }
}
```

### Creating a trajectory


