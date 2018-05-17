# Path-Planning-Project


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
