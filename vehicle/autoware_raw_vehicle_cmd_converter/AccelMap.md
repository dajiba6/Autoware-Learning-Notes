## AccelMap
This class contains all the information about the acceleration map. It can provide the appropriate throttle command by specifying the velocity and acceleration values, achieving this through the use of a lookup table.
### getThrottle
- Obtain the matching table between throttle and acceleration when velocity confirmed.
- Using the matching table to find the right throttle value when acceleration confirmed.
```cpp
bool AccelMap::getThrottle(const double acc, double vel, double & throttle) const
  // construct vec
  std::vector<double> interpolated_acc_vec;

  // Iterate through all the rows, interpolate values based on velocity, and store these values in the vector.
  // We now have a matching table between throttle and acceleration with velocity confirmed.
  //? Why do we need to perform a lookup for each line?
  //? Why not find the index in the first line and then use the index for all subsequent lookups?
  for (const std::vector<double> & accelerations : accel_map_) {
    interpolated_acc_vec.push_back(
      autoware::interpolation::lerp(vel_index_, accelerations, clamped_vel))
  }

  // If the target acceleration is less than the minimum value in the matching table, return false.
  if (acc < interpolated_acc_vec.front()) {
    return false;
  }
  
  // If the value exceeds the maximum, return the maximum throttle value from throttle_index_ (a vector obtained from the readAccelMapFromCSV function, which contains all throttle values in the map).
  if (interpolated_acc_vec.back() < acc) {
    throttle = throttle_index_.back();
    return true;
  }
  
  // If it is in bewteen, get the impolated value.
  throttle = autoware::interpolation::lerp(interpolated_acc_vec, 
  throttle_index_, acc);
  return true;
```
