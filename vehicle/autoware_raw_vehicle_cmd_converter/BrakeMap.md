# BrakeMap
This class contains all the information about the brake map. It can provide the appropriate brake command by specifying the velocity and acceleration values, achieving this through the use of a lookup table.
## exp
```
Brake data: (vel, brake -> acc)

      0.0,   10.0,  20.0 (vel)

0,   -1.0,  -11.0, -21.0

0.5, -2.0,  -22.0, -42.0

1.0, -3.0,  -33.0, -46.0

(brake)
```
## getBrake
```cpp
bool BrakeMap::getBrake(const double acc, const double vel, double & brake)
{
  // create vec
  std::vector<double> interpolated_acc_vec;

  // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
  for (const std::vector<double> & accelerations : brake_map_) {
    interpolated_acc_vec.push_back(
      autoware::interpolation::lerp(vel_index_, accelerations, clamped_vel));
  }

  // calculate brake
  // When the desired acceleration is smaller than the brake area, return max brake on the map
  // When the desired acceleration is greater than the brake area, return min brake on the map
  if (acc < interpolated_acc_vec.back()) {
    brake = brake_index_.back();
    return true;
  }
  if (interpolated_acc_vec.front() < acc) {
    brake = brake_index_.front();
    return true;
  }
  // reverse vec to make lerp working correctly
  std::reverse(std::begin(interpolated_acc_vec), std::end(interpolated_acc_vec));
  brake = autoware::interpolation::lerp(interpolated_acc_vec, brake_index_rev_, acc);

  return true;
}
```