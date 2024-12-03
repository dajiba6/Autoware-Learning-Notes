# Command
Enter dev container
- [x] What is the difference between devel and normal?
  ans:  
  1. use different image.
  2. devel mode will set current folder as /workspace.
```shell
./docker/run.sh --devel
```
build workspace
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
//--symlink-install: use symlink instead of copying files 
//--cmake-args: add cmake args
//-DCMAKE_BUILD_TYPE=Release: set build type to release
```
create package
```bash
ros2 pkg create --build-type ament_cmake my_package
```
build pkg
```bash
colcon build --packages-select my_package
```
