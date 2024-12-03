# Google Test
[autoware unit test documentation](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/unit-testing.html)

## Build test
```bash
cd /workspace/
colcon build --packages-select my_pkg
```
## Run test
```bash
colcon test --packages-select my_pkg
```
get job-wise information of all executed tests:
```bash
colcon test-result --all
```
To print the tests' details while the tests are being run, use the --event-handlers console_cohesion+ option to print the details directly to the console:
```bash
colcon test --event-handlers console_cohesion+ --packages-select my_pkg
```