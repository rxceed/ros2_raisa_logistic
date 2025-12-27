# Just-in-Time Logistic Delivery

## Dependencies
- ROS2 Jazzy
- Tensorflow C API <br>
**Additional notes:** <br>
reeman_api_service package depends on nlohmann/json library and libCURL. Try installing both library (if not installed yet) if build problems arise because of reeman_api_service

## Installation
- Clone this repository
```bash
git clone https://github.com/rxceed/ros2_raisa_logistic.git
```
- Go to the root directory of the workspace
```bash
cd ./ros2_raisa_logistic
```
- Source ROS2 Jazzy environment
```bash
source /opt/ros/jazzy/setup.bash
```
- Install ROS2 dependencies using rosdep
```bash
rosdep install --from-paths src --ignore-src -r -y
```
- Install Tensorflow C API <br>
Tensorflow C for Linux can be downloaded from Tensorflow's official site: https://www.tensorflow.org/install/lang_c. After downloading, extract the **include** and **bin** folder inside **tensorflow** folder inside **activity_pattern_recognition** package.
```bash
|-- ros2_raisa_logistic
|   |-- src
|   |   |-- activity_pattern_recognition
|   |   |   |-- include
|   |   |   |-- src
|   |   |   |-- tensorflow
|   |   |   |   |-- bin <- paste the bin folder here
|   |   |   |   |-- include <- paste the include folder here
|   |   |   |
|   |   |   |-- CMakeLists.txt
|   |   |   |-- LICENSE
|   |   |   |-- package.xml
```
- Change the API host in secret.hpp inside reeman_api_service package <br>
The header file is located in *ros2_raisa_logistic/src/reeman_api_service/inlcude/reeman_api_service/secret.hpp*<br>
Change the host in the address string into the IP address of the Reeman ROS API host. Example: The address of POST_NAV_URL is originally (after repository cloning) "http://host/cmd/nav", if the IP address of the ROS API
is 10.7.101.100, then change it to "http://10.7.101.100/cmd/nav"
- Build the packages using the build script
```bash
bash build.sh
```

## Running the program
- Go to the workspace root directory
- Run the launch scripts in the following order:
```bash
# Run first script
bash 1-run_services.bash
#Run second script
bash 2-run_publishers.bash
#Run third script
bash 3-run_navigations.bash
```
Do note that the run scripts ***must be*** ran in that order, or else the required services or topics may not be available nor published.<br>
All run bash scripts already inlcudes sourcing ROS2 Jazzy environment in it's script, so it should be safe to use even if the environment hasn't been sourced before.


