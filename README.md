# Just-in-Time Logistic Delivery

## Dependencies
- ROS2 Jazzy
- Tensorflow C API

## Installation
- Clone this repository
```bash
git clone https://github.com/rxceed/ros2_raisa_logistic.git
```
- Go to the root directory of the workspace
```bash
cd ./ros2_raisa_logistic
```
- Install ROS2 dependencies using rosdep
```bash
rosdep install --from-paths src --ignore-src -r -y
```
- Install Tensorflow C API
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
bash 3-run_navigations
```
Do note that the run scripts ***must be*** ran in that order, or else the required services or topics may not be available nor published.



