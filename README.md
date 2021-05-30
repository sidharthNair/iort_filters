
# IoRT Filters

`iort_filters` provides two example `insitu` extensions that use `iort_lib` to overlay IoT data onto an image stream as part of a demonstration of the IoT-Robotics Integration project. Other major components of the project are available at the links below:

1. [`iort_endpoint`](https://github.com/PaperFanz/iort_endpoint) - open source firmware for an ESP32 microcontroller to serve as an AWS IoT Core thing
2. [`insitu`](https://github.com/PaperFanz/insitu) - open source situational awareness package with extension support
3. [`iort_lib`](https://github.com/PaperFanz/iort_lib) - a ROS library that wraps calls to a web API for fetching IoT data from AWS Timestream
4. [`iort_filters`](https://github.com/PaperFanz/iort_filters) - this repository

You should go through these four links in order and follow the setup isntructions in `README.md`.

## Cloning and Building

Simply clone the repository into your catkin workspace and run `catkin build`:

```sh
cd ~/catkin_ws/src
git clone https://github.com/PaperFanz/iort_filters
catkin build iort_filters
```
**NOTE:** if you chose to clone `iort_filters` to a different catkin workspace than `insitu` and `iort_lib` then you must install those projects' exported headers to your system include directory:

```sh
# in the insitu/iort_lib workspace
sudo catkin install insitu
sudo catkin install iort_lib
```

## Testing

You should now be able to discover the filters `IoRT_Plot` and `QRFilter` in the `Add Filter` modal dialogue in `insitu`.
