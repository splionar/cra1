# Instructions to reproduce results

### 1. Clone this repository and go to its directory
```bash
git clone https://github.com/splionar/cra1.git
cd cra1
```
### 2. Build docker image in Duckiebot
```bash
dts devel build -f --arch arm32v7 -H [ROBOT_NAME].local 
```

### 3. Run docker image in Duckiebot with the following options
```bash
docker -H [ROBOT_NAME].local run -it -e MAP_FILE=[MAP_FILENAME] -v /data/config/calibrations/:/code/catkin_ws/src/cra1/calibrations/ --rm --net=host --privileged duckietown/cra1:v1-arm32v7
```
Choice of MAP_FILENAME: [hud, calibration_pattern, lane, intersection_4way]. Default is set to lane.
