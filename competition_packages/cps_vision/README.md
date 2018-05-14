# cps_vision

This package is for CPS Challenge vision part, please fork this package to make further change

## Dependences:
opencv: https://opencv.org/

vision_opencv: https://github.com/ros-perception/vision_opencv.git

cv_bridge: http://wiki.ros.org/cv_bridge

image_pipeline: https://github.com/ros-perception/image_pipeline (for calibration)

## The transformation between the camera and the drone frame
![Alt text](/frames.png?raw=true)

## Run vision node:
`rosrun cps_vision cps_vision`
This node publishes two topics:
- /navigation/marker_find : Int32 (indicating if target is found (1) or not (0))
- /navigation/marker_position : Float64MultiArray (relative position (x, y) in drone's body frame)

## Run data collection node:
`rosrun cps_vision pic_collection`