# Wave Move

## Prereqs

Image publisher for testing (optional)

```
sudo apt install ros-kinetic-image-publisher
 ```

## Running

With python 2 start the controller
```
python ai_controller.py
```
An existing node should be publishing image data to a topic but if you want to simulate this use the included dummy launch file

```
roslaunch camera.launch
```

or run directly from image_publisher

```
rosrun image_publisher image_publisher /home/T1000/wave_move/images/0.jpg
```

## Topics

Publishes on
```
/mobile_base_controller/cmd_vel
```
Subscribes to
```
/image_publisher/image_raw
```
and optionally for dead man switch
```
/vesc/ai
```
