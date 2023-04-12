# vector-robot

ROS wrapper and startup code for the Anki Vector robot

**Contents**

1. [Usage](#usage)
2. [Topics](#topics)
    1. [Read-only](#read-only-topics)
    2. [Write](#write-topics)
3. [Custom messages](#custom-messages)
4. [Demos](#demos)

## Usage

1. Install ROS and `rospy`
2. [Setup the Vector Linux SDK](https://developer.anki.com/vector/docs/install-linux.html)
3. Complete the Vector robot authentication process with `python3 -m anki_vector.configure`
4. Clone this repository into your Catkin workspace
5. Install additional dependencies (if needed) with `pip3 install -r requirements.txt`.
6. Install `cv_bridge` from source for Python 3 with `catkin build` to enable camera functionality. [This Stack Overflow thread](https://stackoverflow.com/questions/49221565/unable-to-use-cv-bridge-with-ros-kinetic-and-python3) provides instructions for doing so.
6. Install this package with `catkin_make install` (from the root of your workspace) or `catkin build anki_vector_ros`.
7. Run `source ~/catkin_ws/devel/setup.bash`
8. Run chmod +x <file> on anki_vector_core.py
8. Launch an instance of the core `vector_ros` node:

```
roslaunch launch/vector_core.launch [serial:=<Vector serial number>]
```

The node may also be run directly via Python if `roscore` is running:

```
python3 nodes/anki_vector_core.py [--serial <Vector serial number>]
```

You may also wish to create your own `.launch` files incorporating custom nodes. See [`hello_world.launch`](./launch/hello_world.launch) for an example.

**Notes**
1. Ensure you aren't using a VPN before connecting to Vector
2. The `serial` parameter is only required if there are multiple Vectors listed in your `.anki_vector/sdk_config.ini` file. Otherwise, the package will connect to the default Vector.

### Using the camera

By default, Vector's camera feed is turned off to conserve its battery, and there will be nothing published to the `/camera` topic. To launch the core Vector ROS node with camera enabled, run:

```
python3 nodes/anki_vector_core.py --camera
```

The `--camera` flag can also be set in the default `.launch` file:

```
roslaunch launch/vector_core.launch camera:=true
```

#### Camera troubleshooting

Even after installing `cv_bridge` from source, you may receive the following error from running the camera thread:

```
ImportError: dynamic module does not define module export function (PyInit_cv_bridge_boost)
```

In order to fix this, you'll need to recompile `tf2_ros` for Python 3. [This thread](https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/) provides steps for doing so, and remember to change `python3.6m` and `libpython3.6m.so` in the last step to match your specific Python version.

## Topics

The `vector_ros` node creates a series of ROS topics for interfacing with the Anki Vector sensors and outputs. Each topic is designated to either be read from or written to. Please note that this package is currently not feature-complete, particularly for features involving NavMaps and custom object tracking, but feature requests are always appreciated!

Some of these topics send/receive custom messages instead of built-in ROS messages. For details, see [custom message definitions below](#custom-messages).

### Read-only topics

Subscribe to these topics to access Vector's sensor readings

* `/accel`: `Vector3` reading of the robot's XYZ acceleration
* `/gyro`: `Vector3` reading of the robot's XYZ tilt
* `/battery`: Current voltage level of the robot's battery. Low battery is considered at 3.6 V or less.
* `/carry_object`: integer ID of the object the robot is currently carrying (-1 if none)
* `/head_angle`: Vector's head angle (in radians)
* `/head_tracking_object`: integer ID of the object that the head is tracking to (-1 if none)
* `/wheel_speed/left`: Vector's left wheel speed in mm/sec
* `/wheel_speed/right`: Vector's right wheel speed in mm/sec
* `/lift_height`: height of Vector's lift from the ground in mm
* `/localized_object` integer ID of the object that the robot is localized to (-1 if none)
* `/pose`: `Pose` message containing information about a robot's position and rotation with respect to an origin, its pose angle, and pitch (in radians)
* `/proximity`: `Proximity` message containing information with the robot's proximity to obstacles
* `/status`: `RobotStatus` message with information about the robot's sensors and position
* `/touch`: `Touch` message with the state and raw touch value of the robot's touch sensor
* `/events/object`: `Object` message providing the pose, object type, ID, and other attributes of the objects that Vector detects with its camera. One method to begin detecting is to send a message to `/behavior/look_in_place` 
* `/cube/info`: `LightCube` message providing various sensor readings of Vector's light cube
* `/behavior/response`: `Response` message with the results of select commands issued to `/behavior`

#### Camera required

Enable publishing to these topics by specifying the `--camera` flag when launching the core ROS node

* `/camera`: ROS `Image` representation of the robot's front-facing camera. Use `cv_bridge` to decode this message into a OpenCV-compatible format
* `/events/face`: `Face` message providing the pose, ID, and other attributes of the faces that Vector detects with its camera. One method to begin detecting is to send a message to `/behavior/find_faces`

### Write topics

Publish to these topics to make Vector move and perform other functions. Note that for all movement-based topics, Vector will prevent itself from falling from surfaces (e.g. desks), even when programmed to continue moving.

#### Pre-programmed `/behavior` routines

* `/behavior/find_faces`: Receives a `Bool` message with a `true` value to turn in place and look for faces
* `/behavior/look_in_place`: Receives a `Bool` message with a value of `true` to turn in place

Note that the following routines only execute if Vector is currently detecting the specified object or face:

* `/behavior/go_to_object`: Receives an `Int16` message with the ID of an object, which Vector drives towards
* `/behavior/wheelie`: Receives an `Int16` message with the ID of Light Cube, which Vector pops a wheelie with
* `/behavior/roll_cube`: Receives an `Int16` message with the ID of Light Cube, which Vector rolls towards itself
* `/behavior/roll_visible_cube`: Receives a `Bool` message with a `true` value to roll a LightCube if visible
* `/behavior/dock_cube`: Receives an `Int16` message with the ID of Light Cube, which Vector drives towards and hooks onto
* `/behavior/pickup_object`: Receives an `Int16` message with the ID of Light Cube, which Vector drives towards and picks up
* `/behavior/place_object_ground`: Places the `LightCube` with the specified `Int16` ID onto the ground
* `/behavior/turn_face`: Turns to the `Face` with the specified ID. Note that publishing to this topic will have no effect if the face isn't currently in view.
* `/behavior/drive_charger`: Receives a `Bool` message to trigger driving on or off Vector's charger. A `true` value makes the robot drive on its charger, while a `false` value makes it drive off its charger.

#### High-level driving

* `/behavior/drive_straight`: Receives a custom `Dist` message, making the robot drive straight for the specified distance and speed
* `/behavior/head_angle`: Receives a `Float32` message and turns Vector's head to the specified angle, in radians. Range in degrees is [-22, 45], with other values clamped
* `/behavior/turn_in_place`: Receives a `Float32` message and turns Vector in place by the specified amount, in radians. Positive values turn counterclockwise, while negative values turn clockwise.
* `/behavior/lift_height`: Receives a `Float32` message and sets Vector's lift to the desired height. This is clamped between 0.0 (representing the bottom position) and 1.0 (representing the top position)
* `/behavior/go_to_pose`: Receives a `Pose` message and goes to the specified position. Note that the `angle_z`, `angle`, and `pitch` properties are ignored; quarternion values should be used instead.

#### Low-level motor control

* `/motors/head`: Receives a `Float32` message to set Vector's head motor speed. Positive values represent up, negative values represent down. Measured in rad/sec.
* `/motors/lift`: Receives a `Float32` message to set Vector's lift motor speed. Positive values represent up, negative values represent down. Measured in rad/sec.
* `/motors/wheels`: Receives a `Drive` message and sets the velocity of the left and right treads in mm/sec.
* `/motors/stop`: Receives a boolean value of `True` to stop all motors

#### Media and animations

* `/anim/play`: Plays an animation, via a `String` message containing an animation name. See [here](./animations.md) for a full list of animation names.
* `/anim/play_trigger`: Plays an animation trigger, via a `String` message containing an animation trigger name. See [here](./animations.md#animation-trigger-list) for a full list of animation trigger names.
* `/behavior/say_text`: Receives a `String` message with text to synthesize into speech
* `/behavior/eye_color`: Receives a custom `Color` message and changes Vector's eye color accordingly
* `/audio/play`: Receives a `String` message containing the absolute path of a `.wav` file and plays it. Audio format must be 8000-16025 Hz, 16-bit, mono.
* `/audio/vol`: Recieves an integer 0-100 and sets the audio volume accordingly. This must be sent before a message is passed onto `/audio/vol` to play a file with the set volume; it does not modify sounds that are currently playing.
* `/screen/color`: Receives a `Color` message and sets Vector's screen to the chosen color for the specified duration.
* `/screen/image`: Receives an `String` message containing the absolute path on an image and displays it on Vector's screen. Resizes image as necessary
* `/screen/display_duration`: Receives a `Float32` to set the display duration for colors and images on the screen. This must be set before publishing to `/screen` subtopics to take effect. Default is 5 seconds.

#### Light cube behaviors

* `/cube/color_profile`: Set a `ColorProfile` for the light cube, modifying the brightness of the red, blue, and green LEDs. Must be called before setting the colors of the cube to have an effect.
* `/cube/lights`: Receives a `Color` message and sets the cube's four corners to the specified color
* `/cube/lights_off`: Receives a `Bool` message with a `True` value to turn the cube's lights off
* `/cube/flash_lights`: Receives a `Bool` message with a `True` value to rapidly flash the cube's lights, with white lights

## Custom Messages

#### `Dist`

Specifies how Vector should drive straight

* `distance`: distance to drive in mm (`Float32`)
* `speed`: velocity in mm/sec (`Float32`). Note that the maximum internal speed is 220 mm/sec.

#### `Drive`

Enables manual, fine-grained control over Vector's motors

* `left`: left motor velocity in mm/sec (`Float32`)
* `right`: right motor velocity in mm/sec (`Float32`)
* `left_acc`: left motor acceleration in mm/sec^2 (`Float32`)
* `right_acc`: right motor acceleration in mm/sec^2 (`Float32`)


#### `Pose`

Represents Vector's position in the world

* `x`: X position in mm (`Float32`)
* `y`: Y position in mm (`Float32`)
* `z`: Z position in mm (`Float32`)
* `q0`, `q1`, `q2`, `q3`: quarternion values representing Vector's rotation
* `angle_z`: rotation in the z axis in radians (`Float32`)

#### `Color`

Represents an RGB color combination

* `red`: int value 0-255
* `green`: int value 0-255 
* `blue`: int value 0-255


#### `ColorProfile`

Multiplies the intensity of a light cube's light color by the given `Float32` value

* `red_multiplier`
* `green_multiplier`
* `blue_multiplier`

#### `Proximity`

Reading from Vector's proximity sensor

* `distance`: `Float32` value, in mm
* `found_object`: boolean value indicating if object is found by the sensor
* `is_lift_in_fov`: boolean value indicating if Vector's lift is blocking its sensor
* `signal_quality`: `Float32` representing likelihood of reported distance being a solid surface
* `unobstructed`: boolean value confirming if no objects are detected

#### `Touch`

Reading from Vector's touch sensor

* `is_being_touched`: Vector's conclusion of if it is being touched (boolean)
* `raw_touch_value`: `Float32` representing the detected sensitivity from the touch sensor

#### `Object`

* `timestamp`: time in "robot time" (relative to SDK startup) for when Vector saw the object (`Int32`)
* `object_id`: integer object ID
* `object_type`: integer representing the object type
  * 0: invalid object
  * 1: unknown object
  * 2: Light Cube. Note that Vector can recognize at most one Light Cube at a time
  * 6: charger
  * 15: custom object
* `is_active`: retrieves the state of the object with respect to Vector's connection
* `img_rect`: `ImageRect` message enclosing the object in a bounding box with respect to Vector's camera
* `pose`: `Pose` message for object's location with respect to vector
* `top_face_orientation_rad`: offset of the object's top side, in radians (`Float32`)

#### `Face`

* `timestamp`: time in "robot time" (relative to SDK startup) for when Vector saw the object (`Int32`)
* `face_id`: integer face ID. May change once Vector loses track of the face.
* `img_rect`: `ImageRect` message enclosing the face in a bounding box with respect to Vector's camera
* `pose`: `Pose` message for object's location with respect to vector

#### `ImageRect`

Provides a bounding box for a tracked object with respect to Vector's camera

* `x_top_left`: x-coordinate of top-left corner of the box (`Float32`)
* `y_top_left`: y-coordinate of top-left corner of the box (`Float32`)
* `width`: width of the box in pixels (`Float32`)
* `height`: height of the box in pixels (`Float32`)

#### `LightCube`

Represents the state of a light cube

* `object_id`: internal integer ID representing the connected cube
* `is_visible`: boolean value providing if Vector can see the cube
* `is_moving`: boolean value providing if the cube's accelerometer detects that it is moving
* `last_moved_robot_timestamp`: the time the object was last moved in robot time
* `last_moved_start_robot_timestamp`: the time the object more recently started moving in robot time
* `last_moved_time`: the time the object was last moved in SDK time
* `last_tapped_robot_timestamp`: the time the object was last tapped in robot time
* `last_tapped_time`: the time the object was last tapped in SDK time
* `top_face_orientation_rad`: angular distance from the current reported up axis
* `pose`: the current `Pose` of the cube with respect to the robot. If the cube is currently not visible, this has zeroed values or will retain its last known position

#### `RobotStatus`

Various boolean values representing Vector's state. You may use this message to determine when all of Vector's nodes are online.

* `are_motors_moving`
* `are_wheels_moving`
* `is_animating`
* `is_being_held`
* `is_button_pressed`
* `is_carrying_block`
* `is_charging`
* `is_cliff_detected`
* `is_docking_to_marker`
* `is_falling`
* `is_head_in_pos`
* `is_in_calm_power_mode`
* `is_lift_in_pos`
* `is_on_charger`
* `is_pathing`
* `is_picked_up`
* `is_robot_moving`

#### `Response`

The status and (optionally) result of a command published to `/behavior`

* `status`: an integer [status code](https://sdk-resources.anki.com/vector/docs/proto.html#anki_vector/messaging/response_status.proto) indicating if the command has been received
* `result`: an integer [action result code](https://sdk-resources.anki.com/vector/docs/proto.html#Anki.Vector.external_interface.ActionResult.ActionResultCode) indicating if the action (e.g. picking up a cube) has been completed successfully or possible reasons for failure. Some response types do not have a result code exposed by the API and will hold a value of `-1`.
* `type`: a string indicating the type of the response. Refer [here](https://sdk-resources.anki.com/vector/docs/proto.html) to see possible options


## Demos

The [`launch`](./launch/) and [`sample_nodes`](./sample_nodes/) folders contain several executable roslaunch and Python files with examples of using this package. For standalone Python files, ensure the `vector_ros` node is running beforehand.

* `hello_world.launch`: launches a script where Vector drives in a straight line, then stops
* `lab_demo.launch`: continuous lab demo where Vector drives off its base, runs an idle animation, and prompts the user for various interactions and responds to them. These include petting Vector, [tucking it into sleep](https://drive.google.com/file/d/17Y_z-SmcwopfTtOYCYtyy1H7Fr6nSKx8/view?usp=sharing), [giving it a fistbump, and giving it a cube](https://drive.google.com/file/d/1HXg0o75CS0so_P3DUJmZOWOWpET62BLm/view?usp=sharing) to perform an action. The demo uses a ROS node-based architecture to communicate between prompts and responses.
* `display_image.py`: displays an image for a specified duration on Vector's screen
* `play_sound.py`: plays a sound file in the correct format (see above) at a specified volume
* `text_to_speech.py`: script to continuously try out Vector's voice synthesizer, converting user input text to speech
* `animate.py`: showcase animation sequence where Vector moves its forklift up and down, drives off its base, says hello, and changes its eye color
* `math_demo.py`: Vector asks several simple math questions and responds to the user accordingly. Relies the computer's microphone and an internet connection for audio input/recognition. [Click here](https://drive.google.com/file/d/17Zbq8zSnWRLT9JR43gHkNIH8i1KcZ3vu/view?usp=sharing) for a video demo!



## My setup

# Vector2.0_ROS

!!! Make sure you are on the same wifi network

## Create Virtual Environment
$ sudo apt-get install python3-venv

$ python3 -m venv venv

$ source venv/bin/activate

$ sudo apt-get update

$ sudo apt-get install python3

$ sudo apt install python3-pip

$ sudo apt-get install python3-pil.imagetk

$ python3 -m pip install --user anki_vector
	Possible error: No module name 'setuptools_rust'

	$ python -m ensurepip --upgrade
	$ python -m pip install --upgrade pip
	$ python3 -m pip install anki_vector

$ python3 -m anki_vector.configure

	enter credentials
	
	This will generate sdk_config.ini

### May have to install additional packages

$ pip install pyyaml

$ pip install rospkg

$ pip install opencv-python

$ pip install protobuf==3.20.*
