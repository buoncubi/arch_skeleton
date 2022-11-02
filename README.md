# Robot Architecture Skeleton
**A ROS-based exercise for the Experimental Robotics Laboratory course held at the University of Genoa.**  
Author: *Luca Buoncomapgni*

---

## Introduction

This repository contains ROS-based software that simulates a behavioural architecture.
The objective is twofold: show examples of ROS software, and provide some guidelines 
for bootstrapping software architectures for robotics.

In particular, this repository showcases the usage of *roslaunch* and
*parameters*, as well as the implementation of ROS *nodes*, *servers* and *actions*, with related 
*messages*; including arrays of custom items. These examples are provided in Python 3.

In addition, the architecture has been bootstrapped with an approach based on the following 
procedure.
 1. Define each software component with a random-based dummy implementation, i.e., mind only 
    their required and provided interfaces. The purpose is to test the flow of (meaningless) 
    data in the architecture for evaluating the synchronization of its components. In this
    phase, you should investigate available libraries and arrange them in your architecture
    in a suitable way.

 2. Define a simple keyboard-based interface to inject into the architecture the relevant stimulus
   for the robot behaviour, i.e., sensor data or state variables that it needs to *sense*. The 
   objective is to test the software without introducing uncontrollable sources of errors, e.g., 
   that might be due to sensors.

 3. Implement the components in charge to control the robot's behaviour. In this example, it will
    be a Finite States Machine, which is tested with the simple interfaces developed in 2.
 
 4. Make the interfaces developed in 2 based on randomised approaches and not on a keyboard
   interface anymore. This is done to stress the logic of your architecture and the 
   synchronization of its software components. In this phase, you should also introduce border 
   case situations to further stress the architecture.

 5. Write scripts to automatically evaluate if the randomized robot behaviour is consistent, 
   e.g., though *if* statements and *timestamp* comparison.

 6. Develop the actual implementation (and configure the relevant dependencies) of a single 
   software component developed in 1 with a dummy implementation.

 7. Test the component implemented in 6 through the script developed in 5.

 8. Go to 6 and loop until each component is implemented.

 9. Test the overall architecture, finalize the documentation and refactor the code appropriately.

This repository contains the components of an architecture based on the 1-st, 2-nd and 4-th 
steps, while the 3-rd is the main task that you should tackle during the exercise. As far as the 
exercise is concerned, the 5-th step is optional, and the 6-th will be addressed in the next 
parts of the Experimental Robotics Laboratory course; as far as some specific components are 
concerned.

## Scenario

The scenario involves a pet-like robot with the following behaviour.
 - Normally, the robot moves randomly in the environment (e.g., in a room).
 - When the battery is low, the robot immediately stops and waits for charging. For simplicity, we 
   do not control the robot toward a specific location to recharge its battery.
 - When a user issues specific speech-based commands a human-robot interaction phase starts or 
   stops, i.e., *called* (e.g., "Come here!", "Hi!", etc.) or *greeted* (e.g., "Bye", "Stop", 
   etc.), respectively.
 - When the interaction starts, the robot moves toward the user and waits for a pointing gesture. 
   The gesture is performed by the user to indicate a position in the environment.
 - When the pointing gesture is provided, the robot moves toward such a position.
 - When the pointed position is reached, the robot comes back to the user.
 - The above three points are repeated until the interaction ends or the battery is low. When the
   interaction ends due to a speech-based command and the battery is not low, the robot returns
   to move randomly.

### Assumptions

For simplicity and showing purposes, we consider a scenario with the following assumptions.
 - The robot moves in a 2D environment without obstacles.
 - Given a current and target position, the robot plans a trajectory to follow, i.e., a list of via 
   points. Then, it follows the trajectory by reaching each via point. The distance between via
   points is small enough to disregard the positions within two via points (e.g., for 
   representing the current robot pose).
 - The user can only say simple (and predefined) commands to the robot through the speech-based 
   interface. The speech-based commands can be given at any time, and they might be wrongly
   detected.
 - A pointing gesture is valid only when the interaction occurs. If more gestures are provided,
   the robot should consider the more recent one.
 - The user can point to a 2D location at any time. The location might be out of the environment, 
   i.e, it can refer to a position that is unreachable by the robot.
 - The battery can become low at any time, and the robot should immediately react to this event.
 - The user does not move.

### Synchronization

An important aspect of this exercise is the synchronization between the robot and the user. In
particular, the user should never wait for the robot to complete an action, except when it is
recharging its battery. This implies that the Finite States Machine should never be blocked. In
other words, the Finite States Machine should process speech-based, gesture-based, and 
battery-based events as soon as they occur. Furthermore, we consider that the Finite States 
Machine does not allow for concurrent states.

## Project Structure

### Package List

This repository contains a ROS package named `arch_skeleton` that includes the following resources.
 - [CMakeList.txt](CMakeList.txt): File to configure this package.
 - [package.xml](package.xml): File to configure this package.
 - [setup.py](setup.py): File to `import` python modules from the `utilities` folder into the 
   files in the `script` folder. 
 - [launcher/](launcher/): Contains the configuration to launch this package.
    - [manual_sense.launch](launcher/manual_sense.launch): It launches this package allowing 
       for keyboard-based interface.
    - [random_sense.launch](launcher/random_sense.launch): It launches this package with 
      random-based stimulus.
 - [msg/](msg/): It contains the message exchanged through ROS topics.
    - [Gesture.msg](msg/Gesture.msg): It is the message representing detected pointing gestures.
    - [Speech.msg](msg/Speech.msg): It is the message representing speech-based commands.
    - [Point.msg](msg/Point.msg): It is the message representing a 2D point.
 - [srv/](srv/): It Contains the definition of each server used by this software.
    - [GetPose.srv](srv/GetPose.srv): It defines the request and response to get the current 
      robot position.
    - [SetPose.srv](srv/SetPose.srv): It defines the request and response to set the current 
      robot position.
 - [action/](action/): It contains the definition of each action server used by this software.
    - [Plan.action](action/Plan.action): It defines the goal, feedback and results concerning 
      motion planning.
    - [Control.action](action/Control.action): It defines the goal, feedback and results 
      concerning motion controlling.
 - [scripts/](scripts/): It contains the implementation of each software components.
    - [speech.py](scripts/speech.py): It is a dummy implementation of the speech-based 
      commands detection algorithm.
    - [gesture.py](scripts/gesture.py): It is a dummy implementation of the gesture-based
      commands detection algorithm.
    - [robot_state.py](scripts/robot_state.py): It implements the robot state including:
      current position, and battery level.
    - [planner.py](scripts/planner.py): It is a dummy implementation of a motion planner.
    - [controller.py](scripts/controller.py): It is a dummy implementation of a motion 
      controller.
 - [utilities/arch_skeleton/](utilities/arch_skeleton/): It contains auxiliary python files, 
   which are exploited by the files in the `scripts` folder.
    - [architecture_name_mapper.py](scripts/architecture_name_mapper.py): It contains the name 
      of each *node*, *topic*, *server*, *actions* and *parameters* used in this architecture.
 - [diagrams/](diagrams/): It contains the diagrams shown below in this README file.

### Dependencies

The software exploits [roslaunch](http://wiki.ros.org/roslaunch) and 
[rospy](http://wiki.ros.org/rospy) for using python with ROS. Rospy allows defining ROS nodes, 
services and related messages.

Also, the software uses [actionlib](http://wiki.ros.org/actionlib/DetailedDescription) to define
action servers. In particular, this software is based on 
[SimpleActionServer](http://docs.ros.org/en/jade/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html#a2013e3b4a6a3cb0b77bb31403e26f137).
Thus, you should use the [SimpleActionClient](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html)
to solve the exercise.

The Finite States Machine that you will implement based on the software components provided in 
this repository should be based on [SMACH](http://wiki.ros.org/smach). You can check the 
[tutorials](http://wiki.ros.org/smach/Tutorials) related to SMACH, for an overview of its 
functionalities. In addition, you can exploit the [smach_viewer](http://wiki.ros.org/smach_viewer)
node to visualize and debug the implemented Finite States Machine.

## Software Components

It follows the details of each software component implemented in this repository, which is available
in the `scripts/` folder.

### The `speech-eval` Node, its Message and Parameters

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/speech-eval.png" width="600">

The `speech-eval` node is a simple publisher that produces `Speech` messages in the 
`/sensor/speech` topic. Each generated message has two fields: a time `stamp` and a 
`command`. The latter is a string equal to `"Hello"`, when the interaction should start, or
`"Bye"` when the interaction should end. Such keywords can be configured through the 
`config/speech_commands` parameter (detailed below).

This node allows publishing messages from the keyboard or in a randomized manner, and this can 
be chosen with the `test/random_sense/active` parameter detailed below. When random messages are
published, the `test/random_sense/speech_time` parameter is used to delay the generated 
commands, which might not always be consistent for accounting perception errors (e.g., the command 
`"???"` is sometimes published).

To observe the behaviour of the `speech-eval` node you can run the following commands.
```bash
roscore
# Open a new terminal.
rosparam set config/speech_commands '["Hello", "Bye"]'
rosrun arch_skeleton speech.py 
# Open a new terminal
rostopic echo /sensor/speech
```
With `rosparam` you might also set the `test/random_sense/active` and  
`test/random_sense/speech_time` parameters (detailed below) to see how messages are differently
published.

### The `gesture-eval` Node, its Message and Parameters

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/gesture-eval.png" width="600">

The `gesture-eval` node is a simple publisher that produces `Gesture` messages in the 
`sensor/gesture` topic. Each generated message has two fields: a time `stamp` and a `coordinate`.
The latter is of type `Point` (defined in the `msg/` folder), which has two `float` sub-fields, 
i.e., `x` and `y`.

This node allows publishing messages from the keyboard or in a randomized manner, and this can be
chosen with the `test/random_sense/active` parameter detailed below. When random messages are
published, the `test/random_sense/gesture_time` parameter is used to delay the generated 
messages, which encode a `coordinate` with random `x` and `y` based on the 
`config/environment_size` parameter detailed below. To simulate possible perception error, this 
node might generate `coordinates` that are out of the environment.


To observe the behaviour of the `gesture-eval` node you can run the following commands.
```bash
roscore
# Open a new terminal.
rosparam set config/environment_size '[10,10]'
rosrun arch_skeleton gesture.py 
# Open a new terminal
rostopic echo /sensor/gesture 
```
With `rosparam` you might also set the `test/random_sense/active` and  
`test/random_sense/gesture_time` parameters (detailed below) to see how messages are differently 
published.

### The `robot-state` Node, its Messages and Parameters

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/robot-state.png" width="900">

The `robot-state` is a node that encodes the knowledge shared among the other components, and it 
implements two services (i.e., `state/set_pose` and `state/get_pose`) and a publisher (i.e., 
`state/battery_low`). 

The services allow setting and getting the current robot position, which is shared between the 
`planner` and the `controller` as detailed below. In particular, the `state/set_pose` requires a 
`Point` to be set and returns nothing, while the `state/get_pose` requires nothing and return
a `Point` encoding the robot pose. 

Note that a client should set the initial robot position when the architecture startups. 

Also, note that, for more general architectures, the robot pose might be published in a topic, 
instead of being provided through a server. This is because many components might require the 
current robot pose, which might change frequently. However, this example does not consider such a case.

Moreover, the `robot-state` also implements a publisher of `Boolean` messages into the `state/
battery_low` topic. This message is published when the batter changes state. We consider two 
possible states: low battery (i.e., `True` is published) and recharged (i.e., `False` is 
published).

The battery-related publisher allows publishing messages from the keyboard or in a randomized 
manner, and this can be chosen with the `test/random_sense/active` parameter detailed below. 
When random messages are published, the `test/random_sense/battery_time` parameter is used to 
delay the published messages.

To observe the behaviour of the `robot-state` node you can run the following commands.
```bash
roscore
# Open a new terminal.
rosrun arch_skeleton robot-state.py 
# Open a new terminal
rostopic echo /state/battery_low 
# Open a new terminal 
rosservice call /state/set_pose "pose: { x: 1.11,  y: 2.22}"
rosservice call /state/get_pose "{}" 
```
With `rosparam` you might also set the `test/random_sense/active` and  
`test/random_sense/battery_time` parameters (detailed below) to see how messages are 
differently published.

### The `planner` Node, its Message and Parameters

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/planner.png" width="900">

The `planner` node implements an action server named `motion/planner`. This is done by the 
means of the `SimpleActionServer` class based on the `Plan` action message. This action server 
requires the `state/get_pose/` service of the `robot-state` node, and a `target` point given as goal.

Given the current and target points, this component returns a plan as a list of `via_points`, 
which are randomly generated for simplicity. The number of `via_points` can be set with the 
`test/random_plan_points` parameter addressed below. Moreover, each `via_point` is provided 
after a delay to simulate computation, which can be tuned through the `test/random_plan_time` 
parameter. When a new `via_points` is generated, the updated plan is provided as `feedback`. When
all the `via_points` have been generated, the plan is provided as `results`.

While the picture above shows the actual implementation of the action server, you should not 
interact with it through the shown topics directly. Instead, you should use a 
[SimpleActionClient](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html), 
for instance, as:
```python
import actionlib
from arch_skeleton.msg import PlanAction, PlanGoal
...
# Initialize the client and, eventually, wait for the server.
client = actionlib.SimpleActionClient('motion/planner', PlanAction)
client.wait_for_server()
...
def feedback_callback(feedback):
    # Do something when feedback is provided.
    pass  
...
def done_callback(status, results):
    # Do something when results are provided.
    pass  
...
# Send a new `goal`, which is a message of type `PlanGoal`.
client.send_goal(goal, done_cb = done_callback, feedback_cb = feedback_callback)
...
# Get the action server state.
client.get_state()
...
# Cancel all goals (or the current goal only, i.e., `client.cancel_goal()`).
client.cancel_all_goals()
```

To observe the behaviour of the `planner` you can run the following commands.
``` bash
roscore
# Open a new terminal.
rosrun arch_skeleton robot_states.py
# Open a new terminal.
rosservice call /state/set_pose "pose: { x: 0.11,  y: 0.22}"
rosparam set config/environment_size '[10,10]'
rosrun arch_skeleton planner.py
# Open a new terminal.
rosrun actionlib_tools axclient.py /motion/planner
```
Then, a GUI should appear. Set the goal you want to reach and hit the send button. Eventually, you
can cancel the goal as well. Also, you can change the `test/random_plan_points` and 
`test/random_plan_time` parameters (detailed below) to tune the behaviour of the planner.

The last command of the above fragment of code requires the `actionlib-tools` package, which can
be installed done by typing:
```bash
sudo apt update
sudo apt install ros-noetic-actionlib-tools
```


### The `controller` Node, its Message and Parameters

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/controller.png" width="900">

The `controller` node implements an action server named `motion/controller`. This is done by 
the means of the `SimpleActionServer` class based on the `Control` action message. This action 
server requires the `state/set_pose/` service of the `robot-state` node and a plan given as a 
list of `via_points` by the `planner`.

Given the plan and the current robot position, this component iterates for each planned 
`via_point` and waits to simulate the time spent moving the robot to that location. The 
waiting time can be tuned through the `test/random_motion_time` parameter detailed below. Each 
time a `via_point` is reached the `state/set_pose` service is invoked, and a `feedback` is 
provided. When the last `via_point` is reached, the action service provides a result by 
propagating the current robot position, which has been already updated through the 
`state/set_pose` service.

Similarly to the `planner` above, instead of using the raw topics, you can rely on a 
`SimpleActionClient`, which should be instantiated as:
```python
client = actionlib.SimpleActionClient('motion/controller', ControlAction)
```
This client would accept goals of type `ControlGoal`.

To observe the behaviour of the `controller` you can run the following commands.
``` bash
roscore
# Open a new terminal.
rosrun arch_skeleton robot_states.py
# Open a new terminal.
rosservice call /state/set_pose "pose: { x: 0.11,  y: 0.22}"
#rosparam set config/environment_size '[10,10]'
rosrun arch_skeleton controller.py
# Open a new terminal.
rosrun actionlib_tools axclient.py /motion/controller
```
Then, the same GUI seen for the `planner` should appear. In this case, you can test goals 
formatted as:
```yaml
via_points: 
  - 
    x: 0.109999999404
    y: 0.219999998808
  - 
    x: 3.61638021469
    y: 5.05489301682
  - 
    x: 0.292526483536
    y: 6.59786701202
  - 
    x: 4.33828830719
    y: 7.73262834549
  - 
    x: 6.0
    y: 6.0
```
You can also change the `test/random_motion_time` parameter (detailed below) to tune
the behaviour of the controller.

## Launching the Software

This software has been based on ROS Noetic, and it has been developed with this Docker-based
[environment](https://hub.docker.com/repository/docker/carms84/exproblab), which already 
provides the required dependencies listed above. 

### Installation

Follow these steps to install the software.
 - Clone this repository inside your ROS workspace (which should be sourced in your `.bashrc`).
 - Run `chmod +x <file_name>` for each file inside the `scripts` folder.
 - Run `catkin_make` from the root of your ROS workspace.
 - Install `xterm` by entering the command `sudo apt install -y xterm`.

### Launchers

Use the following command to launch the software with a keyboard-base interface for speech, 
gesture and battery level.
```bash
roslaunch arch_skeleton manual_sense.launch
```

Use the following command to launch the software with randomized stimulus, 
i.e., speech, gesture and battery level.
```bash
roslaunch arch_skeleton random_sense.launch
```

Note that the architecture launched with these commands does nothing, except generate stimulus, 
i.e., battery level, speech and gesture commands. In order to invoke the motion planner 
and controller, you need to implement the Finite States Machine as described below.

Check the `roslouch` outcome to get the path where logs are stored. usually, it is `~/.ros/log/`.
That folder should also contain a link to the `latest` produced log.

### ROS Parameters

This software requires the following ROS parameters.
 
 - `config/environment_size`: It represents the environment boundaries as a list of two float
   numbers, i.e., `[x_max, y_max]`. The environment will have the `x`-th coordinate spanning
   in the interval `[0, x_max)`, while the `y`-th coordinate in `[0, y_max)`.

 - `config/user_pose`: It represents the user's position as a list of two float numbers,
   i.e., `[x, y]`. This pose should be within the `environmet_size`.

 - `config/speech_commands`: It defines the keywords that the user can say to start and end
   the interaction. It must be a list made of two strings (e.g., `["Hello", "Bye"]`) that define
   the keyword to start and end the interaction, respectively.

 - `state/initial_pose`: It represents the initial robot pose as a list of two float numbers, 
   i.e., `[x, y]`. This pose should be within the `environmet_size`.

 - `test/random_plan_points`: It represents the number of via points in a plan, and it should be
   a list of two integer numbers `[min_n, max_n]`. A random value within such an interval will be
   chosen to simulate plans of different lengths.

 - `test/random_plan_time`: It represents the time required to compute the next via point of the 
   plan, and it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. 
   A random value within such an interval will be chosen to simulate the time required to 
   compute the next via points.

 - `test/random_motion_time`: It represents the time required to reach the next via point, and 
   it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. A random
   value within such an interval will be chosen to simulate the time required to reach the next 
   via points. 

 - `test/random_sense/active`: It is a boolean value that activates (i.e., `True`) or 
   deactivates (`False`) the random-based generation of stimulus (i.e., speech, gesture and 
   battery level). If this parameter is `True`, then the three parameters below are also 
   required.  If it is `False`, then the three parameters below are not used.
 

In addition, the `random_sense.launch` also requires the following three parameters. This 
occurs because `test/random_sense/active` has been set to `True`.

 - `test/random_sense/gesture_time`: It indicates the time passed within two randomly generated 
   pointing gestures. It should be a list of two float numbers, i.e., `[min_time, max_time]` in 
   seconds, and the time passed between gestures will be a random value within such an interval.

 - `test/random_sense/speech_time`: It indicates the time passed within two randomly generated
   commands based on speech. It should be a list of two float numbers, i.e., 
   `[min_time, max_time]` in seconds, and the time passed between speech-based commands will be 
   a random value within such an interval.

 - `test/random_sense/battery_time`: It indicates the time passed within battery state changes 
   (i.e., low/high). It should be a list of two float numbers, i.e., `[min_time, max_time]` in 
   seconds, and the time passed between changes in battery levels will be a random value within 
   such an interval.

---

## The exercise

Develop a Finite States Machine based on the SMACH library that implements the behaviour of the 
robot. Use only the software components provided in this repository to develop such a Finite 
States Machine.

Debug your implementation with the `manual_sense.launch` configuration. Then, test it in a log 
term running through the `random_sense.launch` configuration. 

Optionally, write a script that automatically checks if an anomalous behaviour occurs while 
using the `random_sense.launch` configuration.

### The Finite States Machine

The Finite States Machine to be developed should implement the scenario introduced at the 
beginning of this README file. 

In addition, the Finite States Machine should have the following functionalities.
 - It sets, in the `robot-state` node, the initial robot pose given through the 
   `state/initial_pose` parameter.
 - It subscribes to the `sensor/speech` topic to get speech-based commands.
 - It subscribes to the `sensor/gesture` topic to get pointing gestures.
 - It subscribes to the `state/battery_low` topic to get the battery state.
 - It processes each speech, gesture, and battery message as soon as they are provided.
 - It uses the `planner` action server and cancels it if necessary.
 - It uses the `controller` action server and cancels it if necessary.

Note that, in Python, ROS subscribes run on separate threads. Thus, you 
need to use `mutexes` to assure data consistency across concurrent threads.

# A Solution

A possible solution to this exercise is available in the [Wiki](https://github.com/buoncubi/arch_skeleton/wiki/Noetic-Py3-Solution)
of this repository.

---
