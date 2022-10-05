# Robot Architecture Skeleton
**An exercise for the Experimental Robotics Laboratory course held at the University of 
Genoa.**  
Author: *Luca Buoncomapgni*  
---

## Introduction

This repository contains a ROS-based software that simulates a behavioral architecture.
The objective is twofold: show examples of ROS-based software, and provide some guidelines 
to bootstrap of software architecture for robotics.

In particular, the examples that this repository showcase include: the usage of `roslaunch` and
*parameters*, as well as the implementation of ROS *nodes*, *servers* and *actions*, with related 
*messages*; including arrays of custom items. These examples are provided in Python 2.

In addition, the architecture bootstrapping approach is based on the following procedure.
 1. Define each software component with a random-based dummy implementation, i.e., mind only 
    their required and provided interfaces. The purpose is to test the flow of (meaningless) 
    data in the architecture for evaluate the synchronization of its components.
 2. Define a simple keyboard-based interface to inject in the architecture the data that the
    robot should *sense*, i.e., sensor data or state variables. The objective is to test the
    software without introducing further sources of errors, e.g., that might be given for 
    sensors.
 3. Implement the components in charge to control the robot behavior. In this example it will be
    a Finite States Machine, which is tested with the simple interfaces developed in 2.
 4. Make the interfaces developed in 2 based on randomizes approaches, and not on a keyboard
    interface anymore. This is done to stress the implementation of the architecture and the 
    synchronization of its software components.
 5. Write a script to automatically evaluate if the randomized robot behavior is consistent, 
    e.g., though `if` statements and `timestamp` comparison.
 6. Develop the actual implementation of a component that was defined in 1 but with a dummy 
    implementation.
 7. Go to 5 and repeat, i.e., develop a component and test before to focus on another component.

This repository contains the components of an architecture based on the 1-st, 2-nd and 4-th 
steps, while the 3-rd is the main task that you should tackle during the exercise. As far as the 
exercise is concerned, the 5-th step is optional, while the 6-th and 7-th will be addressed in 
the next parts of the Experimental Robotics Laboratory course.

## Scenario

The scenario involves a pet-like robot with the following behavior.
 - Normally, the robot moves random in the environment (e.g., a room).
 - When the battery is low, the robot immediately stops and wait for charging. For simplicity, we 
   do not control the robot toward a specific location to recharge its battery.
 - When a user issue specific speech-based commands a human-robot interaction phase starts or 
   stops, i.e., `called` (e.g., "Come here!", "Hi!", etc.), `greeated` (e.g., "Bye", "Stop", 
   etc.), respectively.
 - When the interaction starts, the robot moves toward the user and waits for a pointing gesture 
   performed by him/her to indicate a position in the environment.
 - When the pointing gesture is provided, the robot moves toward such a position.
 - When the pointed position is reached, the robot comes back to the user.
 - The above three points are repeated until the interaction ends or the battery is low.

### Assumptions

For simplicity and showing purposes we consider a scenario with the following assumptions.
 - The robot moves in a 2D environment without obstacles.
 - Given a current and target position, the robot plans a trajectory to follow as a list of via 
   points. Then it follows the trajectory by reaching each via point. The distance between via
   points is small enough for disregarding the positions within two via points (e.g., for 
   representing the current robot pose).
 - The user can only say simple (and predefined) commands to the robot through the speech-based 
   interface. The speech-based commands can be given at any time, and they might be wrongly      
   detected.
 - The user can point a 2D location at any time. The location might be out of the environment, 
   i.e, it can refer to a position that is unreachable by the robot.
 - The battery low event can occur at any time.
 - The user does not move.

### Synchronization

An important aspect of this exercise is the synchronization between the robot and the user. In
particular, the user should never wait for the robot to complete an action, except when it is
recharging its battery. This implies that the Finite States Machine should never be blocking. In
other words, the Finite States Machine should process speech-based, gesture-based, and 
battery-based events as soon as they occur. In addition, we consider that the Finite States 
Machine does not allow for concurrent states.

## Software Structure

### Package List

This repository is a ROS package named `arch_skeleton` that includes the following resources.
 - `CMakeList.txt`: File to configure this package.
 - `package.xml`: File to configure this package.
 - `launcher/`: Contains the configuration to launch this package.
    - `manual_sense.launch`: It launches this package allowing for keyboard-based interface.
    - `random_sense.launch`: It launches this package with random-based sense functionalities.
 - `msg/`: It contains the message exchanged through ROS topics.
    - `Gesture.msg`: It is the message representing detected pointing gestures.
    - `Speech.msg`: It is the message representing the speech-based command.
    - `Point.msg`: It is the message representing a 2D point `(x,y)`
 - `srv/`: It Contains the definition of each server used by this software.
    - `GetPose.srv`: It defines the request and response to get the current robot position.
    - `SetPose.srv`: It defines the request and response to set the current robot position.
 - `action/`: It contains the definition of each action server used by this software.
    - `Plan.action`: It defines the goal, feedback and results concerning motion planning.
    - `Control.action`: It defines the goal, feedback and results concerning motion controlling.
 - `script/`: It contains the implementation of each software components.
    - `speech.py`: It is a simple Simulation of incoming speech-based commands.
    - `gesture.py`: It is a simple Simulation of incoming gesture-based commands.
    - `robot_state.py`: It implements the robot state including: current position, and 
       battery level.
    - `planner.py`: It is a simple Simulation of a motion planner.
    - `controller.py`: It is a simple Simulation of a motion controller.
    - `architecture_name_mapper.py`: It contains the name of ROS each node, topic, server, 
       actions and parameters used in this software.
 - `diagrams/`: It contains the diagrams shown below in this README.

### Dependencies

The software exploits [roslaunch](http://wiki.ros.org/roslaunch) and [rospy](http://wiki.ros.org/
rospy) to use python with ROS. Rospy allow defining ROS nodes, services and related messages.

Also, the software uses [actionlib](http://wiki.ros.org/actionlib/DetailedDescription) to define
action servers. In particular, this implementation is based on 
[SimpleActionServer](http://docs.ros.org/en/jade/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html#a2013e3b4a6a3cb0b77bb31403e26f137), 
and you should use the [SimpleActionClient](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html).

The Finite States Machine that you will implement based on the software components provided in 
this repository should be based on [SMACH](http://wiki.ros.org/smach). You can check the 
[tutorials](http://wiki.ros.org/smach/Tutorials) related to SMACH, for an overview of its 
functionalities. In addition, you can exploit the [smach_viewer](http://wiki.ros.org/smach_viewer)
node to visualize and debug the implemented Finite States Machine.

## Software Components

It follows the details of each software component implemented in this repository, and available
in the `script/` folder.

### The `speech-eval` Node and Its Messages

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/speech-eval.png" width="600">

The `speech-eval` node is a simple publisher that produces `Speech` messages in the 
`/sensor/speech` topic. Each generated message has two fields: a time `stamp` and a 
`command`. The latter is a string equal to `"Hello"`, when the interaction should start, or
`"Bye"` when the interaction should end. Such a keywords can be configured through the 
`config/speech_commands` parameter (detailed below).

This node allows publishing messages from the keyboard or in a randomized manner, and this can be chosen with the `test/random_sense/active` parameter detailed below. When random messages are
published, the `test/random_sense/speech_time` parameter is used to delay the generated 
commands, which might not always be consistent to simulate perception errors (e.g., the command 
`"???"` is sometimes published).

To observe the behavior of the `speech-eval` node you can run the following commands.
```bash
roscore
# Open a new terminal.
rosparam set config/speech_commands '["Hello", "Bye"]'
rosrun arch_skeleton speech.py 
# Open a new terminal
rostopic echo /sensor/speech
```
With `rosparam` you might also set the `test/random_sense/active` and  
`test/random_sense/speech_time` parameters to see how messages are differently published.

### The `gesture-eval` Node and Its Messages

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/gesture-eval.png" width="600">

The `gesture-eval` node is a simple publisher that produces `Gesture` messages in the 
`sensor/gesture` topic. Each generated message has two fields: a time `stamp` and a `coordinate`.
The latter is of type `Point` (defied in the `msg/` folder), which has two `float` sub-fields, 
i.e., `x` and `y`.

This node allows publishing messages from the keyboard or in a randomized manner, and this can be
chosen with the `test/random_sense/active` parameter detailed below. When random messages are
published, the `test/random_sense/gesture_time` parameter is used to delay the generated 
messages, which encode a `coordinate` with random `x` and `y` based on the 
`config/environment_size` parameter detailed below. To simulate possible perception error, this 
node might generate `coordinates` that are out of the environment.

To observe the behavior of the `gesture-eval` node you can run the following commands.
```bash
roscore
# Open a new terminal.
rosparam set config/environment_size '[10,10]'
rosrun arch_skeleton gesture.py 
# Open a new terminal
rostopic echo /sensor/speech 
```
With `rosparam` you might also set the `test/random_sense/active` and  
`test/random_sense/gesture_time` parameters to see how messages are differently published.

### The `robot-state` Node and Its Messages

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/robot-state-eval.png" width="600">

The `robot-state` is a node that encodes the knowledge shared among the other components, and it 
implements two services (i.e., `state/set_pose` and `state/get_pose`) and a publisher (i.e., 
`state/battery_low`). 

The services allow setting and getting the current robot position, which is shared between the `planner` and the `controller` detailed below. In particular, the `state/set_pose` requires a 
`Point` to be set, and returns nothing, while the `state/get_pose` requires nothing, and return
the a `Point` encoding the robot pose. 

Note that a client should be used to set the initial robot position when the architecture 
startups. 

Also note that, for more general architectures, the robot pose might be published in a topic rather than provided it through a server. This because many components might require the current robot pose, which might change frequently.

Moreover, the `robot-state` also implements a publisher of `Boolean` messages into the `state/
battery_low` topic. This message is published when the batter change state. We consider two 
possible states: low battery (i.e., `True` is published) and recharged (i.e., `False` is 
published).

The battery-related publisher allows publishing messages from the keyboard or in a randomized 
manner, and this can be chosen with the `test/random_sense/active` parameter detailed below. 
When random messages are published, the `test/random_sense/battery_time` parameter is used to 
delay the published messages.

To observe the behavior of the `robot-state` node you can run the following commands.
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
`test/random_sense/battery_time` parameters to see how messages are differently published.

### The `planner` Node and Its Messages

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/planner.png" width="900">

The `planner` nodes implements an action server named `motion/planner`. This is done by the 
means of the `SimpleActionServer` class based on the `Plan` action message. This action server 
requires the `state/get_pose/` service of the `robot-state` node, and a `target` point given as goal.

Given the current and target points, this component returns a plan as a list of `via_points`, which are randomly generated for simplicity. The number of `via_points` can be set with the 
`test/random_plan_points` parameter addressed below. Moreover, each `via_point` is provided 
after a delay to simulate computation, which can be tune through the `test/random_plan_time` 
parameter. A new `via_points` is generated, the updated plan is provided as *feedback*. When
all the `via_points` have been generated, the plan is provided as *result*.

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
# Get the action server state.
client.get_state()
...
# Cancel all goal (or a single goal, i.e., `client.cancel_goal()`).
client.cancel_all_goals()
```

To observe the behavior of the `planner` you can run the following commands.
``` bash
roscore
# Open a new terminal.
rosrun arch_skeleton robot_states.py
# Open a new terminal.
rosservice call /state/set_pose "pose: { x: 0.11,  y: 0.22}"
rosparam set config/environment_size '[10,10]'
rosrun arch_skeleton planner.py
# Open a new terminal.
rosrun actionlib axclient.py /motion/planner
```
Then a GUI should appear. Set the goal you want to reach and hit the send button. Eventually you
can cancel the goal as well.

### The `controller` Node and Its Messages

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/controller.png" width="900">

The `controller` nodes implements an action server named `motion/controller`. This is done by 
the means of the `SimpleActionServer` class based on the `Control` action message. This action 
server requires the `state/set_pose/` service of the `robot-state` node, and a plan given as a 
list of `via_point` by the `planner`.

Given the plan and the current robot position, this component iterates for each planned 
`via_point`, and waits to simulate the time spent to move the robot to that location. The 
waiting time can be tune through the `test/random_motion_time` parameter detailed below. Each 
time a `via_point` is reached the `state/set_pose` service is invoked, and a *feedback* is 
provided. When the last `via_point` is reached, the action service provides a result by 
propagating the current robot position, which is already been updated through the 
`state/set_pose` service.

Similarly to the `planner` action server, instead of using the raw topic you can rely on a 
`SimpleActionClient`, which can be instantiated as:
```python
client = actionlib.SimpleActionClient('motion/controller', ControlAction)
```

To observe the behavior of the `controller` you can run the following commands.
``` bash
roscore
# Open a new terminal.
rosrun arch_skeleton robot_states.py
# Open a new terminal.
rosservice call /state/set_pose "pose: { x: 0.11,  y: 0.22}"
#rosparam set config/environment_size '[10,10]'
rosrun arch_skeleton controller.py
# Open a new terminal.
rosrun actionlib axclient.py /motion/controller
```
Then the same GUI seen for the `planner` should appear. In this case you can test goals 
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

## Launching the Software

This software has been based on ROS Kinetic, and it has been developed with [this]() Docker-based
[environment](https://hub.docker.com/repository/docker/carms84/exproblab), which already 
provides the required dependencies listed above. 

### Installation

Follow this steps to install the software.
 - Clone this repository inside your ROS workspace (which should be sources in your `.bashrc`).
 - Run `chmod +x <file_name>` for each file inside the `script` folder.
 - Run `carking_make` from the root of you workspace.

### Launchers

Use the following command to launch the software with a keyboard-base interface for speech, 
gesture and battery level.
```bash
roslaunch arch_skeleton manual_sense.launch
```

Use the following command to launch the software with randomized speech, gesture and battery level data.
```bash
roslaunch arch_skeleton random_sense.launch
```

Note that the architecture launched with these commands does nothing expect generate sensing 
data, i.e., battery level, speech and gesture commands. In order to invoke the motion planner 
and controller, you need to implement the Finite States Machine as described below.

### ROS Parameters

This software requires the following ROS parameters.
 - `config/environment_size`: It represents the environment boundaries as a list of two float
   numbers, i.e., `[x_max, y_max]`. The environment will have the `x`-th coordinate spanning
   in the interval `[0, x_max)`, while the `y`-th coordinate in `[0, y_max]`.
 - `config/user_pose`: It represents the position of the user as a list of two float numbers,
   i.e., `[x, y]`. This pose should be within the `environmet_size`.
 - `config/speech_commands`: It defines the keywords that the user can say to start and end
   the interaction. It must be a list made of two strings (e.g., `["Hello", "Bye"]`) that define
   the keyword to start and end the interaction, respectively.
 - `state/initial_pose`: It represents the initial robot pose as a list of two float numbers, 
   i.e., `[x, y]`. This pose should be within the `environmet_size`.
 - `test/random_motion_time`: It represents the time required to reach the next via point, and 
   it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. A random
   value within such an interval will be chosen to simulate the time required to reach the next 
   via points. 
 - `test/random_plan_points`: It represent the number of via points in a plan, and it should be
   a list of two integer number `[min_n, max_n]`. A random value within such an interval will be
   chosen to simulate plans of different length.
 - `test/random_plan_time`: It represents the time required to compute the next via point of the 
   plan, and it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. 
   A random value within such an interval will be chosen to simulate the time required to 
   compute the next via points. 
 - `test/random_sense/active`: It is a boolean value that activates (i.e., `True`) or 
   deactivates (`False`) the random-based generation of the data related to speech, gesture and 
   battery level. If this parameter is `True`, then the three parameters below are also 
   required.  If it is `False`, then the three parameters below are not required. 

In addition, the `random_sense.launch` also requires the following three parameters.
 - `test/random_sense/gesture_time`: It indicates the time passed within two randomly generated 
   pointing gestures. It should be a list of two float numbers, i.e., `[min_time, max_time]` in 
   seconds, and the time passed between gestures will be a random value within such an interval.
 - `test/random_sense/speech_time`: It indicates the time passed within two randomly generated
   commands based on speech. It should be a list of two float numbers, i.e., 
   `[min_time, max_time]` in seconds, and the time passed between speech-based commands will be 
   a random value within such an interval.
 - `test/random_sense/battery_time`: It indicates the time passed within battery state changes 
   (i.e., low/high). It should be a list of two float numbers, i.e., `[min_time, max_time]` in 
   seconds, and the time passed between changes of battery levels will be a random value within 
   such an interval.

---
## The exercise

Implement a Finite States Machine based on the SMACH library that implements the behavior of the 
robot based only on the software components provided in this repository. 

Debug your implementation with the `manual_sense.launch` configuration. Then test it in a log term running through the `random_sense.launch` configuration. 

Optionally, write a script that automatically checks if an anomalous behavior occurs while using the `random_sense.launch` configuration.

### The Finite States Machine

The Finite States Machine to be implemented should have the following functionalities.
 - It sets, in the `robot-state` node, the initial robot pose given through the 
   `state/initial_pose` parameter.
 - It subscribes to the `sensor/speech` topic to get speech-based commands.
 - It subscribes to the `sensor/gesture` topic to get pointing gestures.
 - It subscribes to the `state/battery_low` topic to get the battery state.
 - It processes each speech, gesture, and battery message as soon as they are provided.
 - It uses the `planner` action server and cancel it if necessary.
 - It uses the `controller` action server and cancel it if necessary.

Note that, differently from C++, Python subscribers runs on a separate threads. Thus, you 
need to use `mutexes` to assure data consistency across concurrent threads.

---
