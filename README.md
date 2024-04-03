# example_action

## Dependencies

* Ubuntu 22 operating system 
* ros2 humble must be installed

## Installation

* clone this project into your ros2 workspace src directory.
* build workspace

```bash
colcon build --symlink-install
```

## Running Example

* source your ros2 workspace where this package was built

```bash
source ros2_ws/install/setup.bash
```

* Run the launch file

```bash
ros2 launch example_action example_nodes.launch.py
```

* You should then see the action nodes communicating like the following:

```
[INFO] [launch]: All log files can be found below ...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [example_action_server-1]: process started with pid [767275]
[INFO] [example_action_client-2]: process started with pid [767277]
[example_action_client-2] [INFO] [1712113230.955569374] [timer_action_client]: Sending goal
[example_action_server-1] [INFO] [1712113230.955830857] [timer_action_server]: Received goal request with duration: 10
[example_action_client-2] [INFO] [1712113230.956075228] [timer_action_client]: Goal accepted by server, waiting for result
[example_action_server-1] [INFO] [1712113230.956085922] [timer_action_server]: Publish feedback: 10
[example_action_client-2] [INFO] [1712113230.956140060] [timer_action_client]: Time remaining: 10
[example_action_server-1] [INFO] [1712113231.956244293] [timer_action_server]: Publish feedback: 9
[example_action_client-2] [INFO] [1712113231.956368792] [timer_action_client]: Time remaining: 9
[example_action_server-1] [INFO] [1712113232.956425288] [timer_action_server]: Publish feedback: 8
[example_action_client-2] [INFO] [1712113232.956613157] [timer_action_client]: Time remaining: 8
[example_action_server-1] [INFO] [1712113233.956611019] [timer_action_server]: Publish feedback: 7
[example_action_client-2] [INFO] [1712113233.956758720] [timer_action_client]: Time remaining: 7
[example_action_server-1] [INFO] [1712113234.956792511] [timer_action_server]: Publish feedback: 6
[example_action_client-2] [INFO] [1712113234.956909937] [timer_action_client]: Time remaining: 6
[example_action_server-1] [INFO] [1712113235.956998755] [timer_action_server]: Publish feedback: 5
[example_action_client-2] [INFO] [1712113235.957130804] [timer_action_client]: Time remaining: 5
[example_action_server-1] [INFO] [1712113236.957162397] [timer_action_server]: Publish feedback: 4
[example_action_client-2] [INFO] [1712113236.957370266] [timer_action_client]: Time remaining: 4
[example_action_server-1] [INFO] [1712113237.957312962] [timer_action_server]: Publish feedback: 3
[example_action_client-2] [INFO] [1712113237.957525743] [timer_action_client]: Time remaining: 3
[example_action_server-1] [INFO] [1712113238.957475049] [timer_action_server]: Publish feedback: 2
[example_action_client-2] [INFO] [1712113238.957614127] [timer_action_client]: Time remaining: 2
[example_action_server-1] [INFO] [1712113239.957606203] [timer_action_server]: Publish feedback: 1
[example_action_client-2] [INFO] [1712113239.957740158] [timer_action_client]: Time remaining: 1
[example_action_server-1] [INFO] [1712113240.957888130] [timer_action_server]: Goal succeeded
[example_action_client-2] [INFO] [1712113240.957994580] [timer_action_client]: Goal succeeded! Time elapsed: Timer completed
```
