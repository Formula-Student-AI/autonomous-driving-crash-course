# map_lib

`map_lib` is a robust and comprehensive library designed to handle all your map and cone-related needs. Whether you're constructing cone objects, creating map objects or associating observations with map landmarks, `map_lib` has got you covered.

## Features

### Cone Construction (`cone.hpp`)
Create cone objects defined by their colour, position and optional covariance. 

### Map Creation (`map.hpp`)
Build map objects, which are essentially collections of cones. The module includes functions like `add` and `remove` for easy manipulation of cones within the map.

### Map Capture (`map_capture.hpp`)
`MapCallback()` method receives a map message and save it as a CSV file without modify any of the object's member variables. There are 2 parameters `save_path_` and `map_sub_`.

#### `map_sub_`
This object is used to subscribe to a topic where map messages are published. When a message is published to this topic, the MapCallback method is called with the message as its argument.

### Map Publisher (`map_publisher.hpp`)
`GetSelectedMapPath()` method locates the path to the selected map in the `setup.json` file when using the EUFS Testing App. This module requires three arguments: the path to the map file, `frame_id`, and `frequency` which is the frequency at which it is published. `GetSelectedMapPath()` is called when the `map_path` argument is empty. When publishing, `MapPublisher()` takes `map_msg_` which is a map message and publish the map.

### CSV Writer (`writer.hpp`)
Create a CSV file and write the characteristics of the cone objects as strings. By using `SaveCones()` method. Inside `SaveCones()` methods, other smaller methods are used to get information of a cone and change the cone object to a characteristic string.

### CSV Reader (`reader.hpp`)
Reads the CSV file and constructs a map using `LoadCones()`. This uses a series of methods to check the header of the file and cone colours. It then translates the track relative to the car's start position.

### Association (`association.hpp`)
Create an association object that matches observations to map landmarks. This module fills a container based on three parameters: `begin` (Start of the range), `end` (End of the range), and `step` (Step size going from begin to end). Main use case for this is to fill the `Association::unmatched` attribute with the indices of the observations as std::iota cannot be used in this case.

### Single Association (`single.hpp`)
Create a Single Association Object and takes a list of observations and a map of cones objects as argument, iterates over each observation and for each observation. It iterates over the map to find the closest conewithin the maximum distance. If a match is found, it updates the best metric and the best map index.

## Input / Output Topics

### Input Topics

To create the Cone Objects, the user must provide the path to the CSV file which comprises:
 * `cone`: the x-y coordinates, cone direction, variance, and covariance data.
 * `car_start` (optional): x-y coordinates, and rotation in radians.
The second parameter asked from the user is the frame-id, which is set to "map" as the default value if frame-id is not specified.

### Output Topics

`map_lib` outputs five arrays: blue cones, yellow cones, orange cones, big orange cones, and unknown cones.

Topic Name                | Message Type
------------------------- | ------------------------
`ConeArrayWithCovariance` | `eufs_msgs::msg::ConeArrayWithCovariance`
......                    | .......
