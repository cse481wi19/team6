# IF - Intelligent Fetcher (CSE 481C Winter 2019 Team 6)

## Components:

### [Object Detection]:

#### - the [Object Detector](perception/src/object_detector.cpp):
Takes in a `sensor_msgs::PointCloud2& msg` point cloud and publish all detected object 3D bounding boxes (and their orientations) as `visualization_msgs::Marker` to the specified output topic.

Detected object 3D bounding boxes will be of type `visualization_msgs::Marker::CUBE` and their corresponding orientations will be of type `visualization_msgs::Marker::ARROW`. Although they are both published to the `/object_markers` topic, they can be easily distinguished by checking `marker.type`. At the same time, bounding box & orientation of the same object will share the same `marker.id`. (`marker.pose` of the bounding box already has orientation info included; the arrows are mainly for visualizing & debugging purposes)

For more details, check out [ObjectDetector::visualizeBoundingBox](perception/src/object_detector.cpp#L159).

#### - the [Object Detection Node](perception/src/object_detection.cpp):
A constantly running Node that utilizes the Object Detector class.
Takes `/cloud_in` as the input point cloud topic and sets Object Detector's output topic to the `/object_markers`.

### [Arm Motion Planning]:
...

### [Mapping & Navigation]:
...

### [Facial Detection]:
...

### [Master Node]:
...
