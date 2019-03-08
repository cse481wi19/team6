# IF - Intelligent Fetcher

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
#### - the [Arm Motion Planner](robot_api/src/robot_api/arm_motion_planner.py)
MainFunction: `pick_up(self, obj_marker, obs_marker_list=None):` 
obj_marker is type `visualization_msgs::Marker` and obs_marker_list should be a list of `visualization_msgs::Marker`, but it default to None. (planned but not implemented:) The function is expected to return 0 if sucessfully pick up the object, 1 if the planning failed.

#### Usage:
1. Run `roscore`
2. Run `roslaunch fetch_moveit_config move_group.launch`
3. In simulation: Run `roslaunch fetch_gazebo playground.launch`, `rosrun rviz rviz`.
4. Run `rosrun applications publish_saved_cloud.py <bag file path or empty on real robot>` (e.g. `rosrun applications publish_saved_cloud.py /home/team6/catkin_ws/src/cse481wi19/saved_clouds/real_floor_0.bag`)
5. Run perception `rosrun perception object_detection /cloud_in:=/mock_point_cloud` or `rosrun perception object_detection /cloud_in:=/head_camera_depth` on real robot.

#### - [Example Script](applications/scripts/arm_motion_planner_demo.py)
Run `rosrun applications arm_motion_planner_demo.py`

### [Mapping & Navigation]:
#### - the [Navigator](/src/map_annotator/src/map_annotator/navigation.py)
API:
class Navigator()
  current_pose(): get the current position of the robot in map frame.
    return: current a PoseStamped of the current position with frame_id map.
  goto(goal=PoseStamped, timeout=secs): goto a given target position in the given timeout time
    params: a PoseStamped with a position and a header with a frame_id
            a integer with the seconds of time we want to wait
    return: True if it gets to the given goal in the given time
            False otherwise

#### Usage:
1. Run `roslaunch applications nav_rviz.launch` (See Note)
2. Create Navigator()
3. Call current_pose() and goto(goal=PoseStamped, timeout=secs)

#### Note:
nav_rviz.launch will launch the map recorded in the real world.
If you want to test on simulator, run `roslaunch applications nav_rviz_smi.launch` for 1.

...

### [Facial Detection]:
...

### [Master Node]:
...
