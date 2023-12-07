import rosbag

# path = '/root/catkin_ws/src/TC-Driver/csv_to_rosbag/resource/global_wpnts.bag'
path = '/root/catkin_ws/src/TC-Driver/F110_ROS_Simulator/maps/f/global_wpnts.bag'
bag = rosbag.Bag(path)
# for topic, msg, t in bag.read_messages(topics=['/global_waypoints']):
for topic, msg, t in bag.read_messages():
    print(topic)
    with open('resource/f_output.txt', 'w') as file:
        print(msg, file=file)
bag.close()