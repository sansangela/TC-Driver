#!/usr/bin/env python

import rospy
import rosbag
import csv
from f110_msgs.msg import WpntArray, Wpnt

def csv_to_rosbag(csv_file, output_bag):
    with open(csv_file, 'r') as csvfile:
        header_line = csvfile.readline().strip().replace('# ', '')
        fieldnames = [name.strip() for name in header_line.split(';')]
        csv_reader = csv.DictReader(csvfile, fieldnames=fieldnames, delimiter=';')

        wpnt_array_msg = WpntArray()
        id = 0
        with rosbag.Bag(output_bag, 'w') as bag:
            for row in csv_reader:
                print(row)
                
                wpnt_msg = Wpnt()
                wpnt_msg.id = int(id)
                wpnt_msg.s_m = float(row['s_racetraj_m'])
                # wpnt_msg.d_m = float(row['d_m'])
                wpnt_msg.d_m = float(0)   #TODO
                wpnt_msg.x_m = float(row['x_ref_m'])
                wpnt_msg.y_m = float(row['y_ref_m'])
                wpnt_msg.d_right = float(row['width_right_m'])
                wpnt_msg.d_left = float(row['width_left_m'])
                wpnt_msg.psi_rad = float(row['psi_racetraj_rad'])
                wpnt_msg.kappa_radpm = float(row['kappa_racetraj_radpm'])
                wpnt_msg.vx_mps = float(row['vx_racetraj_mps'])
                wpnt_msg.ax_mps2 = float(row['ax_racetraj_mps2'])
                wpnt_array_msg.wpnts.append(wpnt_msg)

                id += 1
                
            bag.write('/global_waypoints', wpnt_array_msg)  # Replace '/your_topic' with your desired ROS topic


if __name__ == '__main__':
    rospy.init_node('csv_to_rosbag_node')
    csv_file = 'resource/wean_2_raceline_track.csv'  # Replace with the path to your CSV file
    output_bag = 'resource/global_wpnts.bag'  # Replace with the desired output ROS bag file
    
    try:
        csv_to_rosbag(csv_file, output_bag)
        rospy.loginfo("CSV data converted to ROS bag successfully.")
    except Exception as e:
        rospy.logerr("Error converting CSV data to ROS bag: %s", str(e))
