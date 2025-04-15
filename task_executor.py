#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class TaskExecutor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('task_executor', anonymous=True)
        
        # Setup subscribers
        self.task_sub = rospy.Subscriber(
            "/task_plan",
            String,
            self.task_callback
        )
        
        # Setup publishers
        self.pose_pub = rospy.Publisher(
            "/robot_arm/command",
            PoseStamped,
            queue_size=1
        )
        
        # Task state
        self.current_task = None
        self.task_steps = []
        self.current_step = 0
        
        rospy.loginfo("Task executor initialized")

    def task_callback(self, msg):
        """Process new task plan"""
        try:
            # Parse task plan
            task_plan = json.loads(msg.data)
            self.task_steps = task_plan
            self.current_step = 0
            self.execute_next_step()
            
        except Exception as e:
            rospy.logerr(f"Error processing task plan: {e}")

    def execute_next_step(self):
        """Execute the next step in the task plan"""
        if self.current_step >= len(self.task_steps):
            rospy.loginfo("Task completed!")
            return
            
        step = self.task_steps[self.current_step]
        rospy.loginfo(f"Executing step {self.current_step + 1}: {step['subtask']}")
        
        if step['skill'] == 'Pick':
            self.execute_pick(step)
        elif step['skill'] == 'Place':
            self.execute_place(step)
        else:
            rospy.logerr(f"Unknown skill: {step['skill']}")
        
        self.current_step += 1
        # Execute next step after a delay
        rospy.Timer(rospy.Duration(2.0), self.execute_next_step, oneshot=True)

    def execute_pick(self, step):
        """Execute a pick operation"""
        try:
            # Create pick pose (you'll need to get the actual position from block tracking)
            pick_pose = PoseStamped()
            pick_pose.header.frame_id = "base_link"
            pick_pose.header.stamp = rospy.Time.now()
            pick_pose.pose.position.x = 0.5  # TODO: Get from block tracking
            pick_pose.pose.position.y = 0.0  # TODO: Get from block tracking
            pick_pose.pose.position.z = 0.1  # TODO: Get from block tracking
            pick_pose.pose.orientation.w = 1.0
            
            # Publish pick pose
            self.pose_pub.publish(pick_pose)
            
        except Exception as e:
            rospy.logerr(f"Error executing pick: {e}")

    def execute_place(self, step):
        """Execute a place operation"""
        try:
            # Create place pose (you'll need to get the actual position from task)
            place_pose = PoseStamped()
            place_pose.header.frame_id = "base_link"
            place_pose.header.stamp = rospy.Time.now()
            place_pose.pose.position.x = 0.5  # TODO: Get from task
            place_pose.pose.position.y = 0.2  # TODO: Get from task
            place_pose.pose.position.z = 0.1  # TODO: Get from task
            place_pose.pose.orientation.w = 1.0
            
            # Publish place pose
            self.pose_pub.publish(place_pose)
            
        except Exception as e:
            rospy.logerr(f"Error executing place: {e}")

def main():
    try:
        executor = TaskExecutor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 