import rclpy
from rclpy.node import Node
from threading import Event
import numpy as np
import pinocchio as pin
import time 
from std_msgs.msg import Int8
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point, Quaternion
from rclpy.executors import SingleThreadedExecutor

def transform_to_se3(trans):
    return pin.XYZQUATToSE3([trans.translation.x, trans.translation.y, trans.translation.z,
                             trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])

def input_accept(prompt):
    choice = input(prompt + " [y]/n: ")
    if(choice == "" or choice.lower()[0] == 'y'):
        return True
    return False


def compute_barycenter(pose_list, fp_iter=100, callback=None):
    """
    bi invariant barycenter
    """
    guess = pose_list[0]
    if callback is not None:
        callback(guess)

    for _ in range(fp_iter):
        guess =  pin.exp(
            pin.Motion(
                np.stack([pin.log(p * guess.inverse()).np for p in pose_list], axis=0).mean(axis=0)
            )
        ) * guess
        if callback is not None:
            callback(guess)
    return guess

def compute_covariance(barycenter, pose_list):
    N = len(pose_list)
    logs_l_riem = np.stack([
        np.concatenate([
            pin.log3((barycenter.inverse() * p).rotation),
            (barycenter.inverse() * p).translation
        ], axis=0)
        for p in pose_list
    ], axis=0)
    V_l_riem = np.einsum('ib,ic->bc', logs_l_riem, logs_l_riem) / (N-1)
    return V_l_riem

def visp_meas_filter(node, n, tracker_topic):
    """
    Wait for n consecutive measures in Tracking Mode and return the last one.

    (Ensure that the measure is not a false positive)

    Parameters:
    -----------
        n (int): number of consecutive measurements to wait for.
        tracker_topic (str): prefix topic to subscribe to.

    Returns:
    --------
        success (bool): True if the average was successful, False otherwise.
        pose (Pose): the last measure of the n consecutives.
        stamp (Time): Timestamp of the last measure.
    """
    pose = None
    for i in range(n):
        status = wait_for_message(node, tracker_topic + "/status", Int8)
        if status is None: # Timeout
            node.get_logger().warn("Timeout when waiting for tracker status")
            return False, None, None
        if(status.data != 3): # Not tracking
            node.get_logger().warn("Tracker not in tracking mode")
            return False, None, None
        if i == n-1:
            transf_marker = wait_for_message(node, tracker_topic + "/object_position", PoseStamped)
            pose = transf_marker.pose
            stamp = transf_marker.header.stamp
    return True, pose, stamp


import rclpy
from rclpy.executors import SingleThreadedExecutor

def wait_for_message(calling_node, topic_name, topic_type, timeout_sec=5.0):
    """
    Attend un message sur un topic, depuis l'intérieur d'un noeud existant.
    
    Args:
        calling_node (Node): Le noeud qui appelle cette fonction.
        topic_name (str): Le nom du topic.
        topic_type (MsgType): Le type du message.
        timeout_sec (float): Timeout en secondes.

    Returns:
        Le message reçu, ou None si timeout.
    """
    received_msgs = []
    
    def callback(msg):
        calling_node.get_logger().info(f"Message received on {topic_name}")
        received_msgs.append(msg)

    # Crée un abonné temporaire sur le noeud appelant
    subscription = calling_node.create_subscription(topic_type, topic_name, callback, 10)
    
    # Attend le message en faisant tourner l'executor du noeud appelant
    start_time = calling_node.get_clock().now()
    while rclpy.ok() and (calling_node.get_clock().now() - start_time).nanoseconds / 1e9 < timeout_sec:
        rclpy.spin_once(calling_node, timeout_sec=0.1)
        if received_msgs:
            break # Message reçu, on sort de la boucle
    
    # Nettoie l'abonné temporaire
    calling_node.destroy_subscription(subscription)
    
    if received_msgs:
        return received_msgs[0]
    else:
        return None