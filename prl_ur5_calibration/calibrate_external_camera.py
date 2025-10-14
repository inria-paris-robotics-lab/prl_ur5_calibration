#!/usr/bin/env python
import os
import rclpy
from rclpy.node import Node
import tf_transformations as transformations
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

from prl_ur5_calibration.utils import visp_meas_filter

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        # Declare parameters
        self.declare_parameter("camera_name", "camera")
        self.declare_parameter("tracker_node", "visp_auto_tracker")
        self.declare_parameter("sample_nb", 20)
        self.declare_parameter("run_loop", False)


        # self.timer_marker = self.create_timer(1.0, self.get_marker)
        # self.timer_run = self.create_timer(5.0, self.run)

         # Variables d'état pour la logique de filtrage
        self.consecutive_tracking_count = 0
        self.last_valid_pose_stamped = None
        self.pose_marker = None
        
        # Récupérer les noms de topics une seule fois
        tracker_node_name = self.get_parameter("tracker_node").get_parameter_value().string_value
        status_topic = f"/{tracker_node_name}/status"
        pose_topic = f"/{tracker_node_name}/object_position"

        self.get_logger().info(f"Subscribing to {status_topic} and {pose_topic}")

        # Abonnés (subscribers) persistants
        self.status_subscriber = self.create_subscription(Int8, status_topic, self.status_callback, 10)
        self.pose_subscriber = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)

        self.main_timer = self.create_timer(1.0, self.calibration_step)
    
    def calibration_step(self):
        """
        Cette fonction est maintenant la boucle principale.
        Elle essaie d'obtenir une mesure. Si elle réussit, elle effectue la calibration et s'arrête.
        Sinon, elle attend que le timer la rappelle pour réessayer.
        """
        self.get_logger().info(f"Checking for stable tracking... (Consecutive count: {self.consecutive_tracking_count})")
        
        sample_nb_required = self.get_parameter("sample_nb").get_parameter_value().integer_value

        # Condition de succès : avons-nous assez de mesures consécutives ET une pose valide ?
        if self.consecutive_tracking_count >= sample_nb_required and self.last_valid_pose_stamped is not None:
            self.get_logger().info(f"Success! Acquired {self.consecutive_tracking_count} consecutive tracking statuses.")

            # Si on arrive ici, c'est que la mesure a réussi !
            self.get_logger().info("Stable marker pose acquired! Proceeding to calibration.")

            # Utilise la dernière pose reçue, qui est garantie d'être pendant une période de tracking stable
            pose = self.last_valid_pose_stamped.pose
            
            # Étape 2: Exécuter la logique de calibration (le contenu de votre ancienne fonction run)
            # Inverse the transformation from camera-to-marker to marker-to-camera
            quat = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            trans = pose.position.x, pose.position.y, pose.position.z

            quat_mat = transformations.quaternion_matrix(quat)
            trans_mat = transformations.translation_matrix(trans)
            concat_mat = transformations.concatenate_matrices(trans_mat, quat_mat)
            marker_T_camera = transformations.inverse_matrix(concat_mat)
            
            # Les transformations suivantes sont correctes
            marker_initial_pos = [0.0, 0.0, 0.008]
            marker_initial_quat = [0.0, 0.0, 0.0, 1.0]
            init_quat_mat = transformations.quaternion_matrix(marker_initial_quat)
            init_trans_mat = transformations.translation_matrix(marker_initial_pos)
            world_T_marker = transformations.concatenate_matrices(init_trans_mat, init_quat_mat)
            world_T_camera = np.matmul(world_T_marker, marker_T_camera)
            res_trans = transformations.translation_from_matrix(world_T_camera)
            res_quat = transformations.quaternion_from_matrix(world_T_camera)

            # User-friendly print the result
            camera_name = self.get_parameter("camera_name").get_parameter_value().string_value
            sample_nb = self.get_parameter("sample_nb").get_parameter_value().integer_value
            pretty_str = self.make_pretty_str(sample_nb, camera_name, res_trans, res_quat)
            self.get_logger().info(pretty_str)

            # Réinitialiser pour la prochaine boucle (si run_loop est True)
            self.consecutive_tracking_count = 0
            self.last_valid_pose_stamped = None
            
            # Étape 3: Arrêter le noeud si run_loop est False
            if not self.get_parameter("run_loop").get_parameter_value().bool_value:
                self.get_logger().info(f"Done calibrating camera {camera_name}. Shutting down.")
                self.main_timer.cancel()
                self.destroy_node()
        else:
            self.get_logger().warn("Condition not met (not enough consecutive tracking messages). Waiting for more consecutive tracking messages.")
    def status_callback(self, msg):
        """Ce callback est appelé à chaque nouveau message de status."""
        if msg.data == 3:  # 3 = "Tracking"
            self.consecutive_tracking_count += 1
            if self.consecutive_tracking_count == self.get_parameter("sample_nb").get_parameter_value().integer_value-1:
                self.get_logger().info(f"Reached required consecutive tracking count: {self.consecutive_tracking_count}")
                self.last_valid_pose_stamped = self.pose_marker
        else:
            # Si le tracker n'est pas en mode "Tracking", on réinitialise le compteur.
            self.get_logger().warn("Tracker is not in tracking mode (status != 3). Resetting count.")
            self.consecutive_tracking_count = 0

    def pose_callback(self, msg):
        """Ce callback met simplement à jour la dernière pose reçue."""
        self.pose_marker = msg
    
    def make_pretty_str(self, sample_nb, camera_name, trans, rot):
        euler = euler_from_quaternion(rot)
        return F"""\n
                ############################################
                # Generated from calibrate_external_camera #
                ############################################
                # For camera : {camera_name}
                # (Filtered on {sample_nb} samples)

                pose:
                    x: {trans[0]:.6f}
                    y: {trans[1]:.6f}
                    z: {trans[2]:.6f}
                    roll:  {euler[0]:.3f}
                    pitch: {euler[1]:.3f}
                    yaw:   {euler[2]:.3f}

                    (degrees :
                    roll:  {euler[0]*180/3.1415926:.1f} deg
                    pitch: {euler[1]*180/3.1415926:.1f} deg
                    yaw:   {euler[2]*180/3.1415926:.1f} deg
                    )
                """
def main(args=None):

    rclpy.init(args=args)
    calibration_node = CameraCalibrationNode()

    calibration_node.get_logger().info(f"Starting calibration for camera {calibration_node.get_parameter('camera_name').get_parameter_value().string_value}, using tracker node {calibration_node.get_parameter('tracker_node').get_parameter_value().string_value}, averaging on {calibration_node.get_parameter('sample_nb').get_parameter_value().integer_value} samples")
    # Start the calibration
    rclpy.spin(calibration_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
