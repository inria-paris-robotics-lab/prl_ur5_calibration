import rospy
import tf
from geometry_msgs.msg import Transform, Vector3, Quaternion
from fiducial_msgs.msg import FiducialTransformArray

rospy.init_node('Test', log_level=rospy.INFO)

# # Get approximate transform of QR
# tf_listener = tf.TransformListener()
# from_f = "/left_camera_color_optical_frame"
# to_f = "/prl_ur5_base"
# tf_listener.waitForTransform(from_f, to_f, rospy.Time(0), rospy.Duration(4.0))
# trans, rot = tf_listener.lookupTransform(from_f, to_f, rospy.Time(0))
# cMo = Transform(translation=Vector3(*trans), rotation=Quaternion(*rot))

# Get approximate transform of QR
msg = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
cMo = msg.transforms[0].transform

# Init tracker
from visp_tracker.srv import Init, InitRequest
rospy.wait_for_service('init_tracker')
srv_init_tracker = rospy.ServiceProxy('init_tracker', Init)

init_req = InitRequest()
init_req.initial_cMo = cMo

init_req.tracker_param.angle_appear = 75
init_req.tracker_param.angle_disappear = 75

init_req.moving_edge.mask_size = 5
init_req.moving_edge.range = 10
init_req.moving_edge.threshold = 5000
init_req.moving_edge.mu1 = 0.5
init_req.moving_edge.mu2 = 0.5
init_req.moving_edge.sample_step = 4
# init_req.moving_edge.strip =  ??
init_req.moving_edge.first_threshold = 0.01 # ??

init_req.klt_param.max_features = 10000
init_req.klt_param.window_size = 5
init_req.klt_param.quality = 0.05
init_req.klt_param.min_distance = 20
init_req.klt_param.harris = 0.01
init_req.klt_param.size_block = 3
init_req.klt_param.pyramid_lvl = 3
init_req.klt_param.mask_border = 0

init_success = srv_init_tracker(init_req)

print(init_success)