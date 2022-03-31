import rospy
import tf2_ros
import tf_conversions
import numpy as np
from numpy.typing import ArrayLike
from geometry_msgs.msg import TransformStamped, Transform
from typing import Optional, Tuple, Union

class TfInterface:

    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

    def set_tf(self,
               parent_frame_id: str,
               child_frame_id: str,
               pos: ArrayLike,
               orientation: Optional[ArrayLike] = [0, 0, 0, 1],
               eul_deg: Optional[bool] = False
               ) -> None:
        """Set the position and orientation of a child frame with respect to a parent frame."""

        # Handle euler angles
        if len(orientation) == 3:
            if eul_deg:
                eul = np.deg2rad(orientation)
            else:
                eul = np.array(orientation)
            quat = tf_conversions.transformations.quaternion_from_euler(eul)
        else:
            quat = np.array(orientation)

        # Pack transform message and broadcast
        self.tf_broadcaster.sendTransform(self.pack_tf_msg(parent_frame_id, child_frame_id, pos, orientation))

    def get_tf_msg(self, parent_frame_id: str, child_frame_id: str) -> Union[TransformStamped, None]:
        msg = None
        try:
            msg = self.tf_buffer.lookup_transform(parent_frame_id, child_frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(f'Did not recieve the frame {child_frame_id} in {parent_frame_id}!')
        return msg

    def get_tf(self, parent_frame_id: str, child_frame_id: str) -> Tuple[ArrayLike]:
        """Return position and orientation of child frame with respect to a parent frame."""
        msg = self.get_tf_msg(parent_frame_id, child_frame_id)
        if msg is not None:
            pos = self.msg_to_pos(msg)
            orientation = self.msg_to_quat(msg)
        else:
            pos = None
            orientation = None
        return pos, orientation

    @staticmethod
    def pack_tf_msg(parent_frame_id: str, child_frame_id: str, pos: ArrayLike, rot: ArrayLike) -> TransformStamped:
        msg = TransformStamped(transform=TfInterface.pack_tf(pos, rot))
        msg.header.frame_id = parent_frame_id
        msg.child_frame_id = child_frame_id
        msg.header.stamp = rospy.Time.now()
        return msg

    @staticmethod
    def pack_tf(pos: ArrayLike, rot: ArrayLike):
        msg = Transform()
        msg.translation.x = pos[0]
        msg.translation.y = pos[1]
        msg.translation.z = pos[2]
        msg.rotation.x = rot[0]
        msg.rotation.y = rot[1]
        msg.rotation.z = rot[2]
        msg.rotation.w = rot[3]
        return msg

    @staticmethod
    def tf_msg_to_pos(msg: Transform) -> ArrayLike:
        return np.array([msg.translation.x, msg.translation.y, msg.translation.z])

    @staticmethod
    def msg_to_pos(msg: TransformStamped) -> ArrayLike:
        return TfInterface.tf_msg_to_pos(msg.transform)

    @staticmethod
    def tf_msg_to_quat(msg: Transform) -> ArrayLike:
        return np.array([msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w])

    @staticmethod
    def msg_to_quat(msg: TransformStamped) -> ArrayLike:
        return TfInterface.tf_msg_to_quat(msg.transform)

    @staticmethod
    def tf_msg_to_eul(msg: Transform) -> ArrayLike:
        return np.asarray(tf_conversions.transformations.euler_from_quaternion(TfInterface.tf_msg_to_quat(msg)))

    @staticmethod
    def msg_to_eul(msg: TransformStamped) -> ArrayLike:
        return TfInterface.tf_msg_to_eul(msg.transform)

    @staticmethod
    def msg_to_pos_quat(msg: TransformStamped) -> Tuple[ArrayLike]:
        return self.msg_to_pos(msg), self.msg_to_quat(msg)

    @staticmethod
    def msg_to_pos_eul(msg: TransformStamped) -> Tuple[ArrayLike]:
        return self.msg_to_pos(msg), self.msg_to_eul(msg)

    @staticmethod
    def msg_to_matrix(msg: TransformStamped) -> ArrayLike:
        p = TfInterface.msg_to_pos(msg)
        q = TfInterface.msg_to_quat(msg)
        return TfInterface.pos_quat_to_matrix(p, q)

    @staticmethod
    def pos_quat_to_matrix(pos: ArrayLike, quat: Optional[ArrayLike] = [0,0,0,1]) -> ArrayLike:
        T = tf_conversions.transformations.quaternion_matrix(quat)
        T[:3, 3] = pos
        return T
