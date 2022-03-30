import rospy
import tf2_ros
import tf_conversions
import numpy as np
from numpy.typing import ArrayLike
from geometry_msgs.msg import TransformStamped
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
    def pack_tf_msg(parent_frame_id: str, child_frame_id: str, pos: ArrayLike, rotation: ArrayLike) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = parent_frame_id
        msg.child_frame_id = child_frame_id
        msg.transform.translation.x = pos[0]
        msg.transform.translation.y = pos[1]
        msg.transform.translation.z = pos[2]
        msg.transform.rotation.x = rotation[0]
        msg.transform.rotation.y = rotation[1]
        msg.transform.rotation.z = rotation[2]
        msg.transform.rotation.w = rotation[3]
        return msg

    @staticmethod
    def msg_to_pos(msg: TransformStamped) -> ArrayLike:
        return np.array([getattr(msg.transform.translation, d) for d in 'xyz'])

    @staticmethod
    def msg_to_quat(msg: TransformStamped) -> ArrayLike:
        return np.array([getattr(msg.transform.rotation, d) for d in 'xyzw'])

    @staticmethod
    def msg_to_eul(msg: TransformStamped) -> ArrayLike:
        return np.asarray(tf_conversions.transformations.euler_from_quaternion(TfInterface.msg_to_quat(msg)))

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
