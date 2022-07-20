import rospy
from typing import List, Optional
from sensor_msgs.msg import JointState

def resolve_joint_order(msg: JointState, joint_names: List[str], ns: str = '') -> JointState:

    # Initialize output joint state message
    out = JointState(header=msg.header, name=joint_names)

    # Iterate over joint names
    for name in joint_names:

        # Check if name appears in the input joint state message
        if name in msg.name:
            # True -> name appears in joint state message

            # Get index of name in input joint state message
            if ns:
                name = f'{ns}/{name}'
            idx = msg.name.index(name)

            # Append position
            try:
                out.position.append(msg.position[idx])
            except IndexError:
                pass

            # Append velocity
            try:
                out.velocity.append(msg.velocity[idx])
            except IndexError:
                pass

            # Append effort
            try:
                out.effort.append(msg.effort[idx])
            except IndexError:
                pass

    return out

def get_joint_state(topic='joint_states', timeout=None, joint_names=None, ns=''):
    msg = rospy.wait_for_message(topic, JointState, timeout=timeout)
    if joint_names:
        msg = resolve_joint_order(msg, joint_names, ns=ns)
    return msg

class JointStatePublisher(rospy.Publisher):

    def __init__(self, topic_name, joint_names, queue_size=1):
        self._msg = JointState(name=joint_names)
        super().__init__(topic_name, JointState, queue_size=queue_size)

    def publish(self, q):
        self._msg.position = q
        self._msg.header.stamp = rospy.Time.now()
        super().publish(self._msg)
