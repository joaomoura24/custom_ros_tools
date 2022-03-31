from typing import List, Optional
from sensor_msgs.msg import JointState

def resolve_joint_order(msg: JointState, joint_names: List[str]) -> JointState:
    out = JointState(name=joint_names)
    for name in joint_names:
        if name in msg.name:
            idx = msg.name.index(name)
            out.position.append(msg.position[idx])
            out.velocity.append(msg.velocity[idx])
            out.effort.append(msg.effort[idx])
        else:
            out.position.append(0.0)
            out.velocity.append(0.0)
            out.effort.append(0.0)
    return out
