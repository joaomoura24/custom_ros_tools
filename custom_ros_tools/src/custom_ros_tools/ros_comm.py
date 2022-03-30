import rospy
from typing import Callable, Optional, Union
from types import TracebackType
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE
from ik_ros.srv import JointNames, JointNamesResponse


_error_handler_type = Optional[
    Union[
        Callable[[type, type, Exception, TracebackType], None],
        None
    ]
]

class ToggleService(rospy.Service):

    def __init__(self,
                 name: str,
                 enable_handler: Callable[..., bool],
                 disable_handler: Callable[..., bool],
                 buff_size: Optional[int] = DEFAULT_BUFF_SIZE,
                 error_handler: _error_handler_type = None):
        super(ToggleService, self).__init__(name, SetBool, self.toggle,
                                            buff_size=buff_size, error_handler=error_handler)
        self.enable_handler = enable_handler
        self.disable_handler = disable_handler

    def toggle(self, req: SetBoolRequest) -> SetBoolResponse:
        if req.data:
            success, message = self.enable_handler()
        else:
            success, message = self.disable_handler()
        return SetBoolResponse(success=success, message=message)


def get_srv_handler(
        srv_name: str,
        srv_type: type,
        presistent: Optional[bool] = False,
        headers: Optional[Union[dict, None]] = None) -> rospy.ServiceProxy:
    handler = None
    rospy.wait_for_service(srv_name)
    try:
        handler = rospy.ServiceProxy(srv_name, srv_type, persistent=persistent, headers=headers)
    except Exception as err:
        rospy.logerr(f"failed to retrieve service proxy: {str(err)}")
    return handler
