from rospy import Subscriber
from keyboard.msg import Key

"""
This requires http://wiki.ros.org/keyboard
"""

class KeyDownSubscriber(Subscriber):

    """Subscriber for keydown events from the keyboard."""

    def __init__(self, key_action_map):
        """Constructor for the KeyDownSubscriber class.

        Note, the class subscribes to the "keyboard/keydown" topic.

        Parameters
        ----------

        key_action_map: dict[Callable]
            Dictionary associcating actions (values of dictionary)
            with keys of the keyboard (keys of dictionary).

        """

        # Set class attributes
        self._key_action_map = key_action_map

        # Initialize class
        super().__init__('keyboard/keydown', Key, self._callback)

    def _callback(self, msg):
        handler = self._key_action_map.get(msg.code)
        if handler:
            handler()
