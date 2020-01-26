package robot_comm;

public interface robot_HandMoveTo$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_HandMoveTo$Service";
  static final java.lang.String _DEFINITION = "# Service to set the robot hand to Move To a position\n\nfloat64 handPose\n---\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
