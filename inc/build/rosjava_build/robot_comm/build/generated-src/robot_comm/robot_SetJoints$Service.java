package robot_comm;

public interface robot_SetJoints$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetJoints$Service";
  static final java.lang.String _DEFINITION = "# Service to Set Joints\n\nfloat64 j1\nfloat64 j2\nfloat64 j3\nfloat64 j4\nfloat64 j5\nfloat64 j6\nfloat64 j7\n---\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
