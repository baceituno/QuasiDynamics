package robot_comm;

public interface robot_GetJoints$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_GetJoints$Service";
  static final java.lang.String _DEFINITION = "# Service to get joint angles of the robot\n\n---\nfloat64 j1\nfloat64 j2\nfloat64 j3\nfloat64 j4\nfloat64 j5\nfloat64 j6\nfloat64 j7\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
