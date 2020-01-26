package robot_comm;

public interface robot_GetFK$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_GetFK$Service";
  static final java.lang.String _DEFINITION = "# Service to get the forward kinematics of the robot\n\nfloat64 j1\nfloat64 j2\nfloat64 j3\nfloat64 j4\nfloat64 j5\nfloat64 j6\nfloat64 j7\n\n---\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz\n\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
