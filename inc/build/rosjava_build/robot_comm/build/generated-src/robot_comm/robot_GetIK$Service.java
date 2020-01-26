package robot_comm;

public interface robot_GetIK$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_GetIK$Service";
  static final java.lang.String _DEFINITION = "# Service to get inverse kinematics of the robot\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz\nfloat64 robotAngle\n\n---\nfloat64 j1\nfloat64 j2\nfloat64 j3\nfloat64 j4\nfloat64 j5\nfloat64 j6\nfloat64 j7\nfloat64 errorNum\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
