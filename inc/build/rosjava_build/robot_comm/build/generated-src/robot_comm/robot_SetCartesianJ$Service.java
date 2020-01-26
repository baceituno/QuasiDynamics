package robot_comm;

public interface robot_SetCartesianJ$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetCartesianJ$Service";
  static final java.lang.String _DEFINITION = "# Service to Set Cartesian coordinates, but to move there with a joint move\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz\n---\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
