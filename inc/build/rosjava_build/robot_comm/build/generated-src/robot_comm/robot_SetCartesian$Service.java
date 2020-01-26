package robot_comm;

public interface robot_SetCartesian$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetCartesian$Service";
  static final java.lang.String _DEFINITION = "# Service to Set Cartesians\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz\n---\nint64 ret\nstring msg";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
