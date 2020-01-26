package robot_comm;

public interface robot_HandGripOut$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_HandGripOut$Service";
  static final java.lang.String _DEFINITION = "#Service to grip the Yumi hand out with certain amount of force\n\nfloat64 handForce\n---\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
