package robot_comm;

public interface robot_SetAcc$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetAcc$Service";
  static final java.lang.String _DEFINITION = "# Service to Set the max Acceleration of the robot.\n\nfloat64 acc  # mm/s^2\nfloat64 deacc  # mm/s^2\n---\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
