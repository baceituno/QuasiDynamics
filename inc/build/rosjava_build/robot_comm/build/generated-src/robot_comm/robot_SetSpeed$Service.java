package robot_comm;

public interface robot_SetSpeed$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetSpeed$Service";
  static final java.lang.String _DEFINITION = "# Service to Set the max Speed of the robot.\n\nfloat64 tcp  # mm/s\nfloat64 ori  # deg/s\n---\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
