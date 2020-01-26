package robot_comm;

public interface robot_DeactivateCSSRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_DeactivateCSSRequest";
  static final java.lang.String _DEFINITION = "geometry_msgs/Pose ToPose\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  geometry_msgs.Pose getToPose();
  void setToPose(geometry_msgs.Pose value);
}
