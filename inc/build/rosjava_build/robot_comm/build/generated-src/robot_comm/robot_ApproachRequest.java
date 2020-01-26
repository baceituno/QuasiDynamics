package robot_comm;

public interface robot_ApproachRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_ApproachRequest";
  static final java.lang.String _DEFINITION = "geometry_msgs/Pose pose\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
}
