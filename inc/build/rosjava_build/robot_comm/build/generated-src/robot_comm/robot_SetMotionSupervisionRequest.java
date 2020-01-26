package robot_comm;

public interface robot_SetMotionSupervisionRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetMotionSupervisionRequest";
  static final java.lang.String _DEFINITION = "# Service to Set the Motion Supervision of the robot\n# Accepts a valeu between 1 and 300\n# Recommended is around 50\n\nfloat64 supervision\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double getSupervision();
  void setSupervision(double value);
}
