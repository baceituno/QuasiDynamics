package robot_comm;

public interface robot_HandSetSpeedRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_HandSetSpeedRequest";
  static final java.lang.String _DEFINITION = "# Service to set the max Speed of the robot hand.\n\nfloat64 handSpeed\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double getHandSpeed();
  void setHandSpeed(double value);
}
