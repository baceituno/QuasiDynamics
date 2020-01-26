package robot_comm;

public interface robot_HandSetForceRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_HandSetForceRequest";
  static final java.lang.String _DEFINITION = "# Service to set the max force of the robot hand.\n\nfloat64 handForce\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double getHandForce();
  void setHandForce(double value);
}
