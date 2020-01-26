package robot_comm;

public interface robot_HandGripInRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_HandGripInRequest";
  static final java.lang.String _DEFINITION = "#Service to grip the Yumi hand in with certain amount of force\n\nfloat64 handForce\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double getHandForce();
  void setHandForce(double value);
}
