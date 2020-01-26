package robot_comm;

public interface robot_HandGripOutRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_HandGripOutRequest";
  static final java.lang.String _DEFINITION = "#Service to grip the Yumi hand out with certain amount of force\n\nfloat64 handForce\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double getHandForce();
  void setHandForce(double value);
}
