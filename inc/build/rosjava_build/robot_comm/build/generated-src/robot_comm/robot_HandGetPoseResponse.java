package robot_comm;

public interface robot_HandGetPoseResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_HandGetPoseResponse";
  static final java.lang.String _DEFINITION = "float64 pose\nint64 ret\nstring msg";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double getPose();
  void setPose(double value);
  long getRet();
  void setRet(long value);
  java.lang.String getMsg();
  void setMsg(java.lang.String value);
}
