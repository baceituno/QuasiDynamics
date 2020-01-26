package robot_comm;

public interface robot_HandGetPressureResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_HandGetPressureResponse";
  static final java.lang.String _DEFINITION = "float64 pressure\nint64 ret\nstring msg";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double getPressure();
  void setPressure(double value);
  long getRet();
  void setRet(long value);
  java.lang.String getMsg();
  void setMsg(java.lang.String value);
}