package robot_comm;

public interface robot_HandGripOutResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_HandGripOutResponse";
  static final java.lang.String _DEFINITION = "int64 ret\nstring msg";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  long getRet();
  void setRet(long value);
  java.lang.String getMsg();
  void setMsg(java.lang.String value);
}
