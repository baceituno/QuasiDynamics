package robot_comm;

public interface robot_GetJointsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_GetJointsResponse";
  static final java.lang.String _DEFINITION = "float64 j1\nfloat64 j2\nfloat64 j3\nfloat64 j4\nfloat64 j5\nfloat64 j6\nfloat64 j7\nint64 ret\nstring msg";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double getJ1();
  void setJ1(double value);
  double getJ2();
  void setJ2(double value);
  double getJ3();
  void setJ3(double value);
  double getJ4();
  void setJ4(double value);
  double getJ5();
  void setJ5(double value);
  double getJ6();
  void setJ6(double value);
  double getJ7();
  void setJ7(double value);
  long getRet();
  void setRet(long value);
  java.lang.String getMsg();
  void setMsg(java.lang.String value);
}
