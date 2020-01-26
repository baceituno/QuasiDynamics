package robot_comm;

public interface robot_JointsLog extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_JointsLog";
  static final java.lang.String _DEFINITION = "string date\nstring time\nfloat64 timeStamp\nfloat64 j1\nfloat64 j2\nfloat64 j3\nfloat64 j4\nfloat64 j5\nfloat64 j6\nfloat64 j7\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.lang.String getDate();
  void setDate(java.lang.String value);
  java.lang.String getTime();
  void setTime(java.lang.String value);
  double getTimeStamp();
  void setTimeStamp(double value);
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
}
