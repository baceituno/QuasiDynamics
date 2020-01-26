package robot_comm;

public interface robot_ForceLog extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_ForceLog";
  static final java.lang.String _DEFINITION = "string date\nstring time\nfloat64 timeStamp\nfloat64 fx\nfloat64 fy\nfloat64 fz\nfloat64 tx\nfloat64 ty\nfloat64 tz\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.lang.String getDate();
  void setDate(java.lang.String value);
  java.lang.String getTime();
  void setTime(java.lang.String value);
  double getTimeStamp();
  void setTimeStamp(double value);
  double getFx();
  void setFx(double value);
  double getFy();
  void setFy(double value);
  double getFz();
  void setFz(double value);
  double getTx();
  void setTx(double value);
  double getTy();
  void setTy(double value);
  double getTz();
  void setTz(double value);
}
