package robot_comm;

public interface robot_CartesianLog extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_CartesianLog";
  static final java.lang.String _DEFINITION = "string date\nstring time\nfloat64 timeStamp\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.lang.String getDate();
  void setDate(java.lang.String value);
  java.lang.String getTime();
  void setTime(java.lang.String value);
  double getTimeStamp();
  void setTimeStamp(double value);
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getZ();
  void setZ(double value);
  double getQ0();
  void setQ0(double value);
  double getQx();
  void setQx(double value);
  double getQy();
  void setQy(double value);
  double getQz();
  void setQz(double value);
}
