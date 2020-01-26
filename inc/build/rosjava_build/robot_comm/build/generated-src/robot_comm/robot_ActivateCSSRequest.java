package robot_comm;

public interface robot_ActivateCSSRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_ActivateCSSRequest";
  static final java.lang.String _DEFINITION = "int32 refFrame\nfloat64 refOrient_q0\nfloat64 refOrient_qx\nfloat64 refOrient_qy\nfloat64 refOrient_qz\nint32 softDir\nfloat64 stiffness\nfloat64 stiffnessNonSoftDir\nint32 allowMove\nfloat64 ramp\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  int getRefFrame();
  void setRefFrame(int value);
  double getRefOrientQ0();
  void setRefOrientQ0(double value);
  double getRefOrientQx();
  void setRefOrientQx(double value);
  double getRefOrientQy();
  void setRefOrientQy(double value);
  double getRefOrientQz();
  void setRefOrientQz(double value);
  int getSoftDir();
  void setSoftDir(int value);
  double getStiffness();
  void setStiffness(double value);
  double getStiffnessNonSoftDir();
  void setStiffnessNonSoftDir(double value);
  int getAllowMove();
  void setAllowMove(int value);
  double getRamp();
  void setRamp(double value);
}
