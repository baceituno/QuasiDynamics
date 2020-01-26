package robot_comm;

public interface robot_SetInertiaRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetInertiaRequest";
  static final java.lang.String _DEFINITION = "# Service to Set the Inertia of the tool of the robot in cartesian coordinates\n\nfloat64 m\nfloat64 cgx\nfloat64 cgy\nfloat64 cgz\nfloat64 ix\nfloat64 iy\nfloat64 iz\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double getM();
  void setM(double value);
  double getCgx();
  void setCgx(double value);
  double getCgy();
  void setCgy(double value);
  double getCgz();
  void setCgz(double value);
  double getIx();
  void setIx(double value);
  double getIy();
  void setIy(double value);
  double getIz();
  void setIz(double value);
}
