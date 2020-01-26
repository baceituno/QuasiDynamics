package robot_comm;

public interface robot_ActivateCSS$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_ActivateCSS$Service";
  static final java.lang.String _DEFINITION = "int32 refFrame\nfloat64 refOrient_q0\nfloat64 refOrient_qx\nfloat64 refOrient_qy\nfloat64 refOrient_qz\nint32 softDir\nfloat64 stiffness\nfloat64 stiffnessNonSoftDir\nint32 allowMove\nfloat64 ramp\n---\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
