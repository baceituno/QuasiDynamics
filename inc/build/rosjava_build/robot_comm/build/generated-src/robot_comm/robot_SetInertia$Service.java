package robot_comm;

public interface robot_SetInertia$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetInertia$Service";
  static final java.lang.String _DEFINITION = "# Service to Set the Inertia of the tool of the robot in cartesian coordinates\n\nfloat64 m\nfloat64 cgx\nfloat64 cgy\nfloat64 cgz\nfloat64 ix\nfloat64 iy\nfloat64 iz\n---\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
