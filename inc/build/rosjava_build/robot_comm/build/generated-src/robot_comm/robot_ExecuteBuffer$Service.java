package robot_comm;

public interface robot_ExecuteBuffer$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_ExecuteBuffer$Service";
  static final java.lang.String _DEFINITION = "# Service to execute cartesian pose buffer\n# If simultaneous is true, buffers for both arms have to be the same size\nbool simultaneous\nbool useHandPose\n---\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
