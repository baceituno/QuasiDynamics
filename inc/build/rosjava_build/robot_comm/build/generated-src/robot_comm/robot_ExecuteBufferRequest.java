package robot_comm;

public interface robot_ExecuteBufferRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_ExecuteBufferRequest";
  static final java.lang.String _DEFINITION = "# Service to execute cartesian pose buffer\n# If simultaneous is true, buffers for both arms have to be the same size\nbool simultaneous\nbool useHandPose\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  boolean getSimultaneous();
  void setSimultaneous(boolean value);
  boolean getUseHandPose();
  void setUseHandPose(boolean value);
}
