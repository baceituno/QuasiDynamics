package robot_comm;

public interface robot_ExecuteJointPosBufferRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_ExecuteJointPosBufferRequest";
  static final java.lang.String _DEFINITION = "# Service to Set Joints\n\nbool simultaneous\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  boolean getSimultaneous();
  void setSimultaneous(boolean value);
}
