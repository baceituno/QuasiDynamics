package robot_comm;

public interface robot_IOSignalRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_IOSignalRequest";
  static final java.lang.String _DEFINITION = "# Service to Set IOSignals\n\nint32 output_num  # 1-4\nint32 signal      # 1: on, 0: off\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  int getOutputNum();
  void setOutputNum(int value);
  int getSignal();
  void setSignal(int value);
}
