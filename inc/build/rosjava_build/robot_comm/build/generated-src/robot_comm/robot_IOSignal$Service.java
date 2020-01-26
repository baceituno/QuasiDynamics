package robot_comm;

public interface robot_IOSignal$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_IOSignal$Service";
  static final java.lang.String _DEFINITION = "# Service to Set IOSignals\n\nint32 output_num  # 1-4\nint32 signal      # 1: on, 0: off\n---\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
