package robot_comm;

public interface robot_SetTrackDist$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetTrackDist$Service";
  static final java.lang.String _DEFINITION = "# Service to Set the tracking distance of the robot while in non-blocking mode\n\nfloat64 pos_dist  # mm\nfloat64 ang_dist  # deg\n---\nint64 ret\nstring msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
