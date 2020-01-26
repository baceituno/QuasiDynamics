package robot_comm;

public interface robot_SetTrackDistRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetTrackDistRequest";
  static final java.lang.String _DEFINITION = "# Service to Set the tracking distance of the robot while in non-blocking mode\n\nfloat64 pos_dist  # mm\nfloat64 ang_dist  # deg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double getPosDist();
  void setPosDist(double value);
  double getAngDist();
  void setAngDist(double value);
}
