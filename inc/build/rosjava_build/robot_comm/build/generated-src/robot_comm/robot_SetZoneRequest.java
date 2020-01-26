package robot_comm;

public interface robot_SetZoneRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetZoneRequest";
  static final java.lang.String _DEFINITION = "# Service to Set the Zone of the robot\n# Mode - Name in RAPID - Linear  - Orientation\n#   0        fine         0 mm        0\uFFFD\uFFFD\n#   1         z0          0.3 mm      0.03\uFFFD\uFFFD  <- Default and recommended value.\n#   2         z1          1 mm        0.1\uFFFD\uFFFD\n#   3         z5          5 mm        0.8\uFFFD\uFFFD\n#   4         z10         10 mm       1.5\uFFFD\uFFFD\n\nint64 mode\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  long getMode();
  void setMode(long value);
}
