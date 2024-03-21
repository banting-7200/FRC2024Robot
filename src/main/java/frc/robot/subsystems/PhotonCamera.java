package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PhotonCamera {

  private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  private static NetworkTable table;
  private static PhotonCamera instance = null;

  public PhotonCamera() {
    table =
        NetworkTableInstance.getDefault()
            .getTable("photonvision")
            .getSubTable("FHD_Camera"); // gets the network table with key
  }

  public static synchronized PhotonCamera getInstance() {
    if (instance == null) {
      instance = new PhotonCamera();
    }
    return instance;
  }

  public boolean has_targets() { // is target in view?

    // bool
    boolean has_target = table.getEntry("hasTarget").getBoolean(false);

    return has_target;
  }

  public double[] note_positions() {

    // (x, y, z, qw, qx, qy, qz)
    double[] note_pos = table.getEntry("targetPose").getDoubleArray(new double[] {});

    return note_pos;
  }

  public double getNoteYaw() {
    double noteYaw = table.getEntry("targetYaw").getDouble(0);
    return noteYaw;
  }

  public double getNotePitch() {
    double notePitch = table.getEntry("targetPitch").getDouble(0);
    return notePitch;
  }

  public double getNoteSkew() {
    double noteSkew = table.getEntry("targetSkew").getDouble(0);
    return noteSkew;
  }

  public double getNoteArea() {
    double noteArea = table.getEntry("targetArea").getDouble(0);
    return noteArea;
  }
}
