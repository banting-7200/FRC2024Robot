package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PhotonCamera extends SubsystemBase {

  private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  private static NetworkTable table;
  private static PhotonCamera instance = null;

  public PhotonCamera() {
    table =
        NetworkTableInstance.getDefault()
            .getTable("photonvision")
            .getSubTable("Microsoft_LifeCam_HD-3000"); // gets the network table with key
  }

  public static synchronized PhotonCamera getInstance() {
    if (instance == null) {
      instance = new PhotonCamera();
    }
    return instance;
  }

  public boolean hasTarget() { // is target in view?
    boolean hasTarget = table.getEntry("hasTarget").getBoolean(false);
    return hasTarget;
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
