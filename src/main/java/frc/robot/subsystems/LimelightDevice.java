package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightDevice extends SubsystemBase {
  private NetworkTable mainTable;
  public int mode;

  private double k_XP = 0.01;
  private double k_fowardP = 5;
  private double targetArea = 0.4;

  // NetworkTableEntry m_XP, m_YP, m_targetArea;

  public LimelightDevice() { // initializes device
    mainTable =
        NetworkTableInstance.getDefault().getTable("limelight"); // gets the network table with key
    // "limelight"
    mode = 0;

    // m_XP = ShuffleboardInfo.getInsatnce().
  }

  public void setLight(boolean on) { // toggles lights (true for on, false for off)
    mainTable.getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  public void setMode(int selection) {
    mode = selection;
    mainTable.getEntry("pipeline").setNumber(selection);
  }

  public int
      getTagData() { // posts limelight apriltag pipeline data to SmartDashboard & returns id of
    // apriltag. Returns -1 if no tag is detected.

    double ttarget = mainTable.getEntry("tv").getDouble(0); // may have to be double then converted
    double tx = mainTable.getEntry("tx").getDouble(0);
    double ty = mainTable.getEntry("ty").getDouble(0);
    double ta = mainTable.getEntry("ta").getDouble(0);
    double tid = mainTable.getEntry("tid").getDouble(-1);
    boolean tdetected = ttarget == 0 ? false : true;

    SmartDashboard.putBoolean("AprilTag Detected", tdetected);
    SmartDashboard.putNumber("Tag X", tx);
    SmartDashboard.putNumber("Tag Y", ty);
    SmartDashboard.putNumber("Tag Area", ta);
    SmartDashboard.putNumber("AprilTag ID", tid);

    return (int) tid;
  }

  public boolean tagDetected() {
    double ttarget = mainTable.getEntry("tv").getDouble(0);
    return ttarget == 0 ? false : true;
  }

  public Pose2d
      getFakeTagPose() { // Can't take negative values because it is trying to pathfind out of the
    // feild
    return new Pose2d(2.98, 4, new Rotation2d(33));
  }

  public double getTagArea() {
    return mainTable.getEntry("ta").getDouble(0);
  }

  public double getTX() {
    return mainTable.getEntry("tx").getDouble(0);
  }

  public boolean
      getNoteData() { // posts limelight note pipelinedata to SmartDashboard & returns true if note
    // is
    // detected

    double ntarget = mainTable.getEntry("tv").getDouble(0); // may have to be double then converted
    double nx = mainTable.getEntry("tx").getDouble(0);
    double ny = mainTable.getEntry("ty").getDouble(0);
    double na = mainTable.getEntry("ta").getDouble(0);
    boolean ndetected = ntarget == 0 ? false : true;

    SmartDashboard.putBoolean("Note Detected", ndetected);
    SmartDashboard.putNumber("Note X", nx);
    SmartDashboard.putNumber("Note Y", ny);
    SmartDashboard.putNumber("Note Size", na);

    return ndetected;
  }
}
