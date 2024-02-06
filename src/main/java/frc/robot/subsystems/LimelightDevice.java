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

  public LimelightDevice() { // initializes device
    mainTable =
        NetworkTableInstance.getDefault()
            .getTable("limelight"); // gets the network table with key "limelight"
    mode = 0;
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

  /*public double[] getTagPose(){

  }*/

  public void printTagData() {
    String[] keyWords = {
      "x: ", "y: ", "z: ", "roll: ", "pitch: ", "yaw: ",
    };
    double[] tagTransform =
        mainTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    for (int i = 0; i < tagTransform.length; i++) {
      SmartDashboard.putNumber(keyWords[i], tagTransform[i]);
    }
  }

  public Pose2d getTagPose() {
    double[] tagTransform =
        mainTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    // Todo: determine which tag transform entries are equivilent to a top down 2d view of the feild
    // (x, y)
    // Determine how to correct for rotation (is pitch, yaw, or roll the value we want)
    // Adjust april tag offsets
    // Configure position of limelight relative to robot(once mounted).
    return new Pose2d(tagTransform[0], tagTransform[2], new Rotation2d(tagTransform[6]));
  }

  public Pose2d getFakeTagPose() {
    return new Pose2d(3, 4, new Rotation2d(33));
  }

  public boolean
      getNoteData() { // posts limelight note pipelinedata to SmartDashboard & returns true if note
    // is detected

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
