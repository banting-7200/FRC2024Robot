package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightDevice extends SubsystemBase {
  private NetworkTable limeLightTable;
  public int mode;

  public LimelightDevice() { // initializes device
    limeLightTable =
        NetworkTableInstance.getDefault().getTable("limelight"); // gets the network table with key
    // "limelight"
    mode = 0;

    // m_XP = ShuffleboardInfo.getInsatnce().
  }

  public void setLight(boolean on) { // toggles lights (true for on, false for off)
    limeLightTable.getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  public void setMode(int selection) {
    mode = selection;
    limeLightTable.getEntry("pipeline").setNumber(selection);
  }

  public int getTagID() { // returns id of apriltag or -1 if no tag is detected.
    double tid = limeLightTable.getEntry("tid").getDouble(-1);
    SmartDashboard.putNumber("AprilTag ID", tid);
    return (int) tid;
  }

  public boolean tagDetected() { // returns true if tag is detected
    double ttarget = limeLightTable.getEntry("tv").getDouble(0);
    boolean tdetected = ttarget == 0 ? false : true;
    SmartDashboard.putBoolean("AprilTag Detected", tdetected);
    return tdetected;
  }

  public double getTagArea() { // return tag area
    double ta = limeLightTable.getEntry("ta").getDouble(0);
    SmartDashboard.putNumber("Tag Area", ta);
    return ta;
  }

  public double getTagX() { // return tag x value (horizontal across camera screen)
    double tx = limeLightTable.getEntry("tx").getDouble(0);
    SmartDashboard.putNumber("Tag X", tx);
    return tx;
  }

  public double getTagY() { // return tag y value (vertical across camera screen)
    double ty = limeLightTable.getEntry("ty").getDouble(0);
    SmartDashboard.putNumber("Tag Y", ty);
    return ty;
  }

  public void putTagData() { // publishes tag data to SmartDashboard
    double ttarget = limeLightTable.getEntry("tv").getDouble(0); // may have to be double then converted
    double tx = limeLightTable.getEntry("tx").getDouble(0);
    double ty = limeLightTable.getEntry("ty").getDouble(0);
    double ta = limeLightTable.getEntry("ta").getDouble(0);
    double tid = limeLightTable.getEntry("tid").getDouble(-1);
    boolean tdetected = ttarget == 0 ? false : true;

    SmartDashboard.putBoolean("AprilTag Detected", tdetected);
    SmartDashboard.putNumber("Tag X", tx);
    SmartDashboard.putNumber("Tag Y", ty);
    SmartDashboard.putNumber("Tag Area", ta);
    SmartDashboard.putNumber("AprilTag ID", tid);
  }

  public boolean
      noteDetected() { // posts limelight note pipelinedata to SmartDashboard & returns true if note
    // is
    // detected
    double ntarget = limeLightTable.getEntry("tv").getDouble(0); // may have to be double then converted
    double nx = limeLightTable.getEntry("tx").getDouble(0);
    double ny = limeLightTable.getEntry("ty").getDouble(0);
    double na = limeLightTable.getEntry("ta").getDouble(0);
    boolean ndetected = ntarget == 0 ? false : true;

    SmartDashboard.putBoolean("Note Detected", ndetected);
    SmartDashboard.putNumber("Note X", nx);
    SmartDashboard.putNumber("Note Y", ny);
    SmartDashboard.putNumber("Note Size", na);
    return ndetected;
  }

  public Pose2d getFakeTagPose() { // input fake tag values for simulation
    // Can't take negative values because it is trying to pathfind out of the feild
    return new Pose2d(2.98, 4, new Rotation2d(33));
  }
}
