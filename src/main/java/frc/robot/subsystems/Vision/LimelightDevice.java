package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagID;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;

public class LimelightDevice extends SubsystemBase {
  private NetworkTable mainTable;
  public int mode;
  private static LimelightDevice instance = null;
  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();
  public int speakerMiddleTag;
  public int speakerSideTag;
  public int ampTag;

  public static synchronized LimelightDevice getInstance() {
    if (instance == null) {
      instance = new LimelightDevice();
    }
    return instance;
  }

  public void shuffleSetUp() {
    shuffle.setTab("Limelight Data");

    shuffle.addCamera("Limelight Stream", "Limelight", "10.72.0.11:5800");

    shuffle.setLayout("Limelight Config");
    shuffle.setBoolean("LEDs On", false);
    shuffle.setNumber("Pipeline", -1);

    shuffle.setLayout("Tag", 2, 4);
    shuffle.setBoolean("Tag Detected", false);
    shuffle.setNumber("Tag ID", -1);
    shuffle.setNumber("Tag X", -1);
    shuffle.setNumber("Tag Y", -1);
    shuffle.setNumber("Tag Area", -1);
    shuffle.setLayout(null);
  }

  public void shuffleUpdate() {
    double ledMode = mainTable.getEntry("ledMode").getDouble(0);
    double pipeline = mainTable.getEntry("pipeline").getDouble(0);
    double ttarget = mainTable.getEntry("tv").getDouble(0);
    double tid = mainTable.getEntry("tid").getDouble(-1);
    double tx = mainTable.getEntry("tx").getDouble(0);
    double ty = mainTable.getEntry("ty").getDouble(0);
    double ta = mainTable.getEntry("ta").getDouble(0);

    boolean ledOn = ledMode == 3 ? true : false;
    boolean tdetected = ttarget == 0 ? false : true;

    shuffle.setBoolean("LEDs On", ledOn);
    shuffle.setNumber("Pipeline", pipeline);
    shuffle.setBoolean("Tag Detected", tdetected);
    shuffle.setNumber("Tag ID", tid);
    shuffle.setNumber("Tag X", tx);
    shuffle.setNumber("Tag Y", ty);
    shuffle.setNumber("Tag Area", ta);
  }

  public void setLight(boolean on) { // toggles lights (true for on, false for off)
    mainTable.getEntry("ledMode").setNumber(on ? 3 : 1);
    shuffle.setBoolean("LEDs On", on);
  }

  public void setMode(int selection) { // sets pipeline
    mainTable.getEntry("pipeline").setNumber(selection);
    shuffle.setNumber("Pipeline", selection);
  }

  public boolean tagDetected() { // returns true if tag is detected
    double ttarget = 0;
    try {
      ttarget = mainTable.getEntry("tv").getDouble(0);
    } catch (NullPointerException e) {
      System.out.println("ttarget ERROR, EXCEPTION: " + e);
    }
    boolean tdetected = ttarget == 0 ? false : true;
    shuffle.setBoolean("Tag Detected", tdetected);
    return tdetected;
  }

  public int getTagID() { // returns id of apriltag or -1 if no tag is detected.
    int tid = 0;
    try {
      tid = (int) mainTable.getEntry("tid").getDouble(-1);
    } catch (NullPointerException e) {
      System.out.println("tid ERROR, EXCEPTION: " + e);
    }
    shuffle.setNumber("Tag ID", tid);
    return tid;
  }

  public double getTagArea() { // return tag area
    double ta = 0;
    try {
      ta = mainTable.getEntry("ta").getDouble(0);

    } catch (NullPointerException e) {
      System.out.println("Tag Area ERROR, EXCEPTION: " + e);
    }
    shuffle.setNumber("Tag Area", ta);
    return ta;
  }

  public double getTagX() { // return tag x value (horizontal across camera screen)
    double tx = 0;
    try {
      tx = mainTable.getEntry("tx").getDouble(0);
    } catch (NullPointerException e) {
      System.out.println("tx ERROR, EXCEPTION: " + e);
    }
    shuffle.setNumber("Tag X", tx);
    return tx;
  }

  public double getTagY() { // return tag y value (vertical across camera screen)
    double ty = 0;
    try {
      ty = mainTable.getEntry("ty").getDouble(0);
    } catch (NullPointerException e) {
      System.out.println("ty ERROR, EXCEPTION: " + e);
    }

    shuffle.setNumber("Tag Y", ty);
    return ty;
  }

  public double calculateArmShootAngle() {
    double angleToGoalDegrees = Arm.limelightMountAngle + getTagY();
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    double distanceFromLimelightToGoal =
        Arm.limelightToSpeakerHeight / Math.tan(angleToGoalRadians);
    double distanceFromArmToGoal = distanceFromLimelightToGoal + Arm.armToLimelightDistance;

    return (Math.atan2(Arm.armToSpeakerHeight, distanceFromArmToGoal) + Arm.armToShooterAngle)
        * (180.0 / 3.14159);
  }

  public Pose2d getFakeTagPose() { // input fake tag values for simulation
    // Can't take negative values because it is trying to pathfind out of the field
    return new Pose2d(2.98, 4, new Rotation2d(33));
  }

  public void refreshRelevantTags(
      boolean isRed) { // sets april tag ids depending on alliance from driverstation
    if (isRed) {
      speakerMiddleTag = AprilTagID.redSpeakerMiddle;
      speakerSideTag = AprilTagID.redSpeakerSide;
      ampTag = AprilTagID.redAmp;
    } else {
      speakerMiddleTag = AprilTagID.blueSpeakerMiddle;
      speakerSideTag = AprilTagID.blueSpeakerSide;
      ampTag = AprilTagID.blueAmp;
    }
  }

  public int getSpeakerMiddleTag() {
    return speakerMiddleTag;
  }

  public int getSpeakerSideTag() {
    return speakerSideTag;
  }

  public int getAmpTag() {
    return ampTag;
  }
}
