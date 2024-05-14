package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagID;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;
import org.photonvision.PhotonCamera;

public class AprilTagSubsystem extends SubsystemBase {
  private NetworkTable mainTable;
  public int mode;
  private static AprilTagSubsystem instance = null;
  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();
  public int speakerMiddleTag;
  public int speakerSideTag;
  public int ampTag;

  private PhotonCamera cam;

  public AprilTagSubsystem() {
    cam = new PhotonCamera("Arducam_OV9281_USB_Camera"); // gets the network table with key
  }

  public static synchronized AprilTagSubsystem getInstance() {
    if (instance == null) {
      instance = new AprilTagSubsystem();
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
    boolean tdetected = tagDetected();
    double tid = getTagID();
    double tx = getTagX();
    double ty = getTagY();
    double ta = getTagArea();

    shuffle.setBoolean("Tag Detected", tdetected);
    shuffle.setNumber("Tag ID", tid);
    shuffle.setNumber("Tag X", tx);
    shuffle.setNumber("Tag Y", ty);
    shuffle.setNumber("Tag Area", ta);
  }

  /*
  OLD CODE USED FOR LIMELIGHT

  public void setLight(boolean on) { // toggles lights (true for on, false for off)
    mainTable.getEntry("ledMode").setNumber(on ? 3 : 1);
    shuffle.setBoolean("LEDs On", on);
  }


  public boolean getLight() {
    double light;
    light = (double) mainTable.getEntry("lightMode").getNumber(3);
    return light == 1 ? false : true;
  }

  public void setMode(int selection) { // sets pipeline
    mainTable.getEntry("pipeline").setNumber(selection);
    shuffle.setNumber("Pipeline", selection);
  }
  */
  public boolean tagDetected() { // returns true if tag is detected
    boolean tdetected = cam.getLatestResult().hasTargets();
    shuffle.setBoolean("Tag Detected", tdetected);
    return tdetected;
  }

  public int getTagID() { // returns id of apriltag or -1 if no tag is detected.
    int tid = cam.getLatestResult().getBestTarget().getFiducialId();
    shuffle.setNumber("Tag ID", tid);
    return tid;
  }

  public double getTagArea() { // return tag area
    double ta = cam.getLatestResult().getBestTarget().getArea();
    shuffle.setNumber("Tag Area", ta);
    return ta;
  }

  public double getTagX() { // return tag x value (horizontal across camera screen)
    double tx = cam.getLatestResult().getBestTarget().getPitch();
    try {
      tx = mainTable.getEntry("targetYaw").getDouble(0);
    } catch (NullPointerException e) {
      System.out.println("tx ERROR, EXCEPTION: " + e);
      tx = 0;
    }
    shuffle.setNumber("Tag X", tx);
    return tx;
  }

  public double getTagY() { // return tag y value (vertical across camera screen)
    double ty = cam.getLatestResult().getBestTarget().getYaw();
    shuffle.setNumber("Tag Y", ty);
    return ty;
  }

  public double calculateArmShootAngle() {
    double maxAngle = 29.38;
    double minAngle = 28.52;

    double maxDist = 0.41;
    double minDist = 0.60;

    double tagDist = getTagArea();
    double goalAngle;
    goalAngle = maxDist - tagDist; // how far back the bot is from subwoofer as a positive number
    goalAngle *=
        ((maxAngle - minAngle)
            / (maxDist - minDist)); // converts from area to angle and makes proportionate;
    goalAngle = maxAngle - goalAngle; // adds difference to minAngle
    shuffle.setNumber("Arm Limelight Calc", goalAngle);
    return tagDist != 0 && getTagID() == getSpeakerMiddleTag() ? goalAngle : Arm.speakerArmAngle;
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
