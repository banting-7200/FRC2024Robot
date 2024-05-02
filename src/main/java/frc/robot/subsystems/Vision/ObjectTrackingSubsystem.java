package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectTrackingSubsystem extends SubsystemBase {

  private static ObjectTrackingSubsystem instance = null;
  private PhotonCamera cam;

  public ObjectTrackingSubsystem() {
   cam = new PhotonCamera("Arducam USB Camera"); // gets the network table with key
  }

  public static synchronized ObjectTrackingSubsystem getInstance() {
    if (instance == null) {
      instance = new ObjectTrackingSubsystem();
    }
    return instance;
  }

  public boolean hasTarget() { // is target in view?
    boolean hasTarget = cam.getLatestResult().hasTargets();
    return hasTarget;
  }

  public double getNoteYaw() {
    double noteYaw = cam.getLatestResult().getBestTarget().getYaw();
    return noteYaw;
  }

  public double getNotePitch() {
    double notePitch = cam.getLatestResult().getBestTarget().getPitch();
    return notePitch;
  }

  public double getNoteSkew() {
    double noteSkew = cam.getLatestResult().getBestTarget().getSkew();
    return noteSkew;
  }

  public double getNoteArea() {
    double noteArea = cam.getLatestResult().getBestTarget().getArea();
    return noteArea;
  }
}
