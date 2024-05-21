package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;
import frc.robot.subsystems.Vision.ObjectTrackingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.time.Clock;

public class DriveToNote extends Command {

  private SwerveSubsystem swerveSubsystem = null;
  private final ObjectTrackingSubsystem photonCam;

  private final PIDController positionController;
  private final PIDController rotationController;

  private double c_noteArea;
  private double d_noteArea;

  ShuffleboardSubsystem suffle = ShuffleboardSubsystem.getInstance();

  private Clock currentTime = Clock.systemDefaultZone();
  private long startedMillis;
  private long currentMillis;

  private NoteAutoStateMachine stateInstance;

  public DriveToNote(
      SwerveSubsystem swerveSubsystem, ObjectTrackingSubsystem photonCam, double d_noteArea) {
    this.swerveSubsystem = swerveSubsystem;
    this.photonCam = photonCam;

    this.d_noteArea = d_noteArea;

    // Initialize PID controlers with P, I, and D values as well as a setpoint
    positionController = new PIDController(0.045, 0.00005, 0);
    positionController.setSetpoint(d_noteArea);

    rotationController = new PIDController(0.03, 0.00008, 0);

    rotationController.setSetpoint(0);
    rotationController.setTolerance(2, 4);

    addRequirements(swerveSubsystem, photonCam);
  }

  public DriveToNote(
      SwerveSubsystem swerveSubsystem,
      ObjectTrackingSubsystem photonCam,
      double d_noteArea,
      NoteAutoStateMachine stateInstance) {
    this(swerveSubsystem, photonCam, d_noteArea);
    this.stateInstance = stateInstance;
  }

  @Override
  public void initialize() {
    startedMillis = currentTime.millis();
  }

  @Override
  public void execute() {
    double fowardAdjust = 0;
    double rotationAdjust = 0;

    if (photonCam.hasTarget()) {
      // System.out.println("Current yaw: " + photonCam.getNoteYaw());
      c_noteArea = photonCam.getNoteArea();
      fowardAdjust =
          positionController.calculate(
              c_noteArea,
              d_noteArea); // Calculate the adjustments needed to be made to drive up to the note
      rotationAdjust =
          rotationController.calculate(
              photonCam.getNoteYaw(),
              0); // Calculate the adjustments needed to be made to rotate towards the note

      // Drive and rotate the bot proportionally to the error
      swerveSubsystem.drive(new Translation2d(fowardAdjust, 0), rotationAdjust, false);
    }
    if (rotationController.atSetpoint()) {
      System.out.println("at rotation setpoint");
    }
    if (positionController.atSetpoint()) {
      System.out.println("at position setpoint");
    }
  }

  @Override
  public boolean isFinished() { // Only finish when both PID controllers have reached there setpoint
    return (rotationController.atSetpoint() && positionController.atSetpoint())
        || !photonCam.hasTarget()
        || currentMillis - startedMillis > 10000;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.lock(); // Lock the swerve base
    if (stateInstance != null) { // Advance to the next state in the state machine
      if (!photonCam.hasTarget()) {
        stateInstance.MoveToState(NoteAutoStateMachine.States.Search);
      } else {
        stateInstance.MoveToState(NoteAutoStateMachine.States.PickUp);
      }
    }
    if (!interrupted) {
      System.out.println("Ended Note Drive Successfully");
    } else {
      System.out.println("Interrupted Note Drive Commmand");
    }
  }
}
