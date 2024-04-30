package frc.robot.commands.swervedrive.auto;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision.PhotonCamera;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToNote extends Command {

  private SwerveSubsystem swerveSubsystem = null;
  private final PhotonCamera photonCam;

  private final PIDController positionController;
  private final PIDController rotationController;

  private double c_noteArea;
  private double d_noteArea;

 /*  private long startedMillis;
 private long currentMillis;
 
 private Command s_command;
 
 private Pose2d initialPose2d; */
  
 private NoteAutoStateMachine stateInstance;

  public DriveToNote(
      SwerveSubsystem swerveSubsystem,
      PhotonCamera photonCam,
      double d_noteArea) {
    this.swerveSubsystem = swerveSubsystem;
    this.photonCam = photonCam;

    this.d_noteArea = d_noteArea;

    positionController = new PIDController(1, 0, 0);
    positionController.setSetpoint(d_noteArea);

    rotationController = new PIDController(0.035, 0.0001, 0);

    rotationController.setSetpoint(0);
    rotationController.setTolerance(2, 4);

    addRequirements(swerveSubsystem, photonCam);
  }

  public DriveToNote(SwerveSubsystem swerveSubsystem,
      PhotonCamera photonCam,
      double d_noteArea, NoteAutoStateMachine stateInstance) {
    this(swerveSubsystem, photonCam, d_noteArea);
    this.stateInstance = stateInstance;
    
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println("Current yaw: " + photonCam.getNoteYaw());
    double fowardAdjust = 0;
    double rotationAdjust = 0;
    if (photonCam.hasTarget()) {
      c_noteArea = photonCam.getNoteArea();
      fowardAdjust = positionController.calculate(c_noteArea, d_noteArea);
      rotationAdjust = rotationController.calculate(photonCam.getNoteYaw(), 0);
      swerveSubsystem.drive(
          new Translation2d(fowardAdjust, 0),
          rotationAdjust,
          false);
    } 
  }

  @Override
  public boolean isFinished() {
    return rotationController.atSetpoint() && positionController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    /*
     * swerveSubsystem.setOdometry(initialPose2d);
     */
    swerveSubsystem.lock();
    if(stateInstance != null){
       stateInstance.MoveToState(NoteAutoStateMachine.States.PickUp);
    }
    if (!interrupted) {
      System.out.println("Ended Note Drive Successfully");
    } else {
      System.out.println("Interrupted Note Drive Commmand");
    }
  }
}
