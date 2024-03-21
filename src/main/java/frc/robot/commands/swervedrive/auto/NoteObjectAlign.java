package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonCamera;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class NoteObjectAlign extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final PhotonCamera photonCam;

  private final PIDController positionController;
  private final PIDController rotationController;

  private double tagArea;
  private double targetArea;

  private long startedMillis;
  private long currentMillis;

  public NoteObjectAlign(
      SwerveSubsystem swerveSubsystem, PhotonCamera photonCam, double targetArea) {
    this.swerveSubsystem = swerveSubsystem;
    this.photonCam = photonCam;

    this.targetArea = targetArea;

    positionController = new PIDController(1, 0, 0);
    positionController.setSetpoint(targetArea);

    rotationController = new PIDController(0.035, 0.0001, 0);
    rotationController.setSetpoint(0);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println("Currently Executing Tag Align command");

    double fowardAdjust = 0;
    double rotationAdjust = 0;

    if (photonCam.has_targets()) {
      tagArea = photonCam.getNoteArea();
      fowardAdjust = positionController.calculate(tagArea, targetArea);
      rotationAdjust = rotationController.calculate(photonCam.getNoteYaw(), 0);
      swerveSubsystem.drive(new Translation2d(fowardAdjust, 0), rotationAdjust, false);
    } else {
      swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
    }
  }

  @Override
  public boolean isFinished() {
    return positionController.atSetpoint() && rotationController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.lock();
    if (!interrupted) {
      System.out.println("Ended Tag Align successfully");
    } else {
      System.out.println("Interrupted Tag Align Commmand");
    }
  }
}
