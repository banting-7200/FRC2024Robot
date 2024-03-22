package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooter.intakeCommand;
import frc.robot.subsystems.ArmAndHead.ShooterSubsystem;
import frc.robot.subsystems.Vision.PhotonCamera;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class NoteObjectAlign extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final PhotonCamera photonCam;

  private final PIDController positionController;
  private final PIDController rotationController;
  private ShooterSubsystem shooter;

  private double c_noteArea;
  private double d_noteArea;

  private long startedMillis;
  private long currentMillis;

  private Command s_command;

  public NoteObjectAlign(
      SwerveSubsystem swerveSubsystem,
      PhotonCamera photonCam,
      double d_noteArea,
      ShooterSubsystem shooter) {
    this.swerveSubsystem = swerveSubsystem;
    this.photonCam = photonCam;

    this.d_noteArea = d_noteArea;
    this.shooter = shooter;

    positionController = new PIDController(1, 0, 0);
    positionController.setSetpoint(d_noteArea);

    rotationController = new PIDController(0.035, 0.0001, 0);
    rotationController.setSetpoint(0);

    addRequirements(swerveSubsystem, photonCam);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double fowardAdjust = 0;
    double rotationAdjust = 0;

    if (photonCam.hasTarget()) {
      c_noteArea = photonCam.getNoteArea();
      fowardAdjust = positionController.calculate(c_noteArea, d_noteArea);
      rotationAdjust = rotationController.calculate(photonCam.getNoteYaw(), 0);
      swerveSubsystem.drive(new Translation2d(fowardAdjust, 0), rotationAdjust, false);
      if (c_noteArea == (d_noteArea / 2)) {
        System.out.println("Running intake command within Note Align!");
        s_command = new intakeCommand(6000, shooter);
      }
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
