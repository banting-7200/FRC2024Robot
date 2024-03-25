package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonCamera;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class NoteObjectAlign extends Command {

  private SwerveSubsystem swerveSubsystem = null;
  private final PhotonCamera photonCam;

  private final PIDController positionController;
  private final PIDController rotationController;
  private ShooterSubsystem shooter;

  private double c_noteArea;
  private double d_noteArea;

  private XboxController driverXbox;
  private BooleanSupplier isRedAliance;

  private long startedMillis;
  private long currentMillis;

  public NoteObjectAlign(
      SwerveSubsystem swerveSubsystem, PhotonCamera photonCam, double d_noteArea) {
    this.swerveSubsystem = swerveSubsystem;
    this.photonCam = photonCam;

    this.d_noteArea = d_noteArea;

    positionController = new PIDController(1, 0, 0);
    positionController.setSetpoint(d_noteArea);

    rotationController = new PIDController(0.035, 0.0001, 0);

    rotationController.setSetpoint(0);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    System.out.println("Currently Executing Tag Align command");

    double fowardAdjust = 0;
    double rotationAdjust = 0;

    if (photonCam.has_targets()) {
      c_noteArea = photonCam.getNoteArea();
      rotationAdjust = rotationController.calculate(photonCam.getNoteYaw(), 0);
      swerveSubsystem.drive(new Translation2d(fowardAdjust, 0), rotationAdjust, false);
    } else {

      swerveSubsystem.drive(
          new Translation2d(
              MathUtil.applyDeadband(-joystickSquaredY.get(),
                  OperatorConstants.LEFT_Y_DEADBAND),
              MathUtil.applyDeadband(-joystickSquaredX.get(),
                  OperatorConstants.LEFT_X_DEADBAND)),
          0,
          true);
    }
    if (s_command == null) {
      s_command.schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return rotationController.atSetpoint();
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
