package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Vision.PhotonCamera;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class NoteObjectAlign extends Command {

  private SwerveSubsystem swerveSubsystem = null;
  private final PhotonCamera photonCam;

  private final PIDController positionController;
  private final PIDController rotationController;

  private double c_noteArea;
  private double d_noteArea;

  private XboxController driverXbox;
  private BooleanSupplier isRedAliance;

  private long startedMillis;
  private long currentMillis;

  private Command s_command;

  public final Supplier<double[]> leftJoystick;

  public final Supplier<double[]> rightJoystick;

  public NoteObjectAlign(
      SwerveSubsystem swerveSubsystem,
      PhotonCamera photonCam,
      Supplier<double[]> leftJoystick,
      Supplier<double[]> rightJoystick,
      BooleanSupplier isRedAlliance) {
    this.swerveSubsystem = swerveSubsystem;
    this.photonCam = photonCam;

    this.d_noteArea = d_noteArea;

    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;

    this.isRedAliance = isRedAlliance;

    positionController = new PIDController(1, 0, 0);
    positionController.setSetpoint(d_noteArea);

    rotationController = new PIDController(0.035, 0.0001, 0);

    rotationController.setSetpoint(0);
    rotationController.setTolerance(2, 4);

    addRequirements(swerveSubsystem, photonCam);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    System.out.println("Current yaw: " + photonCam.getNoteYaw());
    double rotationAdjust = 0;
    if (photonCam.hasTarget()) {
      c_noteArea = photonCam.getNoteArea();
      rotationAdjust = rotationController.calculate(photonCam.getNoteYaw(), 0);
      swerveSubsystem.drive(
          new Translation2d(
              MathUtil.applyDeadband(leftJoystick.get()[1], OperatorConstants.LEFT_X_DEADBAND),
              MathUtil.applyDeadband(leftJoystick.get()[0], OperatorConstants.LEFT_Y_DEADBAND)),
          rotationAdjust,
          false);
    } else {
      swerveSubsystem.driveFieldOriented(leftJoystick.get(), rightJoystick.get());
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.lock();
    if (!interrupted) {
      System.out.println("Ended Note Align successfully");
    } else {
      System.out.println("Interrupted Note Align Commmand");
    }
  }
}
