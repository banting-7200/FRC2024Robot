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

  public final Supplier<Double> joystickSquaredX =
      () -> {
        double[] d = swerveSubsystem.squareifyInput(driverXbox.getLeftX(), driverXbox.getLeftY());
        return isRedAliance.getAsBoolean() ? d[0] * -1 : d[0];
      };
  public final Supplier<Double> joystickSquaredY =
      () -> {
        double[] d = swerveSubsystem.squareifyInput(driverXbox.getLeftX(), driverXbox.getLeftY());
        return isRedAliance.getAsBoolean() ? d[1] * -1 : d[1];
      };

  public NoteObjectAlign(
      SwerveSubsystem swerveSubsystem,
      PhotonCamera photonCam,
      XboxController driverXbox,
      BooleanSupplier isRedAlliance) {
    this.swerveSubsystem = swerveSubsystem;
    this.photonCam = photonCam;

    this.d_noteArea = d_noteArea;

    this.driverXbox = driverXbox;
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
              MathUtil.applyDeadband(-joystickSquaredY.get(), OperatorConstants.LEFT_Y_DEADBAND),
              MathUtil.applyDeadband(-joystickSquaredX.get(), OperatorConstants.LEFT_X_DEADBAND)),
          rotationAdjust,
          false);
    } else {

      swerveSubsystem.drive(
          new Translation2d(
              MathUtil.applyDeadband(-joystickSquaredY.get(), OperatorConstants.LEFT_Y_DEADBAND),
              MathUtil.applyDeadband(-joystickSquaredX.get(), OperatorConstants.LEFT_X_DEADBAND)),
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
