package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision.LimelightDevice;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class AprilTagOrbit extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final LimelightDevice limelightSubsystem;

  private final PIDController rotationController;

  private IntSupplier tagToAlign;

  private Supplier<double[]> leftStick;
  private Supplier<double[]> rightStick;

  public AprilTagOrbit(
      SwerveSubsystem swerveSubsystem,
      LimelightDevice limelightSubsystem,
      IntSupplier tagToAlign, Supplier<double[]> leftStick, Supplier<double[]> rightStick) {
    this.swerveSubsystem = swerveSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    this.tagToAlign = tagToAlign;

    this.leftStick = leftStick;
    this.rightStick = rightStick;

    rotationController = new PIDController(0.035, 0.0001, 0);
    rotationController.setSetpoint(0);

    addRequirements(swerveSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Started April Tag Orbit");
  }

  @Override
  public void execute() {
    double rotationAdjust = 0;

    if (tagToAlign.getAsInt() == limelightSubsystem.getTagID()) {
      rotationAdjust = rotationController.calculate(limelightSubsystem.getTagX(), 0);
      swerveSubsystem.drive(new Translation2d(leftStick.get()[0], leftStick.get()[1]), rotationAdjust, true);
    } else {
      swerveSubsystem.driveFieldOriented(leftStick.get(), rightStick.get());
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    //swerveSubsystem.lock();
    if (!interrupted) {
      System.out.println("Ended Tag Orbit Successfully");
    } else {
      System.out.println("Interrupted Tag Orbit Commmand");
    }
  }
}

