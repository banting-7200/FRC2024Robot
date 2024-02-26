package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightDevice;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.IntSupplier;

public class AprilTagAlign extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final LimelightDevice limelightSubsystem;

  private final PIDController positionController;
  private final PIDController rotationController;

  private double tagArea;
  private double targetArea;

  private IntSupplier tagToAlign;

  public AprilTagAlign(
      SwerveSubsystem swerveSubsystem,
      LimelightDevice limelightSubsystem,
      double targetArea,
      IntSupplier tagToAlign) {
    this.swerveSubsystem = swerveSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    this.targetArea = targetArea;
    this.tagToAlign = tagToAlign;

    positionController = new PIDController(1, 0, 0);
    positionController.setSetpoint(targetArea);

    rotationController = new PIDController(0.035, 0.0001, 0);
    rotationController.setSetpoint(0);

    addRequirements(swerveSubsystem, limelightSubsystem);
  }

  public AprilTagAlign(
      SwerveSubsystem swerveSubsystem,
      LimelightDevice limelightSubsystem,
      double targetArea,
      int tagToAlign) {
    this(swerveSubsystem, limelightSubsystem, targetArea, () -> tagToAlign);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double fowardAdjust = 0;
    double rotationAdjust = 0;

    if (tagToAlign.getAsInt() == limelightSubsystem.getTagID()) {
      tagArea = limelightSubsystem.getTagArea();
      fowardAdjust = positionController.calculate(tagArea, targetArea);
      rotationAdjust = rotationController.calculate(limelightSubsystem.getTagX(), 0);
      swerveSubsystem.drive(new Translation2d(fowardAdjust, 0), rotationAdjust, false);
    }

    SmartDashboard.putNumber("tagArea", tagArea);
    SmartDashboard.putNumber("targetArea", targetArea);
    SmartDashboard.putBoolean("at position", positionController.atSetpoint());

    SmartDashboard.putNumber("tag offset", rotationAdjust);
    SmartDashboard.putBoolean("at rotation", rotationController.atSetpoint());
  }

  @Override
  public boolean isFinished() {
    return positionController.atSetpoint() && rotationController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.lock();
  }
}
