package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightDevice;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AprilTagAlign extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final LimelightDevice limelightSubsystem;

  private final PIDController positionController;
  private final PIDController rotationController;

  private double tagArea;
  private double targetArea = 2;

  public AprilTagAlign(SwerveSubsystem swerveSubsystem, LimelightDevice limelightSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    positionController = new PIDController(1, 0, 0);
    positionController.setSetpoint(targetArea);

    rotationController = new PIDController(0.1, 0, 0);
    rotationController.setSetpoint(0);

    addRequirements(swerveSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    tagArea = limelightSubsystem.getTagArea();
    double fowardAdjust = positionController.calculate(tagArea, targetArea);
    double rotationAdjust = rotationController.calculate(limelightSubsystem.getTagX(), 0);
    swerveSubsystem.drive(new Translation2d(fowardAdjust, 0), rotationAdjust, true);

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
}
