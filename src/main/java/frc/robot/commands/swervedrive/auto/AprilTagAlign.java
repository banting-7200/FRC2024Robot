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

  private final PIDController controller;

  private double tagArea;
  private double targetArea = 2;

  public AprilTagAlign(SwerveSubsystem swerveSubsystem, LimelightDevice limelightSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    controller = new PIDController(1, 0, 0);
    controller.setSetpoint(targetArea);

    addRequirements(swerveSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    tagArea = limelightSubsystem.getTagArea();
    double fowardAdjust = controller.calculate(tagArea, targetArea);
    swerveSubsystem.drive(new Translation2d(fowardAdjust, 0), 0, true);

    SmartDashboard.putNumber("tagArea", tagArea);
    SmartDashboard.putNumber("targetArea", targetArea);
    SmartDashboard.putBoolean("at setpoint", controller.atSetpoint());
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
