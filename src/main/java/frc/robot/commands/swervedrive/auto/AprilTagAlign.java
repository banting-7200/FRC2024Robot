package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.maxCommandWaitTime;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;
import frc.robot.subsystems.Vision.LimelightDevice;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.time.Clock;
import java.util.function.IntSupplier;

public class AprilTagAlign extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final LimelightDevice limelightSubsystem;
  private ShuffleboardSubsystem shuffle;

  private final PIDController positionController;
  private final PIDController rotationController;

  private double tagArea;
  private double targetArea;

  private IntSupplier tagToAlign;

  private Clock currentTime = Clock.systemDefaultZone();
  private long startedMillis;
  private long currentMillis;

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
  public void initialize() {
    shuffle = ShuffleboardSubsystem.getInstance();
    startedMillis = currentTime.millis();
  }

  @Override
  public void execute() {

    currentMillis = currentTime.millis(); // records current time

    double fowardAdjust = 0;
    double rotationAdjust = 0;

    if (tagToAlign.getAsInt() == limelightSubsystem.getTagID()) {
      tagArea = limelightSubsystem.getTagArea();
      fowardAdjust = positionController.calculate(tagArea, targetArea);
      rotationAdjust = rotationController.calculate(limelightSubsystem.getTagX(), 0);
      swerveSubsystem.drive(new Translation2d(fowardAdjust, 0), rotationAdjust, false);
    }
    shuffle.setTab("Debug");
    shuffle.setNumber("Tag Area", tagArea);
    shuffle.setNumber("Target Area", targetArea);
    shuffle.setBoolean("At Position?", positionController.atSetpoint());

    shuffle.setNumber("Tag Offset", rotationAdjust);
    shuffle.setBoolean("At Rotation?", rotationController.atSetpoint());
  }

  @Override
  public boolean isFinished() {
    return (positionController.atSetpoint() && rotationController.atSetpoint())
        || currentMillis - startedMillis > maxCommandWaitTime.aprilTagAlignWaitTime;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.lock();
  }
}
