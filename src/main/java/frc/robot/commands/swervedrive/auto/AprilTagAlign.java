package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTags;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;
import frc.robot.subsystems.Vision.AprilTagSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.time.Clock;
import java.util.function.IntSupplier;

public class AprilTagAlign extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final AprilTagSubsystem limelightSubsystem;

  private ShuffleboardSubsystem shuffle;

  private final PIDController positionController;
  private final PIDController rotationController;

  private double tagArea;
  private double targetArea;

  private double tagYaw;

  private IntSupplier tagToAlign;

  private Clock currentTime = Clock.systemDefaultZone();
  private long startedMillis;
  private long currentMillis;

  boolean onlyRotate;

  private NoteAutoStateMachine stateInstance;

  public AprilTagAlign(
      SwerveSubsystem swerveSubsystem,
      AprilTagSubsystem limelightSubsystem,
      double targetArea,
      IntSupplier tagToAlign,
      boolean onlyRotate) {
    this.swerveSubsystem = swerveSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    this.targetArea = targetArea;
    this.tagToAlign = tagToAlign;

    this.onlyRotate = onlyRotate;

    positionController = new PIDController(4.7, 0.001, 0);
    positionController.setSetpoint(targetArea);

    rotationController = new PIDController(0.06, 0.00008, 0.01);
    rotationController.setSetpoint(0);

    addRequirements(swerveSubsystem, limelightSubsystem);
  }

  public AprilTagAlign(
      SwerveSubsystem swerveSubsystem,
      AprilTagSubsystem limelightSubsystem,
      double targetArea,
      int tagToAlign,
      boolean onlyRotate) {
    this(swerveSubsystem, limelightSubsystem, targetArea, () -> tagToAlign, onlyRotate);
  }

  public AprilTagAlign(
      SwerveSubsystem swerveSubsystem,
      AprilTagSubsystem limelightSubsystem,
      double targetArea,
      int tagToAlign,
      boolean onlyRotate,
      NoteAutoStateMachine stateInstance) {
    this(swerveSubsystem, limelightSubsystem, targetArea, () -> tagToAlign, onlyRotate);
    this.stateInstance = stateInstance;
  }

  @Override
  public void initialize() {
    shuffle = ShuffleboardSubsystem.getInstance();
    startedMillis = currentTime.millis();
    System.out.println("Started INTI for April Tag Align");
    System.out.println("Current tag target ID: " + limelightSubsystem.getTagID());
  }

  @Override
  public void execute() {
    System.out.println("Currently Executing Tag Align command");

    currentMillis = currentTime.millis(); // records current time

    double fowardAdjust = 0;
    double rotationAdjust = 0;

    if (tagToAlign.getAsInt() == limelightSubsystem.getTagID()) {
      tagArea =
          limelightSubsystem
              .getTagArea(); // Todo: Might have to change translation from being dependant on tag
      // area to being dependant on distance. Confirm acuracy with further
      tagYaw = limelightSubsystem.getTagX();
      // testing.
      if (!onlyRotate) fowardAdjust = positionController.calculate(tagArea, targetArea);
      rotationAdjust = rotationController.calculate(tagYaw, 0);
      swerveSubsystem.drive(new Translation2d(-fowardAdjust, 0), rotationAdjust, false);
    } else {
      swerveSubsystem.drive(new Translation2d(0, 0), 2, false);
    }
    shuffle.setTab("Debugging");
    shuffle.setNumber("Tag Area", tagArea);
    shuffle.setNumber("Target Area", targetArea);
    shuffle.setBoolean("At Position?", positionController.atSetpoint());

    shuffle.setNumber("Tag Offset", rotationAdjust);
    shuffle.setBoolean("At Rotation?", rotationController.atSetpoint());
  }

  @Override
  public boolean isFinished() {
    return (tagArea >= AprilTags.maxDist && tagArea <= AprilTags.minDist)
        && (tagYaw > -0.2 && tagYaw < 0.2)
    /* || currentMillis - startedMillis > maxCommandWaitTime.aprilTagAlignWaitTime */ ;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.setOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    swerveSubsystem.lock();
    // swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
    /*  if (stateInstance != null) {
      stateInstance.MoveToState(NoteAutoStateMachine.States.Shoot);
    } */
    if (!interrupted) {
      System.out.println("Ended Tag Align successfully");
    } else {
      System.out.println("Interrupted Tag Align Commmand");
    }
  }
}
