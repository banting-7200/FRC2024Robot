package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmAndHead.ArmSubsystem;
import frc.robot.subsystems.ArmAndHead.ShooterSubsystem;
import frc.robot.subsystems.Vision.LimelightDevice;
import java.time.Clock;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

public class LimelightArmMovement extends Command {

  private ArmSubsystem arm;
  private LimelightDevice limelight;
  private ShooterSubsystem shooter;

  private BooleanSupplier limelightButton;

  private boolean inAlignMode = true;
  private boolean hasSeenNote = false;
  private boolean stoppedPullBack = false;

  private IntSupplier rpm;

  Clock currentTime = Clock.systemDefaultZone();

  long startedMillis; // time when started
  long sinceNoteLeft; // time since note left shooter

  public LimelightArmMovement(
      ArmSubsystem arm,
      LimelightDevice limelight,
      ShooterSubsystem shooter,
      IntSupplier rpm,
      BooleanSupplier limelightButton) {
    this.arm = arm;
    this.limelight = limelight;
    this.shooter = shooter;

    this.rpm = rpm;
    this.limelightButton = limelightButton;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    inAlignMode = true;
    hasSeenNote = false;
    stoppedPullBack = false;

    sinceNoteLeft = 0;

    startedMillis = currentTime.millis();
    arm.disableBrake();
  }

  @Override
  public void execute() {
    if (currentTime.millis() - startedMillis < 100) { // until 100 millis pass
      shooter.spinIntakeToPositiveRPM(2000); // reverse intake
      shooter.spinShootNegativeToRPM(500);
    } else if (!stoppedPullBack) {
      shooter.spinShootToRPM(rpm.getAsInt());
      shooter.stopIntakeMotor();
      stoppedPullBack = true;
    }

    if (!limelightButton.getAsBoolean() && inAlignMode) {
      arm.stopArm();
      inAlignMode = false;
    }

    if (inAlignMode) {
      arm.moveToAngle(limelight.calculateArmShootAngle(), Arm.limelightStopRange);
    } else {
      shooter.spinIntakeToNegativeRPM(rpm.getAsInt());
      if (shooter.shooterHasNote() && !hasSeenNote) {
        sinceNoteLeft = currentTime.millis();
        hasSeenNote = true;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return ((currentTime.millis() - sinceNoteLeft) > 500 && hasSeenNote);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShootMotor();
    shooter.stopIntakeMotor();
    System.out.println("Move to limelight position command finished. interupted: " + interrupted);
  }
}
