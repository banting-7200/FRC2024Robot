package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.Constants.maxCommandWaitTime;
import frc.robot.subsystems.ArmAndHead.ArmSubsystem;
import java.time.Clock;

public class UntuckArm extends Command {

  private ArmSubsystem arm;
  private boolean ranUntuckCommand = false;

  Clock timer = Clock.systemDefaultZone();
  long startTime;

  public UntuckArm(ArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.disableBrake();
    System.out.println("Untuck Arm");
    startTime = timer.millis();
    // System.out.println("start time is: " + startTime);
    ranUntuckCommand = false;
  }

  @Override
  public void execute() {
    if (arm.getEncoderPosition() >= Arm.tuckSafeMin) {
      if (!ranUntuckCommand) {
        arm.deployShooter();
        ranUntuckCommand = true;
      }
    } else {
      // move to safe tuck pos
      arm.moveToAngle(Arm.tuckSafeMin + 2);
    }
    // System.out.println("is it safe to untuck: " + (arm.getEncoderPosition() >= Arm.tuckSafeMin));
  }

  @Override
  public boolean isFinished() {
    return !arm.isTucked() || (timer.millis() - startTime) > maxCommandWaitTime.unTuckWaitTime;
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
    // System.out.println("elapsed time is: " + (timer.millis() - startTime));
  }
}
