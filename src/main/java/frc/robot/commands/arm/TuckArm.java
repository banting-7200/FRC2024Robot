package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;
import java.time.Clock;

public class TuckArm extends Command {

  private ArmSubsystem arm;
  private boolean ranTuckCommand = false;

  Clock timer = Clock.systemDefaultZone();
  long startTime;

  public TuckArm(ArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.disableBrake();
    System.out.println("Tuck Arm");
    startTime = timer.millis();
    System.out.println("start time is: " + startTime);
    ranTuckCommand = false;
  }

  @Override
  public void execute() {
    //System.out.println(ranTuckCommand);
    if (arm.getEncoderPosition() >= Arm.tuckSafeMin) {
      if (!ranTuckCommand) {
        arm.tuckShooter();
        ranTuckCommand = true;
      }
    } else {
      // move to safe tuck pos
      arm.moveToAngle(Arm.tuckSafeMin + 2);
    }
    //System.out.println("is it safe to tuck: " + (arm.getEncoderPosition() >= Arm.tuckSafeMin));
  }

  @Override
  public boolean isFinished() {
    return arm.isTucked();
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
    System.out.println("elapsed time is: " + (timer.millis() - startTime));
  }
}
