package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.Constants.maxCommandWaitTime;
import frc.robot.subsystems.ArmAndHead.ArmSubsystem;
import frc.robot.subsystems.Feedback.LightSubsystem;
import java.time.Clock;

public class TuckArm extends Command {

  private ArmSubsystem arm;
  private boolean ranTuckCommand = false;
  private LightSubsystem lights = LightSubsystem.getInstance();

  Clock timer = Clock.systemDefaultZone();
  long startTime;

  public TuckArm(ArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.disableBrake();
    System.out.println("Started Tuck Arm");
    startTime = timer.millis();
    // System.out.println("start time is: " + startTime);
    ranTuckCommand = false;
  }

  @Override
  public void execute() {
    if (arm.getEncoderPosition() >= Arm.tuckSafeMin) {
      if (!ranTuckCommand) {
        arm.tuckShooter();
        ranTuckCommand = true;
      }
    } else {
      // move to safe tuck pos
      arm.moveToAngle(Arm.tuckSafeMin + 1);
    }
    // System.out.println("is it safe to tuck: " + (arm.getEncoderPosition() >=
    // Arm.tuckSafeMin));
  }

  @Override
  public boolean isFinished() {
    return arm.isTucked() || (timer.millis() - startTime) > maxCommandWaitTime.tuckArmWaitTime;
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
    System.out.println("Ended Tuck Arm");
    /*
     * if (!interrupted) {
     * lights.SetLightState(LightStates.CarryingNote);
     * }
     */
    // System.out.println("elapsed time is: " + (timer.millis() - startTime));
  }
}
