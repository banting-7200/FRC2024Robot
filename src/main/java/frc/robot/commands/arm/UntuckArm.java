package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;

public class UntuckArm extends Command {

  private ArmSubsystem arm;
  private boolean reachedSetpoint;
  private boolean safeToUntuck = false;
  private boolean ranUntuckCommand = false;

  public UntuckArm(ArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    System.out.println("Untuck Arm");
    if (!arm.isTucked()) {
      reachedSetpoint = true;
    } else if (arm.getEncoderPosition() >= Arm.tuckSafeMin
        && arm.getEncoderPosition() <= Arm.tuckSafeMax) {
      safeToUntuck = true;
    }
    arm.disableBrake();
  }

  @Override
  public void execute() {
    if (safeToUntuck) {
      if (!ranUntuckCommand) {
        arm.deployShooter();
        ranUntuckCommand = true;
      } else if (!arm.isTucked()) {
        reachedSetpoint = true;
      }
    } else {
      // move to safe tuck pos
      safeToUntuck = arm.moveToAngle(Arm.tuckSafeMin);
    }
    System.out.println("is it safe to untuck: " + safeToUntuck);
  }

  @Override
  public boolean isFinished() {
    return reachedSetpoint;
  }

  @Override
  public void end(boolean interrupted) {
    arm.enableBrake();
    arm.stopArm();
  }
}
