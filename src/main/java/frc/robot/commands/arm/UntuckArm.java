package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;

public class UntuckArm extends Command {

  private ArmSubsystem arm;
  private boolean ranUntuckCommand = false;

  public UntuckArm(ArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.disableBrake();
    System.out.println("Untuck Arm");
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
    System.out.println("is it safe to untuck: " + (arm.getEncoderPosition() >= Arm.tuckSafeMin));
  }

  @Override
  public boolean isFinished() {
    return !arm.isTucked();
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
      }
}
