package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAndHead.ArmSubsystem;
import frc.robot.subsystems.Vision.LimelightDevice;

public class LimelightArmMovement extends Command {

  private ArmSubsystem arm;
  private LimelightDevice limelight;

  private boolean reachedSetpoint = false;
  private boolean isBrakeEnabled = false;

  public LimelightArmMovement(ArmSubsystem arm, LimelightDevice limelight) {
    this.arm = arm;
    this.limelight = limelight;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.disableBrake();
  }

  @Override
  public void execute() {
    if(reachedSetpoint && !isBrakeEnabled){
      arm.stopArm();
      isBrakeEnabled = true;
    }else if(isBrakeEnabled){
      arm.disableBrake();
      isBrakeEnabled = false;
    }

    reachedSetpoint = arm.moveToAngle(limelight.calculateArmShootAngle());
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
    System.out.println("Move to position command finished. interupted: " + interrupted);
  }
}
