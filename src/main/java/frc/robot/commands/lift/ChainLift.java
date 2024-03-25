package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmAndHead.ArmSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ChainLift extends Command {
  private ArmSubsystem arm;
  private SwerveSubsystem swerve;

  private boolean reachedSetpoint = false;

  public ChainLift(ArmSubsystem arm, SwerveSubsystem swerve) {
    this.arm = arm;
    this.swerve = swerve;

    addRequirements(arm, swerve);
  }

  @Override
  public void initialize() {
    swerve.setMotorBrake(true);
    if (!arm.isHookDeployed()) arm.deployHook();
  }

  @Override
  public void execute() {
    reachedSetpoint = arm.moveToAngle(Arm.liftArmAngle, Arm.stopRange);
  }

  @Override
  public boolean isFinished() {
    return reachedSetpoint;
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
  }
}
