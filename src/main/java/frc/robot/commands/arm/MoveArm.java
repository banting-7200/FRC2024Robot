package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmAndHead.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class MoveArm extends Command {
  private ArmSubsystem arm;
  private DoubleSupplier axis;
  private double rampRate;

  public MoveArm(ArmSubsystem arm, DoubleSupplier axis) {
    this.arm = arm;
    this.axis = axis;
    rampRate = Arm.motorRampRate;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.disableBrake();
  }

  @Override
  public void execute() {

    double motorTargetSpeed = axis.getAsDouble() * arm.motorManualSpeed;
    double motorSpeed = arm.getMotorSpeed();
    if ((motorTargetSpeed - motorSpeed) > rampRate) {
      arm.setMotorSpeed(motorSpeed += rampRate);
    } else if ((motorSpeed - motorTargetSpeed) > rampRate) {
      arm.setMotorSpeed(motorSpeed -= rampRate);
    } else {
      arm.setMotorSpeed(motorTargetSpeed);
    }
  }

  @Override
  public boolean isFinished() {
    return axis.getAsDouble() == 0;
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
  }
}
