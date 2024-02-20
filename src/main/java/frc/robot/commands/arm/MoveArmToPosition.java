package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPosition extends Command {

  private ArmSubsystem arm;
  private boolean reachedSetpoint;
  // private boolean safeToMove = true;

  private double angleSetpoint;

  public MoveArmToPosition(ArmSubsystem arm, double angleSetpoint) {
    this.arm = arm;
    this.angleSetpoint = angleSetpoint;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    /*
     * if (arm.isTucked()) {
     * new UntuckArm(arm);
     * }
     */
    // supplier assignment goes here
    arm.disableBrake();
    System.out.println("Move Arm To Position");
  }

  @Override
  public void execute() {
    reachedSetpoint = arm.moveToAngle(angleSetpoint);
  }

  // Might need an angle supplier function if an object of theis command is crated
  // thats constanly referneced

  @Override
  public boolean isFinished() {
    System.out.println("At Setpoint");
    return reachedSetpoint;
  }

  @Override
  public void end(boolean interrupted) {
    arm.enableBrake();
    arm.stopArm();
  }
}
