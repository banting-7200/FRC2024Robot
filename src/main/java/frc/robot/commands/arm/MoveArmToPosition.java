package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;

public class MoveArmToPosition extends Command {

  private ArmSubsystem arm;
  private LightSubsystem lights = LightSubsystem.getInstance();
  private boolean reachedSetpoint;
  // private boolean safeToMove = true;

  private double /*DoubleSupplier*/ angleSetpoint;

  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  public MoveArmToPosition(ArmSubsystem arm, double /*DoubleSupplier*/ angleSetpoint) {
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
    System.out.println("Move Arm To Position. setpoint is: " + angleSetpoint);
    // see if setting set point here is good
  }

  @Override
  public void execute() {
    reachedSetpoint = arm.moveToAngle(angleSetpoint);
    shuffle.setNumber("command setpoint", angleSetpoint);
    shuffle.setBoolean("reached setpoint", reachedSetpoint);
    // System.out.println("execute arm");
  }

  // Might need an angle supplier function if an object of this command is created
  // thats constanly referenced

  @Override
  public boolean isFinished() {
    return reachedSetpoint;
  }

  @Override
  public void end(boolean interrupted) {
    arm.enableBrake();
    arm.stopArm();
    System.out.println("Move to position command finished. interupted: " + interrupted);
  }
}
