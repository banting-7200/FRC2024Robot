package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import java.time.Clock;
import java.util.function.DoubleSupplier;

public class MoveArmToPosition extends Command {

  private ArmSubsystem arm;
  private LightSubsystem lights = LightSubsystem.getInstance();
  private boolean reachedSetpoint;
  // private boolean safeToMove = true;

  private DoubleSupplier angleSetpoint;

  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  Clock timer = Clock.systemDefaultZone();
  long startTime;

  public MoveArmToPosition(ArmSubsystem arm, DoubleSupplier angleSetpoint) {
    this.arm = arm;
    this.angleSetpoint = angleSetpoint;

    addRequirements(arm);
  }

  public MoveArmToPosition(ArmSubsystem arm, double angleSetpoint) {
    this(arm, () -> angleSetpoint);
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
    System.out.println("Move Arm To Position. setpoint is: " + angleSetpoint.getAsDouble());
    startTime = timer.millis();
    System.out.println("start time is: " + startTime);
    // see if setting set point here is good
  }

  @Override
  public void execute() {
    reachedSetpoint = arm.moveToAngle(angleSetpoint.getAsDouble());
    shuffle.setNumber("command setpoint", angleSetpoint.getAsDouble());
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
    arm.stopArm();
    System.out.println("Move to position command finished. interupted: " + interrupted);
    System.out.println("elapsed time is: " + (timer.millis() - startTime));
  }
}
