package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm extends Command {
    private ArmSubsystem arm;
    private DoubleSupplier axis;

    public MoveArm(ArmSubsystem arm, DoubleSupplier axis) {
        this.arm = arm;
        this.axis = axis;

        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        arm.setMotor(axis.getAsDouble() * Arm.motorSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopArm();
        arm.enableBrake();
    }
}
