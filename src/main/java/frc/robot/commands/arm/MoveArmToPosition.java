package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveArmToPosition extends Command {

    private ArmSubsystem arm;
    private boolean reachedSetpoint;
    //private boolean safeToMove = true;

    private double angleSetpoint;

    public MoveArmToPosition(ArmSubsystem arm, int angleSetpoint) {
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
    }

    @Override
    public void execute() {
        reachedSetpoint = arm.moveToAngle(angleSetpoint);
    }

    // Might need an angle supplier function if an object of theis command is crated
    // thats constanly referneced

    @Override
    public boolean isFinished() {
        return reachedSetpoint;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopArm();
    }
}
