package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;

public class MoveArmToIntake extends Command {

    private ArmSubsystem arm;
    private boolean reachedSetpoint;
    private boolean safeToMove = false;

    public MoveArmToIntake(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (arm.isTucked()) {
            if (arm.getEncoderPosition() >= Arm.tuckSafeMin && arm.getEncoderPosition() <= Arm.tuckSafeMax) {
                safeToMove = true;
            }
        }
    }

    @Override
    public void execute() {
        if (safeToMove) {
            if (arm.isTucked())//might need a delay to untuck here
                arm.deployShooter();

            reachedSetpoint = arm.moveToAngle(Arm.intakeArmAngle);
        } else {
            // move to safe untuck pos
            safeToMove = arm.moveToAngle(Arm.tuckSafeMin);
        }
        System.out.println("is it safe to move: " + safeToMove);
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
