package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;

public class TuckArm extends Command {

    private ArmSubsystem arm;
    private boolean reachedSetpoint;
    private boolean safeToTuck = false;

    public TuckArm(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (arm.getEncoderPosition() >= Arm.tuckSafeMin && arm.getEncoderPosition() <= Arm.tuckSafeMax) {
            safeToTuck = true;
        }
    }

    @Override
    public void execute() {
        if (safeToTuck) {
            if (!arm.isTucked())//might need a delay to untuck here
                arm.tuckShooter();

            reachedSetpoint = arm.moveToAngle(Arm.tuckArmAngle);
        } else {
            // move to safe tuck pos
            safeToTuck = arm.moveToAngle(Arm.tuckSafeMin);
        }
        System.out.println("is it safe to tuck: " + safeToTuck);
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
