package frc.robot.commands.shooter;

import java.time.Clock;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class shootCommand extends Command {

    public ShooterSubsystem shooter;
    private boolean hasNotBeenDetected = false;
    Clock currentTime = Clock.systemDefaultZone();
    long startedMillis = currentTime.millis();
    long currentMillis;
    int rpm;

    public shootCommand(int rpm) {
        this.rpm = rpm;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        currentMillis = currentTime.millis();
        shooter.spinIntakeToRPM(rpm);
        if (currentMillis - startedMillis > 5000) {
            shooter.spinShooterToRPM(rpm);
        }
    }

    public boolean isFinished() {
        return false; // shooter.hasNote() == false && currentMillis > startedMillis - 50;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShootMotor();
        shooter.stopIntakeMotor();
    }
}
