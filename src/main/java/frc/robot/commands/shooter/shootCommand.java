package frc.robot.commands.shooter;

import java.time.Clock;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class shootCommand extends Command {

    public ShooterSubsystem shooter = new ShooterSubsystem(new ArmSubsystem());
    private boolean hasNotBeenDetected = false;
    Clock currentTime = Clock.systemDefaultZone();
    long startedMillis = currentTime.millis();
    long currentMillis;
    long sinceNoteLeft;
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
        shooter.spinShootToRPM(rpm);
        if(shooter.hasNote() == true){
            sinceNoteLeft = currentTime.millis();
        }
    }

    public boolean isFinished() {
        return shooter.hasNote() == false && currentMillis - sinceNoteLeft > 5000;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShootMotor();
        shooter.stopIntakeMotor();
    }
}
