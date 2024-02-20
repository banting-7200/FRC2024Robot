package frc.robot.commands.shooter;

import java.time.Clock;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class readyNoteCommand extends Command {
    public ShooterSubsystem shooter;
    Clock currentTime = Clock.systemDefaultZone();
    long currentMillis;
    long intakeActivatedMillis = currentTime.millis();
    boolean hasBeenStowed = false;
    long timeHasBeenIn = 0;
    boolean isReadyNextStage = false;
    int rpm;

    
    public readyNoteCommand(int rpm, ShooterSubsystem shooter) {
        this.rpm = rpm;
        this.shooter = shooter;
    }




    @Override
    public void initialize() {
        System.out.println("Time at ready note was activated! " + intakeActivatedMillis);

    }

    @Override
    public void execute() {
        currentMillis = currentTime.millis();
        shooter.spinIntakeToRPM(-rpm);
        timeHasBeenIn = currentMillis - intakeActivatedMillis;
    }

    public boolean isFinished() {
        return timeHasBeenIn > 250;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopIntakeMotor();
        System.out.println("Ready Note Command ShutDown");
    }

}