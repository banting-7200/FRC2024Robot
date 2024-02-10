package frc.robot.commands.shooter;

import java.time.Clock;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class readyNoteCommand extends Command {
    private ShooterSubsystem shooter;
    Clock currentTime = Clock.systemDefaultZone();
    long currentMillis;
    long intakeActivatedMillis;
    boolean hasBeenStowed = false;
    long timeHasBeenIn = 0;
    boolean isReadyNextStage = false;

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (shooter.hasNote() == true) {
            hasBeenStowed = true;
            intakeActivatedMillis = currentTime.millis();
        }
        currentMillis = currentTime.millis();
        if (hasBeenStowed == true
                && (intakeActivatedMillis - 10 < currentMillis && currentMillis < intakeActivatedMillis + 10)) {
            shooter.spinIntakeToRPM(-30);
            timeHasBeenIn = currentMillis - intakeActivatedMillis;
        }
    }

    public boolean isFinished() {
        return timeHasBeenIn > 250;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopIntakeMotor();
    }

}
