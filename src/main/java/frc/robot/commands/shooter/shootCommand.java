package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.time.Clock;

public class shootCommand extends Command {

    public ShooterSubsystem shooter;
    private boolean hasNotBeenDetected = false;
    Clock currentTime = Clock.systemDefaultZone();
    long startedMillis = currentTime.millis();
    long currentMillis;
    long sinceNoteLeft;
    long sinceIntakeMotor;
    int rpm;
    Boolean hasSeenNote;
    

  public shootCommand(int rpm, ShooterSubsystem shooter) {
    this.rpm = rpm;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    currentMillis = currentTime.millis();
    shooter.spinIntakeToRPM(rpm);
    shooter.spinShootToRPM(rpm);
    if (shooter.hasNote() == true) {
      sinceNoteLeft = currentTime.millis();
      System.out.println("Note still inside");
    }
  }

    @Override
    public void initialize() {
        sinceIntakeMotor = currentTime.millis();
        sinceNoteLeft = currentTime.millis();

    }

    @Override
    public void execute() {
        currentMillis = currentTime.millis(); // records current time
        shooter.spinShootToRPM(rpm); // spins the shooters
        if ((currentMillis - sinceIntakeMotor) > 1000) { // waits for 250 ms for it to turn on the shoot motor
            shooter.spinIntakeToNegativeRPM(rpm); // runs the shoot motor
            System.out.println("Run Shooter motor");
        }
        if (shooter.shooterHasNote() == true) {
            sinceNoteLeft = currentTime.millis(); // if it see's the note it will set the since note left time for current time
            hasSeenNote = true;

        }

    }

    public boolean isFinished() {
        return (currentMillis - sinceNoteLeft) > 1000 && hasSeenNote == true;

    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShootMotor();
        shooter.stopIntakeMotor();
        System.out.println("Shooting Done");
    }
}
