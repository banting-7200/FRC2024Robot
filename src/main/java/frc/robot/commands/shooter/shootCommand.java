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
  int rpm;

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

  public boolean isFinished() {
    return shooter.hasNote() == false && currentMillis - sinceNoteLeft > 3000;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShootMotor();
    shooter.stopIntakeMotor();
  }
}
