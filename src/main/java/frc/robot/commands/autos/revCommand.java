package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAndHead.ShooterSubsystem;
import java.time.Clock;

public class revCommand extends Command {

  public ShooterSubsystem shooter;
  Clock currentTime = Clock.systemDefaultZone();

  long startedMillis;
  long currentMillis;
  long passedMillis;

  int commandDuration = 500;

  public revCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    startedMillis = currentTime.millis();
    shooter.stopIntakeMotor();
    shooter.stopShootMotor();
  }

  @Override
  public void execute() {
    currentMillis = currentTime.millis();
    passedMillis = currentMillis - startedMillis;

    if (passedMillis < commandDuration) {
      shooter.spinShootToRPM(5000);
    }
  }

  public boolean isFinished() {
    return passedMillis > commandDuration;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      shooter.stopShootMotor();
    }
  }
}
