package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAndHead.ShooterSubsystem;
import java.time.Clock;

public class feedCommand extends Command {

  public ShooterSubsystem shooter;
  Clock currentTime = Clock.systemDefaultZone();

  long startedMillis;
  long currentMillis;
  long passedMillis;

  long commandDuration = 500;

  public feedCommand(ShooterSubsystem shooter){
  }

  @Override
  public void initialize() {
    startedMillis = currentTime.millis();
    shooter.stopIntakeMotor();
  }

  @Override
  public void execute() {
    currentMillis = currentTime.millis();
    passedMillis = currentMillis - startedMillis;

    if (passedMillis < commandDuration) {
      shooter.spinIntakeToNegativeRPM(5000);
      shooter.spinShootToRPM(1000);
    }
  }

  public boolean isFinished() {
    return !shooter.shooterHasNote() && passedMillis > commandDuration;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShootMotor();
    shooter.stopIntakeMotor();
  }
}
