package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LightSubsystem.lightStates;

import java.time.Clock;
import frc.robot.subsystems.LightSubsystem;

public class readyNoteCommand extends Command {
  public ShooterSubsystem shooter;
  Clock currentTime = Clock.systemDefaultZone();
  long currentMillis;
  long intakeActivatedMillis = currentTime.millis();
  boolean hasBeenStowed = false;
  long timeHasBeenIn = 0;
  boolean isReadyNextStage = false;
  LightSubsystem lights = LightSubsystem.getInstance();
  int rpm;

  public readyNoteCommand(int rpm, ShooterSubsystem shooter) {
    this.rpm = rpm;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    lights.SetLightState(lightStates.ReadyToSPEAKER);
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
    lights.SetLightState(lightStates.ReadyToSPEAKER);
  }
}
