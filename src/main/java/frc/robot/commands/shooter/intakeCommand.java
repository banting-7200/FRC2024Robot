package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.time.Clock;

public class intakeCommand extends Command {
  public ShooterSubsystem shooter;
  double rpm;
  Clock currentTime = Clock.systemDefaultZone();
  long startedMillis = 0;
  boolean override = false;
  int openOrClosedCounter = 0;
  boolean stopDryRun = false;
  boolean notelock = false;
  boolean notelock2 = false;
  boolean shooterHasNotePrev = false;

  public intakeCommand(double rpm, ShooterSubsystem shooter) {
    this.rpm = rpm;
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    System.out.println("I GOT TO INIT");
  }

  @Override
  public void execute() {

    if (notelock == false) {
      shooter.spinIntakeToNegativeRPM(6000);
      if (shooter.shooterHasNote() == true && override == false) {
        startedMillis = currentTime.millis() + 100;
        stopDryRun = true;
      }
      if (shooter.shooterHasNote() == true && override == true) {
        notelock = true;
      }
      if (currentTime.millis() > startedMillis && stopDryRun == true) {
        shooter.spinIntakeToPositiveRPM(1000);
        override = true;
      }
    } else {
      if (notelock2 == false) {
        shooter.spinIntakeToNegativeRPM(2000);
        if (shooter.shooterHasNote() == false && shooterHasNotePrev == true) {
          notelock2 = true;
        }
      } else {
        shooter.spinIntakeToNegativeRPM(0);
      }
    }
    shooterHasNotePrev = shooter.shooterHasNote();
  }

  public boolean isFinished() {
    return shooter.shooterHasNote() == false && openOrClosedCounter == 1;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopIntakeMotor();
    System.out.println("Intake Command ShutDown");
  }
}

// Jas sux
