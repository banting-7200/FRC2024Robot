package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LightSubsystem.lightStates;
import frc.robot.subsystems.ShooterSubsystem;
import java.time.Clock;

public class intakeCommand extends Command {
  public ShooterSubsystem shooter;
  double intakeRPM;
  double pullBackRPM;
  double correctPositioningRPM;
  Clock currentTime = Clock.systemDefaultZone();
  long startedMillis = 0;
  boolean override = false;
  int openOrClosedCounter = 0;
  boolean stopDryRun = false;
  boolean notelock = false;
  boolean notelock2 = false;
  boolean shooterHasNotePrev = false;
  boolean hasNote;

  private LightSubsystem lights = LightSubsystem.getInstance();

  public intakeCommand(
      double intakeRPM,
      double pullBackRPM,
      double correctPositioningRPM,
      ShooterSubsystem shooter) {
    this.intakeRPM = intakeRPM;
    this.pullBackRPM = pullBackRPM;
    this.correctPositioningRPM = correctPositioningRPM;
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    System.out.println("I GOT TO INIT");
    startedMillis = 0;
    override = false;
    openOrClosedCounter = 0;
    stopDryRun = false;
    notelock = false;
    notelock2 = false;
    shooterHasNotePrev = false;
  }

  @Override
  public void execute() {
    /* if (hasNote == true) { */
    if (notelock == false) {
      shooter.spinIntakeToNegativeRPM(intakeRPM);
      if (shooter.shooterHasNote() == true && override == false) {
        startedMillis = currentTime.millis() + 100;
        stopDryRun = true;
      }
      if (shooter.shooterHasNote() == true && override == true) {
        notelock = true;
      }
      if (currentTime.millis() > startedMillis && stopDryRun == true) {
        shooter.spinIntakeToPositiveRPM(pullBackRPM);
        override = true;
      }
    } else {
      if (notelock2 == false) {
        shooter.spinIntakeToNegativeRPM(correctPositioningRPM);
        if (shooter.shooterHasNote() == false && shooterHasNotePrev == true) {
          notelock2 = true;
        }
      } else {
        shooter.spinIntakeToNegativeRPM(0);
      }
    }
    shooterHasNotePrev = shooter.shooterHasNote();
    /* } */
  }

  public boolean isFinished() {
    return shooter.shooterHasNote() == false && openOrClosedCounter == 1;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopIntakeMotor();
    lights.SetLightState(lightStates.NotePickedUp);
    System.out.println("Intake Command ShutDown");
    System.out.println("Has Note state is currently: " + shooter.getHasNoteState());
    /*  if (!interrupted) {
      shooter.setHasNoteState(true);
    } */
  }
}

// Jas sux
