/*
 * Essentially this command is supposed to pull in the note and once it does
 * it will pull it back and then correct it until the IR sensor sees it and
 * doesn't see it again. Therefore why there are three different 3 different
 * RPM's.
 */

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.maxCommandWaitTime;
import frc.robot.subsystems.ArmAndHead.ShooterSubsystem;
import frc.robot.subsystems.Feedback.LightSubsystem;
import frc.robot.subsystems.Feedback.LightSubsystem.LightStates;
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
  long commandInitMillis;

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
    commandInitMillis = currentTime.millis();
    hasNote = shooter.getHasNoteState();
  }

  @Override
  public void execute() {
    if (hasNote == false) {
      /* Get note into position */
      if (notelock == false) { // if note is not in position
        shooter.spinIntakeToNegativeRPM(intakeRPM); // intake
        if (shooter.shooterHasNote() == true && override == false) {
          startedMillis = currentTime.millis() + 100; // run for an extra 100ms after note has been detected
          stopDryRun = true; // 100ms is finished
        }
        if (shooter.shooterHasNote() == true && override == true) {// If we have the note and can override
          notelock = true;// enable the first lock
        }
        if (currentTime.millis() > startedMillis && stopDryRun == true) {// If the timer has elapsed and we have entered
                                                                         // the dry run
          shooter.spinIntakeToPositiveRPM(pullBackRPM);// Pull back the note
          override = true;
        }
      } else {
        if (notelock2 == false) {// If we have not started correcting
          shooter.spinIntakeToNegativeRPM(correctPositioningRPM);// Start Correcting
          if (shooter.shooterHasNote() == false && shooterHasNotePrev == true) {// If we no longer have a note, but a
                                                                                // note was detected last robot loop
            notelock2 = true;
          }
        } else {// Stop correcting
          shooter.spinIntakeToNegativeRPM(0);
        }
      }
      shooterHasNotePrev = shooter.shooterHasNote();// Store the status of the note this loop to check against the next
                                                    // loop.
    }
  }

  public boolean isFinished() {
    return (shooter.shooterHasNote() == false && openOrClosedCounter == 1)
        || (currentTime.millis() - commandInitMillis > maxCommandWaitTime.intakeCommandWaitTime);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopIntakeMotor();
    lights.SetLightState(LightStates.NotePickedUp);
    System.out.println("Intake Command ShutDown");
    System.out.println("Has Note state is currently: " + shooter.getHasNoteState());
    if (!interrupted) {
      shooter.setHasNoteState(true);
    }
  }
}

// Jas sux