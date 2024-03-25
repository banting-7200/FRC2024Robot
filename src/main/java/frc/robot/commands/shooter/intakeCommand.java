/*
 * Essentially this command is supposed to pull in the note and once it does
 * it will pull it back and then correct it until the IR sensor sees it and
 * doesn't see it again. Therefore why there are three different 3 different
 * RPM's.
 */

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.maxCommandWaitTime;
import frc.robot.subsystems.ArmAndHead.ShooterSubsystem;
import frc.robot.subsystems.Feedback.LightSubsystem;
import frc.robot.subsystems.Feedback.LightSubsystem.LightStates;
import java.time.Clock;

public class intakeCommand extends Command {
  public ShooterSubsystem shooter;
  double intakeRPM;

  Clock currentTime = Clock.systemDefaultZone();
  long startedMillis = 0;
  boolean stopDryRun = false;
  boolean readyNote = false;
  long commandInitMillis;
  long seenMillis = 0;

  XboxController driveController;

  private LightSubsystem lights = LightSubsystem.getInstance();

  public intakeCommand(double intakeRPM, ShooterSubsystem shooter) {
    this.intakeRPM = intakeRPM;
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // System.out.println("Has Note state is currently@IntakeStart: " + shooter.getHasNoteState());
    // System.out.println("I GOT TO INIT");
    startedMillis = 0;
    stopDryRun = false;
    commandInitMillis = currentTime.millis();
    readyNote = shooter.shooterHasNote();
  }

  @Override
  public void execute() {
    /* if (readyNote == false) { */

    shooter.spinIntakeToNegativeRPM(intakeRPM); // intake
    if (shooter.shooterHasNote() && seenMillis == 0) {
      seenMillis = currentTime.millis();
    }
    /*     if (shooter.shooterHasNote() && stopDryRun == false) {
      startedMillis =
          currentTime.millis() + 100; // run for an extra 100ms after note has been detected
      stopDryRun = true; // 100ms is finished
      System.out.println("Stage 1@intakeExecute");
    }
    if (currentTime.millis() > startedMillis
        && stopDryRun == true) { // If the timer has elapsed and we have entered
      // the dry run
      shooter.spinIntakeToPositiveRPM(0); // Pull back the note
      shooter.setHasNoteState(true);
      System.out.println("Stage 2@intakeExecute");
    } */
    // System.out.println("Current timer: " + (currentTime.millis() - startedMillis));
  }

  /* } */

  public boolean isFinished() {
    return (shooter.shooterHasNote() && currentTime.millis() - seenMillis > 500)
        || (currentTime.millis() - commandInitMillis > maxCommandWaitTime.intakeCommandWaitTime);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopIntakeMotor();
    if (shooter.shooterHasNote()) lights.SetLightState(LightStates.NotePickedUp);
    // System.out.println("Intake Command ShutDown");
    // System.out.println("Has Note state is currently@IntakeEnd: " + shooter.getHasNoteState());
    /*
     * if (!interrupted) {
     * shooter.setHasNoteState(true);
     * }
     */
  }
}
// Jas sux
