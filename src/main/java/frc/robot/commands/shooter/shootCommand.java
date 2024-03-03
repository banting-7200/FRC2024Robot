/*
 * This command is extremely simple as it shoots the note and ends after one second of it leaving
 * the shooter.
 */

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.maxCommandWaitTime;
import frc.robot.subsystems.ArmAndHead.ShooterSubsystem;
import frc.robot.subsystems.Feedback.LightSubsystem;
import frc.robot.subsystems.Feedback.LightSubsystem.LightStates;
import java.time.Clock;
import java.util.function.IntSupplier;

public class shootCommand extends Command {

  public ShooterSubsystem shooter;
  LightSubsystem lights = LightSubsystem.getInstance();
  Clock currentTime = Clock.systemDefaultZone();
  long startedMillis; // time when started
  long currentMillis; // current time
  long sinceNoteLeft; // time since note left shooter
  long sinceIntakeMotor; // time since intake motor turned on
  IntSupplier rpm; // Int supplier for different shoot rpm's(speaker/amp)
  Boolean hasSeenNote = false; // if note has been detected yet
  IntSupplier waitTime; // Int supplier for different wait times(speaker/amp)
  boolean hasNote; // This variables is used for telling whether the note is already in the shooter

  // or

  // not

  public shootCommand(IntSupplier rpm, ShooterSubsystem shooter, IntSupplier waitTime) {
    this.rpm = rpm;
    this.shooter = shooter;
    this.waitTime = waitTime;

    addRequirements(shooter);
  }

  public shootCommand(int rpm, ShooterSubsystem shooter, int waitTime) {
    this(() -> rpm, shooter, () -> waitTime);
  }

  @Override
  public void initialize() {
    sinceNoteLeft = currentTime.millis();
    startedMillis = currentTime.millis();
    hasSeenNote = false;
    hasNote = shooter.getHasNoteState();
    shooter.stopIntakeMotor(); // Stop the intake motor
  }

  @Override
  public void execute() {
    if (hasNote == true) {
      currentMillis = currentTime.millis(); // records current time
      if (currentMillis - startedMillis
          < 100) { // runs until current millis minus started millis until bigger than 100
        shooter.spinIntakeToPositiveRPM(3000);
      } else if ((currentMillis - startedMillis) > waitTime.getAsInt()) {
        // waits for 250 ms for it to turn on the shoot
        // motor
        shooter.spinIntakeToNegativeRPM(rpm.getAsInt()); // runs the shoot motor
        System.out.println("Run Shooter motor");
      } else {
        shooter.stopIntakeMotor();
      }
      if (shooter.shooterHasNote() == true) {
        sinceNoteLeft =
            currentTime
                .millis(); // if it see's the note it will set the since note left time for current
        // time
        hasSeenNote = true;
        System.out.println("SAW THE NOTE");
      }
      if (currentMillis - startedMillis > 250) {
        shooter.spinShootToRPM(rpm.getAsInt());
      }
    }

    if ((currentMillis - startedMillis)
        > waitTime.getAsInt()) { // waits for 250 ms for it to turn on the shoot
      // motor
      shooter.spinIntakeToNegativeRPM(rpm.getAsInt()); // runs the shoot motor
      System.out.println("Run Shooter motor");
    }
    if (shooter.shooterHasNote() == true) {
      sinceNoteLeft =
          currentTime
              .millis(); // if it see's the note it will set the since note left time for current
      // time
      hasSeenNote = true;
      System.out.println("SAW THE NOTE");
    }
    System.out.println("Current  HAS NOTE STATE: " + shooter.getHasNoteState());
  }

  public boolean isFinished() {
    return ((currentMillis - sinceNoteLeft) > 1000 && hasSeenNote == true)
        || (currentMillis - startedMillis > maxCommandWaitTime.shootCommandWaitTime);
  }

  @Override
  public void end(boolean interrupted) {

    shooter.stopShootMotor();
    shooter.stopIntakeMotor();
    System.out.println("Shooting Done");
    lights.SetLightState(LightStates.ReadyForPickup);
    System.out.println("Has Note state is currently: " + shooter.getHasNoteState());
    if (!interrupted) {
      shooter.setHasNoteState(false);
    }
  }
}
