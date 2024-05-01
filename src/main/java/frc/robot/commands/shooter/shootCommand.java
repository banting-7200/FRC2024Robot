/*
 * This command is extremely simple as it shoots the note and ends after one second of it leaving
 * the shooter.
 */

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.copilotController;
import frc.robot.Constants.maxCommandWaitTime;
import frc.robot.commands.swervedrive.auto.NoteAutoStateMachine;
import frc.robot.subsystems.ArmAndHead.ShooterSubsystem;
import frc.robot.subsystems.Feedback.LightSubsystem;
import frc.robot.subsystems.Feedback.LightSubsystem.LightStates;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;
import java.time.Clock;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

public class shootCommand extends Command {

  public ShooterSubsystem shooter;

  LightSubsystem lights = LightSubsystem.getInstance();
  Clock currentTime = Clock.systemDefaultZone();

  long startedMillis; // time when started
  long currentMillis; // current time
  long sinceNoteLeft; // time since note left shooter
  long passedMillis;

  IntSupplier rpm; // Int supplier for different shoot rpm's(speaker/amp)
  Boolean hasSeenNote = false; // if note has been detected yet
  IntSupplier waitTime; // Int supplier for different wait times(speaker/amp)
  BooleanSupplier isSpeakerShot;

  Joystick controller;

  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  private NoteAutoStateMachine stateInstance;

  public shootCommand(
      IntSupplier rpm,
      ShooterSubsystem shooter,
      IntSupplier waitTime,
      BooleanSupplier isSpeakerShot) {
    this.rpm = rpm;
    this.shooter = shooter;
    this.waitTime = waitTime;
    this.isSpeakerShot = isSpeakerShot;
    this.controller = null;

    addRequirements(shooter);
  }

  public shootCommand(
      int rpm, ShooterSubsystem shooter, int waitTime, BooleanSupplier isSpeakerShot) {
    this(() -> rpm, shooter, () -> waitTime, isSpeakerShot);
  }

  public shootCommand(
      int rpm, ShooterSubsystem shooter, int waitTime, boolean isSpeakerShot, NoteAutoStateMachine stateInstance) {
    this(() -> rpm, shooter, () -> waitTime, () -> isSpeakerShot);
    this.stateInstance = stateInstance;
  }

  public shootCommand(
      IntSupplier rpm,
      ShooterSubsystem shooter,
      IntSupplier waitTime,
      BooleanSupplier isSpeakerShot,
      Joystick controller) {
    this(rpm, shooter, waitTime, isSpeakerShot);
    this.controller = controller;
  }

  @Override
  public void initialize() {
    sinceNoteLeft = 0;
    startedMillis = currentTime.millis();
    hasSeenNote = false;

    shooter.stopIntakeMotor(); // Stop the intake motor
    System.out.println("Started shoot command");
    System.out.println("is Shoot state: " + isSpeakerShot.getAsBoolean());
  }

  @Override
  public void execute() {
    currentMillis = currentTime.millis(); // record current time
    passedMillis = currentMillis - startedMillis;
    if (isSpeakerShot.getAsBoolean()) {
      if (passedMillis < 100 // pullback
      ) { // until 100 millis pass
        shooter.spinIntakeToPositiveRPM(2000); // reverse intake
        shooter.spinShootNegativeToRPM(500);
      } else if (passedMillis > waitTime.getAsInt()
          && (controller == null
              || !controller.getRawButton(copilotController.shootButton))) { // feed in
        // waits for 250 ms for it to turn on the shoot
        // motor
        shooter.spinIntakeToNegativeRPM(rpm.getAsInt()); // feeds to intake motor
        // System.out.println("Run Shooter motor");
      } else {
        shooter.stopIntakeMotor();
      }
      if (!shooter.shooterHasNote() && passedMillis > 100 + waitTime.getAsInt() && !hasSeenNote) {
        // if it see's the note it will set the since note left time for current
        // time
        sinceNoteLeft = currentMillis;
        hasSeenNote = true;
        // System.out.println("SAW THE NOTE");
      }
      if (passedMillis > 450 /*&& !pause*/) { // rev up
        shooter.spinShootToRPM(rpm.getAsInt());
      }
    } else {
      if (!shooter.shooterHasNote() && passedMillis > 100 + waitTime.getAsInt() && !hasSeenNote) {
        // if it see's the note it will set the since note left time for current
        // time
        sinceNoteLeft = currentMillis;
        hasSeenNote = true;
        // System.out.println("SAW THE NOTE");
      }
      shooter.spinShootToRPM(rpm.getAsInt());
      shooter.spinIntakeToNegativeRPM(rpm.getAsInt());
    }
  }

  // System.out.println("Current  HAS NOTE STATE: " + shooter.shooterHasNote());

  public boolean isFinished() {
    return ((currentMillis - sinceNoteLeft) > /* shuffle.getNumber("shoot Ramp Down") */ 500
            && hasSeenNote == true)
        || (currentMillis - startedMillis > maxCommandWaitTime.shootCommandWaitTime);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShootMotor();
    shooter.stopIntakeMotor();
    System.out.println("Shooting Done");
    if (!shooter.shooterHasNote())
      lights.SetLightState(LightStates.ReadyForPickup);
    if(stateInstance != null){
      stateInstance.MoveToState(NoteAutoStateMachine.States.Search);
    }
    
    // System.out.println("Has Note state is currently: " +
    // shooter.shooterHasNote());
  }
}
