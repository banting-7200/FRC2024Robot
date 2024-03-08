/*
 * This command is extremely simple as it shoots the note and ends after one second of it leaving
 * the shooter.
 */

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.copilotController;
import frc.robot.Constants.maxCommandWaitTime;
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
  IntSupplier rpm; // Int supplier for different shoot rpm's(speaker/amp)
  Boolean hasSeenNote = false; // if note has been detected yet
  IntSupplier waitTime; // Int supplier for different wait times(speaker/amp)
  boolean hasNote; // This variables is used for telling whether the note is already in the shooter
  BooleanSupplier isSpeakerShot;

  Joystick controller;

  // or
  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  // not

  public shootCommand(
      IntSupplier rpm,
      ShooterSubsystem shooter,
      IntSupplier waitTime,
      BooleanSupplier isSpeakerShot) {
    this.rpm = rpm;
    this.shooter = shooter;
    this.waitTime = waitTime;
    this.isSpeakerShot = isSpeakerShot;

    addRequirements(shooter);
  }

  public shootCommand(
      int rpm, ShooterSubsystem shooter, int waitTime, BooleanSupplier isSpeakerShot) {
    this(() -> rpm, shooter, () -> waitTime, isSpeakerShot);
    this.controller = null;
  }

   public shootCommand(
      int rpm, ShooterSubsystem shooter, int waitTime, BooleanSupplier isSpeakerShot, Joystick controller) {
    this(() -> rpm, shooter, () -> waitTime, isSpeakerShot);
    this.controller = controller;
  }

  @Override
  public void initialize() {
    sinceNoteLeft = currentTime.millis();
    startedMillis = currentTime.millis();
    hasSeenNote = false;
    hasNote = shooter.shooterHasNote();
    shooter.stopIntakeMotor(); // Stop the intake motor
    System.out.println("is Shoot state: " + isSpeakerShot.getAsBoolean());
  }

  @Override
  public void execute() {
    if (hasNote == true || controller != null ? controller.getRawButton(copilotController.brakeButton) : false) { // if shooter has note in it
      currentMillis = currentTime.millis(); // record current time
      if (currentMillis - startedMillis < 100
          && isSpeakerShot.getAsBoolean()) { // until 100 millis pass
        shooter.spinIntakeToPositiveRPM(2000); // reverse intake
        shooter.spinShootNegativeToRPM(500);
      } else if ((currentMillis - startedMillis) > waitTime.getAsInt()) {
        // waits for 250 ms for it to turn on the shoot
        // motor
        shooter.spinIntakeToNegativeRPM(rpm.getAsInt()); // runs the shoot motor
        // System.out.println("Run Shooter motor");
      } else {
        shooter.stopIntakeMotor();
      }
      if (shooter.shooterHasNote() == true) {
        sinceNoteLeft =
            currentTime
                .millis(); // if it see's the note it will set the since note left time for current
        // time
        hasSeenNote = true;
        // System.out.println("SAW THE NOTE");
      }
      if (currentMillis - startedMillis > 350) {
        shooter.spinShootToRPM(rpm.getAsInt());
      }
  }

    // System.out.println("Current  HAS NOTE STATE: " + shooter.shooterHasNote());
  }

  public boolean isFinished() {
    return ((currentMillis - sinceNoteLeft) > /* shuffle.getNumber("shoot Ramp Down") */ 3000
            && hasSeenNote == true)
        || (currentMillis - startedMillis > maxCommandWaitTime.shootCommandWaitTime);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShootMotor();
    shooter.stopIntakeMotor();
    // System.out.println("Shooting Done");
    if (!shooter.shooterHasNote()) lights.SetLightState(LightStates.ReadyForPickup);
    // System.out.println("Has Note state is currently: " + shooter.shooterHasNote());
  }
}
