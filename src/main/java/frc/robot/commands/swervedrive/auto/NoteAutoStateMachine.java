package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Shooter;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.arm.TuckArm;
import frc.robot.commands.arm.UntuckArm;
import frc.robot.commands.shooter.shootCommand;
import frc.robot.subsystems.ArmAndHead.ArmSubsystem;
import frc.robot.subsystems.ArmAndHead.ShooterSubsystem;
import frc.robot.subsystems.Vision.AprilTagSubsystem;
import frc.robot.subsystems.Vision.ObjectTrackingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class NoteAutoStateMachine {
  // Enum to classify the different states of the auto
  public enum States {
    MoveToIntake,
    Search,
    Drive,
    PickUp,
    TargetAlign,
    Shoot
  }

  int tagToSearch = 1;

  // Instances of the commands the autos impliment
  private DriveToNote driveToNote;
  private AprilTagAlign aprilTagAlign;
  private Command shootCommand;
  private Command botToShootPosition;
  private Command armToIntakePosition;
  private Command prepForShoot;
  private SearchNote searchNote;
  private intakeNote intakeNote;

  // A command variable to store the command being currently run
  private Command currentCommand;

  public NoteAutoStateMachine(
      SwerveSubsystem swerveSubsystem,
      ObjectTrackingSubsystem photonCamera,
      AprilTagSubsystem limelightDevice,
      ShooterSubsystem shooter,
      ArmSubsystem arm) {
    driveToNote =
        new DriveToNote(
            swerveSubsystem,
            photonCamera,
            20.50, // 28.50 original value
            this); // See what the best note area is for our applications

    shootCommand =
        new shootCommand(
            Shooter.speakerShootRPM,
            shooter,
            Shooter.speakerWaitTime,
            true,
            swerveSubsystem,
            this); // The shoot rpm may need to be calibrated to our new target

    aprilTagAlign =
        new AprilTagAlign(
            swerveSubsystem,
            limelightDevice,
            0.50,
            tagToSearch, // 6
            false,
            this); // Make the tag id which ever tag we decide goes on the new target

    prepForShoot =
        aprilTagAlign
            .andThen(new TuckArm(arm))
            .andThen(new MoveArmToPosition(arm, Arm.speakerArmAngle))
            .finallyDo(() -> MoveToState(States.Shoot));

    armToIntakePosition =
        new UntuckArm(arm).andThen(new MoveArmToPosition(arm, Arm.intakeArmAngle, this));

    intakeNote = new intakeNote(swerveSubsystem, shooter, this);
    searchNote = new SearchNote(swerveSubsystem, photonCamera, limelightDevice, this);
  }

  // Method to switch between the diffrent states of the auto
  public void MoveToState(States newState) {
    Cancel();

    switch (newState) {
      case MoveToIntake:
        searchNote.trackAprilTags(false);
        currentCommand = armToIntakePosition;
        break;

      case Search:
        currentCommand = searchNote;
        break;

      case Drive:
        currentCommand = driveToNote;
        break;

      case PickUp:
        searchNote.trackAprilTags(true, tagToSearch);
        currentCommand = intakeNote;
        break;

      case TargetAlign:
        currentCommand = prepForShoot;
        break;

      case Shoot:
        currentCommand = shootCommand;
        break;
    }
    currentCommand.schedule();
  }

  // A function to shut down the state machine
  public void Cancel() {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
  }
}
