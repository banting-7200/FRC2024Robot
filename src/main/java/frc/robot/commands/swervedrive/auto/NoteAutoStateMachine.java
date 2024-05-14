package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Shooter;
import frc.robot.commands.arm.MoveArmToPosition;
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
    Search,
    Drive,
    PickUp,
    TargetAlign,
    Shoot
  }

  // Instances of the commands the autos impliment
  private DriveToNote driveToNote;
  private AprilTagAlign aprilTagAlign;
  private shootCommand shootCommand;
  private MoveArmToPosition armToShootPosition;
  private Command armToIntakePosition;
  private PrepForShoot prepForShoot;
  private SearchNote searchNote;
  private intakeNote intakeNote;

  private ArmSubsystem arm;

  // A command variable to store the command being currently run
  private SequentialCommandGroup currentCommand = new SequentialCommandGroup();

  public NoteAutoStateMachine(
      SwerveSubsystem swerveSubsystem,
      ObjectTrackingSubsystem photonCamera,
      AprilTagSubsystem limelightDevice,
      ShooterSubsystem shooter,
      ArmSubsystem arm) {
    this.arm = arm;
    driveToNote =
        new DriveToNote(
            swerveSubsystem,
            photonCamera,
            28.50,
            this); // See waht the best note area is for our applications

    shootCommand =
        new shootCommand(
            Shooter.speakerShootRPM,
            shooter,
            Shooter.speakerWaitTime,
            true,
            this); // The shoot rpm may need to be calibrated to our new target

    aprilTagAlign =
        new AprilTagAlign(
            swerveSubsystem,
            limelightDevice,
            0.60,
            16, // 6
            false); // Make the tag id which ever tag we decide goes on the new target
    armToShootPosition = new MoveArmToPosition(arm, Arm.speakerArmAngle);
    prepForShoot = new PrepForShoot(aprilTagAlign, armToShootPosition, this);

    armToIntakePosition =
        new UntuckArm(arm).andThen(new MoveArmToPosition(arm, Arm.intakeArmAngle));

    intakeNote = new intakeNote(swerveSubsystem, shooter, this);
    searchNote = new SearchNote(swerveSubsystem, photonCamera, this);
  }

  // Method to switch between the diffrent states of the auto
  public void MoveToState(States newState) {
    Cancel();

    switch (newState) {
      case Search:
        if (!arm.isTucked()) {
          currentCommand.addCommands(searchNote);
        } else {
          currentCommand = armToIntakePosition.andThen(searchNote);
        }
        break;

      case Drive:
        currentCommand.addCommands(driveToNote);
        break;

      case PickUp:
        currentCommand.addCommands(intakeNote);
        break;

      case TargetAlign:
        currentCommand.addCommands(prepForShoot);
        break;

      case Shoot:
        currentCommand.addCommands(shootCommand);
        ;
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
