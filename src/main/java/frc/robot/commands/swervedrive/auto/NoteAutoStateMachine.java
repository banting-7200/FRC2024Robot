package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Shooter;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.swervedrive.auto.intakeNote;
import frc.robot.commands.shooter.shootCommand;
import frc.robot.subsystems.ArmAndHead.ArmSubsystem;
import frc.robot.subsystems.ArmAndHead.ShooterSubsystem;
import frc.robot.subsystems.Vision.LimelightDevice;
import frc.robot.subsystems.Vision.PhotonCamera;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.swervedrive.auto.SearchNote;
//src\main\java\frc\robot\commands\swervedrive\auto\SearchNote.java
public class NoteAutoStateMachine {
  //Enum to classify the different states of the auto
  public enum States {
    Search,
    Drive,
    PickUp,
    TargetAlign,
    Shoot
  }

  //Instances of the commands the autos impliment
  private DriveToNote driveToNote;
  private AprilTagAlign aprilTagAlign;
  private shootCommand shootCommand;
  private MoveArmToPosition armToPosition;
  private PrepForShoot prepForShoot;
  private SearchNote searchNote;
  private intakeNote intakeNote;

   NoteAutoStateMachine(SwerveSubsystem swerveSubsystem, PhotonCamera photonCamera,
      LimelightDevice limelightDevice, ShooterSubsystem shooter, ArmSubsystem arm) {
    driveToNote = new DriveToNote(swerveSubsystem, photonCamera, 0, this);
    aprilTagAlign = new AprilTagAlign(swerveSubsystem, limelightDevice, 0, 0, false);//Make the tag id which ever tag we decide goes on the new target
    shootCommand = new shootCommand(Shooter.speakerShootRPM, shooter, Shooter.speakerWaitTime, true);//The shoot rpm may need to be calibrated to our new target
    armToPosition = new MoveArmToPosition(arm, Arm.speakerArmAngle);

    intakeNote = new intakeNote(swerveSubsystem, shooter);
    searchNote = new SearchNote(swerveSubsystem, photonCamera);
  }

  //Method to switch between the diffrent states of the auto
  public void MoveToState(States newState) {
    switch(newState){
      case Search:
        searchNote.schedule();
        break;

      case Drive:
        driveToNote.schedule();
        break;
          
      case PickUp:
        intakeNote.schedule();        
      break;

      case TargetAlign:
      
        break;

      case Shoot:
        shootCommand.schedule();
        break;
    }
  }
}
