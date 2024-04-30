package frc.robot.commands.swervedrive.auto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision.PhotonCamera;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SearchNote extends Command {

private SwerveSubsystem swerveSubsystem;
  private final PhotonCamera photonCamera;
  
  public SearchNote(
      SwerveSubsystem swerveSubsystem,
      PhotonCamera photonCamera
      ) {

    this.swerveSubsystem = swerveSubsystem;
    this.photonCamera = photonCamera;
      
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem, photonCamera);
  }


@Override
public void initialize() {
   System.out.println("Started Search for note Orbit");

  }

  @Override
  public void execute() {
    System.out.println("Looking for note...");
    swerveSubsystem.drive(null, 2, false);
   }

  @Override
  public boolean isFinished() {
    return photonCamera.hasTarget();
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(null, 0, false);
    if (!interrupted) {
      System.out.println("Successfully ended Search Note command");
    } else {
      System.out.println("Failed at Search Note command");

    }
  }
}
