package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision.ObjectTrackingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SearchNote extends Command {

  private SwerveSubsystem swerveSubsystem;
  private final ObjectTrackingSubsystem photonCamera;
  private NoteAutoStateMachine stateInstance;

  public SearchNote(
      SwerveSubsystem swerveSubsystem,
      ObjectTrackingSubsystem photonCamera,
      NoteAutoStateMachine stateInstance) {

    this.swerveSubsystem = swerveSubsystem;
    this.photonCamera = photonCamera;

    this.stateInstance = stateInstance;

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
    swerveSubsystem.drive(new Translation2d(0, 0), 2, false);
  }

  @Override
  public boolean isFinished() {
    return photonCamera.hasTarget();
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
    stateInstance.MoveToState(NoteAutoStateMachine.States.Drive);
    if (!interrupted) {
      System.out.println("Successfully ended Search Note command");
    } else {
      System.out.println("Failed at Search Note command");
    }
  }
}
