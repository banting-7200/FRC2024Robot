package frc.robot.commands.swervedrive.auto;
import java.time.Clock;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.ArmAndHead.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class intakeNote extends Command {

  private SwerveSubsystem swerveSubsystem;
  private ShooterSubsystem shooter;
  private Clock currentTime = Clock.systemDefaultZone();
  private long startTime;
  private long passedTime;


  public intakeNote(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooter) {
    this.swerveSubsystem = swerveSubsystem;
    this.shooter = shooter;
    addRequirements(swerveSubsystem, shooter);
  }

  @Override
  public void initialize() {
    startTime = currentTime.millis();
    passedTime = currentTime.millis();

  }

  @Override
  public void execute() {
    passedTime = currentTime.millis() - startTime;
    shooter.spinIntakeToNegativeRPM(Shooter.intakeRPM);
    swerveSubsystem.drive(new Translation2d(0.5, 0.0), 0.0, false);
  }

  @Override
  public boolean isFinished() {
    return passedTime > 3000;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopIntakeMotor();
  }
}
