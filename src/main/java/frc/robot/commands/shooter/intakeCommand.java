package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class intakeCommand extends Command {
  public ShooterSubsystem shooter;
  int rpm;
  int openOrClosedCounter;

  public intakeCommand(int rpm, ShooterSubsystem shooter) {
    this.rpm = rpm;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    openOrClosedCounter = 0;
  }

  @Override
  public void execute() {
    shooter.spinIntakeToNegativeRPM(rpm);
    if (shooter.shooterHasNote() == true) {
      openOrClosedCounter = 1;
    } else if (openOrClosedCounter == 1 && shooter.shooterHasNote() == false) {
      openOrClosedCounter++;
    }
  }

  public boolean isFinished() {
    return openOrClosedCounter > 1;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopIntakeMotor();
    System.out.println("Intake Command ShutDown");
  }
}

// Jas sux
