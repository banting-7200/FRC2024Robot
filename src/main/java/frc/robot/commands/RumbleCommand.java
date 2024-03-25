package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import java.time.Clock;

public class RumbleCommand extends Command {
  long rumbleStartTime;
  Clock currentTime = Clock.systemDefaultZone();
  double rumbleStrength, rumbleTime;
  XboxController driverController;

  public RumbleCommand(double rumbleStrength, double rumbleTime, XboxController driverController) {
    this.rumbleStrength = rumbleStrength;
    this.rumbleTime = rumbleTime;
    this.driverController = driverController;
  }

  @Override
  public void initialize() {
    rumbleStartTime = currentTime.millis();
    driverController.setRumble(RumbleType.kBothRumble, rumbleStrength);
  }

  @Override
  public boolean isFinished() {
    return currentTime.millis() - rumbleStartTime > rumbleTime;
  }

  @Override
  public void end(boolean interrupted) {
    driverController.setRumble(RumbleType.kBothRumble, 0);
  }
}
