package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class intakeCommand extends Command {
    public ShooterSubsystem shooter = new ShooterSubsystem(new ArmSubsystem());
    private boolean noteHasEntered = false;
    int rpm;

    public intakeCommand(int rpm) {
        this.rpm = rpm;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        if (noteHasEntered == false) {
            shooter.spinIntakeToRPM(rpm);
        } else if (shooter.hasNote()) {
            noteHasEntered = true;
        }
    }

    public boolean isFinished() {
        return shooter.hasNote();
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopIntakeMotor();
    }

    
}
