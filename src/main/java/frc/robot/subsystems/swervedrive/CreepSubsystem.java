package frc.robot.subsystems.swervedrive;
import frc.robot.RobotContainer;

public class CreepSubsystem {
    public static final double maxNormalSpeed = 14.5;
    public static final double creepSpeed = 5;

    public Double getMaximumSpeed() {
        if (RobotContainer.creepBoolean.get()) {
            return maxNormalSpeed;
        } else {
            return creepSpeed+1;
        }
    }  
}
