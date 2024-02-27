package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightDevice;
import frc.robot.subsystems.PhotonVisionDevice;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class NoteObjectAlign extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final PhotonVisionDevice photonVisionDevice;

    private final PIDController positionController;
    private final PIDController rotationController;

    private double tagArea;
    private double targetArea;

    public NoteObjectAlign(
            SwerveSubsystem swerveSubsystem,
            PhotonVisionDevice photonVisionDevice,
            double targetArea) {
        this.swerveSubsystem = swerveSubsystem;
        this.photonVisionDevice = photonVisionDevice;

        this.targetArea = targetArea;

        positionController = new PIDController(1, 0, 0);
        positionController.setSetpoint(targetArea);

        rotationController = new PIDController(0.035, 0.0001, 0);
        rotationController.setSetpoint(0);

        addRequirements(swerveSubsystem, photonVisionDevice);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double fowardAdjust = 0;
        double rotationAdjust = 0;

        if (photonVisionDevice.noteDetected()) {
            tagArea = photonVisionDevice.noteArea();
            fowardAdjust = positionController.calculate(tagArea, targetArea);
            rotationAdjust = rotationController.calculate(photonVisionDevice.getNoteAngleOffset(), 0);
            swerveSubsystem.drive(new Translation2d(fowardAdjust, 0), rotationAdjust, false);
        }
    }

    @Override
    public boolean isFinished() {
        return positionController.atSetpoint() && rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.lock();
    }
}
