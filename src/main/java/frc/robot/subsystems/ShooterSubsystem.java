package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX armShooter;
    private TalonFX armIntake;

    VelocityVoltage m_velocity = new VelocityVoltage(0);

    private DigitalInput noteSensor;

    private ArmSubsystem arm;

    ShooterSubsystem(ArmSubsystem armClass) {
        armShooter = new TalonFX(Shooter.shooterID);
        armIntake = new TalonFX(Shooter.intakeID);

        noteSensor = new DigitalInput(Shooter.sensorPin);

        arm = armClass;

        // PID config
        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = Shooter.shooterF;// shoot motor feedfoward
        slot0Configs.kP = Shooter.shooterP;// shoot motor Proprotional
        slot0Configs.kI = Shooter.shooterI;// shoot motor Integral
        slot0Configs.kD = Shooter.shooterD;// shoot motor Derivitive

        var slot1Configs = new Slot1Configs();
        slot1Configs.kV = Shooter.intakeF;// inatke motor feedfoward
        slot1Configs.kP = Shooter.intakeP;// inatke motor Proprotional
        slot1Configs.kI = Shooter.intakeI;// inatke motor Integral
        slot1Configs.kD = Shooter.intakeD;// inatke motor Derivitive

        armShooter.getConfigurator().apply(slot0Configs);// pass shoot motor PID configs to the motor
        armIntake.getConfigurator().apply(slot1Configs);// pass intake motor PID configs to the motor
    }

    public void spinIntakeToRPM(double targetRPM) {
        if (arm.isTucked() == false) {
            m_velocity.Slot = 1;
            armIntake.setControl(m_velocity.withVelocity(targetRPM / 60));// convert rpm to rps then apply
        }
    }

    public void spinShooterToRPM(double targetRPM) {
        if (arm.isTucked() == false) {
            m_velocity.Slot = 0;
            armShooter.setControl(m_velocity.withVelocity(targetRPM / 60));// convert rpm to rps then apply
        }
    }

    public void stopShootMotor() {
        armShooter.stopMotor();
    }

    public void stopIntakeMotor() {
        armIntake.stopMotor();
    }

    public boolean hasNote() {
        return !noteSensor.get();// Inverted because the IR sensor returns true when there is no note.
    }
}
