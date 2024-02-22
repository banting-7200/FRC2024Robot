package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX m_shoot;
  private TalonFX m_intake;

  VelocityVoltage m_velocity = new VelocityVoltage(0);

    private DigitalInput shootIR;

  private ArmSubsystem arm;

  public ShooterSubsystem(ArmSubsystem _arm) {
    m_shoot = new TalonFX(Shooter.shooterID);
    m_intake = new TalonFX(Shooter.intakeID);

        shootIR = new DigitalInput(Shooter.shootIR); // haha shooter lol, get it?

    arm = _arm;

    // PID config
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = Shooter.shooterF; // shoot motor feedfoward
    slot0Configs.kP = Shooter.shooterP; // shoot motor Proprotional
    slot0Configs.kI = Shooter.shooterI; // shoot motor Integral
    slot0Configs.kD = Shooter.shooterD; // shoot motor Derivitive

    var slot1Configs = new Slot1Configs();
    slot1Configs.kV = Shooter.intakeF; // inatke motor feedfoward
    slot1Configs.kP = Shooter.intakeP; // inatke motor Proprotional
    slot1Configs.kI = Shooter.intakeI; // inatke motor Integral
    slot1Configs.kD = Shooter.intakeD; // inatke motor Derivitive

    m_shoot.getConfigurator().apply(slot0Configs); // pass shoot motor PID configs to the motor
    m_intake.getConfigurator().apply(slot1Configs); // pass intake motor PID configs to the motor
  }

  public void spinIntakeToRPM(double targetRPM) {
    if (arm.isTucked() == false) {
      m_velocity.Slot = 1;
      m_intake.setControl(m_velocity.withVelocity(targetRPM / 60)); // convert rpm to rps then apply
    }
  }

    public void spinIntakeToPositiveRPM(double targetRPM) {
        if (arm.isTucked() == false) {
            m_velocity.Slot = 1;
            m_intake.setControl(m_velocity.withVelocity(targetRPM / 60));// convert rpm to rps then apply
        }
    }
     public void spinIntakeToNegativeRPM(double targetRPM) {
        if (arm.isTucked() == false) {
            m_velocity.Slot = 1;
            m_intake.setControl(m_velocity.withVelocity(-targetRPM / 60));// convert rpm to rps then apply
        }
    }

  public boolean hasNote() {
    return !noteSensor.get(); // Inverted because the IR sensor returns true when there is no note.
  }

    public boolean shooterHasNote() {
        return !shootIR.get();// Inverted because the IR sensor returns true when there is no note.
    }

    public void stopShootMotor() {
        m_shoot.stopMotor();
    }

    public void stopIntakeMotor() {
        m_intake.setControl(m_velocity.withVelocity(0));
    }
}
