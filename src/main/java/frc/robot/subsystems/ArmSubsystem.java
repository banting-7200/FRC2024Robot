package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax leftArmMotor, rightArmMotor;
    private RelativeEncoder rightEncoder;
    private SparkPIDController pidController;

    private SolenoidActions shooterSolenoid = new SolenoidActions(Arm.shooterSolenoid);

    ArmSubsystem() {
        leftArmMotor = new CANSparkMax(Arm.leftArmMotorID, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(Arm.rightArmMotorID, MotorType.kBrushless);

        leftArmMotor.restoreFactoryDefaults();
        rightArmMotor.restoreFactoryDefaults();

        leftArmMotor.follow(rightArmMotor);
        rightArmMotor.setInverted(true);

        leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightEncoder = rightArmMotor.getEncoder();
        pidController = rightArmMotor.getPIDController();
        pidController.setFeedbackDevice(rightEncoder);

        setPID(Arm.p, Arm.i, Arm.d, Arm.f, Arm.iz);
    }

    private void setPID(double p, double i, double d, double f, double iz) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(f);
        pidController.setIZone(iz);
    }

    public void moveToAngle(double angle) {
        // Todo: implement pid movement to angle
    }

    public boolean isTucked() {
        return shooterSolenoid.getState();
    }
}
