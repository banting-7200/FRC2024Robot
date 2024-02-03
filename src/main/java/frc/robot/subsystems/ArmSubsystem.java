package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
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

    public ArmSubsystem() {
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
        pidController.setOutputRange(Arm.pidOutputMin, Arm.pidOutputMax);
        
        int smartMotionSlot = Arm.smartMotionSlot;
        pidController.setSmartMotionMaxVelocity(Arm.maxMotorVelocity, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(Arm.minMotorVelocity, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(Arm.maxMotorAccel, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(Arm.allowedPIDError, smartMotionSlot);
    }

    public void moveToAngle(double angle) {
        pidController.setReference(angle, CANSparkBase.ControlType.kSmartMotion);
        System.out.println("motor angle: " + rightEncoder.getPosition());
    }

    public boolean isTucked() {
        return shooterSolenoid.getState();
    }
}
