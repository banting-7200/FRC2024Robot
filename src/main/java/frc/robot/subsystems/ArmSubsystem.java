package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax leftArmMotor, rightArmMotor;
    private RelativeEncoder rightEncoder;
    private SparkPIDController pidController;

    SolenoidActions shooterSolenoidActions = new SolenoidActions(Constants.m_pH.makeSolenoid(Arm.s_Channel));

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
        rightEncoder.setPositionConversionFactor(42);// Amount of ticks in a Neo encoder rotation unit. Converted for gravity feedfoward later on.
        pidController = rightArmMotor.getPIDController();
        pidController.setFeedbackDevice(rightEncoder);

        setPID(Arm.p, Arm.i, Arm.d, Arm.f, Arm.iz);
    }

    private void setPID(double p, double i, double d, double f, double iz) {
        // Configure PID
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(f);
        pidController.setIZone(iz);
        pidController.setOutputRange(Arm.pidOutputMin, Arm.pidOutputMax);

        // Configure smart motion
        int smartMotionSlot = Arm.smartMotionSlot;
        pidController.setSmartMotionMaxVelocity(Arm.maxMotorVelocity, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(Arm.minMotorVelocity, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(Arm.maxMotorAccel, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(Arm.allowedPIDError, smartMotionSlot);
    }

    public void moveToAngle(double angle) {
        // Arbirtary feedfoward to account for gravity acting on the arm
        int kMeasuredPosHorizontal = 840; // Default position measured when arm is horizontal from example. Todo: find
                                          // the value for our arm.
        double kTicksPerDegree = 42 / 360; // Todo: find the ammount of ticks per degree relative to all the gearboxes
                                           // and factor that into the calculation.
        double currentPos = rightEncoder.getPosition();
        double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
        double radians = java.lang.Math.toRadians(degrees);
        double cosineScalar = java.lang.Math.cos(radians);

        double maxGravityFF = 0.07;// Todo: Find the best gravity feed foward for our arm

        pidController.setReference(angle, CANSparkBase.ControlType.kSmartMotion, 0, maxGravityFF * cosineScalar);// Todo: find what pidslot the pid is or find a way to apply arb feedfoward without it
        System.out.println("motor angle: " + rightEncoder.getPosition());
    }

    // ask felix if it would be better to keep solonoid action or integrate it all
    // into arm subsystem

    public boolean isTucked() {
        return !shooterSolenoidActions.getState();
    }
}
