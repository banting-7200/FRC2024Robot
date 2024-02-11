package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
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

        leftArmMotor.follow(rightArmMotor, true);

        leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightEncoder = rightArmMotor.getEncoder();
        rightEncoder.setPositionConversionFactor(42);// Amount of ticks in a Neo encoder rotation unit. Converted for gravity feedfoward later on.
        pidController = rightArmMotor.getPIDController();
        pidController.setFeedbackDevice(rightEncoder);

        setPID(Arm.p, Arm.i, Arm.d, Arm.f, Arm.iz);

          if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(rightArmMotor, DCMotor.getNEO(1));
    }
    }

    private void setPID(double p, double i, double d, double f, double iz) {
        int smartMotionSlot = Arm.smartMotionSlot;
        
        // Configure PID
        pidController.setP(p, smartMotionSlot);
        pidController.setI(i, smartMotionSlot);
        pidController.setD(d, smartMotionSlot);
        pidController.setFF(f, smartMotionSlot);
        pidController.setIZone(iz, smartMotionSlot);
        pidController.setOutputRange(Arm.pidOutputMin, Arm.pidOutputMax, smartMotionSlot);

        // Configure smart motion
      
        pidController.setSmartMotionMaxVelocity(Arm.maxMotorVelocity, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(Arm.minMotorVelocity, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(Arm.maxMotorAccel, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(Arm.allowedPIDError, smartMotionSlot);
    }

    public void moveToAngle(double angle) {
        pidController.setReference(angle, CANSparkMax.ControlType.kSmartMotion, Arm.smartMotionSlot, getArbFF());// Todo: find what pidslot the pid is or find a way to apply arb feedfoward without it
        System.out.println("motor angle: " + rightEncoder.getPosition());
    }

    public double getArbFF(){
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

        return maxGravityFF * cosineScalar;
    }

    //Todo: add a degrees to motor rotations conversion to account for the gearboxes
    
    // Solenoid movement functions
    public void tuckShooter(){
        shooterSolenoidActions.setOff();
    }

    public void deployShooter(){
        shooterSolenoidActions.setOn();
    }

    public void toggleShooterState(){
        shooterSolenoidActions.toggle();
    }

    public boolean isTucked() {
        return !shooterSolenoidActions.getState();
    }

    @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
