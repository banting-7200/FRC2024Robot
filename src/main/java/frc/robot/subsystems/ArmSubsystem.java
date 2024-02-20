package frc.robot.subsystems;

import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax leftArmMotor, rightArmMotor;
    private AbsoluteEncoder rightEncoder;
    private SparkPIDController pidController;

    DoubleSolenoidActions shooterSolenoidActions = new DoubleSolenoidActions(Constants.m_pH.makeDoubleSolenoid(Arm.sForward_Channel, Arm.sReverse_Channel));
    SolenoidActions brakeSolenoidActions = new SolenoidActions(Constants.m_pH.makeSolenoid(Arm.b_Channel));
    DoubleSolenoidActions hookSolenoidActions = new DoubleSolenoidActions(Constants.m_pH.makeDoubleSolenoid(Arm.hForward_Channel, Arm.hReverse_Channel));

    ShuffleboardSubsystem shuffleboard;

    public ArmSubsystem() {
        leftArmMotor = new CANSparkMax(Arm.leftArmMotorID, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(Arm.rightArmMotorID, MotorType.kBrushless);

        shuffleboard = ShuffleboardSubsystem.getInstance();

        leftArmMotor.restoreFactoryDefaults();
        rightArmMotor.restoreFactoryDefaults();

        leftArmMotor.follow(rightArmMotor, true);

        leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);// make kBreak
        rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightEncoder = rightArmMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        rightEncoder.setPositionConversionFactor(42);// Amount of ticks in a Neo encoder rotation unit. Converted for
                                                     // gravity feedfoward later on.
        pidController = rightArmMotor.getPIDController();
        pidController.setFeedbackDevice(rightEncoder);
        pidController.setPositionPIDWrappingEnabled(true);

        setPID();

        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(rightArmMotor, DCMotor.getNEO(1));
        }
    }

    private void setPID() {
        int smartMotionSlot = Arm.smartMotionSlot;

       /*ShuffleboardTab tab = Shuffleboard.getTab("PID Tuning");
        GenericEntry pinput = tab.add("P", Arm.p).withPosition(0, 0).getEntry();
        GenericEntry iinput = tab.add("I", Arm.i).withPosition(0, 1).getEntry();
        GenericEntry dinput = tab.add("D", Arm.d).withPosition(0, 2).getEntry();
        GenericEntry finput = tab.add("F", Arm.f).withPosition(0, 3).getEntry();
        GenericEntry izinput = tab.add("IZ", Arm.iz).withPosition(0, 4).getEntry();
        double pval = pinput.getDouble(0);
        double ival = iinput.getDouble(0);
        double dval = dinput.getDouble(0);
        double fval = finput.getDouble(0);
        double izval = izinput.getDouble(0);
        GenericEntry minPIDinput = tab.add("pidOutputMin", Arm.pidOutputMin).withPosition(1, 0).getEntry();
        GenericEntry maxPIDinput = tab.add("pidOutputMax", Arm.pidOutputMax).withPosition(1, 1).getEntry();
        double pidMinval = minPIDinput.getDouble(0);
        double pidMaxval = maxPIDinput.getDouble(0);
        GenericEntry maxVelocityinput = tab.add("maxMotorVelocity", Arm.maxMotorVelocity).withPosition(1, 2).getEntry();
        GenericEntry maxMotorAccel = tab.add("maxMotorAccel", Arm.maxMotorAccel).withPosition(1, 3).getEntry();
        GenericEntry allowedPIDError = tab.add("allowedPIDError", Arm.allowedPIDError).withPosition(1, 4).getEntry();
        double maxVelocityinputval = maxVelocityinput.getDouble(0);
        double maxMotorAccelval = maxMotorAccel.getDouble(0);
        double allowedPIDErrorval = allowedPIDError.getDouble(0);*/

        // Configure PID
        pidController.setP(Arm.p, smartMotionSlot);
        pidController.setI(Arm.i, smartMotionSlot);
        pidController.setD(Arm.d, smartMotionSlot);
        pidController.setFF(Arm.f, smartMotionSlot);
        pidController.setIZone(Arm.iz, smartMotionSlot);
        pidController.setOutputRange(Arm.pidOutputMax, Arm.pidOutputMax, smartMotionSlot);

        // Configure smart motion
        pidController.setSmartMotionMaxVelocity(Arm.maxMotorVelocity, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(Arm.minMotorVelocity, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(Arm.maxMotorAccel, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(Arm.allowedPIDError, smartMotionSlot);
    }

    public boolean moveToAngle(double angle) {
        pidController.setReference(angle, CANSparkMax.ControlType.kSmartMotion, Arm.smartMotionSlot, getArbFF());
        System.out.println("motor angle: " + rightEncoder.getPosition());
        return Math.abs(rightEncoder.getPosition() - angle) < 0.99;
    }

    public double getArbFF() {
        // Arbirtary feedfoward to account for gravity acting on the arm
        double kTicksPerDegree = 42 / (360 * 400);// Todo: hardcode gear ratio
        double currentPos = rightEncoder.getPosition();
        double degrees = (currentPos - Arm.kMeasuredPosHorizontal) / kTicksPerDegree;
        double radians = java.lang.Math.toRadians(degrees);
        double cosineScalar = java.lang.Math.cos(radians);

        return Arm.maxGravityFF * cosineScalar;
    }

    double degreesToRotations(double degrees) {
        return degrees * 400;
    }

    public void getLimitSwitch() {
        System.out.println("Limit switch foward: " + rightArmMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed()
                + ", Limit switch reverse: " + rightArmMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed());// Change type
    }

    public void stopArm() {
        rightArmMotor.stopMotor();
    }

    public double getEncoderPosition() {
        return rightEncoder.getPosition();
    }

    // Solenoid movement functions
    
    // Shooter
    public void tuckShooter() {
        shooterSolenoidActions.setReverse();
        System.out.println("Shooter tucked");
    }

    public void deployShooter() {
        shooterSolenoidActions.setForward();
        System.out.println("Shooter deployed");
    }

    public void toggleShooterState() {
        shooterSolenoidActions.toggle();
        System.out.println("Shooter toggled");
    }

    public void disableShooterSolenoids() {
        shooterSolenoidActions.setOff();
        System.out.println("Shooter solenoids disabled");
    }

    public boolean isTucked() {
        return !shooterSolenoidActions.getState();
    }

    // Hook
    public void deployHook() {
        hookSolenoidActions.setForward();
        System.out.println("Hook Deployed");
    }

    public void stowHook() {
        hookSolenoidActions.setReverse();
        System.out.println("Hook Stowed");
    }

    public void toggleHook(){
        hookSolenoidActions.toggle();
        System.out.println("Hook Toggled");
    }

    public void disableHookSolenoids(){
        hookSolenoidActions.setOff();
         System.out.println("Hook disabled");
    }

    // Brake
    public void enableBrake() {
        brakeSolenoidActions.setOff();
    }

    public void disableBrake() {
        brakeSolenoidActions.setOn();
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
    }
}
