package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax leftArmMotor, rightArmMotor;
  private AbsoluteEncoder rightEncoder;
  private SparkPIDController pidController;

  DoubleSolenoidActions shooterSolenoidActions =
      new DoubleSolenoidActions(
          Constants.m_pH.makeDoubleSolenoid(Arm.sForward_Channel, Arm.sReverse_Channel));
  SolenoidActions brakeSolenoidActions =
      new SolenoidActions(Constants.m_pH.makeSolenoid(Arm.b_Channel));
  DoubleSolenoidActions hookSolenoidActions =
      new DoubleSolenoidActions(
          Constants.m_pH.makeDoubleSolenoid(Arm.hForward_Channel, Arm.hReverse_Channel));

  ShuffleboardSubsystem shuffleboard;

  public ArmSubsystem() {
    leftArmMotor = new CANSparkMax(Arm.leftArmMotorID, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(Arm.rightArmMotorID, MotorType.kBrushless);

    shuffleboard = ShuffleboardSubsystem.getInstance();

    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();

    leftArmMotor.follow(rightArmMotor, true);

    leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake); // make kBreak
    rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    rightEncoder = rightArmMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    rightEncoder.setPositionConversionFactor(
        42 /* * Arm.armGearRatio */); // Amount of ticks in a Neo encoder rotation unit. Converted
    // for gravity feedfoward later on.
    pidController = rightArmMotor.getPIDController();
    pidController.setFeedbackDevice(rightEncoder);
    pidController.setPositionPIDWrappingEnabled(true);

    shuffleboard.setNumber("arm P", Arm.p);
    shuffleboard.setNumber("arm I", Arm.i);
    shuffleboard.setNumber("arm D", Arm.d);
    shuffleboard.setNumber("arm F", Arm.f);
    shuffleboard.setNumber("arm IZ", Arm.iz);
    shuffleboard.setNumber("arm min output", Arm.pidOutputMin);
    shuffleboard.setNumber("arm max output", Arm.pidOutputMax);
    setPID();

    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(rightArmMotor, DCMotor.getNEO(1));
    }
  }

  private void setPID() {
    int smartMotionSlot = Arm.smartMotionSlot;

    double[] PIDvalues = shuffleboard.getPID("arm");
    // Configure PID
    if (Arm.p != PIDvalues[0]) {
      Arm.p = PIDvalues[0];
    }
    if (Arm.i != PIDvalues[1]) {
      Arm.i = PIDvalues[1];
    }
    if (Arm.d != PIDvalues[2]) {
      Arm.d = PIDvalues[2];
    }
    if (Arm.f != PIDvalues[3]) {
      Arm.f = PIDvalues[3];
    }
    if (Arm.iz != PIDvalues[4]) {
      Arm.iz = PIDvalues[4];
    }
    if (Arm.pidOutputMin != shuffleboard.getNumber("arm min output")) {
      Arm.pidOutputMin = shuffleboard.getNumber("arm min output");
    }
    if (Arm.pidOutputMax != shuffleboard.getNumber("arm max output")) {
      Arm.pidOutputMax = shuffleboard.getNumber("arm max output");
    }
    pidController.setP(PIDvalues[0], smartMotionSlot);
    pidController.setI(PIDvalues[1], smartMotionSlot);
    pidController.setD(PIDvalues[2], smartMotionSlot);
    pidController.setFF(PIDvalues[3], smartMotionSlot);
    pidController.setIZone(PIDvalues[4], smartMotionSlot);
    pidController.setOutputRange(
        shuffleboard.getNumber("arm min output"),
        shuffleboard.getNumber("arm max output"),
        smartMotionSlot);

    // Configure smart motion
    /*pidController.setSmartMotionMaxVelocity(Arm.maxMotorVelocity, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(Arm.minMotorVelocity, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(Arm.maxMotorAccel, smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(Arm.allowedPIDError, smartMotionSlot);*/
  }

  public boolean moveToAngle(double angle) {
    pidController.setReference(
        angle, CANSparkMax.ControlType.kPosition, Arm.smartMotionSlot, getArbFF());
    // System.out.println("motor angle: " + rightEncoder.getPosition());
    return Math.abs(rightEncoder.getPosition() - angle) < 0.99;
  }

  public double getArbFF() {
    // Arbirtary feedfoward to account for gravity acting on the arm
    double kTicksPerDegree = 42 / (360 * Arm.armGearRatio);
    double currentPos = rightEncoder.getPosition();
    double degrees = (currentPos - Arm.kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

    return Arm.maxGravityFF * cosineScalar;
  }

  double degreesToRotations(double degrees) {
    return degrees * Arm.armGearRatio;
  }

  public void getLimitSwitch() {
    shuffleboard.setBoolean(
        "Limit switch foward",
        rightArmMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed());
    shuffleboard.setBoolean(
        "Limit switch reverse",
        rightArmMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed());
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

  public void toggleHook() {
    hookSolenoidActions.toggle();
    System.out.println("Hook Toggled");
  }

  public void disableHookSolenoids() {
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

  public boolean getBrake() {
    return !brakeSolenoidActions.getState();
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
