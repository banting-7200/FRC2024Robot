package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.Solenoids.DoubleSolenoidActions;
import frc.robot.subsystems.Solenoids.SolenoidActions;
import java.time.Clock;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax leftArmMotor, rightArmMotor;
  private AbsoluteEncoder rightEncoder;
  private SparkPIDController pidController;

  private DigitalInput solenoidSwitch;

  DoubleSolenoidActions shooterSolenoidActions =
      new DoubleSolenoidActions(
          Constants.m_pH.makeDoubleSolenoid(Arm.sForward_Channel, Arm.sReverse_Channel));
  SolenoidActions brakeSolenoidActions =
      new SolenoidActions(Constants.m_pH.makeSolenoid(Arm.b_Channel));
  DoubleSolenoidActions hookSolenoidActions =
      new DoubleSolenoidActions(
          Constants.m_pH.makeDoubleSolenoid(Arm.hForward_Channel, Arm.hReverse_Channel));

  private Clock currentTime = Clock.systemDefaultZone();
  private long stateChangeTimestamp;
  private boolean lastShooterState;

  ShuffleboardSubsystem shuffleboard;

  public ArmSubsystem() {
    leftArmMotor = new CANSparkMax(Arm.leftArmMotorID, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(Arm.rightArmMotorID, MotorType.kBrushless);

    solenoidSwitch = new DigitalInput(Arm.solenoidSwitchID);

    shuffleboard = ShuffleboardSubsystem.getInstance();

    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();

    leftArmMotor.follow(rightArmMotor, true);

    leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake); // make kBreak
    rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    rightArmMotor.setClosedLoopRampRate(Arm.motorRampRate);
    rightArmMotor.setSmartCurrentLimit(Arm.currentLimit);

    rightEncoder = rightArmMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    rightEncoder.setPositionConversionFactor(
        42 /* * Arm.armGearRatio */); // Amount of ticks in a Neo encoder rotation unit. Converted
    // for gravity feedfoward later on.
    pidController = rightArmMotor.getPIDController();
    pidController.setFeedbackDevice(rightEncoder);
    pidController.setPositionPIDWrappingEnabled(true);

    lastShooterState = !solenoidSwitch.get();

    shuffleboard.setNumber("arm P", Arm.p);
    shuffleboard.setNumber("arm I", Arm.i);
    shuffleboard.setNumber("arm D", Arm.d);
    shuffleboard.setNumber("arm F", Arm.f);
    shuffleboard.setNumber("arm IZ", Arm.iz);
    shuffleboard.setNumber("arm min output", Arm.pidOutputMin);
    shuffleboard.setNumber("arm max output", Arm.pidOutputMax);
    shuffleboard.setNumber("stop range", Arm.stopRange);
    shuffleboard.setNumber("gravity FF", Arm.maxGravityFF);
    shuffleboard.setNumber("ramp rate", Arm.motorRampRate);
    shuffleboard.setNumber("current limit", Arm.currentLimit);
    shuffleboard.setNumber("solenoid delay", Arm.s_stateChangeDelay);

    shuffleboard.setNumber("output current right", rightArmMotor.getOutputCurrent());
    shuffleboard.setNumber("output current left", leftArmMotor.getOutputCurrent());
    setPID();

    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(rightArmMotor, DCMotor.getNEO(1));
    }
  }

  public void setOutputVoltage() {
    shuffleboard.setNumber("output current right", rightArmMotor.getOutputCurrent());
    shuffleboard.setNumber("output current left", leftArmMotor.getOutputCurrent());
  }

  public void setPID() {
    int smartMotionSlot = Arm.smartMotionSlot;
/* public void setPID() {
    int smartMotionSlot = Arm.smartMotionSlot;

    /* double[] PIDvalues = shuffleboard.getPID("arm");
    Arm.p = PIDvalues[0];
    Arm.i = PIDvalues[1];
    Arm.d = PIDvalues[2];
    Arm.f = PIDvalues[3];
    Arm.iz = PIDvalues[4]; */

    // Configure PID
    // if (Arm.p != PIDvalues[0]) {
    //   Arm.p = PIDvalues[0];
    // }
    // if (Arm.i != PIDvalues[1]) {
    //   Arm.i = PIDvalues[1];
    // }
    // if (Arm.d != PIDvalues[2]) {
    //   Arm.d = PIDvalues[2];
    // }
    // if (Arm.f != PIDvalues[3]) {
    //   Arm.f = PIDvalues[3];
    // }
    // if (Arm.iz != PIDvalues[4]) {
    //   Arm.iz = PIDvalues[4];
    // }
    // if (Arm.pidOutputMin != shuffleboard.getNumber("arm min output")) {
    // Arm.pidOutputMin = shuffleboard.getNumber("arm min output");
    // }
    // if (Arm.pidOutputMax != shuffleboard.getNumber("arm max output")) {
    // Arm.pidOutputMax = shuffleboard.getNumber("arm max output");
    // }
    pidController.setP(Arm.p, smartMotionSlot);
    pidController.setI(Arm.i, smartMotionSlot);
    pidController.setD(Arm.d, smartMotionSlot);
    pidController.setFF(Arm.f, smartMotionSlot);
    pidController.setIZone(Arm.iz, smartMotionSlot);
    pidController.setOutputRange(Arm.pidOutputMin, Arm.pidOutputMax, smartMotionSlot);

    rightArmMotor.setClosedLoopRampRate(Arm.motorRampRate);
    rightArmMotor.setSmartCurrentLimit(Arm.currentLimit);

    // Configure smart motion
    /*
     * pidController.setSmartMotionMaxVelocity(Arm.maxMotorVelocity,
     * smartMotionSlot);
     * pidController.setSmartMotionMinOutputVelocity(Arm.minMotorVelocity,
     * smartMotionSlot);
     * pidController.setSmartMotionMaxAccel(Arm.maxMotorAccel, smartMotionSlot);
     * pidController.setSmartMotionAllowedClosedLoopError(Arm.allowedPIDError,
     * smartMotionSlot);
     */
  }

  public void setMotorSpeed(double speed) {
    if (rightEncoder.getPosition() >= Arm.encoderHardMax
        && speed > 0) { // Check if the arm is beyond the encoder hard
      // max before we
      // move. If it is beyond the hard max then stop the motor and end the movement
      stopArm();
    } else {
      rightArmMotor.set(speed);
    }
  }

  public double getMotorSpeed() {
    return rightArmMotor.get();
  }

  public boolean moveToAngle(double angle) {
    if (rightEncoder.getPosition() >= Arm.encoderHardMax
        && angle
            >= Arm.encoderHardMax) { // Check if the arm is beyond the encoder hard max before we
      // move. If it is beyond the hard max then stop the motor and end the movement
      stopArm();
      return true;
    } else {
      pidController.setReference(
          angle, CANSparkMax.ControlType.kPosition, Arm.smartMotionSlot, getArbFF());
      // System.out.println("motor angle: " + rightEncoder.getPosition());
      return Math.abs(rightEncoder.getPosition() - angle) < shuffleboard.getNumber("stop range");
    }
  }

  public double getArbFF() {
    // Arbirtary feedfoward to account for gravity acting on the arm
    double kTicksPerDegree = 4096 / (360 * Arm.armGearRatio);
    double currentPos = rightEncoder.getPosition();
    double degrees = (currentPos - Arm.kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

    return shuffleboard.getNumber("gravity FF") * cosineScalar;
  }

  double degreesToRotations(double degrees) {
    return degrees * Arm.armGearRatio;
  }

  public void getSwitch() {
    System.out.println(solenoidSwitch.get());
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
    enableBrake();
  }

  public double getEncoderPosition() {
    return rightEncoder.getPosition();
  }

  // Solenoid movement functions

  // Shooter
  public void tuckShooter() {
    shooterSolenoidActions.setReverse();
    stateChangeTimestamp = currentTime.millis();
    System.out.println("Shooter tucked");
  }

  public void deployShooter() {
    shooterSolenoidActions.setForward();
    stateChangeTimestamp = currentTime.millis();
    System.out.println("Shooter deployed");
  }

  public void toggleShooterState() {
    shooterSolenoidActions.toggle();
    stateChangeTimestamp = currentTime.millis();
    // shuffleboard.setNumber("Changed state. current time: ",
    // currentTime.millis());
    System.out.println("Shooter toggled");
  }

  public void disableShooterSolenoids() {
    shooterSolenoidActions.setOff();
    System.out.println("Shooter solenoids disabled");
  }

  public boolean isTucked() {
    shuffleboard.setBoolean("last shooter state", lastShooterState);
    if (currentTime.millis() - stateChangeTimestamp > Arm.s_stateChangeDelay) {
      return lastShooterState;
    }
    stateChangeTimestamp = currentTime.millis();
    lastShooterState = solenoidSwitch.get();
    return solenoidSwitch.get();
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

  public boolean isHookDeployed() {
    return !hookSolenoidActions.isReversed();
  }

  // Brake
  public void enableBrake() {
    brakeSolenoidActions.setOff();
    System.out.println("Brake enabled");
  }

  public void disableBrake() {
    brakeSolenoidActions.setOn();
    System.out.println("Brake disabled");
  }

  public boolean getBrake() {
    return !brakeSolenoidActions.getState();
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
