package frc.robot.subsystems.ArmAndHead;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;
import frc.robot.subsystems.Solenoids.DoubleSolenoidActions;
import frc.robot.subsystems.Solenoids.SolenoidActions;
import java.time.Clock;

public class ArmSubsystem extends SubsystemBase {

  // Motors that control the arm
  private CANSparkMax leftArmMotor, rightArmMotor;
  private AbsoluteEncoder rightEncoder; // Arm encoder
  private SparkPIDController pidController; // PID controller for arm

  SparkLimitSwitch forwardLimitSwitch, reverseLimitSwitch; // Arm limit switches

  private DigitalInput
      solenoidSwitch; // switch attached to the shooter solenoid to query the tuck/untucked state.

  public double motorManualSpeed = Arm.motorManualSpeed;

  DoubleSolenoidActions shooterSolenoidActions =
      new DoubleSolenoidActions(
          Constants.m_pH.makeDoubleSolenoid(
              Arm.sForward_Channel, Arm.sReverse_Channel)); // Creates a double solenoid for
  // the shooter
  SolenoidActions brakeSolenoidActions =
      new SolenoidActions(Constants.m_pH.makeSolenoid(Arm.b_Channel)); // Creates a
  // solenoid for
  // the brake
  DoubleSolenoidActions hookSolenoidActions =
      new DoubleSolenoidActions(
          Constants.m_pH.makeDoubleSolenoid(
              Arm.hForward_Channel, Arm.hReverse_Channel)); // Creates a double solenoid for
  // the hook.

  //
  private Clock currentTime = Clock.systemDefaultZone(); // Clock for using the Millis Function
  private long stateChangeTimestamp; // The timestamp since the last change was made in terrm
  private boolean
      lastShooterState; // A boolean to obfuscate the true state of the shooter solenoid so that it
  // has
  // time to move between states.

  ShuffleboardSubsystem shuffleboard =
      ShuffleboardSubsystem.getInstance(); // Gets shuffleboard instance

  public ArmSubsystem() {
    // Define motor objects for the arm using SparkMax motor controllers
    leftArmMotor = new CANSparkMax(Arm.leftArmMotorID, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(Arm.rightArmMotorID, MotorType.kBrushless);

    // Assign the limit switches to the ones connected to the right spark
    forwardLimitSwitch = rightArmMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    reverseLimitSwitch = rightArmMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    /*
     * Assigns solenoidSwitch to the digital input of pin 2, which is the limit
     * switch that lets the robot know what the shooter's position is, such as when
     * it is tucked or not.
     */
    solenoidSwitch = new DigitalInput(Arm.solenoidSwitchID);

    // restores the motors to a factory default state so motor controllers can be
    // easily switched without extra config.
    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();

    // Allows left motor output to mirror that of the right motor
    // The true state inverses the output of the right motor
    leftArmMotor.follow(rightArmMotor, true);

    // Sets the mode of the motors when not being commanded to brake so the arm does
    // not slip from comamn
    leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Sets the ramp rate when moving the motor through PID to a predefine
    rightArmMotor.setClosedLoopRampRate(Arm.motorPIDRampRate);
    // Sets the current limit in Amperes which is defined in constants.java
    rightArmMotor.setSmartCurrentLimit(Arm.currentLimit);

    // Assigns the right encoder to the encoder connected to the ri
    rightEncoder = rightArmMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    rightEncoder.setPositionConversionFactor(42); // Amount of ticks in a Neo encoder rotation unit.

    // Assigns pidController to the PIDController of the right Arm Spark
    pidController = rightArmMotor.getPIDController();
    pidController.setFeedbackDevice(
        rightEncoder); // Assigns the feedback device for the PID control to the right
    // encoder
    // Lets the PID controller know that the values can be wrapped for the arm. (ex.
    // If Arm encoder reaches 40 and needs to get to one it will)
    pidController.setPositionPIDWrappingEnabled(true);

    // Sets the intial value of the last shooter state to whatever the solenoid
    // switch reports on initialization.
    lastShooterState = solenoidSwitch.get();

    // Sets the PID configs.
    shuffleboard.setTab("Debugging");
    shuffleboard.setPID("arm PID", Arm.p, Arm.i, Arm.d, Arm.f, Arm.iz);
    shuffleboard.setNumber("arm min output", Arm.pidOutputMin);
    shuffleboard.setNumber("arm max output", Arm.pidOutputMax);
    setPID();
    shuffleboard.newCommandButton("apply PID changes", new InstantCommand(() -> setPID()));

    // Enables better motor simulation in sim.
    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(rightArmMotor, DCMotor.getNEO(1));
    }
  }

  public void setOutputVoltage() {
    shuffleboard.setNumber("output current right", rightArmMotor.getOutputCurrent());
    shuffleboard.setNumber("output current left", leftArmMotor.getOutputCurrent());
  }

  public void setPID() {
    /*
     * Local reference to the smart motion slot defined in constants. The smart
     * motion slot tells the spark max which PID configuration slot to set the PID
     * changes to.
     */
    int smartMotionSlot = Arm.smartMotionSlot;

    // Shuffle PID config. Todo: move to debug

    double[] PIDvalues = shuffleboard.getPID("arm PID");

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

    // Re-updating ramp rate and current limit for motor ramp rate
    // rightArmMotor.setClosedLoopRampRate(Arm.motorPIDRampRate);
    // rightArmMotor.setSmartCurrentLimit(Arm.currentLimit);

    // Sets the PID values defined in constants to the smart motion slot on the
    // spark max
    pidController.setP(Arm.p, smartMotionSlot);
    pidController.setI(Arm.i, smartMotionSlot);
    pidController.setD(Arm.d, smartMotionSlot);
    pidController.setFF(Arm.f, smartMotionSlot);
    pidController.setIZone(Arm.iz, smartMotionSlot);
    // Sets the max and min motor output to the smart motion slot on the spark max
    pidController.setOutputRange(Arm.pidOutputMin, Arm.pidOutputMax, smartMotionSlot);
  }

  public void setMotorSpeed(double speed) {
    if ((rightEncoder.getPosition() >= Arm.encoderHardMax && speed > 0)
        || (forwardLimitSwitch.isPressed() && speed > 0)
        || (reverseLimitSwitch.isPressed()
            && speed < 0)) { // Check if the arm is beyond the encoder hard /*
      /* Check if the arm is beyond the encoder hard max before we move.
       * If it is beyond the hard max then stop the motor and end the movement
       */
      stopArm();
    } else {

      // Sets right (and left inversed) arm motor to the speed that is passed in
      rightArmMotor.set(speed);
    }
  }

  /* Simply gets the current speed of the right arm */
  public double getMotorSpeed() {
    return rightArmMotor.get();
  }

  public boolean moveToAngle(double angle) {
    if ((rightEncoder.getPosition() >= Arm.encoderHardMax && angle >= Arm.encoderHardMax)
        || (forwardLimitSwitch.isPressed() && angle > rightEncoder.getPosition())
        || (reverseLimitSwitch.isPressed() && angle < rightEncoder.getPosition())) {
      /*
       * Check if the arm is beyond the encoder hard max before we move
       * If it is beyond the hard max then stop the motor and end the movement
       */
      stopArm();
      return true;
    } else {
      /*
       * tells the PID controller to move to the input angle, withe position control,
       * using the PID configs in the smart motion slot defined in constants with
       * arbitrary feedforwards from the getArbFF function.
       */
      pidController.setReference(
          angle, CANSparkMax.ControlType.kPosition, Arm.smartMotionSlot, getArbFF());
      return Math.abs(rightEncoder.getPosition() - angle) < Arm.stopRange;
      /*
       * Checks if the arm has entered the desired stopping range and returns true if
       * so.
       */
    }
  }

  public double getArbFF() {
    // Arbirtary feedfoward to account for gravity acting on the arm
    double kTicksPerDegree = 4096 / (360 * Arm.armGearRatio);
    double currentPos = rightEncoder.getPosition(); // stores the current psoition of our encoder
    /*
     * Takes the current position of the right encoder and subtracts the encoder
     * reading of the arm when it's completely horizontal divided by the
     * ticksPerDegrees which then gets converted into radians in the next line
     */
    double degrees = (currentPos - Arm.kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians =
        java.lang.Math.toRadians(degrees); // Converts the degrees from encoder values ^
    double cosineScalar =
        java.lang.Math.cos(radians); // Creates a scalar using the translated radians

    // Returns that anti gravity shiiiiiiit
    return Arm.maxGravityFF * cosineScalar;
  }

  /* Simply takes the degrees multiplies it by the ArmGearRatio */
  double degreesToRotations(double degrees) {
    return degrees * Arm.armGearRatio;
  }

  public boolean getIsTuckedRaw() {
    return solenoidSwitch.get();
  }

  /*
   * Simple function to stop the arm. Disables output to the motors and enables
   * the brake.
   */
  public void stopArm() {
    rightArmMotor.stopMotor();
    enableBrake();
  }

  // Return the encoder position of the right arm encoder
  public double getEncoderPosition() {
    return rightEncoder.getPosition();
  }

  // Shooter
  public void tuckShooter() {
    // Sets the shooter Solenoid to the reversed state
    shooterSolenoidActions.setReverse();
    stateChangeTimestamp =
        currentTime.millis(); // Set state change timer allow shooter time to tuck before arm
    // movement.
    System.out.println("Shooter tucked");
  }

  public void deployShooter() {
    // Sets the shooter Solenoid to the forward state
    shooterSolenoidActions.setForward();
    stateChangeTimestamp =
        currentTime.millis(); // Set state change timer allow shooter time untuck before arm
    // movement.
    System.out.println("Shooter deployed");
  }

  // Toggles the shooter between forward reversed state of solenoid
  public void toggleShooterState() {
    shooterSolenoidActions.toggle();
    stateChangeTimestamp =
        currentTime.millis(); // Set state change timer allow shooter time tuck/untuck before arm
    // movement.
    System.out.println("Shooter toggled");
  }

  // Simply disables the shooter solonoid to off as well as printing a statements
  public void disableShooterSolenoids() {
    shooterSolenoidActions.setOff();
    System.out.println("Shooter solenoids disabled");
  }

  public boolean isTucked() {
    // Setting the last shooter state boolean in Shuffleboard to lastShooterState
    shuffleboard.setBoolean("last shooter state", lastShooterState);
    if (currentTime.millis() - stateChangeTimestamp > Arm.s_stateChangeDelay) {
      /*
       * If the time elapsed since the last state change is bigger than the
       * stateChangeDelay then set lastShooterstate as the current SolonoidState
       */
      lastShooterState = solenoidSwitch.get();
    }
    return lastShooterState;
  }

  // HookDeploys the hook
  public void deployHook() {
    hookSolenoidActions.setReverse();
    System.out.println("Hook Deployed");
  }

  // Stows the hook
  public void retractHook() {
    hookSolenoidActions.setForward();
    System.out.println("Hook Retracted");
  }

  // A toggle function for the hook
  public void toggleHook() {
    hookSolenoidActions.toggle();
    System.out.println("Hook Toggled");
  }

  // Disables the Hook's Solonoids
  public void disableHookSolenoids() {
    hookSolenoidActions.setOff();
    System.out.println("Hook disabled");
  }

  /* Function to query if the hook is deployed. */
  public boolean isHookDeployed() {
    return !hookSolenoidActions
        .isReversed(); // Inverted beecause the the deployed state of the hook is forward on the
    // solenoid.
  }

  // Brake
  /*toggles the brake on(applying the brake is disabling the solenoid)*/
  public void enableBrake() {
    brakeSolenoidActions
        .setOff(); // Turns off because solenoid is reversed as a safety feature (if power/air
    // pressure loss brakes enable)
    System.out.println("Brake enabled");
  }

  /*toggles the brake off(disabling the brake is enabling the solenoid*/
  public void disableBrake() {
    brakeSolenoidActions
        .setOn(); // Turns on because solenoid is reversed as a safety feature (if power/air
    // pressure loss brakes enable)
    System.out.println("Brake disabled");
  }

  /* Function to query if the brake is enabled. */
  public boolean isBrakeEnabled() {
    // Returns the state of the brake solenoid if on
    return !brakeSolenoidActions
        .getState(); // Inverted because enabling the brake is disabling the solenoid.
  }

  // Run the motors in sim using REV physics simulation  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  public void setShuffleboard() {
    shuffleboard.setNumber("Arm Encoder", rightEncoder.getPosition());

    shuffleboard.setBoolean("Shooter Tuck Switch", solenoidSwitch.get());

    shuffleboard.setBoolean("Is Brake Enabled", isBrakeEnabled());
    shuffleboard.setBoolean("Is Hook Deployed", isHookDeployed());
    shuffleboard.setBoolean("Is Head Tucked", isTucked());
    shuffleboard.setBoolean("Is  Head Tucked Raw", getIsTuckedRaw());

    shuffleboard.setBoolean("Is Forward Limit Pressed", forwardLimitSwitch.isPressed());
    shuffleboard.setBoolean("Is Reverse Limit Pressed", reverseLimitSwitch.isPressed());

    shuffleboard.setNumber("right output voltage", rightArmMotor.getOutputCurrent());
    shuffleboard.setNumber("left output voltage", leftArmMotor.getOutputCurrent());
  }
}
