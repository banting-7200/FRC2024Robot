package frc.robot.subsystems.ArmAndHead;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;

public class ShooterSubsystem extends SubsystemBase {

  // Init the motor variables used on the shooter
  private TalonFX m_shoot; // Creating new Shoot motor
  private TalonFX m_intake; // Creating new Intake motor

  // Creates a boolean that determines if the note is inside the shooter
  public boolean hasNote = true;

  // PID Velocity control for both motors
  VelocityVoltage m_velocity = new VelocityVoltage(0);
  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  // Init the variables of the ir sensor value and the arm subsystem
  // Used to control the input/output of the note intake
  private DigitalInput shootIR;
  private ArmSubsystem arm;

  // Define the shooter subsystem with the arm subsystem as a param
  public ShooterSubsystem(ArmSubsystem passedInArmSubsystem) {

    // Create Talon Motor Objects with previously defined variables
    m_shoot = new TalonFX(Shooter.shooterID);
    m_intake = new TalonFX(Shooter.intakeID);

    // Sensor value of the IR sensor used to check if the note is loaded
    shootIR = new DigitalInput(Shooter.shootIR); // haha shooter lol, get it?

    // Sets the ArmSubsystem that's passed in to the one that's used in this
    // subsystem.
    this.arm = passedInArmSubsystem;

    // PID config
    var slot0Configs = new Slot0Configs(); // Slot zero for shooter
    slot0Configs.kV = Shooter.shooterF; // shoot motor FeedForward
    slot0Configs.kP = Shooter.shooterP; // shoot motor Proprotional
    slot0Configs.kI = Shooter.shooterI; // shoot motor Integral
    slot0Configs.kD = Shooter.shooterD; // shoot motor Derivitive

    var slot1Configs = new Slot1Configs(); // Slot one for intake
    slot1Configs.kV = Shooter.intakeF; // intake motor FeedForward
    slot1Configs.kP = Shooter.intakeP; // intake motor Proprotional
    slot1Configs.kI = Shooter.intakeI; // intake motor Integral
    slot1Configs.kD = Shooter.intakeD; // intake motor Derivitive

    m_shoot.getConfigurator().apply(slot0Configs); // pass shoot motor PID configs to the motor
    m_intake.getConfigurator().apply(slot1Configs); // pass intake motor PID configs to the motor

    shuffle.setTab("Debugging");
    shuffle.setNumber("shoot Ramp Down", Shooter.shootRampDown);
  }

  /* Spins the intake to a positive passed in RPM */
  public void spinIntakeToPositiveRPM(double targetRPM) {
    m_velocity.Slot = 1;
    m_intake.setControl(m_velocity.withVelocity(targetRPM / 60)); // convert rpm to rps then apply
  }

  /* Spins the intake to a negative passed in RPM */
  public void spinIntakeToNegativeRPM(double targetRPM) {
    m_velocity.Slot = 1;
    m_intake.setControl(m_velocity.withVelocity(-targetRPM / 60)); // convert rpm to rps then apply
  }

  /* Spins shoot motors to passed in RPM */
  public void spinShootToRPM(double targetRPM) {
    m_velocity.Slot = 0;
    m_shoot.setControl(m_velocity.withVelocity(targetRPM / 60)); // convert rpm to rps then apply
  }

  public void spinShootNegativeToRPM(double targetRPM) {
    m_velocity.Slot = 0;
    m_shoot.setControl(m_velocity.withVelocity(-targetRPM / 60)); // convert rpm to rps then apply
  }

  /* Querys the state of the IR sensor */
  public boolean shooterHasNote() {
    return !shootIR.get(); // Inverted because the IR sensor returns true when there is no note.
  }

  /* Stops the shoot motor */
  public void stopShootMotor() {
    m_shoot.stopMotor();
  }

  /*
   * Stops the intake motor by setting the motor to zero with Velocity Volatage
   * control
   */
  public void stopIntakeMotor() {
    /*
     * Stops motor a different way due to the difference in modes that the intake
     * and shooter
     * motors use in Phoenix Tuner X
     */
    m_intake.setControl(m_velocity.withVelocity(0));
  }

  /*
   * This function was used to display the RPM of the Motors for trouble shooting
   */
  public double getIntakeRPM() {
    return m_intake.getVelocity().getValue();
  }

  /*
   * The next two functions simply set the state and get the state of a variable
   * which determines whether the intake and shoot commands can run again
   * depending on
   * the variables state.
   */
  // Todo: test this
  public void setHasNoteState(boolean state) {
    hasNote = state;
  }

  public boolean getHasNoteState() {

    return hasNote;
  }

  public void setShooterShuffleBoard() {
    shuffle.setTab("Debugging");
    shuffle.setBoolean("IR Sensor", shooterHasNote());
    /* shuffle.setNumber("shoot Ramp Down", 0); */
  }
}
