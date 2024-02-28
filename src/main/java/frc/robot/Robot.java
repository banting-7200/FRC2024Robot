// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LimelightDevice;
import frc.robot.subsystems.ShuffleboardSubsystem;
import java.io.File;
import java.io.IOException;
import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static Robot instance;
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  double intakeCommandRPM;
  double shootCommandRPM;
  double shootCommandWaitTime;

  XboxController driverXbox = new XboxController(0);
  /*
   * POVButton dpadDownButton = new POVButton(driverXbox, 180);
   * POVButton dpadLeftButton = new POVButton(driverXbox, 270);
   * POVButton dpadUpButton = new POVButton(driverXbox, 0);
   */

  /*
   * static Joystick CoPilotController = new Joystick(1);
   * JoystickButton joystickIntakeButton = new JoystickButton(CoPilotController,
   * 2);
   * JoystickButton joystickShootButton = new JoystickButton(CoPilotController,
   * 3);
   */

  private Timer disabledTimer;
  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();
  LimelightDevice limelight;

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    shuffle = ShuffleboardSubsystem.getInstance();
    limelight = m_robotContainer.limelight;
    /*shuffle.setNumber("Intake Command RPM", intakeCommandRPM);
    shuffle.setNumber("Shoot Command RPM", shootCommandRPM);
    shuffle.setNumber("Shoot Command Wait Time", shootCommandWaitTime);

    shuffle.setBoolean("IR sensor", shooter.shooterHasNote());
    shuffle.setBoolean("IR sensor", shooter.shooterHasNote());*/
    // m_robotContainer.arm.disableBrake(); // Todo: put this a the start of arm
    // commands

    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();

    // CameraServer.startAutomaticCapture("Front Camera", 0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    /*intakeCommandRPM = shuffle.getNumber("Intake Command RPM");
    shootCommandRPM = shuffle.getNumber("Intake Command RPM");
    shootCommandWaitTime = shuffle.getNumber("Shoot Command Wait Time");
    shuffle.setNumber("Intake Speed", shooter.getIntakeRPM());*/

    // System.out.println("INTAKERPM: " + intakeCommandRPM);

    // System.out.println("shuffleboard input: " +
    // m_robotContainer.getDoubleSupplier());
    shuffle.setNumber("arm encoder reading", m_robotContainer.arm.getEncoderPosition());
    m_robotContainer.arm.getLimitSwitch();
    m_robotContainer.arm.setOutputVoltage();
    // m_robotContainer.arm.getSwitch();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
    m_robotContainer.stopArm();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME)) {
      // m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_robotContainer.setMotorBrake(true);
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // m_autonomousCommand = new UntuckArm(arm).andThen(new MoveArmToPosition(arm,
    // 26)).andThen(new
    // TuckArm(arm));

    // m_autonomousCommand = new shootCommand(1500);
    // m_autonomousCommand = new readyNoteCommand(1500);

    /*
     * The 3 commands above here are what's being run
     * is what's behind the commands that are used to
     * intake the note, ready it, and shoot it, which for now
     * you can set the rpm's that are being used for the motors.
     * As of right now the command for the readyNoteCommand has the rpm
     * already set to negative (as it need to pull the note back to "ready" it)
     * so you don't need to put that in. Another thing, you also need to change the
     * Talon motor
     * id's under src\main\java\frc\robot\Constants.java (if you're on vscode simply
     * hold on CTRL and click on constants for easy access)if you want to use
     * two motors (intake and shoot motor). Also FYI, this code does not allow the
     * robot
     * to drive as those controls have been commented out, to uncomment them simply
     * look for
     * the asterisks and uncomment those lines.
     */

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    limelight.setMode(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // SmartDashboard.putNumber("motor angle", arm.getEncoderPosition());
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_robotContainer.setDriveMode();
    // m_robotContainer.setMotorBrake(true);
    // m_robotContainer.arm.setPID();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() // Controller inputs to create and automate commands
      {
    /*
     * dpadDownButton.onTrue(
     * new PrintCommand("Intake Command STARTED")
     * .andThen(new intakeCommand(intakeCommandRPM, shooter)));
     * dpadLeftButton.onTrue(
     * new PrintCommand("Shoot Command Started")
     * .andThen(new shootCommand(shootCommandRPM, shooter, shootCommandWaitTime)));
     */

    // joystickIntakeButton.onTrue(new intakeCommand(intakeCommandRPM, shooter));
    // joystickShootButton.onTrue(new shootCommand(intakeCommandRPM, shooter,
    // shootCommandWaitTime));

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
