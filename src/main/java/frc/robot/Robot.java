// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Feedback.LightSubsystem;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;
import frc.robot.subsystems.Vision.LimelightDevice;
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

  private static Robot instance; // Creates the Robot.java instance
  private RobotContainer m_robotContainer; // Creates the Robot container instance
  private Command m_autonomousCommand; // The command that stores our auto
  private LimelightDevice limelight = LimelightDevice.getInstance();

  private Timer disabledTimer; // Normal Robot container instance
  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  LightSubsystem lights = LightSubsystem.getInstance();

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

    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot
    // stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();

    // start the camera server
    CameraServer.startAutomaticCapture("Front Camera", 0);
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

    m_robotContainer.setShuffleboard();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true); // Brake the swerve modules
    disabledTimer.reset();
    disabledTimer.start();
    m_robotContainer.stopArm(); // ensure the arm is stopped
    m_robotContainer.resetArmManualSpeed(); // reset the arm speed to it's regular state
    limelight.setLight(
        false); // Turn off the limelight lights so the robot can be more easily approached on
    // disable.
    m_robotContainer.driverXbox.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
    m_robotContainer.refreshTagIDs();
    if (limelight.getLight()) {
      limelight.setLight(false);
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // m_robotContainer.zeroGyroWithAlliance();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    limelight.setLight(true);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.zeroGyroWithAlliance();
    m_robotContainer.stopShootAndIntake();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

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
