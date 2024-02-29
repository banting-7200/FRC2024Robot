// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.copilotController;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.arm.TuckArm;
import frc.robot.commands.arm.UntuckArm;
import frc.robot.commands.shooter.intakeCommand;
import frc.robot.commands.shooter.shootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LightSubsystem.lightStates;
import frc.robot.subsystems.LimelightDevice;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here..

  // CommandJoystick driverController = new
  // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  // CommandJoystick driverController = new
  // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  static XboxController driverXbox = new XboxController(0);
  static Joystick CoPilotController = new Joystick(1);

  // Subsystem Declaration
  /*
   * private final SwerveSubsystem drivebase =
   * new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
   */
  ArmSubsystem arm;
  private ShooterSubsystem shooter;
  LimelightDevice limelight;
  LightSubsystem lights = LightSubsystem.getInstance();
  private static ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  static boolean isRedAliance =
      DriverStation.getAlliance().isPresent()
          ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red
          : false;

  static boolean speakerShot = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    arm = new ArmSubsystem();
    shooter = new ShooterSubsystem(arm);
    limelight = new LimelightDevice();

    configureBindings();

    /*
     * AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
     * () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
     * OperatorConstants.LEFT_Y_DEADBAND),
     * () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
     * OperatorConstants.LEFT_X_DEADBAND),
     * () -> MathUtil.applyDeadband(driverXbox.getRightX(),
     * OperatorConstants.RIGHT_X_DEADBAND),
     * driverXbox::getYButtonPressed,
     * driverXbox::getAButtonPressed,
     * driverXbox::getXButtonPressed,
     * driverXbox::getBButtonPressed);
     *
     * // Applies deadbands and inverts controls because joysticks
     * // are back-right positive while robot
     * // controls are front-left positive
     * // left stick controls translation
     * // right stick controls the desired angle NOT angular rotation
     * Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
     * () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
     * OperatorConstants.LEFT_Y_DEADBAND),
     * () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
     * OperatorConstants.LEFT_X_DEADBAND),
     * () -> driverXbox.getRightX(),
     * () -> driverXbox.getRightY());
     *
     * // Applies deadbands and inverts controls because joysticks
     * // are back-right positive while robot
     * // controls are front-left positive
     * // left stick controls translation
     * // right stick controls the angular velocity of the robot
     * Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
     * () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
     * OperatorConstants.LEFT_Y_DEADBAND),
     * () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
     * OperatorConstants.LEFT_X_DEADBAND),
     * () -> driverXbox.getRawAxis(2));
     *
     * Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
     * () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
     * OperatorConstants.LEFT_Y_DEADBAND),
     * () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
     * OperatorConstants.LEFT_X_DEADBAND),
     * () -> driverXbox.getRawAxis(2));
     *
     * drivebase.setDefaultCommand(
     * !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle :
     * driveFieldOrientedDirectAngleSim);
     */

    shuffle.setTab("arm");
    // shuffle.setLayout("arm", 1, 4);
    shuffle.setNumber("arm angle", 0);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}. Use this method to define your trigger->command mappings. Triggers can be created
   * via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an
   * arbitrary predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public static final DoubleSupplier shuffleboardAngle =
      () -> {
        return shuffle.getNumber("arm angle");
      };

  public static final IntSupplier tagToAlign = () -> isRedAliance ? 4 : 7;

  public static final IntSupplier shooterRPM =
      () -> speakerShot ? Shooter.speakerShootRPM : Shooter.ampShootRPM;

  public static final IntSupplier shooterWaitTime =
      () -> speakerShot ? Shooter.ampWaitTime : Shooter.speakerWaitTime;

  static JoystickButton upButton =
      new JoystickButton(CoPilotController, copilotController.upButton);
  static JoystickButton downButton =
      new JoystickButton(CoPilotController, copilotController.downButton);

  public static final DoubleSupplier axis =
      () -> upButton.getAsBoolean() == true ? 1 : downButton.getAsBoolean() == true ? -1 : 0;

  private void configureBindings() {
    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // // new JoystickButton(driverXbox, 1).onTrue((new
    // // InstantCommand(drivebase::zeroGyro)));
    // // new JoystickButton(driverXbox, 3).onTrue(new
    // // InstantCommand(drivebase::addFakeVisionReading));
    // /*
    // * new JoystickButton(driverXbox,
    // * 2).whileTrue(
    // * Commands.deferredProxy(() -> drivebase.driveToPose(
    // * new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    // * ));
    // */
    // // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new
    // // InstantCommand(drivebase::lock, drivebase)));

    /*
     * Trigger t = new Trigger(creepBoolean);
     * t.onTrue(new InstantCommand(() -> drivebase.setDriveSpeeds(true)))
     * .onFalse(new InstantCommand(() -> drivebase.setDriveSpeeds(false)));
     */
    new JoystickButton(driverXbox, XboxController.Button.kX.value)
        .onTrue(new TuckArm(arm).andThen(new MoveArmToPosition(arm, Arm.tuckArmAngle)));
    // Move to position outlined on shuffleboard position

    new JoystickButton(driverXbox, XboxController.Button.kX.value)
        .onTrue(new TuckArm(arm).andThen(new MoveArmToPosition(arm, Arm.tuckArmAngle)));

    new JoystickButton(driverXbox, XboxController.Button.kA.value)
        .onTrue(new UntuckArm(arm).andThen(new MoveArmToPosition(arm, Arm.intakeArmAngle)));
    new JoystickButton(driverXbox, XboxController.Button.kB.value)
        .onTrue(new UntuckArm(arm).andThen(new MoveArmToPosition(arm, Arm.ampArmAngle)));

    new JoystickButton(driverXbox, XboxController.Button.kY.value)
        .onTrue(Commands.runOnce(() -> arm.toggleShooterState()));

    /*
     * new POVButton(driverXbox, 270).onTrue(new intakeCommand(500, shooter));
     * //D-pad down
     * new POVButton(driverXbox, 180).onTrue(new shootCommand(500, shooter, 1000));
     * new POVButton(driverXbox, 90)
     * .onTrue(new intakeCommand(1500, shooter).andThen(new shootCommand(2000,
     * shooter, 1000)));
     */

    // Todo: make suppliers for conditional commands
    new JoystickButton(CoPilotController, copilotController.upButton)
        .onTrue(new MoveArm(arm, axis));
    new JoystickButton(CoPilotController, copilotController.downButton)
        .onTrue(new MoveArm(arm, axis));

    new JoystickButton(CoPilotController, copilotController.brakeButton)
        .onTrue(
            Commands.runOnce(
                () -> stopArm())); // Does this have to brake everything or just the arm.
    new JoystickButton(CoPilotController, copilotController.pickupButton)
        .onTrue(
            new intakeCommand(
                Shooter.intakeRPM,
                Shooter.pullBackRPM,
                Shooter.correctPositioningRPM,
                shooter)); // From what positions will we intake?

    new JoystickButton(CoPilotController, copilotController.hookButton)
        .onTrue(Commands.runOnce(() -> arm.toggleHook()));
    new JoystickButton(CoPilotController, copilotController.carryButton)
        .onTrue(
            Commands.runOnce(() -> shooter.stopIntakeMotor())
                .andThen(new TuckArm(arm).andThen(new MoveArmToPosition(arm, Arm.tuckArmAngle))));
    new JoystickButton(CoPilotController, copilotController.extendButton)
        .onTrue(Commands.runOnce(() -> arm.toggleShooterState()));

    new JoystickButton(CoPilotController, copilotController.shootButton)
        .onTrue(new shootCommand(shooterRPM, shooter, shooterWaitTime));
    /*
     * new JoystickButton(CoPilotController,
     * copilotController.limelightButton).onTrue(new AprilTagAlign(drivebase,
     * limelight, Arm.speakerAlignTagArea, tagToAlign));
     */

    new JoystickButton(CoPilotController, copilotController.speakerButton)
        .onTrue(
            new MoveArmToPosition(
                    arm,
                    limelight
                        .calculateArmShootAngle()) // Need redundancies for if Limelight doesn't
                // work!!!
                .finallyDo(() -> lights.SetLightState(lightStates.ReadyToSPEAKER))
                .alongWith(Commands.runOnce(() -> speakerShot = true)));
    new JoystickButton(CoPilotController, copilotController.ampButton)
        .onTrue(
            new MoveArmToPosition(arm, Arm.ampArmAngle)
                .finallyDo(() -> lights.SetLightState(lightStates.ReadyToAMP))
                .alongWith(Commands.runOnce(() -> speakerShot = false)));
  }

  public static final BooleanSupplier creepBoolean =
      () -> {
        return driverXbox.getLeftTriggerAxis() > 0.5;
      };

  public double getDoubleSupplier() {
    return shuffleboardAngle.getAsDouble();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand()
  // {
  // An example command will be run in autonomous
  // return drivebase.getAutonomousCommand("New Path", true);
  // }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void stopArm() {
    arm.enableBrake();
    arm.stopArm();
  }

  public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  }

  /*
   * public Pose2d getRobotPose() {
   * return drivebase.getPose();
   * }
   */
}
