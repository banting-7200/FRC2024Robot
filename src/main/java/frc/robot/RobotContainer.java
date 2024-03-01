// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
import frc.robot.subsystems.Vision.LimelightDevice;
import frc.robot.subsystems.ArmAndHead.ArmSubsystem;
import frc.robot.subsystems.ArmAndHead.ShooterSubsystem;
import frc.robot.subsystems.Feedback.LightSubsystem;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;
import frc.robot.subsystems.Feedback.LightSubsystem.lightStates;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here..
    static XboxController driverXbox = new XboxController(0);
    static Joystick CoPilotController = new Joystick(1);

    // Subsystem Declaration
    /*
     * private final SwerveSubsystem drivebase =
     * new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
     */
    private ArmSubsystem arm; // Instance of Arm Subsystem
    private ShooterSubsystem shooter; // Instance of Shooter Subsystem
    private LimelightDevice limelight = LimelightDevice.getInstance();// Getting instance of Limelight Subsystem
    private LightSubsystem lights = LightSubsystem.getInstance();// Getting instance of Light Subsystem
    private static ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();// Getting insstance of Shooter
                                                                                       // Subsystem

    private static boolean isRedAliance = DriverStation.getAlliance().isPresent()
            ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            : false;// This is a local reference to the DriverStation alliance

    static boolean speakerShot = true; // Whether the robot is ready for a Speaker Shot or not

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        arm = new ArmSubsystem(); // Creating a new instance of Arm Subsytem
        shooter = new ShooterSubsystem(arm); // Creating a new instance of Shooter Subsytem

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

        shuffle.setTab("Debugging");
        shuffle.setLayout("Debugging Tools", 1, 4);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}. Use this method to define your trigger->command mappings.
     * Triggers can be created
     * via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructor with an
     * arbitrary predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    public static final DoubleSupplier shuffleboardAngle = () -> {
        return shuffle.getNumber("arm angle");
    };

    public static final IntSupplier tagToAlign = () -> isRedAliance ? 4 : 7;

    public static final IntSupplier shooterRPM = () -> speakerShot ? Shooter.speakerShootRPM : Shooter.ampShootRPM;

    public static final IntSupplier shooterWaitTime = () -> speakerShot ? Shooter.speakerWaitTime : Shooter.ampWaitTime;

    static JoystickButton upButton = new JoystickButton(CoPilotController, copilotController.upButton);
    static JoystickButton downButton = new JoystickButton(CoPilotController, copilotController.downButton);

    // Double supplier that returns the up and down buttons as a 1d axis
    public static final DoubleSupplier axis = () -> upButton.getAsBoolean() == true ? 1
            : downButton.getAsBoolean() == true ? -1 : 0;
    /*
     * Supplies the speaker angle binding a value for its target angle based on
     * weather or not the limelight button is pressed.
     * If it is, the supply a dynamicly calculated angle from the limelight,
     * if not, then supply a predetemined shoot angle.
     */
    public final DoubleSupplier speakerAngle = () -> CoPilotController
            .getRawButton(copilotController.limelightButton) == true
                    ? limelight.calculateArmShootAngle()
                    : Arm.speakerArmAngle;

    private void configureBindings() {

        // SWERVE STUFF
        /*
         * new JoystickButton(driverXbox, 1).onTrue((new
         * InstantCommand(drivebase::zeroGyro)));
         * new JoystickButton(driverXbox, 3).onTrue(new
         * InstantCommand(drivebase::addFakeVisionReading));
         */

        // This binds the creep toggling in swerve subsystem to this trigger
        /*
         * Trigger t = new Trigger(creepBoolean);
         * t.onTrue(new InstantCommand(() -> drivebase.setCreep(true)))//Set creep on
         * .onFalse(new InstantCommand(() -> drivebase.setCreep(false)));//Set creep off
         */

        // Manual arm movement up based on the axis double supplier
        new JoystickButton(CoPilotController, copilotController.upButton)
                .onTrue(new MoveArm(arm, axis));
        // Manual arm movement down based on the axis double supplier
        new JoystickButton(CoPilotController, copilotController.downButton)
                .onTrue(new MoveArm(arm, axis));

        /*
         * // Brakes the swerve modules. Constantly resets odometry while active to
         * trick the swerve into thinking its not moving and stop trying to correct
         * itself.
         * new JoystickButton(CoPilotController, copilotController.brakeButton)
         * .toggleOnTrue(new RepeatCommand(new InstantCommand(drivebase::resetOdometry,
         * drivebase)));
         */

        /*
         * Simply runs intake routine which runs upon when the pickUpButton is clicked.
         * First it intakes the note with 3 different RPMS which do different things,
         * then
         * it tucks the shooter, and if it's not in a safe position it will move it
         * until it is.
         * Once it finishes that it will move the arm into the tuck position and finally
         * it sets
         * the lights for the operator to know what state it's in.
         */
        new JoystickButton(CoPilotController, copilotController.pickupButton)
                .onTrue(
                        new intakeCommand(
                                Shooter.intakeRPM,
                                Shooter.pullBackRPM,
                                Shooter.correctPositioningRPM,
                                shooter).andThen(
                                        new TuckArm(arm)
                                                .andThen(new MoveArmToPosition(arm, Arm.tuckArmAngle))
                                                .finallyDo(
                                                        (boolean interrupted) -> {
                                                            if (!interrupted && shooter.hasNote)
                                                                lights.SetLightState(lightStates.CarryingNote);
                                                        }))); // From what positions will we intake?

        /*
         * Simply toggles the hook solenoid on a button press.
         * This effectivly toggles the hook up or down on button press.
         */
        new JoystickButton(CoPilotController, copilotController.hookButton)
                .onTrue(Commands.runOnce(() -> arm.toggleHook()));

        /*
         * This is a failsafe if the intake command fails.
         * First it stops the intake motor and then runs the tucking sequence like in
         * the intake binding above,
         * first tucking the shooter,
         * then moving to the tuck position,
         * and finally setting the light state if the command was not interrupted and we
         * have a note.
         */
        new JoystickButton(CoPilotController, copilotController.carryButton)
                .onTrue(
                        Commands.runOnce(() -> shooter.stopIntakeMotor())
                                .andThen(
                                        new TuckArm(arm)
                                                .andThen(new MoveArmToPosition(arm, Arm.tuckArmAngle))
                                                .finallyDo(
                                                        (boolean interrupted) -> {
                                                            if (!interrupted && shooter.hasNote)
                                                                lights.SetLightState(lightStates.CarryingNote);
                                                        })));

        // This binding allows for direct control over the shooter solenoid to toggle
        // its state at will.
        new JoystickButton(CoPilotController, copilotController.extendButton)
                .onTrue(Commands.runOnce(() -> arm.toggleShooterState()));

        /*
         * Essentially this command runs the shoot command when the shoot button
         * is ran and it takes 3 different parameters, 2 of which are int suppliers
         * and the other is the shooter instance which the command uses. The int
         * suppliers, shooterRPM and shooterWaitTime change accordingly depending on
         * position of the arm, which can be found if you scroll up.
         */
        new JoystickButton(CoPilotController, copilotController.shootButton)
                .onTrue(new shootCommand(shooterRPM, shooter, shooterWaitTime));

        /*
         * Simply schedules a command to align the robot to the commands supplied april
         * tag
         */

        /*
         * new JoystickButton(CoPilotController,
         * copilotController.limelightButton).onTrue(new AprilTagAlign(drivebase,
         * limelight, Arm.speakerAlignTagArea, tagToAlign));
         */

        /*
         * On button press command the arm to move to the angle supplied by the
         * speakerAngle double supplier.
         * Once the command has finished, it checks if it has been interrupted and if it
         * hasn't that means it has reached its setpoint
         * properly and updates the speaker shot variable to inform the shoot command
         * suppliers that it is shooting
         * at the speaker and sets the light states to a speaker shot.
         */
        new JoystickButton(CoPilotController, copilotController.speakerButton)
                .onTrue(
                        new MoveArmToPosition(arm, speakerAngle)
                                .finallyDo(
                                        (boolean interrupted) -> {
                                            if (!interrupted)
                                                speakerShot = true;
                                            lights.SetLightState(lightStates.ReadyToSPEAKER);
                                        }));

        /*
         * On button press, this binding commands the arm to move to the amp angle
         * it checks if it has been interrupted and if it
         * hasn't that means it has reached its setpoint
         * properly and updates the speaker shot variable to inform the shoot command
         * suppliers that it is shooting
         * at the amp and sets the light states to a amp shot.
         */
        new JoystickButton(CoPilotController, copilotController.ampButton)
                .onTrue(
                        new MoveArmToPosition(arm, Arm.ampArmAngle)
                                .finallyDo(
                                        (boolean interrupted) -> {
                                            if (!interrupted)
                                                speakerShot = false;
                                            lights.SetLightState(lightStates.ReadyToAMP);
                                        }));
    }

    /*
     * What's passed into the trigger for the Creep drive Modes.
     * Converts the analog input of a controller trigger into a boolean input that
     * is true after
     * the trigger is half pressed.
     */
    public static final BooleanSupplier creepBoolean = () -> {
        return driverXbox.getLeftTriggerAxis() > 0.5;
    };

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

    /*
     * This function is called when the program wants to completely stop the arm and
     * is generally used for redundancies and safety.
     * It enables the solenoid brake on the arm and stops the arm motors.
     */
    public void stopArm() {
        arm.enableBrake();
        arm.stopArm();
    }

    /*
     * This sets the brake mode of the swerve drive neos. true sets them to brake
     * mode while false sets them to coast mode.
     */
    public void setMotorBrake(boolean brake) {
        // drivebase.setMotorBrake(brake);
    }

    /*
     * public Pose2d getRobotPose() {
     * return drivebase.getPose();
     * }
     */
}
