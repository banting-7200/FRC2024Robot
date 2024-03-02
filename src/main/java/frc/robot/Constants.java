// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticHub;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class Auton {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0); // 5.0, 0.0, 0.0
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class copilotController {
    public static final int upButton = 2;
    public static final int downButton = 3;

    public static final int brakeButton = 4;
    public static final int pickupButton = 5;

    public static final int hookButton = 6;
    public static final int carryButton = 7;
    public static final int extendButton = 8;

    public static final int shootButton = 9;
    public static final int limelightButton = 10;

    public static final int speakerButton = 11;
    public static final int ampButton = 12;
  }

  public static final class Lights {
    public static final int lightID = 1;
    public static final int lightStringLength = 50;
  }

  public static final class maxCommandWaitTime {
    // todo: Tune All of these later
    public static final int moveArmToPositionWaitTime = 3000;
    public static final int tuckArmWaitTime = 2000;
    public static final int unTuckWaitTime = 2000;
    public static final int intakeCommandWaitTime = 3000;
    public static final int shootCommandWaitTime = 3000;
    public static final int aprilTagAlignWaitTime = 7000;
    public static final int noteObjectAlignWaitTime = 7000;
  }

  /* */
  public static final PneumaticHub m_pH = new PneumaticHub(3);

  public static final class Arm {
    public static final int leftArmMotorID = 33; // 21 Todo: update to an actual on robot value
    public static final int rightArmMotorID = 32; // 20 Todo: update to an actual on robot value

    public static final int solenoidSwitchID = 2;

    public static final int sForward_Channel = 7;
    public static final int sReverse_Channel = 8;
    public static final int hForward_Channel = 6; // Todo: update these values
    public static final int hReverse_Channel = 9; // Todo: update these values
    public static final int b_Channel = 2;

    // default arm pids
    // Todo: calibrate these
    public static final double p = 0.15;
    public static final double i = 0.000009;
    public static final double d = 0.01;
    public static final double f = 0;
    public static final double iz = 1.9;

    public static final double pidOutputMin = -1;
    public static final double pidOutputMax = 1;

    public static final double motorSpeed = 0.2; // 0 - 1 range

    public static final double motorRampRate = 0.6;
    public static final int currentLimit = 20;
    public static final double speakerAlignTagArea = 2;

    public static final double stopRange = 0.3;
    public static final double encoderHardMax = 34;
    public static final long s_stateChangeDelay = 8;

    public static final int smartMotionSlot = 0;

    // Arm movement constants
    public static final double tuckSafeMin =
        25; // Robot hard min 25 ticks, min must be further tuned later
    public static final double armGearRatio = 400;

    public static final double tuckArmAngle = 24;
    public static final double intakeArmAngle = 21.0;
    public static final double ampArmAngle = 31.3;
    public static final double liftArmAngle = 21;
    public static final double speakerArmAngle = 29; // Todo: find a real value for this

    public static final int kMeasuredPosHorizontal =
        21; // Default position measured when arm is horizontal from
    // example. Todo: find the value
    // for our arm.
    public static final double maxGravityFF =
        0.07; // Todo: Find the best gravity feed forward for our arm

    // Arm angle calculation constants
    public static final double armToSpeakerHeight =
        0.1524; // from the motor axel to speaker(goal - arm height from
    // ground)
    public static final double armToShooterAngle = 45;
    public static final double limelightToSpeakerHeight =
        0.1520; // from camera lens to speaker(goal - limelight height
    // from ground)
    public static final double limelightMountAngle = 31;
    public static final double armToLimelightDistance = 0.04;
  }

  public static final class Shooter {
    public static final int shooterID = 30;
    public static final int intakeID = 31;
    public static final int shootIR = 0;

    public static final int intakeRPM = 6000;
    public static final int pullBackRPM = 2000;
    public static final int correctPositioningRPM = 500;
    public static final int ampShootRPM = 2000;
    public static final int speakerShootRPM = 7200;
    public static final int ampWaitTime = 0;
    public static final int speakerWaitTime = 1000;

    // Default PID values to be tuned later
    public static final double shooterF = 0.13;
    public static final double shooterP = 0.11;
    public static final double shooterI = 0;
    public static final double shooterD = 0;

    // Default PID values to be tuned later
    public static final double intakeF = 0.12;
    public static final double intakeP = 0.11;
    public static final double intakeI = 0.5;
    public static final double intakeD = 0.0001;
  }

  public static final class Drivebase {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double creepSpeedMultiplier = 0.7;
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class AprilTagID {
    public static final int blueSourceInner = 1;
    public static final int blueSourceOuter = 2;
    public static final int redSpeakerSide = 3;
    public static final int redSpeakerMiddle = 4;
    public static final int redAmp = 5;
    public static final int blueAmp = 6;
    public static final int blueSpeakerMiddle = 7;
    public static final int blueSpeakerSide = 8;
    public static final int redSourceOuter = 9;
    public static final int redSourceInner = 10;
    public static final int redStageSourceSide = 11;
    public static final int redStageAmpSide = 12;
    public static final int redStageMiddle = 13;
    public static final int blueStageMiddle = 14;
    public static final int blueStageAmpSide = 15;
    public static final int blueStageSourceSide = 16;
  }
}
