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
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class Auton {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

   public static final PneumaticHub m_pH = new PneumaticHub(3);

  public static final class Arm {
    public static final int leftArmMotorID = 33;//21 Todo: update to an actual on robot value
    public static final int rightArmMotorID = 32;//20 Todo: update to an actual on robot value
    
    public static final int sForward_Channel = 0;
    public static final int sReverse_Channel = 15;
    public static final int hForward_Channel = 1;//Todo: update these values
    public static final int hReverse_Channel = 14;//Todo: update these values
    public static final int b_Channel = 2;

    // default arm pids
    // Todo: calibrate these
    public static final double p = 0.0001;
    public static final double i = 0.4;//0.03
    public static final double d = 0;
    public static final double f = 0;
    public static final double iz = 0.08;

    public static final double pidOutputMin = -1;
    public static final double pidOutputMax = 1;

    public static final int smartMotionSlot = 0;
    public static final double maxMotorVelocity = 1700; //max motor velocity in rpm
    public static final double minMotorVelocity = 0;
    public static final double maxMotorAccel = 1400;
    public static final double allowedPIDError = 0;
     
    //Arm movement constants
    //Todo: find the min-max safe range to tuck the shooter
    public static final double tuckSafeMin = 10, tuckSafeMax = 11;// Robot hard max 11 ticks, min must be further tuned later

    public static final double tuckArmAngle = 8;
    public static final double intakeArmAngle = 0;
    public static final double ampArmAngle = 12;

     public static final int kMeasuredPosHorizontal = 840; // Default position measured when arm is horizontal from example. Todo: find the value for our arm.
     public static final double maxGravityFF = 0.07;// Todo: Find the best gravity feed forward for our arm

    //Arm angle calculation constants
    public static final double armToSpeakerHeight = 0.1524;// from the motor axel to speaker(goal - arm height from ground)
    public static final double armToShooterAngle = 45;
    public static final double limelightToSpeakerHeight = 0.1520;// from camera lens to speaker(goal - limelight height from ground)
    public static final double limelightMountAngle = 31;
    public static final double armToLimelightDistance = 0.04;
  }
  public static final class Shooter
  {
      public static final int shooterID = 0;
      public static final int intakeID = 1;
      public static final int sensorPin = 0;
      
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
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }
}
