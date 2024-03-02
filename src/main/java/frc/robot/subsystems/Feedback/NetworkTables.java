package frc.robot.subsystems.Feedback;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables {
  private NetworkTable mainTable;
  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  double absoluteFrontLeftEncoder;
  double absoluteFrontRightEncoder;
  double absoluteBackLeftEncoder;
  double absoluteBackRightEncoder;
  double driveFrontLeftEncoder;
  double driveFrontRightEncoder;
  double driveBackLeftEncoder;
  double driveBackRightEncoder;

  public NetworkTables() {
    mainTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
  }

  public void printData() {
    double rawFrontLeftEncoder =
        mainTable.getEntry("Module[frontleft] Raw Absolute Encoder").getDouble(0);
    System.out.println(rawFrontLeftEncoder);
  }

  public double[] returnEncoderData() {
    absoluteFrontLeftEncoder =
        mainTable.getEntry("Module[frontleft] Raw Absolute Encoder").getDouble(0);
    absoluteFrontRightEncoder =
        mainTable.getEntry("Module[frontright] Raw Absolute Encoder").getDouble(0);
    absoluteBackLeftEncoder =
        mainTable.getEntry("Module[backleft] Raw Absolute Encoder").getDouble(0);
    absoluteBackRightEncoder =
        mainTable.getEntry("Module[backright] Raw Absolute Encoder").getDouble(0);

    return new double[] {
      absoluteFrontLeftEncoder,
      absoluteFrontRightEncoder,
      absoluteBackLeftEncoder,
      absoluteBackRightEncoder
    };
  }

  public double[] returnDriveData() {
    driveFrontLeftEncoder = mainTable.getEntry("Module[frontleft] Raw Drive Encoder").getDouble(0);
    driveFrontRightEncoder =
        mainTable.getEntry("Module[frontright] Raw Drive Encoder").getDouble(0);
    driveBackLeftEncoder = mainTable.getEntry("Module[backleft] Raw Drive Encoder").getDouble(0);
    driveBackRightEncoder = mainTable.getEntry("Module[backright] Raw Drive Encoder").getDouble(0);

    return new double[] {
      driveFrontLeftEncoder, driveFrontRightEncoder, driveBackLeftEncoder, driveBackRightEncoder
    };
  }

  public void setSwerveShuffleboard() {
    double[] driveData = returnDriveData();
    double[] angleData = returnEncoderData();
    shuffle.setTab("Swerve");
    shuffle.setSwerve("Front Left Module", driveData[0], angleData[0]);
    shuffle.setSwerve("Front Right Module", driveData[1], angleData[1]);
    shuffle.setSwerve("Back Left Module", driveData[2], angleData[2]);
    shuffle.setSwerve("Back Right Module", driveData[3], angleData[3]);
  }
}
