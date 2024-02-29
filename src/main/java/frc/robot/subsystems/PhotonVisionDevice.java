package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.math.MathContext;

public class PhotonVisionDevice extends SubsystemBase {
    private NetworkTable photonTable;
    public int mode;

    public PhotonVisionDevice() { // initializes device
        photonTable = NetworkTableInstance.getDefault().getTable("limelight"); // gets the network table with key
    }

    public boolean seeNote() {
        boolean seeNoteBoolean = photonTable.getEntry("hasTarget").getBoolean(false);
        return seeNoteBoolean;
    }

    public double getNoteAngleOffset() {
        double noteRotation = photonTable.getEntry("targetRotation").getDouble(0);
        return noteRotation;
    }

    public double noteDistance() {
        double targetDistance = photonTable.getEntry("targetDistance").getDouble(0);
        return targetDistance;
    }

    public void putTagData() { // publishes tag data to SmartDashboard
        double ttarget = photonTable.getEntry("tv").getDouble(0); // may have to be double then converted
        double tx = photonTable.getEntry("tx").getDouble(0);
        double ty = photonTable.getEntry("ty").getDouble(0);
        double ta = photonTable.getEntry("ta").getDouble(0);
        double tid = photonTable.getEntry("tid").getDouble(-1);
        boolean tdetected = ttarget == 0 ? false : true;

        SmartDashboard.putBoolean("AprilTag Detected", tdetected);
        SmartDashboard.putNumber("Tag X", tx);
        SmartDashboard.putNumber("Tag Y", ty);
        SmartDashboard.putNumber("Tag Area", ta);
        SmartDashboard.putNumber("AprilTag ID", tid);
    }

    public Pose2d getFakeTagPose() { // input fake tag values for simulation
        // Can't take negative values because it is trying to pathfind out of the feild
        return new Pose2d(2.98, 4, new Rotation2d(33));
    }
}
