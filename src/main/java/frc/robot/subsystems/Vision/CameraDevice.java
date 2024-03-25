package frc.robot.subsystems.Vision;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;

public class CameraDevice {
  UsbCamera robotCamera = new UsbCamera("USB Camera 0", 0);
  MjpegServer camServer = new MjpegServer("serve_USB Camera 0", 1181);
  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  public void setCameraShuffle() {
    shuffle.addCamera("Robot Camera", "serve_USB", "10.72.0.11:1181");
  }
}
