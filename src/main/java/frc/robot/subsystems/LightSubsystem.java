package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.Lights;

public class LightSubsystem {
  private final AddressableLED statusLights;
  private final AddressableLEDBuffer statusBuffer;
  private static LightSubsystem instance = null; // creates Singleton instance

  public enum lightStates {
    ReadyForPickup,
    NotePickedUp,
    CarryingNote,
    ReadyToShoot,
    AprilTagFound
  }

  public static synchronized LightSubsystem
      getInstance() { // if no instance has been made, create one.
    // Otherwise, reference the already made instance.
    // Ensures only one instance can be made.
    if (instance == null) {
      instance = new LightSubsystem(Lights.lightID, Lights.lightStringLength);
    }
    return instance;
  }

  private LightSubsystem(int lightPort, int stringLength) {
    statusLights = new AddressableLED(lightPort);
    statusBuffer = new AddressableLEDBuffer(stringLength);

    statusLights.setLength(stringLength);

    statusLights.start();
  }

  // Singleton instance of lights to call in other classes.

  public void SetLightState(lightStates stateToSet) { // Set to predefined colours
    switch (stateToSet) {
      case ReadyForPickup:
        setColor(255, 0, 0); // Red
        break;
      case NotePickedUp:
        setColor(255, 50, 0); // Orange
        break;
      case CarryingNote:
        setColor(0, 0, 255); // Blue
        break;
      case ReadyToShoot:
        setColor(0, 255, 0); // Green
        break;
      default:
        setColor(200, 0, 255); // Purple(for error)
        break;
    }
  }

  public void setColor(int r, int g, int b) { // set to specific colour
    for (int i = 0; i < statusBuffer.getLength(); i++) {
      statusBuffer.setRGB(i, g, r, b); // GRB
    }

    statusLights.setData(statusBuffer);
  }
}
