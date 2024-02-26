package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.Lights;
import frc.robot.subsystems.ShuffleboardSubsystem;
import java.awt.Color;
import frc.robot.subsystems.LimelightDevice;

public class LightSubsystem {
  private final AddressableLED statusLights;
  private final AddressableLEDBuffer statusBuffer;
  private static LightSubsystem instance = null; // creates Singleton instance
  private ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();
  private Color colour1 = new Color(127, 127, 127);
  private Color colour2 = new Color(127, 127, 127);
  private LimelightDevice limelight = new LimelightDevice();

  public enum lightStates {
    ReadyForPickup,
    NotePickedUp,
    CarryingNote,
    ReadyToSPEAKER,
    ReadyToAMP,
    ReadyToShoot,
    AprilTagFound
  }

  public void shuffleSetup() {
    shuffle.setTab("Lights");
    shuffle.setNumber("Mode", -1);
  }

  public static synchronized LightSubsystem getInstance() { // if no instance has been made, create one.
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
    int id = -1;
    switch (stateToSet) {
      case ReadyForPickup: // in pickup position & intake on
        colour1 = Color.RED;
        colour2 = colour1;
        shuffle.setText("Mode", "ReadyForPickup");
        break;
      case NotePickedUp: // note picked up but not in position position
        colour1 = Color.GREEN;
        colour2 = colour1;
        shuffle.setText("Mode", "NotePickedUp");
        break;
      case CarryingNote: // note picked up and in carry position
        colour1 = Color.ORANGE;
        colour2 = colour1;
        shuffle.setText("Mode", "CarryingNote");
        break;
      case ReadyToAMP: // in position to score in AMP
        colour1 = Color.BLUE;
        id = limelight.getTagID();
        if (id == 3 || id == 4 || id == 7 || id == 8) {
          colour2 = colour1;
        } else {
          colour2 = Color.BLACK;
        }
        shuffle.setText("Mode", "ReadyToAMP");
        break;
      case ReadyToSPEAKER: // in position to score in SPEAKER
        colour1 = Color.PINK;
        id = limelight.getTagID();
        if (id == 5 || id == 6) {
          colour2 = colour1;
        } else {
          colour2 = Color.BLACK;
        }
        shuffle.setText("Mode", "ReadyToSPEAKER");
        break;
      default:
        colour1 = Color.BLACK;
        colour2 = colour1;
        shuffle.setText("Mode", "Error");
        break;
    }
    setColourMode();
  }

  public void setColor(int r, int g, int b) { // set to specific colour
    for (int i = 0; i < statusBuffer.getLength(); i++) {
        statusBuffer.setRGB(i, r, g, b);
      }
    statusLights.setData(statusBuffer);
  }

  public void setColourMode(){
    for (int i = 0; i < statusBuffer.getLength(); i++) {
      if (Math.floor(i / 4) % 2 == 0) {
        statusBuffer.setRGB(i, colour1.getGreen(), colour1.getRed(), colour1.getBlue());
      } else {
      statusBuffer.setRGB(i, colour2.getGreen(), colour2.getRed(), colour2.getBlue());
      }
    }
  }
}
