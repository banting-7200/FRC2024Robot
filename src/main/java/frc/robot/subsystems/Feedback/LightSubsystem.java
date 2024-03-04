package frc.robot.subsystems.Feedback;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.Lights;
import frc.robot.subsystems.Vision.LimelightDevice;

public class LightSubsystem {

  private final AddressableLED statusLights;
  private final AddressableLEDBuffer statusBuffer;

  private static LightSubsystem instance = null; // creates Singleton instance

  private ShuffleboardSubsystem shuffle =
      ShuffleboardSubsystem.getInstance(); // Gets shuffleboard instance

  private LimelightDevice limelight = LimelightDevice.getInstance(); // Gets limelight instance

  public enum LightStates {
    ReadyForPickup, // in intake position and intake on
    NotePickedUp, // note detected in intake
    CarryingNote, // in carry position with note in intake
    ReadyToSPEAKER, // in position to score in speaker
    ReadyToAMP, // in position to score in amp
  }

  // Initializes shuffleboard
  public void shuffleSetup() {
    shuffle.setTab("Lights Tab");
    shuffle.setText("Mode", "No Mode Selected");
  }

  // if no instance has been made, create one.
  // Otherwise, reference the already made instance.
  // Ensures only one instance can be made.
  public static synchronized LightSubsystem getInstance() {
    if (instance == null) {
      instance = new LightSubsystem(Lights.lightIDLeft, Lights.lightStringLengthLeft);
    }
    return instance;
  }

  private LightSubsystem(int lightPort, int stringLength) {
    statusLights = new AddressableLED(lightPort); // Inititalizes light string with passed in values
    statusBuffer =
        new AddressableLEDBuffer(stringLength); // Inititalizes light buffer with passed in values
    statusLights.setLength(stringLength);
    statusLights.start();
  }

  public void SetLightState(LightStates stateToSet) { // Set to predefined colours
    int id = -1;
    switch (stateToSet) {
      case ReadyForPickup:
        setSolid(Color.kRed);
        break;
      case NotePickedUp:
        setSolid(Color.kGreen);
        break;
      case CarryingNote:
        setSolid(Color.kOrange);
        break;
      case ReadyToAMP:
        id =
            limelight
                .getTagID(); // todo: remove limelight subsystem from here and replace with function
        // for limelight subsystem
        if (id == 3 || id == 4 || id == 7
            || id == 8) { // todo: set speaker id in constants on startup based on aliance colour
          setSolid(Color.kBlue);
        } else {
          setDashed(Color.kBlue, Color.kBlack);
        }
        break;
      case ReadyToSPEAKER:
        id = limelight.getTagID();
        if (id == 5 || id == 6) { // todo: same here with amp id
          setSolid(Color.kPink);
        } else {
          setDashed(Color.kPink, Color.kBlack);
        }
        break;
      default:
        setSolid(Color.kGray);
        break;
    }
    shuffle.setText("Mode", stateToSet.toString());
  }

  public void setSolid(Color colour) { // set to specific colour
    for (int i = 0; i < statusBuffer.getLength(); i++) {
      statusBuffer.setRGB(i, (int) colour.green, (int) colour.red, (int) colour.blue);
    }
    statusLights.setData(statusBuffer);
    shuffle.setTab("Data");
    shuffle.setColour("Lights", colour);
  }

  public void setDashed(Color colour1, Color colour2) {
    for (int i = 0; i < statusBuffer.getLength(); i++) {
      if (Math.floor(i / 4) % 2 == 0) {
        statusBuffer.setRGB(i, (int) colour1.green, (int) colour1.red, (int) colour1.blue);
      } else {
        statusBuffer.setRGB(i, (int) colour2.green, (int) colour2.red, (int) colour2.blue);
      }
      shuffle.setTab("Data");
      shuffle.setColour("Light Colour", Color.kGray);
    }
    statusLights.setData(statusBuffer);
  }
}
