package frc.robot.subsystems.Feedback;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.Lights;
import frc.robot.subsystems.Vision.LimelightDevice;
import java.awt.Color;

public class LightSubsystem {

  private final AddressableLED statusLights;
  private final AddressableLEDBuffer statusBuffer;

  private static LightSubsystem instance = null; // creates Singleton instance
  private ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();//Gets shuffleboard instance

  private LimelightDevice limelight = LimelightDevice.getInstance();//Gets limelight instance

  public enum LightStates {
    ReadyForPickup, //in intake position and intake on
    NotePickedUp, //note detected in intake
    CarryingNote, //in carry position with note in intake
    ReadyToSPEAKER, //in position to score in speaker
    ReadyToAMP, //in position to score in amp
  }

  //Initializes shuffleboard
  public void shuffleSetup() {
    shuffle.setTab("Lights");
    shuffle.setText("Mode", "No Mode Selected");
  }

  // if no instance has been made, create one.
  // Otherwise, reference the already made instance.
  // Ensures only one instance can be made.
  public static synchronized LightSubsystem getInstance() { 
    if (instance == null) {
      instance = new LightSubsystem(Lights.lightID, Lights.lightStringLength);
    }
    return instance;
  }

  private LightSubsystem(int lightPort, int stringLength) {
    statusLights = new AddressableLED(lightPort);//Inititalizes light string with passed in values
    statusBuffer = new AddressableLEDBuffer(stringLength);//Inititalizes light buffer with passed in values
    statusLights.setLength(stringLength);
    statusLights.start();
  }

  public void SetLightState(LightStates stateToSet) { // Set to predefined colours
    int id = -1;
    switch (stateToSet) {
      case ReadyForPickup:
       setSolid(Color.RED);
        break;
      case NotePickedUp:
       setSolid(Color.GREEN);
        break;
      case CarryingNote:
        setSolid(Color.ORANGE);
        break;
      case ReadyToAMP:
        id = limelight.getTagID(); //todo: remove limelight subsystem from here and replace with function for limelight subsystem
        if (id == 3 || id == 4 || id == 7 || id == 8) { //todo: set speaker id in constants on startup based on aliance colour
          setSolid(Color.BLUE);
        } else {
          setDashed(Color.BLUE, Color.BLACK);
        }
        break;
      case ReadyToSPEAKER:
        id = limelight.getTagID();
        if (id == 5 || id == 6) { //todo: same here with amp id
          setSolid(Color.PINK);
        } else {
          setDashed(Color.PINK, Color.BLACK);
        }
        break;
      default:
        setSolid(Color.GRAY);
        break;
    }
    shuffle.setText("Mode", stateToSet.toString());
  }
  
  public void setSolid(Color colour) { // set to specific colour
    for (int i = 0; i < statusBuffer.getLength(); i++) {
      statusBuffer.setRGB(i, colour.getGreen(), colour.getRed(), colour.getBlue());
    }
    statusLights.setData(statusBuffer);
  }


  public void setDashed(Color colour1, Color colour2) {
    for (int i = 0; i < statusBuffer.getLength(); i++) {
      if (Math.floor(i / 4) % 2 == 0) {
        statusBuffer.setRGB(i, colour1.getGreen(), colour1.getRed(), colour1.getBlue());
      } else {
        statusBuffer.setRGB(i, colour2.getGreen(), colour2.getRed(), colour2.getBlue());
      }
      shuffle.setTab("Lights");
      shuffle.setColour("Light Colour", colour1);
    }
    statusLights.setData(statusBuffer);
  }
}
