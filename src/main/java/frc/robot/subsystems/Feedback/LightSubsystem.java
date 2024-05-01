package frc.robot.subsystems.Feedback;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.Lights;
import frc.robot.subsystems.Vision.AprilTagSubsystem;

public class LightSubsystem {

  private final AddressableLED statusLights;
  private final AddressableLEDBuffer statusBuffer;
  private static I2C arduinoLightI2c = new I2C(I2C.Port.kOnboard, 0x07);
  // private SerialPort arduinoLightSPI = new SerialPort(9600, SerialPort.Port.kMXP); (Not worth it
  // as it requires a lot more wires compared to i2c)
  private static LightSubsystem instance = null; // creates Singleton instance

  private ShuffleboardSubsystem shuffle =
      ShuffleboardSubsystem.getInstance(); // Gets shuffleboard instance

  private AprilTagSubsystem limelight = AprilTagSubsystem.getInstance(); // Gets limelight instance

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
      instance = new LightSubsystem(Lights.lightID, Lights.lightStringLength);
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
        System.out.println("Lights Ready for Pickup");
        break;
      case NotePickedUp:
        setSolid(Color.kGreen);
        break;
      case CarryingNote:
        setSolid(Color.kOrange);
        System.out.println("Lights Carry position");
        break;
      case ReadyToAMP:
        /*
         * //Removed until limelight is back
         * id = limelight
         * .getTagID(); // todo: remove limelight subsystem from here and replace with
         * function
         * // for limelight subsystem
         * if (id == 3 || id == 4 || id == 7
         * || id == 8) { // todo: set speaker id in constants on startup based on
         * aliance colour
         * setSolid(Color.kBlue);
         * } else {
         * setDashed(Color.kBlue, Color.kBlack);
         * }
         */
        setSolid(Color.kBlue);
        break;
      case ReadyToSPEAKER:
        // removed until limelight is back
        /*
         * id = limelight.getTagID();
         * if (id == 5 || id == 6) { // todo: same here with amp id
         * setSolid(Color.kPink);
         * } else {
         * setDashed(Color.kPink, Color.kBlack);
         * }
         * break;
         * default:
         * setSolid(Color.kGray);
         */
        setSolid(Color.kPink);
        break;
    }
    shuffle.setText("Mode", stateToSet.toString());
  }

  public void setSolid(Color colour) { // set to specific colour
    for (int i = 0; i < statusBuffer.getLength(); i++) {
      statusBuffer.setLED(i, colour);
      /*
       * System.out.println(
       * "Led color: R "
       * + statusBuffer.getRed(i)
       * + " B "
       * + statusBuffer.getBlue(i)
       * + " G "
       * + statusBuffer.getGreen(i));
       */
    }
    statusLights.setData(statusBuffer);
    shuffle.setTab("Driver");
    shuffle.setColour("Lights", colour);
  }

  public void UpdateLEDs(String WriteString) // Constructor, pass it a string argument.
      {
    char[] CharArray =
        WriteString
            .toCharArray(); // Create an array of characters.  This breaks up the information into
    // something that can be passed over the I2C bus.
    byte[] RobotStatus =
        new byte
            [CharArray
                .length]; // Characters cannot be passed over I2C, thus we must convert them to
    // bytes. This line creates the byte array.
    for (int i = 0;
        i < CharArray.length;
        i++) // Create a loop that fills the new  byte array. The new byte array is the same size as
    // the character array.
    {
      RobotStatus[i] =
          (byte)
              CharArray[i]; // Pass information slot by slot. This also converts the characters into
      // bytes.
    }
    // arduino.transaction(RobotStatus, RobotStatus.length, null, 0);  //One type of sending info
    // over the I2C bus.  This method asks for a response from the receiving unit. Caused null point
    // exceptions.
    arduinoLightI2c.writeBulk(
        RobotStatus,
        RobotStatus
            .length); // This method sends info one way, without demanding a response from reader
    // unit.
    // ty team 386
  }

  public void setDashed(Color colour1, Color colour2) {
    for (int i = 0; i < statusBuffer.getLength(); i++) {
      if (Math.floor(i / 4) % 2 == 0) {
        statusBuffer.setRGB(i, (int) colour1.green, (int) colour1.red, (int) colour1.blue);
      } else {
        statusBuffer.setRGB(i, (int) colour2.green, (int) colour2.red, (int) colour2.blue);
      }
      shuffle.setTab("Driver");
      shuffle.setColour("Light Colour", colour2);
    }
    statusLights.setData(statusBuffer);
  }
}
