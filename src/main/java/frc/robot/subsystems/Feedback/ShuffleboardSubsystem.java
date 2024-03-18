/* To Do:
 * Find a clean way to sort widgets & figure out why .withPosition doesn't do this.
 * Create final layout
 * Test cameras
 * switch layout previous after pid functions etc. so new functions dont get added to them.
 */

package frc.robot.subsystems.Feedback;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

// import java.awt.Color;

public class ShuffleboardSubsystem {
  private static ShuffleboardSubsystem instance = null;
  ShuffleboardLayout layout;
  ShuffleboardTab tab;
  List<String> entryNames = new ArrayList<String>();
  List<GenericEntry> entries = new ArrayList<GenericEntry>();
  SendableChooser<String> autos;
  SimpleWidget lights;

  private ShuffleboardSubsystem() {
    setTab("Driver");
    lights = tab.add("Lights", true).withWidget(BuiltInWidgets.kBooleanBox);
  }

  // Singleton instance of the shuffleboard class
  public static synchronized ShuffleboardSubsystem getInstance() {
    if (instance == null) {
      instance = new ShuffleboardSubsystem();
    }
    return instance;
  }

  public void setTab(String tabName) { // sets which tab to put values to.
    tab = Shuffleboard.getTab(tabName);
  }

  public void setLayout(
      String layoutName,
      int x,
      int y) { // sets which layout to put values to. Putting values of the same name in two
    // different layouts may mess with code.
    layout = tab.getLayout(layoutName, "List Layout").withSize(x, y);
  }

  public void setLayout(
      String layoutName) { // sets which layout to put values to. Putting values of the same name in
    // two
    // different layouts may mess with code.
    if (layoutName == null) {
      layout = null;
    } else {
      layout = tab.getLayout(layoutName, "List Layout");
    }
  }

  public void setNumber(
      String name, double value) { // creates or sets a double on the shuffleboard.
    GenericEntry entry;
    int index = entryNames.indexOf(name);
    if (index == -1) { // If this entry does not exist
      if (layout != null) { // if theres no layout selected
        entry = layout.add(name, value).withWidget(BuiltInWidgets.kTextView).getEntry();
      } else {
        entry = tab.add(name, value).withWidget(BuiltInWidgets.kTextView).getEntry();
      }
      entryNames.add(name);
      entries.add(entry);
    } else {
      entries.get(index).setDouble(value);
    }
  }

  public void setNumber(String name, double value, int x, int y) {
    GenericEntry entry;
    int index = entryNames.indexOf(name);
    if (index == -1) {
      if (layout != null) {
        entry =
            layout
                .add(name, value)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(x, y)
                .getEntry();
      } else {
        entry =
            tab.add(name, value).withWidget(BuiltInWidgets.kTextView).withPosition(x, y).getEntry();
      }
      entryNames.add(name);
      entries.add(entry);
    } else {
      entries.get(index).setDouble(value);
    }
  }

  public void setNumber(
      String name, double value, BuiltInWidgets type) { // creates or sets a double on the
    // shuffleboard with a specific widget.
    GenericEntry entry;
    int index = entryNames.indexOf(name);
    if (index == -1) {
      if (layout != null) {
        entry = layout.add(name, value).withWidget(type).getEntry();
      } else {
        entry = tab.add(name, value).withWidget(type).getEntry();
      }
      entryNames.add(name);
      entries.add(entry);
    } else {
      entries.get(index).setDouble(value);
    }
  }

  public void setBoolean(
      String name, boolean value) { // creates or sets a boolean on the shuffleboard.
    GenericEntry entry;
    int index = entryNames.indexOf(name);
    if (index == -1) {
      if (layout != null) {
        entry = layout.add(name, value).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
      } else {
        entry = tab.add(name, value).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
      }
      entryNames.add(name);
      entries.add(entry);
    } else {
      entries.get(index).setBoolean(value);
    }
  }

  public double getNumber(String name) { // returns a double from the shuffleboard.
    int index = entryNames.indexOf(name);
    if (index == -1) {
      setNumber(name, 0);
      return 0;
    } else {
      return entries.get(index).getDouble(0);
    }
  }

  public boolean getBoolean(String name) { // returns a boolean from the shuffleboard.
    int index = entryNames.indexOf(name);
    if (index == -1) {
      setBoolean(name, false);
      return false;
    } else {
      return entries.get(index).getBoolean(false);
    }
  }

  public void setSwerve(
      String name,
      double speed,
      double direction) { // create a named list layout of swerve motor and
    // direction values.
    setLayout(name, 2, 2);
    setNumber(
        name + " Speed", speed, BuiltInWidgets.kDial); // todo:see if max and mins needs to be set.
    setNumber(name + " Direction", direction, BuiltInWidgets.kGyro); // same here
    setLayout(null);
  }

  public void setPID(
      String name,
      double p,
      double i,
      double d,
      double f,
      double iz) { // create a named list layout of
    // PID values.
    setTab("PID");
    setLayout(name, 2, 4);
    setNumber(name + " P", p, 0, 0);
    setNumber(name + " I", i, 0, 1);
    setNumber(name + " D", d, 0, 2);
    setNumber(name + " F", f, 0, 3);
    setNumber(name + " IZ", iz, 0, 4);
    setLayout(null);
  }

  public double[] getPID(
      String name) { // return an array of PID values stored under the name they were originally set
    // with.
    double p = getNumber(name + " P");
    double i = getNumber(name + " I");
    double d = getNumber(name + " D");
    double f = getNumber(name + " F");
    double iz = getNumber(name + " IZ");
    return new double[] {p, i, d, f, iz};
  }

  public void newCommandButton(
      String name,
      Command command) { // creates button which automatically runs the command it was set with.
    tab.add(name, command);
  }

  public void newAutoChooser(
      SendableChooser<String>
          inAutos) { // creates drop down containing autos, doesn't add any functionality to the
    // basic function, just contains everything in the subsystem
    setTab("Pre-Match");
    autos = inAutos;
    tab.add("Autos", autos).withSize(2, 1);
    autos.setDefaultOption("No Auto Selected", "(M) Basic Auto");
  }

  public String getAuto() { // returns auto from drop down;
    return (autos.getSelected());
  }

  public void addCamera(String name, String camera, String url) {
    tab.addCamera(name, camera, url).withSize(2, 2); // doesnt add any functionalaty again
  }

  public void setText(String name, String text) { // puts text to the dashboard
    GenericEntry entry;
    int index = entryNames.indexOf(name);
    if (index == -1) {
      if (layout != null) {
        entry = layout.add(name, text).getEntry();
      } else {
        entry = tab.add(name, text).getEntry();
      }
      entryNames.add(name);
      entries.add(entry);
    } else {
      entries.get(index).setString(text);
    }
  }

  public String getText(String name) { // returns text from shuffleboard
    int index = entryNames.indexOf(name);
    if (index == -1) {
      setText(name, " ");
      return " ";
    } else {
      return entries.get(index).getString(" ");
    }
  }

  public void setColour(String name, Color colour) { // custom colour widget
    try {
      GenericEntry entry;
      int index = entryNames.indexOf(name);
      Map<String, Object> map = Map.of("colorWhenTrue", colour.toHexString());
      if (layout != null) {
        entry =
            layout
                .add(name, true)
                .withProperties(map)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .getEntry();
      } else {
        entry =
            tab.add(name, true)
                .withProperties(map)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .getEntry();
      }
      if (index == -1) {
        entryNames.add(name);
        entries.add(entry);
      }

      // lights.withProperties(Map.of("colorWhenTrue", colour.toHexString()));
    } catch (IllegalArgumentException e) {
      System.out.println("Set Color Error, EXEPTION: " + e);
    }
  }
}
