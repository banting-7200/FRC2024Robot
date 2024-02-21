/* To Do:
 * Test if .add will create new component if component with same name is in another tab or layout.
 * Test if declaring layout and tab outside of functions causes any problems.
 * Find a clean way to sort widgets & figure out why .withPosition doesn't do this.
 * Research if widgets can be edited after they are created & if they can be done externally (passed through a function).
 * Figure out a way to set size of window & if magification can be changed (not a lot of room for widgets & switching tabs is annoying).
 * See if widgets can be hidden or moved, or if tab can be switched through code (e.g. after autonomous switch to new tab).
 * Test with actual code.
 * Look into sendables more, seems like it shouldn't be so messy to deal with. Might not be necessary if this code works fine though.
 * Create function that sets up and organizes dashboard so it can be run on robot startup and automatically populate dashboard on new computers.
 */

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;

public class ShuffleboardSubsystem {
  ShuffleboardLayout layout;
  ShuffleboardTab tab = Shuffleboard.getTab("arm");
  List<String> entryNames = new ArrayList<String>();
  List<GenericEntry> entries = new ArrayList<GenericEntry>();
  private static ShuffleboardSubsystem instance = null;

  private ShuffleboardSubsystem() {}

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

  public void setNumber(
      String name, double value) { // creates or sets a double on the shuffleboard.
    GenericEntry entry;
    int index = entryNames.indexOf(name);
    if (index == -1) {
      if (layout != null) {
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
    setNumber(name + " Speed", speed, BuiltInWidgets.kDial);
    setNumber(name + " Direction", direction, BuiltInWidgets.kGyro);
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
    setLayout(name, 1, 4);
    setNumber(name + " P", p);
    setNumber(name + " I", i);
    setNumber(name + " D", d);
    setNumber(name + " F", f);
    setNumber(name + " IZ", iz);
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
    SmartDashboard.putData(name, command);
  }
}
