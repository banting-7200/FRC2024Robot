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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleboardSubsystem {
    ShuffleboardLayout layout;
    ShuffleboardTab tab;
    private static ShuffleboardSubsystem instance = null;

    private ShuffleboardSubsystem(){
    }

    public static synchronized ShuffleboardSubsystem getInstance(){ 
        if(instance==null){
            instance = new ShuffleboardSubsystem();
        }
            return instance;
    }

    public void setTab(String tabName){ //sets which tab to put values to.
        tab = Shuffleboard.getTab(tabName);
    }

    public void setLayout(String layoutName){ //sets which layout to put values to. Putting values of the same name in two different layouts may mess with code.
        layout = tab.getLayout(layoutName);
    }

    public void setNumber(String name, double value){ //creates or sets a double on the shuffleboard.
        GenericEntry entry;
        if(layout!=null){
            entry = layout.add(name,0).withWidget(BuiltInWidgets.kTextView).getEntry();
        } else {
            entry = tab.add(name, 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        }
        entry.setDouble(value);
    }

    public void setNumber(String name, double value, BuiltInWidgets type){ //creates or sets a double on the shuffleboard with a specific widget.
        GenericEntry entry;
        if(layout!=null){
            entry = layout.add(name,0).withWidget(type).getEntry();
        } else {
            entry = tab.add(name, 0).withWidget(type).getEntry();
        }
        entry.setDouble(value);
    }

    public void setBoolean(String name, boolean value){ //creates or sets a boolean on the shuffleboard.
        GenericEntry entry;
        if(layout!=null){
            entry = layout.add(name,false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        } else {
            entry = tab.add(name,false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        }
        entry.setBoolean(value);
    }

    public double getNumber(String name){ //returns a double from the shuffleboard.
        GenericEntry entry = tab.add(name,0).getEntry();
        return entry.getDouble(0);
    }

    public boolean getBoolean(String name){ //returns a boolean from the shuffleboard.
        GenericEntry entry = tab.add(name, false).getEntry();
        return entry.getBoolean(false);
    }

    public void setSwerve(String name, double speed, double direction){ //create a named list layout of swerve motor and direction values.
        setLayout(name);
        setNumber(name+" Speed", speed, BuiltInWidgets.kDial);
        setNumber(name+" Direction", direction, BuiltInWidgets.kGyro);
    }

    public void setPID(String name, double p, double i, double d){ //create a named list layout of PID values. 
        setLayout(name);
        setNumber(name+" P", p);
        setNumber(name+" I", i);
        setNumber(name+" D", d);
    }

    public double[] getPID(String name){ //return an array of PID values stored under the name they were originally set with.
        GenericEntry p = tab.add(name+" P", 0).getEntry();
        GenericEntry i = tab.add(name+" P", 0).getEntry();
        GenericEntry d = tab.add(name+" P", 0).getEntry();
        return new double[] {p.getDouble(0),i.getDouble(0),d.getDouble(0)};
    }
}
