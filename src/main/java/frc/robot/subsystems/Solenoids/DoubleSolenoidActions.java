package frc.robot.subsystems.Solenoids;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DoubleSolenoidActions {
  private DoubleSolenoid McDoubleSolenoid;// mc == master class

  public DoubleSolenoidActions(DoubleSolenoid solenoid) {
    McDoubleSolenoid = solenoid;
    setOff();
  }

  public DoubleSolenoidActions(DoubleSolenoid solenoid, boolean startForward) {
    this(solenoid);
    if (startForward) {
      setForward();
    } else {
      setReverse();
    }
  }

  public void toggle() {
    if (McDoubleSolenoid.get() == DoubleSolenoid.Value.kOff) {//Set the solenoid forward if no state is set.
      McDoubleSolenoid.set(Value.kForward);
    } else {
      McDoubleSolenoid.toggle();
    }
  }

  public void setOff() {
    McDoubleSolenoid.set(Value.kOff);
  }

  public void setReverse() {
    McDoubleSolenoid.set(Value.kReverse);
  }

  public void setForward() {
    McDoubleSolenoid.set(Value.kForward);
  }

  public void reportSolenoid() {
    System.out.println(
        "solenoid state: "
            + McDoubleSolenoid.get()
            + ", forward solenoid channel: "
            + McDoubleSolenoid.getFwdChannel()
            + ", reverse solenoid channel: "
            + McDoubleSolenoid.getRevChannel());
  }

  public boolean isReversed() {
    return McDoubleSolenoid.get() == Value.kReverse;
  }
}
