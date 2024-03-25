package frc.robot.subsystems.Solenoids;

import edu.wpi.first.wpilibj.Solenoid;

public class SolenoidActions {
  private final Solenoid jrSolenoid; // jr = java reticulating

  public SolenoidActions(Solenoid solenoid) {
    jrSolenoid = solenoid;
    setOff();
  }

  public void toggle() {
    jrSolenoid.set(!jrSolenoid.get());
  }

  public void setOff() {
    jrSolenoid.set(false);
  }

  public void setOn() {
    jrSolenoid.set(true);
  }

  public void reportSolenoid() {
    System.out.println(
        "solenoid state: " + jrSolenoid.get() + ", solenoid channel: " + jrSolenoid.getChannel());
  }

  public boolean getState() {
    return jrSolenoid.get();
  }
}
