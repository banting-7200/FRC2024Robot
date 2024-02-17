package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DoubleSolenoidActions {
    private DoubleSolenoid SOLENOID;

    public DoubleSolenoidActions(DoubleSolenoid s) {
        SOLENOID = s;
        setOff();
    }

    public void toggle() {
        SOLENOID.toggle();
    }

    public void setOff() {
        SOLENOID.set(Value.kOff);
    }

    public void setReverse() {
        SOLENOID.set(Value.kReverse);
    }

    public void setForward() {
        SOLENOID.set(Value.kForward);
    }

    public void reportSolenoid() {
        System.out.println("solenoid state: " + SOLENOID.get() + ", forward solenoid channel: " + SOLENOID.getFwdChannel() + ", reverse solenoid channel: " + SOLENOID.getRevChannel());
    }

    public boolean getState() {
        return SOLENOID.get() == Value.kReverse;
    }
}
