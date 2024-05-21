package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import java.time.Clock;

/*The class is a helper for the Note Auto State Machine to schedule two commands at once and advance the satte machine once both finish.*/
public class PrepForShoot extends Command {

  AprilTagAlign tagAlign;
  Command prepShoot;
  NoteAutoStateMachine stateInstance;

  private Clock currentTime = Clock.systemDefaultZone();
  private long startedMillis;
  private long currentMillis;

  public PrepForShoot(Command prepShoot, NoteAutoStateMachine stateInstance) {
    // this.tagAlign = tagAlign;
    this.prepShoot = prepShoot;
    this.stateInstance = stateInstance;
  }

  @Override
  public void initialize() { // Schedule both commands when the helper is initialized
    System.out.println("Scheduled print shoot");
    startedMillis = currentTime.millis();
    prepShoot.schedule();
  }

  @Override
  public boolean isFinished() { // End the helper command only if both commands have ended
    return prepShoot.isFinished() || currentMillis - startedMillis > 10000;
  }

  @Override
  public void end(boolean interrupted) { // Move to the next state once ended
    System.out.println("Finished prep shoot");
    stateInstance.MoveToState(NoteAutoStateMachine.States.Shoot);
  }
}
