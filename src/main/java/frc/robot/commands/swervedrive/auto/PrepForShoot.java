package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.arm.TuckArm;
import java.time.Clock;

/*The class is a helper for the Note Auto State Machine to schedule two commands at once and advance the satte machine once both finish.*/
public class PrepForShoot extends Command {

  AprilTagAlign tagAlign;
  TuckArm tuckArm;
  MoveArmToPosition armToShoot;
  NoteAutoStateMachine stateInstance;

  private Clock currentTime = Clock.systemDefaultZone();
  private long startedMillis;

  boolean finished = false;

  public PrepForShoot(
      AprilTagAlign aprilTagAlign,
      TuckArm tuckArm,
      MoveArmToPosition armToShoot,
      NoteAutoStateMachine stateInstance) {
    this.tagAlign = aprilTagAlign;
    this.tuckArm = tuckArm;
    this.armToShoot = armToShoot;
    this.stateInstance = stateInstance;
  }

  Command prepSequence;

  @Override
  public void initialize() { // Schedule both commands when the helper is initialized
    System.out.println("Scheduled prep shoot");
    startedMillis = currentTime.millis();
    prepSequence =
        tagAlign
            .andThen(tuckArm)
            .andThen(armToShoot)
            .finallyDo(
                () -> {
                  System.out.println("Sequence complete!");
                  finished = true;
                });
    prepSequence.schedule();
  }

  @Override
  public boolean isFinished() { // End the helper command only if both commands have ended
    return finished /* || currentTime.millis() - startedMillis > 6000 */;
  }

  @Override
  public void end(boolean interrupted) { // Move to the next state once ended
    System.out.println("Finished prep shoot");
    stateInstance.MoveToState(NoteAutoStateMachine.States.Shoot);
  }
}
