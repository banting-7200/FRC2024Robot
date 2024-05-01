package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.MoveArmToPosition;

/*The class is a helper for the Note Auto State Machine to schedule two commands at once and advance the satte machine once both finish.*/
public class PrepForShoot extends Command {
  
  AprilTagAlign tagAlign;
  MoveArmToPosition moveArm;
  NoteAutoStateMachine stateInstance;

  public PrepForShoot(AprilTagAlign tagAlign,
  MoveArmToPosition moveArm, NoteAutoStateMachine stateInstance) {
    this.tagAlign = tagAlign;
    this.moveArm = moveArm;
    this.stateInstance = stateInstance;
  }
  
  @Override
  public void initialize() {//Schedule both commands when the helper is initialized
    tagAlign.schedule();
    moveArm.schedule();
  }
  
  @Override
  public boolean isFinished() {//End the helper command only if both commands have ended
    return tagAlign.isFinished() && moveArm.isFinished();
  }
  
  @Override
  public void end(boolean interrupted) {//Move to the next state once ended
    stateInstance.MoveToState(NoteAutoStateMachine.States.Shoot);
  }
}
