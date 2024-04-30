package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.MoveArmToPosition;

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
  public void initialize() {
    tagAlign.schedule();
    moveArm.schedule();
  }
  
  @Override
  public boolean isFinished() {
    return tagAlign.isFinished() && moveArm.isFinished();
  }
  
  @Override
  public void end(boolean interrupted) {
    stateInstance.MoveToState(NoteAutoStateMachine.States.Shoot);
  }
}
