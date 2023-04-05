package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class ConeCone extends SequentialCommandGroup {

  public ConeCone(
    Swerve s_Swerve,
    Elevator m_Elevator,
    Claw m_Claw,
    Limelight m_Limelight
  ) {
    PathPlannerTrajectory traj1 = PathPlanner.loadPath("Cone2GP", 4, 4);
    PathPlannerTrajectory traj3 = PathPlanner.loadPath("GP2Cone", 4, 4);
    addCommands(
      m_Claw.closeAllHold(),
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCone,
        Constants.armTopCone
      ),
      m_Claw.openAllDrop(),
      new WaitCommand(.25),
      m_Claw.motorOff(),
      m_Elevator.setStow(),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(.25),
          m_Elevator.setToFloor(),
          m_Claw.openAllIn()
        ),
        new SequentialCommandGroup(
          s_Swerve.followTrajectoryCommand(traj1, true)
        )
      ),
      m_Claw.closeAllHold(),
      new ParallelCommandGroup(
        m_Elevator.setStow(),
        new InstantCommand(() -> m_Limelight.setToRetroreflectiveTape()),
        new SequentialCommandGroup(
          s_Swerve.followTrajectoryCommand(traj3, false)
        )
      ),
      s_Swerve.moveToGoal(),
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCone,
        Constants.armTopCone
      ),
      m_Claw.openAllDrop(),
      new WaitCommand(.5),
      m_Claw.motorOff(),
      m_Elevator.setStow()
    );
  }
}
