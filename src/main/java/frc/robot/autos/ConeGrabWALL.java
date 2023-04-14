package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class ConeGrabWALL extends SequentialCommandGroup {

  public ConeGrabWALL(Swerve s_Swerve, Elevator m_Elevator, Claw m_Claw) {
    PathPlannerTrajectory traj1 = PathPlanner.loadPath("Cone2GP2", 1.9, 1.9);
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
      m_Claw.closeAllHold()
    );
  }
}
