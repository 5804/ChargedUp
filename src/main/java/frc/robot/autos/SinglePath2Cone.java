package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import java.util.List;

public class SinglePath2Cone extends SequentialCommandGroup {

  public SinglePath2Cone(Swerve s_Swerve, Elevator m_Elevator, Claw m_Claw) {
    PathPlannerTrajectory traj = PathPlanner.loadPath("1Path2Cone", 4, 4);
    // List<EventMarker> trajevents = traj.getMarkers();
    // trajevents.get(0).waypointRelativePos;
    addCommands(
      m_Claw.closeAllHold(),
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCone,
        Constants.armTopCone
      ),
      m_Claw.openAllDrop(),
      new WaitCommand(.5),
      new ParallelCommandGroup(
        m_Elevator.setStow(),
        new SequentialCommandGroup(
          new WaitCommand(.25),
          s_Swerve.followTrajectoryCommand(traj, true)
        )
      )
    );
  }
}
