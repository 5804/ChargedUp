package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import java.util.HashMap;
import java.util.List;

public class ConeLowConeWALL extends SequentialCommandGroup {

  public ConeLowConeWALL(Swerve s_Swerve, Elevator m_Elevator, Claw m_Claw) {
    PathPlannerTrajectory traj = PathPlanner.loadPath(
      "1PathConeMConeWallSAFE",
      2.1,
      2.1
    );
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put(
      "IntakeCone",
      new SequentialCommandGroup(
        m_Elevator.setToFloor(),
        m_Claw.openConeCommand()
      )
    );
    eventMap.put("StowAnyway", m_Elevator.setStow());
    eventMap.put("Wait(0.5)", new WaitCommand(.5));
    eventMap.put("Wait(0.25)", new WaitCommand(.25));
    eventMap.put("DropCone", m_Claw.openAllDrop());
    eventMap.put("ShootCube", m_Claw.openAllOut());
    eventMap.put("IntakeCube", m_Claw.openCubeCommand());
    eventMap.put("FloorIntake", m_Elevator.setToFloor());
    eventMap.put("ConeShoot", m_Claw.openAllShoot());
    eventMap.put(
      "ConeHigh",
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCone,
        Constants.armTopCone
      )
    );
    eventMap.put(
      "CubeHigh",
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCube,
        Constants.armTopCube
      )
    );

    FollowPathWithEvents PathCommand = new FollowPathWithEvents(
      s_Swerve.followTrajectoryCommand(traj, true),
      traj.getMarkers(),
      eventMap
    );

    addCommands(
      m_Claw.closeAllHold(),
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCone,
        Constants.armTopCone
      ),
      m_Claw.openAllDrop(),
      new WaitCommand(.5),
      m_Elevator.setStow(),
      new ParallelCommandGroup( //why did we make a parallel command group with only one thing in it
        new SequentialCommandGroup(new WaitCommand(.25), PathCommand)
      )
    );
  }
}
