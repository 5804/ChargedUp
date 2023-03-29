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

public class ConeGrab extends SequentialCommandGroup {

  public ConeGrab(Swerve s_Swerve, Elevator m_Elevator, Claw m_Claw) {
    PathPlannerTrajectory traj1 = PathPlanner.loadPath("Cone2GP", 4, 4);
    addCommands(
      m_Claw.closeAllHold(),
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCone,
        Constants.armTopCone
      ),
      m_Claw.openAllDrop(),
      new WaitCommand(.25),
      m_Claw.motorOff(),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          m_Elevator.setStow(),
          new WaitCommand(.25),
          m_Elevator.sequentialSetPositions(
            Constants.elevatorFloor,
            Constants.armFloor
          ),
          m_Claw.openAllIn()
        ),
        new SequentialCommandGroup(
          new WaitCommand(.5),
          s_Swerve.followTrajectoryCommand(traj1, true)
        )
      ),
      m_Claw.open1Hold(),
      m_Elevator.setStow()
    );
  }
}
