package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class ConeGrabBalance1PATH extends SequentialCommandGroup {

  public ConeGrabBalance1PATH(
    Swerve s_Swerve,
    Elevator m_Elevator,
    Claw m_Claw
  ) {
    PathPlannerTrajectory traj = PathPlanner.loadPath("GPMobilityCharge", 2, 2);
    addCommands(
      m_Claw.closeAllHold(),
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCone,
        Constants.armTopCone
      ),
      // new WaitCommand(2),
      m_Claw.openAllDrop(),
      new WaitCommand(.5),
      // m_Claw.motorOff(), // dont think we need for this
      m_Elevator.setStow(),
      new ParallelCommandGroup(
        s_Swerve.followTrajectoryCommand(traj, true),
        new SequentialCommandGroup(
          new WaitCommand(2), //? maybe idk i just picked a random number
          m_Elevator.setToFloor(),
          m_Claw.openConeCommand(),
          new RunCommand(s_Swerve::autoBalance, s_Swerve) //er
        )
      )
    );
  }
}
