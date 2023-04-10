package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class ConeGrabBalance extends SequentialCommandGroup {

  public ConeGrabBalance(
    Swerve s_Swerve,
    Elevator m_Elevator,
    Claw m_Claw,
    Limelight m_Limelight
  ) {
    PathPlannerTrajectory traj1 = PathPlanner.loadPath("ConeChargeGrab", 4, 4);
    PathPlannerTrajectory traj3 = PathPlanner.loadPath("GP2Charge", 4, 4);
    addCommands(
      m_Claw.closeAllHold(),
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCone,
        Constants.armTopCone
      ),
      m_Claw.openAllDrop(),
      new WaitCommand(.1),
      m_Claw.motorOff(),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          m_Elevator.setStow(),
          new WaitCommand(.25),
          m_Elevator.setToFloor(),
          m_Claw.openAllIn()
        ),
        new SequentialCommandGroup(
          new WaitCommand(.25),
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
      new RunCommand(s_Swerve::autoBalance, s_Swerve) //maybe wont run it? i dont know actually
    );
  }
}
/*m_Claw.openAllDrop(),
      new WaitCommand(.25),
      m_Claw.motorOff(),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          m_Elevator.setStow(),
          new WaitCommand(.25),
          m_Elevator.setToFloor(),
          m_Claw.openAllIn()
        ),
        new SequentialCommandGroup(
          new WaitCommand(.5),
          s_Swerve.followTrajectoryCommand(traj1, true)
        ) */
