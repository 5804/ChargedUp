package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SnapToAngle extends CommandBase {

  /** Creates a new SnapToAngle. */
  Swerve s_swerve;
  double m_angle;
  PIDController m_thetaController;

  public SnapToAngle(Swerve swerve, double angle) {
    m_angle = angle;
    s_swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_thetaController = new PIDController(0.03, 0, 0.00125);

    m_thetaController.enableContinuousInput(-180, 180); // :(
    m_thetaController.setSetpoint(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationVal = -m_thetaController.calculate(
      -(MathUtil.inputModulus(s_swerve.getYaw().getDegrees(), -180, 180)), // remainder math ?
      m_thetaController.getSetpoint()
    );
    s_swerve.drive(
      new Translation2d(0.0, 0.0),
      rotationVal * (Math.PI * 2),
      true,
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(s_swerve.getYaw().getDegrees()) < 1) {
      return true;
    }
    return false;
  }
}
