package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {

  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private DoubleSupplier thrustTrigger;
  private SendableChooser<Double> speedChoice;

  public TeleopSwerve(
    Swerve s_Swerve,
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier rotationSup,
    BooleanSupplier robotCentricSup,
    DoubleSupplier thrustTrigger
  ) {
    speedChoice = new SendableChooser<Double>();
    speedChoice.setDefaultOption(".5", .5);
    speedChoice.addOption(".2", .2);
    speedChoice.addOption(".5", .5);
    speedChoice.addOption(".6", .6);
    speedChoice.addOption(".7", .7);
    speedChoice.addOption(".8", .8);
    speedChoice.addOption("1", 1.0);
    SmartDashboard.putData("Speed", speedChoice);

    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.thrustTrigger = thrustTrigger;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal = MathUtil.applyDeadband(
      translationSup.getAsDouble(),
      Constants.stickDeadband
    );
    double strafeVal = MathUtil.applyDeadband(
      strafeSup.getAsDouble(),
      Constants.stickDeadband
    );
    double rotationVal = MathUtil.applyDeadband(
      rotationSup.getAsDouble(),
      Constants.stickDeadband
    );

    double thrust = 1 - (double) speedChoice.getSelected();
    double thrustVal =
      1 - thrust + thrust * Math.abs(thrustTrigger.getAsDouble());
    translationVal = translationVal * thrustVal;
    strafeVal = strafeVal * thrustVal;
    rotationVal = rotationVal * thrustVal;

    /* Drive */
    s_Swerve.drive(
      new Translation2d(translationVal, strafeVal)
        .times(Constants.Swerve.maxSpeed),
      rotationVal * Constants.Swerve.maxAngularVelocity,
      !robotCentricSup.getAsBoolean(),
      false
    );
  }
}
