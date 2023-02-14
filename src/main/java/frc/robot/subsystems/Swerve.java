package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {

  public SwerveDriveOdometry swerveOdometry;
  public static SwerveModule[] mSwerveMods;
  public Pigeon2 gyro;
  private GenericEntry gyroAngle;
  Limelight m_Limelight;

  public Swerve(Limelight limelight) {
    m_Limelight = limelight;
    gyro = new Pigeon2(Constants.Swerve.pigeonID, "torch");

    gyro.configFactoryDefault();
    zeroGyro();

    ShuffleboardTab tab = Shuffleboard.getTab("GyroFinal");
    gyroAngle =
      tab
        .add(getName(), 0)
        .withPosition(0, 1)
        .withWidget(BuiltInWidgets.kGyro)
        .getEntry();

    mSwerveMods =
      new SwerveModule[] {
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants),
      };

    /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry =
      new SwerveDriveOdometry(
        Constants.Swerve.swerveKinematics,
        getYaw(),
        getModulePositions()
      );
  }

  public void drive(
    Translation2d translation,
    double rotation,
    boolean fieldRelative,
    boolean isOpenLoop
  ) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(),
          translation.getY(),
          rotation,
          getYaw()
        )
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates,
      Constants.Swerve.maxSpeed
    );

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void alignToGoal() {
    drive(new Translation2d(0, m_Limelight.getSteeringValue()), 0, true, false);
  }

  // WIP

  // public void alignToCenterClockwise() {
  //   drive(new Translation2d(0, 0), -1, true, false);
  // }

  // public void alignToCenterCounterclockwise() {
  //   drive(new Translation2d(0, 0), 1, true, false);
  // }

  // public CommandBase alignToBase() {
  //   if (Math.abs(getYaw().getDegrees() % 360) >= 180) {
  //     return run(() -> alignToCenterCounterclockwise())
  //       .until(() ->
  //         Math.abs(getYaw().getDegrees() % 360) <= 10 ||
  //         Math.abs(getYaw().getDegrees() % 360) >= 350
  //       );
  //   } else {
  //     return run(() -> alignToCenterClockwise())
  //       .until(() ->
  //         Math.abs(getYaw().getDegrees() % 360) <= 10 ||
  //         Math.abs(getYaw().getDegrees() % 360) >= 350
  //       );
  //   }
  // }

  public CommandBase moveToGoalAprilTags() {
    return runOnce(() -> m_Limelight.setToAprilTags())
      .andThen(() -> Timer.delay(.5))
      .andThen(
        run(() -> alignToGoal())
          .until(() -> m_Limelight.getSteeringValue() == 0)
      );
  }

  public CommandBase moveToGoalRetroreflective() {
    return runOnce(() -> m_Limelight.setToRetroreflectiveTape())
      .andThen(() -> Timer.delay(.5))
      .andThen(
        run(() -> alignToGoal())
          .until(() -> m_Limelight.getSteeringValue() == 0)
      );
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      Constants.Swerve.maxSpeed
    );

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
      ? Rotation2d.fromDegrees(360 - gyro.getYaw())
      : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public static void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());
    SmartDashboard.putNumber("gyro", gyro.getYaw());
    gyroAngle.setDouble(gyro.getYaw());
    //SmartDashboard.putNumber("gyro2", getyaw);
    SmartDashboard.putNumber("yawdegrees", getYaw().getDegrees());
    SmartDashboard.putNumber(
      "yawdegreesmodulo",
      Math.abs(getYaw().getDegrees()) % 360
    );

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Cancoder",
        mod.getCanCoder().getDegrees()
      );
      SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Integrated",
        mod.getPosition().angle.getDegrees()
      );
      SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Velocity",
        mod.getState().speedMetersPerSecond
      );
      SmartDashboard.putNumber("X pose", this.getPose().getX());
      SmartDashboard.putNumber("Y pose", this.getPose().getY());
    }
  }

  public CommandBase createCommandForTrajectory(
    PathPlannerTrajectory trajectory
  ) {
    return new PPSwerveControllerCommand(
      trajectory,
      this::getPose, // Functional interface to feed supplier
      Constants.Swerve.swerveKinematics,
      // Position controllers
      new PIDController(Constants.AutoConstants.kPXController, 0, 0),
      new PIDController(Constants.AutoConstants.kPYController, 0, 0),
      new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
      this::setModuleStates,
      this
    );
  }

  public CommandBase drive1m() {
    PathPlannerTrajectory drive1m2 = PathPlanner.loadPath("Drive1m", 4, 3);
    return runOnce(() -> {
        resetOdometry(drive1m2.getInitialPose());
      })
      .andThen(createCommandForTrajectory(drive1m2))
      .andThen(() -> drive(new Translation2d(0, 0), 0, true, false));
  }
}
