package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static final double LIMELIGHT_DEADBAND = 1;
  public static final double MIN_STEER_K = .4;

  // weird motion magic
  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from
   * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
   * configuration.
   */
  public static final int kSlotIdx = 0;
  public final int test = 0;

  /**
   * Talon FX supports multiple (cascaded) PID loops. For
   * now we just want the primary one.
   */
  public static final int kPIDLoopIdx = 0;

  /**
   * set to zero to skip waiting for confirmation, set to nonzero to wait and
   * report to DS if action fails.
   */
  public static final int kTimeoutMs = 10;

  // /**
  //  * Gains used in Motion Magic, to be adjusted accordingly
  //    * Gains(kp, ki, kd, kf, izone, peak output);
  //    */

  public static final double stickDeadband = 0.1;

  public static final class Swerve {

    public static final int pigeonID = 0;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SDSMK3( //TODO: This must be tuned to specific robot
      COTSFalconSwerveConstants.driveGearRatios.SDSMK3_Fast // or standard idk?
    );

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.25); //TODO: This must be tuned to specific robot
    public static final double wheelBase = Units.inchesToMeters(21.25); //TODO: This must be tuned to specific robot
    public static final double wheelCircumference =
      chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert =
      chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert =
      chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { //TODO: This must be tuned to specific robot

      public static final int driveMotorID = 22;
      public static final int angleMotorID = 21;
      public static final int canCoderID = 20;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(
        167.43
      );
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot

      public static final int driveMotorID = 32;
      public static final int angleMotorID = 31;
      public static final int canCoderID = 30;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(
        93.77
      );
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot

      public static final int driveMotorID = 12;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(
        173.67
      );
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        +canCoderID,
        angleOffset
      );
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot

      public static final int driveMotorID = 42;
      public static final int angleMotorID = 41;
      public static final int canCoderID = 40;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(38.4);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }
  }

  public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared =
      Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularSpeedRadiansPerSecondSquared
    );
  }
}
