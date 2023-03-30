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

  //LED constants

  public static final int CANdleID1 = 1;
  public static final int CANdleID2 = 2;

  public static final double LIMELIGHT_DEADBAND = 0.15;
  public static final double MIN_STEER_K = .05;

  // elevator constants
  public static final int elevatorUpperLimit = 97733;
  // this is used to set a threshold where the arm position needs to be considered
  public static final int elevatorLowerThreshold = 50000;
  public static final int elevatorTopCone = 97733;
  public static final int elevatorMidCone = 62000;
  public static final int elevatorTopCube = 79731;
  public static final int elevatorMidCube = 45811;
  public static final int elevatorStow = 0;
  public static final int elevatorFloor = 16499;
  public static final int elevatorShelf = 63166;

  // arm constants
  public static final int armUpperLimit = 1400; // DO NOT TOUCH
  // this is used to set a threshold of where the elevator postion needs to be considered
  public static final int armLowerThreshold = 800;
  // set height for arm pos 1
  public static final int armTopCone = 992;
  // set height for arm pos 2
  public static final int armMidCone = 1254;
  public static final int armTopCube = 1013;
  public static final int armMidCube = 1334;
  public static final int armStow = 1675;
  public static final int armFloor = 529;
  public static final int armShelf = 1120;

  //arm feed forward
  public static final int horizontalPos = 870;
  public static final int ticksPerDegrees = 4096 / 360;
  public static final double maxFF = .03;

  // arm encoder
  public static final double armEncoderOffset = 312.02;

  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from
   * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
   * configuration.
   */
  public static final int kSlotIdx0 = 0;
  public static final int kSlotIdx1 = 1;
  public final int test = 0;
  public static final int kPIDLoopIdx = 0;
  public static int kTimeoutMs = 30;

  // /**
  //  * Gains used in Motion Magic, to be adjusted accordingly
  //    * Gains(kp, ki, kd, kf, izone, peak output);
  //    */

  public static final double stickDeadband = 0.1;

  public static final class Swerve {

    public static int pigeonID = 1;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SDSMK4i( //TODO: This must be tuned to specific robot
      COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1
    );

    /* Drivetrain Constants */

    public static double trackWidth = Units.inchesToMeters(19.75); //TODO: This must be tuned to specific robot
    public static double wheelBase = Units.inchesToMeters(28.5); //TODO: This must be tuned to specific robot

    public static double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );

    /* Module Gear Ratios */
    public static double driveGearRatio = chosenModule.driveGearRatio;
    public static double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static boolean canCoderInvert = chosenModule.canCoderInvert;

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
    public static final double driveKS = (0.18882 / 12);
    public static final double driveKV = (2.6515 / 12);
    public static final double driveKA = (0.37384 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = Math.PI * 2; //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Constants (Offset!) */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {

      public static int driveMotorID = 12;
      public static int angleMotorID = 11;
      public static int canCoderID = 10;
      public static Rotation2d angleOffset = Rotation2d.fromDegrees(190);
      public static SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {

      public static int driveMotorID = 42;
      public static int angleMotorID = 41;
      public static int canCoderID = 40;
      public static Rotation2d angleOffset = Rotation2d.fromDegrees(316.14);
      public static SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {

      public static int driveMotorID = 22;
      public static int angleMotorID = 21;
      public static int canCoderID = 20;
      public static Rotation2d angleOffset = Rotation2d.fromDegrees(127.09);
      public static SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {

      public static int driveMotorID = 32;
      public static int angleMotorID = 31;
      public static int canCoderID = 30;
      public static Rotation2d angleOffset = Rotation2d.fromDegrees(44.91);
      public static SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }
  }

  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared =
      Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 10;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularSpeedRadiansPerSecondSquared
    );
  }
}
