package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  public WPI_TalonFX mainMotor;
  public WPI_TalonFX followerMotor;
  public WPI_TalonFX armMotor;
  public WPI_TalonFX armFollower;
  public WPI_CANCoder armEncoder;
  public double calculatedPOutput = 0;
  public double motorPosition;
  public int smoothing = 0;
  public boolean isStowing = false;
  int upTargetPos = 10000;
  int downTargetPosition = 100;
  int count = 0;

  public Elevator() {
    SupplyCurrentLimitConfiguration elevatorSupplyLimit = new SupplyCurrentLimitConfiguration(
      true,
      25,
      40,
      .1
    );

    if (!Constants.mantis) {
      mainMotor = new WPI_TalonFX(1); // add "torch as second parameter when on canivore"
      followerMotor = new WPI_TalonFX(2); // add "torch as second parameter when on canivore"
      armMotor = new WPI_TalonFX(3);
      armEncoder = new WPI_CANCoder(1);
      armFollower = new WPI_TalonFX(5);

      armMotor.setNeutralMode(NeutralMode.Brake);
      armFollower.setNeutralMode(NeutralMode.Brake);
      armMotor.configNeutralDeadband(.001);
      armMotor.configSupplyCurrentLimit(elevatorSupplyLimit);
      armMotor.setInverted(TalonFXInvertType.CounterClockwise);

      armMotor.configForwardSoftLimitEnable(true);
      armMotor.configForwardSoftLimitThreshold(Constants.armUpperLimit);
      armMotor.configReverseSoftLimitEnable(true);
      armMotor.configReverseSoftLimitThreshold(0);
      armMotor.configSupplyCurrentLimit(elevatorSupplyLimit);
      armMotor.configRemoteFeedbackFilter(armEncoder, 0);
      armFollower.follow(armMotor);
      armFollower.setInverted(TalonFXInvertType.Clockwise); //maybe cw vs ccw
      armFollower.configSupplyCurrentLimit(elevatorSupplyLimit);

      armMotor.configForwardSoftLimitEnable(true);
      armMotor.configForwardSoftLimitThreshold(Constants.armUpperLimit);
      armMotor.configReverseSoftLimitEnable(true);
      armMotor.configReverseSoftLimitThreshold(0);

      armEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition
      );
      armEncoder.configMagnetOffset(Constants.armEncoderOffset);
      armEncoder.configSensorDirection(true);

      mainMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        0,
        30
      );

      armMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.RemoteSensor0,
        0,
        30
      );

      followerMotor.follow(mainMotor); //set the follower motor to mimic the mainmotor

      followerMotor.setInverted(TalonFXInvertType.CounterClockwise); // motors need to be inverted from each other as they face opposite ways.  We need to determine if positive is up or down on the elevator.
      mainMotor.setInverted(TalonFXInvertType.Clockwise);
      mainMotor.setNeutralMode(NeutralMode.Brake);
      followerMotor.setNeutralMode(NeutralMode.Brake);
      mainMotor.configNeutralDeadband(0.001);

      mainMotor.configSupplyCurrentLimit(elevatorSupplyLimit);
      followerMotor.configSupplyCurrentLimit(elevatorSupplyLimit);

      /* Set relevant frame periods to be at least as fast as periodic rate */
      mainMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_13_Base_PIDF0,
        10,
        Constants.kTimeoutMs
      );

      mainMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic,
        10,
        Constants.kTimeoutMs
      );

      /* Set the peak and nominal outputs */
      mainMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
      mainMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
      mainMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
      mainMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

      /* Set Motion Magic gains in slot0 - see documentation */

      mainMotor.config_kF(Constants.kSlotIdx0, 0.060176, Constants.kTimeoutMs);
      mainMotor.config_kP(Constants.kSlotIdx0, 0.2, Constants.kTimeoutMs);
      mainMotor.config_kI(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
      mainMotor.config_kD(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
      mainMotor.config_IntegralZone(Constants.kSlotIdx0, 200);
      mainMotor.configAllowableClosedloopError(Constants.kSlotIdx0, 400);

      /* Set Motion Magic gains in slot1 - see documentation */
      mainMotor.selectProfileSlot(Constants.kSlotIdx1, Constants.kPIDLoopIdx);
      mainMotor.config_kF(Constants.kSlotIdx1, 0.060176, Constants.kTimeoutMs);
      mainMotor.config_kP(Constants.kSlotIdx1, 0.2, Constants.kTimeoutMs);
      mainMotor.config_kI(Constants.kSlotIdx1, 0, Constants.kTimeoutMs);
      mainMotor.config_kD(Constants.kSlotIdx1, 0, Constants.kTimeoutMs);
      mainMotor.config_IntegralZone(Constants.kSlotIdx1, 200);
      mainMotor.configAllowableClosedloopError(Constants.kSlotIdx1, 100);

      /* Set acceleration and vcruise velocity - see documentation */
      mainMotor.configMotionCruiseVelocity(17000, Constants.kTimeoutMs);
      mainMotor.configMotionAcceleration(17000, Constants.kTimeoutMs);

      armMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_13_Base_PIDF0,
        10,
        Constants.kTimeoutMs
      );
      armMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic,
        10,
        Constants.kTimeoutMs
      );

      /* Set the peak and nominal outputs */
      armMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
      armMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
      armMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
      armMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

      /* Set Motion Magic gains in slot0 - see documentation */
      armMotor.selectProfileSlot(Constants.kSlotIdx0, Constants.kPIDLoopIdx);
      armMotor.config_kF(Constants.kSlotIdx0, 3.5, Constants.kTimeoutMs);
      armMotor.config_kP(Constants.kSlotIdx0, 4.1, Constants.kTimeoutMs);
      armMotor.config_kI(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
      armMotor.config_kD(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
      armMotor.configAllowableClosedloopError(Constants.kSlotIdx0, 20);

      /* Set acceleration and vcruise velocity - see documentation */
      armMotor.configMotionCruiseVelocity(280, Constants.kTimeoutMs);
      armMotor.configMotionAcceleration(280, Constants.kTimeoutMs);

      mainMotor.configForwardSoftLimitEnable(true);
      mainMotor.configForwardSoftLimitThreshold(Constants.elevatorUpperLimit);

      armMotor.configForwardSoftLimitEnable(true);
      armMotor.configForwardSoftLimitThreshold(Constants.armUpperLimit);

      //DISABLE MOTION MAGIC
      armMotor.set(ControlMode.PercentOutput, feedForward());
      mainMotor.set(ControlMode.PercentOutput, 0.03);
    }
  }

  public double feedForward() {
    double armPos = armMotor.getSelectedSensorPosition();
    double degrees =
      (armPos - Constants.horizontalPos) / Constants.ticksPerDegrees;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    return Constants.maxFF * cosineScalar;
  }

  //nice run up and down commands
  public CommandBase resetElevatorEncoder() {
    return run(() -> mainMotor.setSelectedSensorPosition(0));
  }

  public CommandBase runDown() {
    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, -.2))
      .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.03))
      .withName("runDown");
  }

  public CommandBase runUp() {
    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, 0.27))
      .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.03))
      .withName("runUp");
  }

  public CommandBase armDown() {
    return run(() -> armMotor.set(TalonFXControlMode.PercentOutput, -0.1))
      .finallyDo(interrupted ->
        armMotor.set(ControlMode.PercentOutput, feedForward())
      )
      .withName("armDown");
  }

  public CommandBase armUp() {
    return run(() -> armMotor.set(TalonFXControlMode.PercentOutput, 0.1))
      .finallyDo(interrupted ->
        armMotor.set(ControlMode.PercentOutput, feedForward())
      )
      .withName("armUp");
  }

  public void armAndElevatorStopPercentMode() {
    // if (!DriverStation.isAutonomous()) {
    armMotor.set(TalonFXControlMode.PercentOutput, feedForward());
    mainMotor.set(TalonFXControlMode.PercentOutput, 0.03);
    // }
  }

  public boolean isStowing() {
    return isStowing;
  }

  public CommandBase StopCommand() {
    // if (!DriverStation.isAutonomous()) {
    return runOnce(() ->
        armMotor.set(TalonFXControlMode.PercentOutput, feedForward())
      )
      .andThen(() -> mainMotor.set(TalonFXControlMode.PercentOutput, 0.03));
    // }
  }

  public CommandBase setToFloor() {
    mainMotor.selectProfileSlot(Constants.kSlotIdx0, Constants.kPIDLoopIdx);
    return runOnce(() ->
        armMotor.set(
          TalonFXControlMode.MotionMagic,
          Constants.armUpperLimit,
          DemandType.ArbitraryFeedForward,
          feedForward()
        )
      )
      .andThen(
        runOnce(() ->
          mainMotor.set(
            TalonFXControlMode.MotionMagic,
            Constants.elevatorFloor,
            DemandType.ArbitraryFeedForward,
            0.03
          )
        )
      )
      .andThen(
        runOnce(() ->
          armMotor.set(TalonFXControlMode.MotionMagic, Constants.armFloor)
        )
      )
      .andThen(
        Commands
          .waitUntil(() ->
            armMotor.getActiveTrajectoryPosition() < Constants.armFloor + 20 &&
            armMotor.getActiveTrajectoryPosition() > Constants.armFloor - 20 &&
            mainMotor.getActiveTrajectoryPosition() <
            Constants.elevatorFloor +
            30000 &&
            mainMotor.getActiveTrajectoryPosition() >
            Constants.elevatorFloor -
            30000
          )
          .withTimeout(2)
      )
      .andThen(runOnce(() -> this.armAndElevatorStopPercentMode()));
  }

  public CommandBase sequentialSetPositions(
    final int elevatorPosition,
    int armPosition
  ) {
    System.out.println(Swerve.mSwerveMods[0].getState().speedMetersPerSecond);
    // if (Math.abs(Swerve.mSwerveMods[0].getState().speedMetersPerSecond) > 0) {
    //   return new InstantCommand(() -> LED.LEDColor(256, 0, 0));
    // } else {
    mainMotor.selectProfileSlot(Constants.kSlotIdx0, Constants.kPIDLoopIdx);
    return Commands.either(
      Commands.none(),
      runOnce(() ->
          armMotor.set(
            TalonFXControlMode.MotionMagic,
            Constants.armUpperLimit,
            DemandType.ArbitraryFeedForward,
            feedForward()
          )
        )
        .andThen(() ->
          System.out.println(
            Swerve.mSwerveMods[0].getState().speedMetersPerSecond
          )
        )
        .andThen(
          Commands
            .waitUntil(() ->
              armMotor.getActiveTrajectoryPosition() >
              Constants.armUpperLimit -
              100
            )
            .withTimeout(1)
        )
        .andThen(
          runOnce(() ->
            mainMotor.set(
              TalonFXControlMode.MotionMagic,
              elevatorPosition,
              DemandType.ArbitraryFeedForward,
              0.03
            )
          )
        )
        .andThen(
          Commands
            .waitUntil(() ->
              mainMotor.getActiveTrajectoryPosition() <
              elevatorPosition +
              30000 &&
              mainMotor.getActiveTrajectoryPosition() > elevatorPosition - 30000
            )
            .withTimeout(1.5)
        )
        .andThen(
          runOnce(() ->
            armMotor.set(TalonFXControlMode.MotionMagic, armPosition)
          )
        )
        .andThen(
          Commands
            .waitUntil(() ->
              armMotor.getActiveTrajectoryPosition() < armPosition + 20 &&
              armMotor.getActiveTrajectoryPosition() > armPosition - 20
            )
            .withTimeout(1)
        )
        .andThen(
          Commands.waitUntil(() ->
            mainMotor.getActiveTrajectoryPosition() < elevatorPosition + 500 &&
            mainMotor.getActiveTrajectoryPosition() > elevatorPosition - 500
          )
        )
        .andThen(runOnce(() -> this.armAndElevatorStopPercentMode())),
      () -> {
        return (
          false
          // Math.abs(Swerve.mSwerveMods[0].getState().speedMetersPerSecond) > 0
        );
      }
    );
  }

  public void stowMethod() {
    mainMotor.selectProfileSlot(Constants.kSlotIdx1, Constants.kPIDLoopIdx);
    armMotor.set(TalonFXControlMode.MotionMagic, Constants.armUpperLimit);
    armMotor.configForwardSoftLimitThreshold(Constants.armStow);
    mainMotor.set(
      TalonFXControlMode.MotionMagic,
      Constants.elevatorStow,
      DemandType.ArbitraryFeedForward,
      0.03
    );
  }

  // Test this
  public CommandBase setStow() {
    isStowing = true;
    mainMotor.selectProfileSlot(Constants.kSlotIdx1, Constants.kPIDLoopIdx);
    return runOnce(() ->
        armMotor.set(TalonFXControlMode.MotionMagic, Constants.armUpperLimit)
      )
      .andThen(new WaitCommand(.25))
      // .andThen(
      //   Commands
      // .waitUntil(() ->
      //   armMotor.getActiveTrajectoryPosition() >
      //   Constants.armUpperLimit -
      //   100 &&
      //   armMotor.getActiveTrajectoryPosition() <
      //   Constants.armUpperLimit +
      //   100
      // )
      // .withTimeout(1)
      //) // set to current upperlimit
      .andThen(
        runOnce(() ->
          armMotor.configForwardSoftLimitThreshold(Constants.armStow)
        )
      ) // set soft limit to be stow position
      .andThen(
        runOnce(() ->
          mainMotor.set(
            TalonFXControlMode.MotionMagic,
            Constants.elevatorStow,
            DemandType.ArbitraryFeedForward,
            0.03
          )
        )
      ) // set elevator to 0
      .andThen(
        Commands
          .waitUntil(() ->
            mainMotor.getActiveTrajectoryPosition() <
            Constants.elevatorStow +
            5000 &&
            mainMotor.getActiveTrajectoryPosition() >
            Constants.elevatorStow -
            5000
          )
          .withTimeout(2.25)
      )
      .andThen(
        runOnce(() ->
          armMotor.set(TalonFXControlMode.MotionMagic, Constants.armStow)
        )
      ) //^wait until finished, set arm to stow
      .andThen(
        Commands
          .waitUntil(() ->
            armMotor.getActiveTrajectoryPosition() > Constants.armStow - 20 &&
            armMotor.getActiveTrajectoryPosition() < Constants.armStow + 20
          )
          .withTimeout(3)
      ) //wait until finished
      .andThen(
        runOnce(() ->
          armMotor.configForwardSoftLimitThreshold(Constants.armUpperLimit)
        )
      )
      // ) //set soft limit back to what it was
      .andThen(runOnce(() -> this.armAndElevatorStopPercentMode()))
      .andThen(() -> isStowing = false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
      "elevatorEncoderVal",
      mainMotor.getSelectedSensorPosition()
    );
    SmartDashboard.putNumber(
      "armEncoderVal",
      armMotor.getSelectedSensorPosition()
    );
    SmartDashboard.putNumber(
      "Active Trajectory Position",
      armMotor.getActiveTrajectoryPosition()
    );
  }
}
