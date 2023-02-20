package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  public WPI_TalonFX mainMotor;
  public WPI_TalonFX followerMotor;
  public WPI_TalonFX armMotor;
  public double calculatedPOutput = 0;
  public double motorPosition;
  public int smoothing = 0;
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

      armMotor.setNeutralMode(NeutralMode.Brake);
      armMotor.configNeutralDeadband(.001);
      armMotor.configSupplyCurrentLimit(elevatorSupplyLimit);
      armMotor.setInverted(false);

      armMotor.configForwardSoftLimitEnable(true);
      armMotor.configForwardSoftLimitThreshold(Constants.armTrueSoftLimit);
      armMotor.configReverseSoftLimitEnable(true);
      armMotor.configReverseSoftLimitThreshold(0);
      armMotor.configSupplyCurrentLimit(elevatorSupplyLimit);

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
      mainMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
      mainMotor.config_kF(Constants.kSlotIdx, 0.0479, Constants.kTimeoutMs);
      mainMotor.config_kP(Constants.kSlotIdx, 0.07, Constants.kTimeoutMs);
      mainMotor.config_kI(Constants.kSlotIdx, 0.001, Constants.kTimeoutMs);
      mainMotor.config_kD(Constants.kSlotIdx, 0.6, Constants.kTimeoutMs);
      mainMotor.config_IntegralZone(0, 50);
      mainMotor.configAllowableClosedloopError(0, 100);

      /* Set acceleration and vcruise velocity - see documentation */
      mainMotor.configMotionCruiseVelocity(16000, Constants.kTimeoutMs);
      mainMotor.configMotionAcceleration(16000, Constants.kTimeoutMs);

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
      armMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
      armMotor.config_kF(Constants.kSlotIdx, 3.602, Constants.kTimeoutMs);
      armMotor.config_kP(Constants.kSlotIdx, 5, Constants.kTimeoutMs);
      armMotor.config_kI(Constants.kSlotIdx, 0.001, Constants.kTimeoutMs);
      armMotor.config_kD(Constants.kSlotIdx, 50, Constants.kTimeoutMs);
      armMotor.configAllowableClosedloopError(0, 20);

      /* Set acceleration and vcruise velocity - see documentation */
      armMotor.configMotionCruiseVelocity(213, Constants.kTimeoutMs);
      armMotor.configMotionAcceleration(213, Constants.kTimeoutMs);

      // /* Zero the sensor once on robot boot up not forever */
      // mainMotor.setSelectedSensorPosition(
      //   0,
      //   Constants.kPIDLoopIdx,
      //   Constants.kTimeoutMs
      // );

      mainMotor.configForwardSoftLimitEnable(true);
      mainMotor.configForwardSoftLimitThreshold(Constants.elevatorUpperLimit);

      armMotor.configForwardSoftLimitEnable(true);
      armMotor.configForwardSoftLimitThreshold(Constants.armUpperLimit);

      //DISABLE MOTION MAGIC
      armMotor.set(ControlMode.PercentOutput, 0.0);
      mainMotor.set(ControlMode.PercentOutput, 0.0);
    }
  }

  //nice run up and down commands
  public CommandBase resetElevatorEncoder() {
    return run(() -> mainMotor.setSelectedSensorPosition(0));
  }

  public CommandBase runDown() {
    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, -0.2))
      .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.0))
      .withName("runDown");
  }

  public CommandBase runUp() {
    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, 0.2))
      .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.0))
      .withName("runUp");
  }

  public CommandBase armDown() {
    return run(() -> armMotor.set(TalonFXControlMode.PercentOutput, -.25))
      .finallyDo(interrupted -> armMotor.set(ControlMode.PercentOutput, 0.0))
      .withName("armDown");
  }

  public CommandBase armUp() {
    return run(() -> armMotor.set(TalonFXControlMode.PercentOutput, .25))
      .finallyDo(interrupted -> armMotor.set(ControlMode.PercentOutput, 0.0))
      .withName("armUp");
  }

  public void armAndElevator() {
    armMotor.set(TalonFXControlMode.PercentOutput, 0);
    mainMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public CommandBase armMM1() {
    return run(() -> armMotor.set(TalonFXControlMode.MotionMagic, 400));
  }

  public CommandBase armMM2() {
    return run(() -> armMotor.set(TalonFXControlMode.MotionMagic, 1200));
  }

  //methods to find percent outputs needed for feedforward etc etc
  public void increasePercentOutput() {
    calculatedPOutput = calculatedPOutput + .05;
    goTo(calculatedPOutput);
  }

  public void decreasePercentOutput() {
    calculatedPOutput = calculatedPOutput - .05;
    goTo(calculatedPOutput);
  }

  public void goTo(double calculatedPercentOutput) {
    mainMotor.set(ControlMode.PercentOutput, calculatedPercentOutput);
  }

  // public CommandBase setPosition(int position) {
  //   double elevatorPosition = mainMotor.getSelectedSensorPosition();
  //   double armPosition = armMotor.getSelectedSensorPosition();
  //   //if isElevator, check if arm is past limit
  //   //  move arm then elevator
  //   //else
  //   //  move elevator to position

  //   //if moving arm and elevator can't support
  //   //  move elevator, then arm
  //   //else
  //   //  move arm

  //   return run(() -> mainMotor.set(TalonFXControlMode.MotionMagic, position));
  // }

  // public CommandBase testposition() {
  //   return runOnce(() ->
  //   armMotor.set(
  //     TalonFXControlMode.MotionMagic,
  //     Constants.armLowerThreshold)
  //   ).alongWith(WaitUntilCommand())

  //Move arm to high threshold
  //check if arm is to high thresheold
  //move elevator to position
  //move arm to position

  // used to set the arm and elevator to positions being set for the button board
  public CommandBase setPosition(final int elevatorPosition, int armPosition) {
    // check to see if the elevator position being set is lower then the threshold
    if (elevatorPosition < Constants.elevatorLowerThreshold) {
      // make sure that if the elevator position is lower, theat the arm position will not break the robot
      if (armPosition < Constants.armLowerThreshold) {
        // arm was set to a breaking position, so instead set it to the lowest possible setting then move elevator
        //except it wasnt :( this doesn't work because some combonations where they are both lower than the threshold are necessary
        return runOnce(() ->
            armMotor.set(
              TalonFXControlMode.MotionMagic,
              Constants.armLowerThreshold
            )
          )
          .andThen(
            runOnce(() ->
              mainMotor.set(TalonFXControlMode.MotionMagic, elevatorPosition)
            )
          )
          .andThen(
            runOnce(() ->
              armMotor.set(TalonFXControlMode.MotionMagic, armPosition) //so we put this here and we just have to be really careful not to send bad positions
            )
          );
      }
      // arm was set in a good position move the arm then the elevator
      return runOnce(() ->
          armMotor.set(TalonFXControlMode.MotionMagic, armPosition)
        )
        .andThen(
          runOnce(() ->
            mainMotor.set(TalonFXControlMode.MotionMagic, elevatorPosition)
          )
        );
    }
    // check to see if the arm is being set lower then the threshold
    else if (armPosition < Constants.armLowerThreshold) {
      // make sure if the arm could break the robot that the elevator position sent is in the acceptable range
      if (elevatorPosition < Constants.elevatorLowerThreshold) {
        // elevator range would break robot, so setit to the lowest position to not break the robot
        return runOnce(() ->
            mainMotor.set(
              TalonFXControlMode.MotionMagic,
              Constants.elevatorUpperLimit
            )
          )
          .andThen(
            runOnce(() ->
              armMotor.set(TalonFXControlMode.MotionMagic, armPosition)
            )
          );
      }
    }
    // no thresholds were broken so move the elevator and then the arm
    return runOnce(() ->
        mainMotor.set(TalonFXControlMode.MotionMagic, elevatorPosition)
      )
      .andThen(
        runOnce(() -> armMotor.set(TalonFXControlMode.MotionMagic, armPosition))
      );
  }

  public CommandBase parallelTest(final int elevatorPosition, int armPosition) {
    return Commands.parallel(
      runOnce(() ->
        armMotor.set(TalonFXControlMode.MotionMagic, Constants.armUpperLimit)
      ),
      Commands
        .waitUntil(null)
        .andThen(
          runOnce(() ->
            mainMotor.set(TalonFXControlMode.MotionMagic, elevatorPosition)
          )
        ),
      Commands
        .waitUntil(null)
        .andThen(
          runOnce(() ->
            armMotor.set(TalonFXControlMode.MotionMagic, armPosition)
          )
        )
    );
  }

  public CommandBase sequentialSetPositions(
    final int elevatorPosition,
    int armPosition
  ) {
    return runOnce(() ->
        armMotor.set(TalonFXControlMode.MotionMagic, Constants.armUpperLimit)
      )
      .andThen(
        Commands.waitUntil(() ->
          armMotor.getActiveTrajectoryPosition() > Constants.armUpperLimit - 100
        )
      )
      .andThen(
        runOnce(() ->
          mainMotor.set(TalonFXControlMode.MotionMagic, elevatorPosition)
        )
      )
      .andThen(
        Commands.waitUntil(() ->
          mainMotor.getActiveTrajectoryPosition() < elevatorPosition + 5000 &&
          mainMotor.getActiveTrajectoryPosition() > elevatorPosition - 5000
        )
      )
      .andThen(
        runOnce(() -> armMotor.set(TalonFXControlMode.MotionMagic, armPosition))
      )
      .andThen(
        Commands
          .waitUntil(() ->
            mainMotor.getActiveTrajectoryPosition() < armPosition + 20 &&
            armMotor.getActiveTrajectoryPosition() > armPosition - 20
          )
          .andThen(
            runOnce(() -> armMotor.set(TalonFXControlMode.PercentOutput, 0))
          )
          .andThen(
            runOnce(() -> mainMotor.set(TalonFXControlMode.PercentOutput, 0))
          )
      );
  }

  // Test this
  public CommandBase setStow() {
    return runOnce(() ->
        armMotor.set(TalonFXControlMode.MotionMagic, Constants.armUpperLimit)
      )
      .andThen(
        Commands.waitUntil(() ->
          armMotor.getActiveTrajectoryPosition() > Constants.armUpperLimit - 100
        )
      ) // set to current upperlimit
      .andThen(
        runOnce(() ->
          armMotor.configForwardSoftLimitThreshold(Constants.armStow)
        )
      ) // set soft limit to be stow position
      .andThen(
        runOnce(() ->
          mainMotor.set(TalonFXControlMode.MotionMagic, Constants.elevatorStow)
        )
      ) // set elevator to 0
      .andThen(
        Commands.waitUntil(() ->
          mainMotor.getActiveTrajectoryPosition() <
          Constants.elevatorStow +
          5000 &&
          mainMotor.getActiveTrajectoryPosition() >
          Constants.elevatorStow -
          5000
        )
      )
      .andThen(
        runOnce(() ->
          armMotor.set(TalonFXControlMode.MotionMagic, Constants.armStow)
        )
      ) //^wait until finished, set arm to stow
      .andThen(
        Commands.waitUntil(() ->
          armMotor.getActiveTrajectoryPosition() > Constants.armStow - 30
        )
      ) //wait until finished
      .andThen(
        runOnce(() ->
          armMotor.configForwardSoftLimitThreshold(Constants.armTrueSoftLimit)
        )
      ) //set soft limit back to what it was
      .andThen(runOnce(() -> armMotor.set(TalonFXControlMode.PercentOutput, 0)))
      .andThen(
        runOnce(() -> mainMotor.set(TalonFXControlMode.PercentOutput, 0))
      );
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
