package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import javax.swing.text.Position;

public class Elevator extends SubsystemBase {

  public WPI_TalonFX mainMotor;
  public WPI_TalonFX followerMotor;
  public double targetVelocity;
  public double calculatedPosition = 0;
  public double motorPosition;

  public Elevator() {
    mainMotor = new WPI_TalonFX(1); // change
    followerMotor = new WPI_TalonFX(2); // change
    mainMotor.configFactoryDefault();
    followerMotor.configFactoryDefault();
    // total clicks of the elevator chasis 491,717
    //mainMotor.configForwardSoftLimitThreshold(3.0);
    mainMotor.configSelectedFeedbackSensor(
      TalonFXFeedbackDevice.IntegratedSensor,
      0,
      10
    );

    followerMotor.follow(mainMotor);
    followerMotor.setInverted(TalonFXInvertType.Clockwise); // subject to change
    mainMotor.setInverted(TalonFXInvertType.CounterClockwise);

    mainMotor.configNeutralDeadband(0.01);

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
    mainMotor.config_kF(Constants.kSlotIdx, 1000, Constants.kTimeoutMs);

    mainMotor.config_kP(Constants.kSlotIdx, .1, Constants.kTimeoutMs);
    mainMotor.config_kI(Constants.kSlotIdx, 0.0, Constants.kTimeoutMs);
    mainMotor.config_kD(Constants.kSlotIdx, 0.0, Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    mainMotor.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    mainMotor.configMotionAcceleration(15000, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    mainMotor.setSelectedSensorPosition(
      0,
      Constants.kPIDLoopIdx,
      Constants.kTimeoutMs
    );
  }

  // public void runUp() {
  //   mainMotor.set(TalonFXControlMode.PercentOutput, .5);
  // }

  public void stop() {
    mainMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  //public CommandBase runUp(){
  //    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, .5))
  //            .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.0))
  //            .withName("runUp");
  //}

  public CommandBase runDown() {
    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, -.5))
      .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.0))
      .withName("runDown");
  }

  public CommandBase runUp() {
    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, .5))
      .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.0))
      .withName("runDown");
  }

  public void increasePosition() {
    calculatedPosition = calculatedPosition + 1;
    //mainMotor.set(TalonFXControlMode.Position, calculatedPosition);

  }

  public void decreasePosition() {
    calculatedPosition = calculatedPosition - 1;
    //mainMotor.set(TalonFXControlMode.Position, calculatedPosition);
  }

  public void setPosition() {
    //setSelectedSensorPosition(0);
    double targetPos = calculatedPosition * 2048 * 10.0;
    mainMotor.set(TalonFXControlMode.MotionMagic, 10000);
  }

  public void reset() {
    mainMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    motorPosition = mainMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber("Elevator", motorPosition);
  }
}
