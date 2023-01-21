package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
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
import javax.swing.text.Position;

public class Elevator extends SubsystemBase {

  public TalonFX mainMotor;
  public TalonFX followerMotor;
  public double targetVelocity;

  public Elevator() {
    mainMotor = new TalonFX(1, "torch"); // change
    followerMotor = new TalonFX(2, "torch"); // change

    mainMotor.configFactoryDefault();
    followerMotor.configFactoryDefault();
    // 491,717
    //mainMotor.configForwardSoftLimitThreshold(3.0);
    mainMotor.setNeutralMode(NeutralMode.Brake);
    followerMotor.setNeutralMode(NeutralMode.Brake);
    followerMotor.follow(mainMotor);
    followerMotor.setInverted(TalonFXInvertType.Clockwise); // subject to change
    mainMotor.setInverted(TalonFXInvertType.CounterClockwise);

    mainMotor.configNeutralDeadband(0.01);
  }

  // public void runUp(){
  //     mainMotor.set(TalonFXControlMode.PercentOutput, .5);
  // }

  //public CommandBase runUp(){
  //    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, .5))
  //            .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.0))
  //            .withName("runUp");
  //}

  //public CommandBase runDown(){
  //    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, -.5))
  //            .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.0))
  //            .withName("runDown");
  //}

  public void runUp() {
    mainMotor.set(TalonFXControlMode.PercentOutput, -0.3);
  }

  public void runDown() {
    mainMotor.set(TalonFXControlMode.PercentOutput, 0.3);
  }

  public void elevatorStop() {
    mainMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void reset() {
    mainMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    double position = mainMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber("sensor position", position);
  }
}
