package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import java.util.function.BooleanSupplier;

public class Claw extends SubsystemBase {

  public DoubleSolenoid clawPiston1;
  public DoubleSolenoid clawPiston2;
  public TalonFX clawMotor;
  public TimeOfFlight TOF;

  public Claw() {
    clawPiston1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
    clawPiston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 2); //idk
    clawMotor = new TalonFX(4);
    clawPiston1.set(Value.kForward);
    clawPiston2.set(Value.kForward);

    TOF = new TimeOfFlight(1);
    TOF.setRangingMode(RangingMode.Short, 30);
  }

  public CommandBase motorForward() {
    return run(() -> clawMotor.set(ControlMode.PercentOutput, .2));
  }

  public CommandBase motorOff() {
    return runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0));
  }

  public CommandBase motorReverse() {
    return run(() -> clawMotor.set(ControlMode.PercentOutput, -.2));
  }

  public CommandBase open1In() {
    return runOnce(() -> clawPiston1.set(Value.kForward))
      .andThen(runOnce(() -> clawPiston2.set(Value.kReverse)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, .5)));
  }

  public CommandBase open1Hold() {
    return runOnce(() -> clawPiston1.set(Value.kForward))
      .andThen(runOnce(() -> clawPiston2.set(Value.kReverse)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0)));
  }

  public CommandBase openAllIn() {
    return runOnce(() -> clawPiston2.set(Value.kReverse))
      .andThen(runOnce(() -> clawPiston1.set(Value.kReverse)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, .2)));
  }

  public void openCube() {
    clawPiston1.set(Value.kReverse);
    clawPiston2.set(Value.kReverse);
    clawMotor.set(ControlMode.PercentOutput, .5);
  }

  public void closeCube() {
    clawPiston1.set(Value.kForward);
    clawPiston2.set(Value.kReverse);
    clawMotor.set(ControlMode.PercentOutput, 0);
  }

  public void openCone() {
    clawPiston1.set(Value.kForward);
    clawPiston2.set(Value.kReverse);
    clawMotor.set(ControlMode.PercentOutput, .5);
  }

  public CommandBase openConeCommand() {
    return runOnce(() -> openCone())
      .andThen(
        Commands.waitUntil(() ->
          TOF.getRange() < 70 && TOF.getRange() > 10 && TOF.isRangeValid()
        )
      )
      .andThen(() -> closeCone())
      .andThen(() -> LED.GreenFlow())
      .andThen(RobotContainer.m_Elevator.setStow())
      .unless(() ->
        RobotContainer.m_Elevator.armMotor.getSelectedSensorPosition() > 1400
      ) //it's possible the unless will stop things bc idk if it moves on
      .finallyDo(interrupted -> {
        closeCone();
        RobotContainer.m_Elevator.armAndElevatorStopPercentMode();
      });
  }

  public CommandBase openCubeCommand() {
    return runOnce(() -> openCube())
      .andThen(
        Commands.waitUntil(() ->
          TOF.getRange() < 70 && TOF.getRange() > 10 && TOF.isRangeValid()
        )
      )
      .andThen(() -> closeCube())
      .andThen(() -> LED.GreenFlow())
      .andThen(RobotContainer.m_Elevator.setStow())
      .unless(() ->
        RobotContainer.m_Elevator.armMotor.getSelectedSensorPosition() > 1400
      )
      .finallyDo(interrupted -> {
        closeCone();
        RobotContainer.m_Elevator.armAndElevatorStopPercentMode();
      });
  }

  public void closeCone() {
    clawPiston1.set(Value.kForward);
    clawPiston2.set(Value.kForward);
    clawMotor.set(ControlMode.PercentOutput, 0);
  }

  public void TOFDistance() {
    TOF.getRange();
  }

  public CommandBase openAllHold() {
    return runOnce(() -> clawPiston2.set(Value.kReverse))
      .andThen(runOnce(() -> clawPiston1.set(Value.kReverse)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0)));
  }

  public CommandBase openAllOut() {
    return runOnce(() -> clawPiston2.set(Value.kReverse))
      .andThen(runOnce(() -> clawPiston1.set(Value.kReverse)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, -.2)));
  }

  public CommandBase openAllDrop() {
    return runOnce(() -> clawPiston2.set(Value.kReverse))
      .andThen(runOnce(() -> clawPiston1.set(Value.kReverse)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0)));
  }

  public CommandBase closeAllHold() {
    return runOnce(() -> clawPiston1.set(Value.kForward))
      .andThen(runOnce(() -> clawPiston2.set(Value.kForward)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0)));
  }

  // public CommandBase LOPEN() {
  //   return run(() -> clawPiston1.set(Value.kForward)); //closes L
  // }

  // public CommandBase LCLOSE() {
  //   return run(() -> clawPiston1.set(Value.kReverse)); //opens L
  // }

  // public CommandBase ROPEN() {
  //   return run(() -> clawPiston2.set(Value.kForward)); //closes R
  // }

  // public CommandBase RCLOSE() {
  //   return run(() -> clawPiston2.set(Value.kReverse)); //opens R
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("TOF", TOF.getRange());
    SmartDashboard.putBoolean("TOF", TOF.getRange() < 40);
  }
}
