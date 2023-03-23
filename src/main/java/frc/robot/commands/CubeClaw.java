// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.Constants;
import com.playingwithfusion.TimeOfFlight;



public class CubeClaw extends CommandBase {
  /** Creates a new AutoClaw. */

 Claw m_claw;
 Elevator m_Elevator;
 //LED m_led;



  public CubeClaw(Claw Claw, Elevator Elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_claw = Claw;
    m_Elevator = Elevator;
    addRequirements(m_claw);
    addRequirements(m_Elevator);
    //addRequirements(m_led);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_claw.openCube();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.closeCube();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("TOFISFINISHED", m_claw.TOF.getRange());
    if (m_claw.TOF.getRange() < 50 && m_claw.TOF.getRange() > 5 && m_claw.TOF.isRangeValid()) {
    m_claw.closeCube();
    m_Elevator.setStow();
    return true;
    }
    else {
      return false;
    }
  }
}
