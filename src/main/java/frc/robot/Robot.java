// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LED;
// import frc.robot.subsystems.LED;
import frc.robot.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Constants constants;

  boolean offYet;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  //
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // LED.CANdleSystem();
    PathPlannerServer.startServer(5811); // 5811 = port number. adjust this according to your needs
    CameraServer.startAutomaticCapture();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Swerve.resetModulesToAbsolute();
    offYet = false;
    LED.Off();
    m_robotContainer.m_Elevator.armAndElevatorStopPercentMode();
    if (DriverStation.isFMSAttached()) {
      LED.Fire();
    } else {
      // LED.Rainbow((int) (((RobotController.getBatteryVoltage()) - 10)) * 18);
    }
  }

  @Override
  public void disabledPeriodic() { //we could do a timer and then some remainder math at intervals of the timer to reset the LEDs so stuff doesnt get stuck on colors
    if (!DriverStation.isFMSAttached()) {
      if (offYet == false) {
        LED.Off();
        offYet = true;
      } else {
        if (RobotController.getBatteryVoltage() > 10) {
          LED.Rainbow(
            //(int) (RobotContainer.m_LEDCount.getSelected())
            ((int) ((RobotController.getBatteryVoltage() - 10) * 18.3))
          );
        } else {
          LED.Off();
        }
      }
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.s_Swerve.zeroGyro();
    Swerve.resetModulesToAbsolute();
    if (DriverStation.getAlliance() == Alliance.Blue) {
      LED.LEDColor(0, 0, 255);
    } else if (DriverStation.getAlliance() == Alliance.Red) {
      LED.LEDColor(255, 0, 0);
    }
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Swerve.resetModulesToAbsolute();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
