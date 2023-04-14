package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */

  public final XboxController driver1 = new XboxController(0);
  private final Joystick buttonBoard = new Joystick(1); //maybe 2 idk fix the ports
  private final Joystick driver2 = new Joystick(2);
  Trigger leftTrigger = new Trigger(() -> driver1.getLeftTriggerAxis() > 0.5);
  Trigger rightTrigger = new Trigger(() -> driver1.getRightTriggerAxis() > 0.5);
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons :) */
  private final JoystickButton back1 = new JoystickButton(
    driver1,
    XboxController.Button.kBack.value
  );
  private final JoystickButton yButton1 = new JoystickButton(
    driver1,
    XboxController.Button.kY.value
  );
  private final JoystickButton leftBumper1 = new JoystickButton( //maybe we wanna change this to leftBumper
    driver1,
    XboxController.Button.kLeftBumper.value
  );
  private final JoystickButton aButton1 = new JoystickButton(
    driver1,
    XboxController.Button.kA.value
  );
  private final JoystickButton bButton1 = new JoystickButton(
    driver1,
    XboxController.Button.kB.value
  );
  private final JoystickButton rightBumper1 = new JoystickButton( //rightBumper
    driver1,
    XboxController.Button.kRightBumper.value
  );
  private final JoystickButton xButton1 = new JoystickButton(
    driver1,
    XboxController.Button.kX.value
  );
  private final JoystickButton start1 = new JoystickButton(
    driver1,
    XboxController.Button.kStart.value
  );
  private final JoystickButton leftStickButton1 = new JoystickButton(
    driver1,
    XboxController.Button.kLeftStick.value
  );

  private final JoystickButton rightStickButton1 = new JoystickButton(
    driver1,
    XboxController.Button.kRightStick.value
  );

  private final POVButton dUp1 = new POVButton(driver1, 0);

  private final POVButton dRight1 = new POVButton(driver1, 90);

  private final POVButton dDown1 = new POVButton(driver1, 180);

  private final POVButton dLeft1 = new POVButton(driver1, 270);

  private final JoystickButton aButton2 = new JoystickButton(
    driver2,
    XboxController.Button.kA.value
  );
  private final JoystickButton bButton2 = new JoystickButton(
    driver2,
    XboxController.Button.kB.value
  );
  private final JoystickButton xButton2 = new JoystickButton(
    driver2,
    XboxController.Button.kX.value
  );
  private final JoystickButton yButton2 = new JoystickButton(
    driver2,
    XboxController.Button.kY.value
  );

  private final JoystickButton leftBumper2 = new JoystickButton(
    driver2,
    XboxController.Button.kLeftBumper.value
  );

  private final JoystickButton rightBumper2 = new JoystickButton(
    driver2,
    XboxController.Button.kRightBumper.value
  );

  private final JoystickButton start2 = new JoystickButton(
    driver2,
    XboxController.Button.kStart.value
  );

  private final JoystickButton back2 = new JoystickButton(
    driver2,
    XboxController.Button.kBack.value
  );

  final JoystickButton b1 = new JoystickButton(buttonBoard, 1);
  final JoystickButton b2 = new JoystickButton(buttonBoard, 2);
  final JoystickButton b3 = new JoystickButton(buttonBoard, 3);
  final JoystickButton b4 = new JoystickButton(buttonBoard, 4);
  final JoystickButton b5 = new JoystickButton(buttonBoard, 5);
  final JoystickButton b6 = new JoystickButton(buttonBoard, 6);
  final JoystickButton b7 = new JoystickButton(buttonBoard, 7);
  final JoystickButton b8 = new JoystickButton(buttonBoard, 8);
  final JoystickButton b9 = new JoystickButton(buttonBoard, 9);
  final JoystickButton b10 = new JoystickButton(buttonBoard, 10);
  final JoystickButton b11 = new JoystickButton(buttonBoard, 11);
  final JoystickButton b12 = new JoystickButton(buttonBoard, 12);

  Trigger bbStickF = new Trigger(() -> buttonBoard.getRawAxis(1) > 0.7);
  Trigger bbStickB = new Trigger(() -> buttonBoard.getRawAxis(1) < -0.7);

  /* Subsystems */
  public final Limelight m_Limelight = new Limelight();
  public final Swerve s_Swerve = new Swerve(m_Limelight);
  public static final Elevator m_Elevator = new Elevator();
  private final Claw m_Claw = new Claw();
  public static final LED m_LED = new LED();

  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  public static SendableChooser<Integer> m_LEDCount;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Swerve.resetModulesToAbsolute();
    Swerve.resetModulesToAbsolute();

    // Shuffleboard
    //   .getTab("SmartDashboard")
    //   .add("LED count", 0)
    //   .withWidget(BuiltInWidgets.kNumberSlider)
    //   .getEntry();
    m_LEDCount = new SendableChooser<>();
    m_autoChooser.setDefaultOption("Nothing", new InstantCommand());
    m_LEDCount.setDefaultOption("0", 0);

    m_LEDCount.addOption("20", 20);
    m_LEDCount.addOption("55", 55);
    m_LEDCount.addOption("100", 100);

    m_autoChooser.addOption(
      "(CEN.) Cone/Mobility/Balance",
      new ConeMBalance(s_Swerve, m_Elevator, m_Claw)
    );

    m_autoChooser.addOption(
      "(CEN.) Cone/Grab/Balance",
      new ConeGrabBalance1PATH(s_Swerve, m_Elevator, m_Claw) //i want to test this but we can change it back if we need bc idk which would be more accurate honestly
    );

    m_autoChooser.addOption(
      " (LOAD) 2 Cone",
      new ConeConeSUB(s_Swerve, m_Elevator, m_Claw)
    );

    m_autoChooser.addOption(
      "(LOAD) Cone/Grab Cone",
      new ConeGrab(s_Swerve, m_Elevator, m_Claw)
    );

    m_autoChooser.addOption(
      "(LOAD) 2 Cone (1 Node)",
      new ConeLowConeSUB(s_Swerve, m_Elevator, m_Claw)
    );

    m_autoChooser.addOption(
      "(WALL) Cone/Grab Cone",
      new ConeGrabWALL(s_Swerve, m_Elevator, m_Claw)
    );

    // m_autoChooser.addOption(
    //   "(TEST) Drive Left",
    //   s_Swerve.followTrajectoryCommand(
    //     PathPlanner.loadPath("Driveleft", 2, 2),
    //     true
    //   )
    // );

    // m_autoChooser.addOption(
    //   "(TEST) Drive Forward",
    //   s_Swerve.followTrajectoryCommand(
    //     PathPlanner.loadPath("Driveforward", 2, 2),
    //     true
    //   )
    // );

    m_autoChooser.addOption(
      "(WALL) 2 Cone",
      new ConeConeWALL(s_Swerve, m_Elevator, m_Claw)
    );

    m_autoChooser.addOption(
      "(LOAD/WALL) Leave Community",
      new Leave(s_Swerve, m_Elevator, m_Claw)
    );

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    SmartDashboard.putData("LED Chooser", m_LEDCount);

    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve,
        () -> -driver1.getRawAxis(translationAxis),
        () -> -driver1.getRawAxis(strafeAxis),
        () -> -driver1.getRawAxis(rotationAxis),
        () -> back1.getAsBoolean(),
        () -> driver1.getLeftTriggerAxis()
      )
    );

    m_Elevator.armAndElevatorStopPercentMode();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    leftBumper1.whileTrue(m_Claw.openCubeCommand());
    leftBumper1.onFalse(m_Elevator.setStow());
    rightBumper1.whileTrue(m_Claw.openConeCommand());
    rightBumper1.onFalse(m_Elevator.setStow());
    aButton1.onTrue(m_Claw.openAllOut());
    aButton1.onFalse(m_Claw.motorOff());
    bButton1.onTrue(m_Claw.openAllDrop());

    xButton1.onTrue(m_Claw.openAllShoot());
    xButton1.onFalse(m_Claw.motorOff());
    leftStickButton1.onTrue(m_Elevator.setToFloor());

    rightStickButton1.onTrue(m_Elevator.setStow());

    rightTrigger.onTrue(
      m_Elevator.sequentialSetPositions(
        Constants.elevatorShelf,
        Constants.armShelf
      )
    );

    //Elevator Arm Presets
    b1.onTrue(
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCone,
        Constants.armTopCone
      )
    );
    b2.onTrue(
      m_Elevator.sequentialSetPositions(
        Constants.elevatorMidCone,
        Constants.armMidCone
      )
    );
    b3.onTrue(
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCube,
        Constants.armTopCube
      )
    );
    b4.onTrue(
      m_Elevator.sequentialSetPositions(
        Constants.elevatorMidCube,
        Constants.armMidCube
      )
    );
    b5.onTrue(m_Elevator.setStow());
    b6.onTrue(
      m_Elevator.sequentialSetPositions(
        Constants.elevatorShelf,
        Constants.armShelf
      )
    );

    b7.whileTrue(m_Elevator.armUp());
    b8.whileTrue(m_Elevator.armDown());
    b9.whileTrue(m_Elevator.runUp());
    b10.whileTrue(m_Elevator.runDown());

    b11.onTrue(
      new InstantCommand(() -> {
        m_LED.LEDColor(255, 140, 0);
        m_Limelight.setToRetroreflectiveTape();
      })
    );
    b12.onTrue(
      new InstantCommand(() -> {
        m_LED.LEDColor(255, 0, 255);
        m_Limelight.setToAprilTags();
      })
    );

    start1.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    back1.onTrue(new InstantCommand(() -> Swerve.resetModulesToAbsolute()));

    dUp1.whileTrue(
      s_Swerve.driveContinuous(new Translation2d(.2, 0), 0, true, false)
    );

    dRight1.whileTrue(
      s_Swerve.driveContinuous(new Translation2d(0, -0.2), 0, true, false)
    );

    dDown1.whileTrue(
      s_Swerve.driveContinuous(new Translation2d(-0.2, 0), 0, true, false)
    );

    dLeft1.whileTrue(
      s_Swerve.driveContinuous(new Translation2d(0, .2), 0, true, false)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }
}
