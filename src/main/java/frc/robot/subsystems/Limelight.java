package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  // todo:
  // convert limelight from rotational angle to horizontal plane

  // get the values from the vision router
  double tx = NetworkTableInstance
    .getDefault()
    .getTable("limelight")
    .getEntry("tx")
    .getDouble(0);
  double tv = NetworkTableInstance
    .getDefault()
    .getTable("limelight")
    .getEntry("tv")
    .getDouble(0);

  @Override
  public void periodic() {
    // output the values to the smart dashboard
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putBoolean("tv", tv >= 1.0);
    SmartDashboard.putBoolean("target valid", tv >= 1.0);
  }

  // centers the robot relative to the april tag
  public void centering() {
    if (isTargetValid()) {}
  }

  // check to see if the target is valid
  public boolean isTargetValid() {
    return tv >= 1.0;
  }
}
