package frc.robot.subsystems.climb;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimbIOInputsAutoLogged extends ClimbIO.ClimbIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ClimbConnected", climbConnected);
    table.put("ClimbPositionDeg", climbPositionDeg);
    table.put("ClimbVelocityDegPerSec", climbVelocityDegPerSec);
    table.put("ClimbVoltage", climbVoltage);
    table.put("ClimbCurrent", climbCurrent);
  }

  @Override
  public void fromLog(LogTable table) {
    climbConnected = table.get("ClimbConnected", climbConnected);
    climbPositionDeg = table.get("ClimbPositionDeg", climbPositionDeg);
    climbVelocityDegPerSec = table.get("ClimbVelocityDegPerSec", climbVelocityDegPerSec);
    climbVoltage = table.get("ClimbVoltage", climbVoltage);
    climbCurrent = table.get("ClimbCurrent", climbCurrent);
  }

  public ClimbIOInputsAutoLogged clone() {
    ClimbIOInputsAutoLogged copy = new ClimbIOInputsAutoLogged();
    copy.climbConnected = this.climbConnected;
    copy.climbPositionDeg = this.climbPositionDeg;
    copy.climbVelocityDegPerSec = this.climbVelocityDegPerSec;
    copy.climbVoltage = this.climbVoltage;
    copy.climbCurrent = this.climbCurrent;
    return copy;
  }
}
