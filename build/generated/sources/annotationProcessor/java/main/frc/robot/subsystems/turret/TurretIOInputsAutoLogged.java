package frc.robot.subsystems.turret;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TurretIOInputsAutoLogged extends TurretIO.TurretIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("TurretConnected", turretConnected);
    table.put("TurretCurrent", turretCurrent);
    table.put("TurretVoltage", turretVoltage);
    table.put("TurretVelocityDegPerSec", turretVelocityDegPerSec);
    table.put("TurretDirDeg", turretDirDeg);
  }

  @Override
  public void fromLog(LogTable table) {
    turretConnected = table.get("TurretConnected", turretConnected);
    turretCurrent = table.get("TurretCurrent", turretCurrent);
    turretVoltage = table.get("TurretVoltage", turretVoltage);
    turretVelocityDegPerSec = table.get("TurretVelocityDegPerSec", turretVelocityDegPerSec);
    turretDirDeg = table.get("TurretDirDeg", turretDirDeg);
  }

  public TurretIOInputsAutoLogged clone() {
    TurretIOInputsAutoLogged copy = new TurretIOInputsAutoLogged();
    copy.turretConnected = this.turretConnected;
    copy.turretCurrent = this.turretCurrent;
    copy.turretVoltage = this.turretVoltage;
    copy.turretVelocityDegPerSec = this.turretVelocityDegPerSec;
    copy.turretDirDeg = this.turretDirDeg;
    return copy;
  }
}
