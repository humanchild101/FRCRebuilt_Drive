package frc.robot.subsystems.intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PivotConnected", pivotConnected);
    table.put("PivotPositionDeg", pivotPositionDeg);
    table.put("PivotVelocityDegPerSec", pivotVelocityDegPerSec);
    table.put("PivotAppliedVolts", pivotAppliedVolts);
    table.put("PivotCurrent", pivotCurrent);
    table.put("WheelsConnected", wheelsConnected);
    table.put("WheelsVelocityDegPerSec", wheelsVelocityDegPerSec);
    table.put("WheelsAppliedVolts", wheelsAppliedVolts);
    table.put("WheelsCurrent", wheelsCurrent);
    table.put("SensorDistanceMillimeters", sensorDistanceMillimeters);
    table.put("LaserCANConnected", laserCANConnected);
    table.put("LaserCANStatus", laserCANStatus);
  }

  @Override
  public void fromLog(LogTable table) {
    pivotConnected = table.get("PivotConnected", pivotConnected);
    pivotPositionDeg = table.get("PivotPositionDeg", pivotPositionDeg);
    pivotVelocityDegPerSec = table.get("PivotVelocityDegPerSec", pivotVelocityDegPerSec);
    pivotAppliedVolts = table.get("PivotAppliedVolts", pivotAppliedVolts);
    pivotCurrent = table.get("PivotCurrent", pivotCurrent);
    wheelsConnected = table.get("WheelsConnected", wheelsConnected);
    wheelsVelocityDegPerSec = table.get("WheelsVelocityDegPerSec", wheelsVelocityDegPerSec);
    wheelsAppliedVolts = table.get("WheelsAppliedVolts", wheelsAppliedVolts);
    wheelsCurrent = table.get("WheelsCurrent", wheelsCurrent);
    sensorDistanceMillimeters = table.get("SensorDistanceMillimeters", sensorDistanceMillimeters);
    laserCANConnected = table.get("LaserCANConnected", laserCANConnected);
    laserCANStatus = table.get("LaserCANStatus", laserCANStatus);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.pivotConnected = this.pivotConnected;
    copy.pivotPositionDeg = this.pivotPositionDeg;
    copy.pivotVelocityDegPerSec = this.pivotVelocityDegPerSec;
    copy.pivotAppliedVolts = this.pivotAppliedVolts;
    copy.pivotCurrent = this.pivotCurrent;
    copy.wheelsConnected = this.wheelsConnected;
    copy.wheelsVelocityDegPerSec = this.wheelsVelocityDegPerSec;
    copy.wheelsAppliedVolts = this.wheelsAppliedVolts;
    copy.wheelsCurrent = this.wheelsCurrent;
    copy.sensorDistanceMillimeters = this.sensorDistanceMillimeters;
    copy.laserCANConnected = this.laserCANConnected;
    copy.laserCANStatus = this.laserCANStatus;
    return copy;
  }
}
