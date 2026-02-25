package frc.robot.subsystems.shooter;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterIOInputsAutoLogged extends ShooterIO.ShooterIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("FuelConnected", fuelConnected);
    table.put("Fuel2Connected", fuel2Connected);
    table.put("AnglerConnected", anglerConnected);
    table.put("WheelsConnected", wheelsConnected);
    table.put("AnglerCurrent", anglerCurrent);
    table.put("FuelCurrent", fuelCurrent);
    table.put("TurretCurrent", turretCurrent);
    table.put("WheelsCurrent", wheelsCurrent);
    table.put("AnglerVoltage", anglerVoltage);
    table.put("FuelVoltage", fuelVoltage);
    table.put("TurretVoltage", turretVoltage);
    table.put("WheelsVoltage", wheelsVoltage);
    table.put("AnglerVelocityDegPerSec", anglerVelocityDegPerSec);
    table.put("FuelVelocityDegPerSec", fuelVelocityDegPerSec);
    table.put("TurretVelocityDegPerSec", turretVelocityDegPerSec);
    table.put("WheelsVelocityDegPerSec", wheelsVelocityDegPerSec);
    table.put("AnglerPos", anglerPos);
    table.put("SensorDistanceMillimeters", sensorDistanceMillimeters);
    table.put("LaserCANConnected", laserCANConnected);
    table.put("LaserCANStatus", laserCANStatus);
  }

  @Override
  public void fromLog(LogTable table) {
    fuelConnected = table.get("FuelConnected", fuelConnected);
    fuel2Connected = table.get("Fuel2Connected", fuel2Connected);
    anglerConnected = table.get("AnglerConnected", anglerConnected);
    wheelsConnected = table.get("WheelsConnected", wheelsConnected);
    anglerCurrent = table.get("AnglerCurrent", anglerCurrent);
    fuelCurrent = table.get("FuelCurrent", fuelCurrent);
    turretCurrent = table.get("TurretCurrent", turretCurrent);
    wheelsCurrent = table.get("WheelsCurrent", wheelsCurrent);
    anglerVoltage = table.get("AnglerVoltage", anglerVoltage);
    fuelVoltage = table.get("FuelVoltage", fuelVoltage);
    turretVoltage = table.get("TurretVoltage", turretVoltage);
    wheelsVoltage = table.get("WheelsVoltage", wheelsVoltage);
    anglerVelocityDegPerSec = table.get("AnglerVelocityDegPerSec", anglerVelocityDegPerSec);
    fuelVelocityDegPerSec = table.get("FuelVelocityDegPerSec", fuelVelocityDegPerSec);
    turretVelocityDegPerSec = table.get("TurretVelocityDegPerSec", turretVelocityDegPerSec);
    wheelsVelocityDegPerSec = table.get("WheelsVelocityDegPerSec", wheelsVelocityDegPerSec);
    anglerPos = table.get("AnglerPos", anglerPos);
    sensorDistanceMillimeters = table.get("SensorDistanceMillimeters", sensorDistanceMillimeters);
    laserCANConnected = table.get("LaserCANConnected", laserCANConnected);
    laserCANStatus = table.get("LaserCANStatus", laserCANStatus);
  }

  public ShooterIOInputsAutoLogged clone() {
    ShooterIOInputsAutoLogged copy = new ShooterIOInputsAutoLogged();
    copy.fuelConnected = this.fuelConnected;
    copy.fuel2Connected = this.fuel2Connected;
    copy.anglerConnected = this.anglerConnected;
    copy.wheelsConnected = this.wheelsConnected;
    copy.anglerCurrent = this.anglerCurrent;
    copy.fuelCurrent = this.fuelCurrent;
    copy.turretCurrent = this.turretCurrent;
    copy.wheelsCurrent = this.wheelsCurrent;
    copy.anglerVoltage = this.anglerVoltage;
    copy.fuelVoltage = this.fuelVoltage;
    copy.turretVoltage = this.turretVoltage;
    copy.wheelsVoltage = this.wheelsVoltage;
    copy.anglerVelocityDegPerSec = this.anglerVelocityDegPerSec;
    copy.fuelVelocityDegPerSec = this.fuelVelocityDegPerSec;
    copy.turretVelocityDegPerSec = this.turretVelocityDegPerSec;
    copy.wheelsVelocityDegPerSec = this.wheelsVelocityDegPerSec;
    copy.anglerPos = this.anglerPos;
    copy.sensorDistanceMillimeters = this.sensorDistanceMillimeters;
    copy.laserCANConnected = this.laserCANConnected;
    copy.laserCANStatus = this.laserCANStatus;
    return copy;
  }
}
