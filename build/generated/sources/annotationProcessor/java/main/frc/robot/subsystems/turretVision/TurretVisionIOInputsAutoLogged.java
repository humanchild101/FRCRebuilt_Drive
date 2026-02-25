package frc.robot.subsystems.turretVision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TurretVisionIOInputsAutoLogged extends TurretVisionIO.TurretVisionIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("HasTrackingTarget", hasTrackingTarget);
    table.put("LatestTargetObservation", latestTargetObservation);
    table.put("PoseObservations", poseObservations);
    table.put("TagIds", tagIds);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    hasTrackingTarget = table.get("HasTrackingTarget", hasTrackingTarget);
    latestTargetObservation = table.get("LatestTargetObservation", latestTargetObservation);
    poseObservations = table.get("PoseObservations", poseObservations);
    tagIds = table.get("TagIds", tagIds);
  }

  public TurretVisionIOInputsAutoLogged clone() {
    TurretVisionIOInputsAutoLogged copy = new TurretVisionIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.hasTrackingTarget = this.hasTrackingTarget;
    copy.latestTargetObservation = this.latestTargetObservation;
    copy.poseObservations = this.poseObservations.clone();
    copy.tagIds = this.tagIds.clone();
    return copy;
  }
}
