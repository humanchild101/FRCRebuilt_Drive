package frc.robot.subsystems.indexer;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IndexerIOInputsAutoLogged extends IndexerIO.IndexerIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IndexerConnected", indexerConnected);
    table.put("IndexerVelocityDegPerSec", indexerVelocityDegPerSec);
    table.put("IndexerAppliedVolts", indexerAppliedVolts);
    table.put("IndexerCurrent", indexerCurrent);
  }

  @Override
  public void fromLog(LogTable table) {
    indexerConnected = table.get("IndexerConnected", indexerConnected);
    indexerVelocityDegPerSec = table.get("IndexerVelocityDegPerSec", indexerVelocityDegPerSec);
    indexerAppliedVolts = table.get("IndexerAppliedVolts", indexerAppliedVolts);
    indexerCurrent = table.get("IndexerCurrent", indexerCurrent);
  }

  public IndexerIOInputsAutoLogged clone() {
    IndexerIOInputsAutoLogged copy = new IndexerIOInputsAutoLogged();
    copy.indexerConnected = this.indexerConnected;
    copy.indexerVelocityDegPerSec = this.indexerVelocityDegPerSec;
    copy.indexerAppliedVolts = this.indexerAppliedVolts;
    copy.indexerCurrent = this.indexerCurrent;
    return copy;
  }
}
