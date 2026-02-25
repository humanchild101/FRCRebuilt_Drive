// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.REPLAY;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Replaying from a log file. */
    REPLAY

    
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }
  public static class Ports {
    // Controllers
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;

//    // Motors
//    public static final int TURRET = 13;
//    public static final int SHOOTER_FUEL1 = 14;
//    public static final int SHOOTER_FUEL2 = 15;
//    public static final int SHOOTER_WHEELS = 16;
//    public static final int SHOOTER_ANGLER = 17;
//
//
//    public static final int INTAKE_PIVOT = 18;
//    public static final int INTAKE_WHEEL = 19;
//    public static final int INDEXER_SENSOR = 20;
//    public static final int INDEXER = 21;
//    public static final int CLIMB = 22;
//    public static final int SHOOTER_CAN = 23;

  }

  public static class AutoConstants {
    public static final double kPXController = 3.0;
    public static final double kPThetaController = 3.0;
  }
}