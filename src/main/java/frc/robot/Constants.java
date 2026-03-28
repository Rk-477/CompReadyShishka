// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class LimelightConstants {
    private LimelightConstants() {}

    // Camera pose relative to robot origin (meters).
    public static final Translation3d kCameraToRobot = new Translation3d(0.0, -0.064, 0.434);
  }

  public static final class AutoAlignConstants {
    private AutoAlignConstants() {}

    public static final double DEFAULT_TARGET_DISTANCE_METERS = 2.0;
    public static final double DEFAULT_TAG_HEIGHT_METERS = 1.12;
    public static final double CAMERA_HEIGHT_METERS = 0.434;

    // Per-tag target distance table; tune on-robot as needed.
    public static final Map<Integer, Double> TARGET_DISTANCE_METERS_BY_TAG = Map.ofEntries(
      
        Map.entry(10, DEFAULT_TARGET_DISTANCE_METERS),
        Map.entry(26, DEFAULT_TARGET_DISTANCE_METERS));
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.05;
    public static final double LEFT_X_DEADBAND = 0.05;
  }
  public static double maximumSpeed = Units.feetToMeters(4.5);
}
