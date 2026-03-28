// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

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
    public static final double REEF_TARGET_DISTANCE_METERS = 2.0;
    public static final double DEFAULT_TAG_HEIGHT_METERS = 1.12;
    public static final double CAMERA_HEIGHT_METERS = 0.434;
    public static final List<Integer> REEF_TAG_IDS =
        List.of( 10, 26);

    // All reef tags share one tunable stand-off distance.
    public static final Map<Integer, Double> TARGET_DISTANCE_METERS_BY_TAG =
        REEF_TAG_IDS.stream()
            .collect(Collectors.toUnmodifiableMap(tagId -> tagId, tagId -> REEF_TARGET_DISTANCE_METERS));
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.05;
    public static final double LEFT_X_DEADBAND = 0.05;
  }
  public static double maximumSpeed = Units.feetToMeters(4.5);
}
