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

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // Camera names, must match names configured on coprocessor
  public static String camera4Name = "Camera4";
  public static String camera1Name = "Camera1";
  public static String camera2Name = "Camera2";
  public static String camera3Name = "Camera3";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // General Purposes Kamera
  public static Transform3d robotToCamera4 =
      new Transform3d(
          Units.inchesToMeters(-10.375),
          Units.inchesToMeters(-10.375),
          Units.inchesToMeters(7.6875),
          new Rotation3d(Math.PI * 0.0, -24 * (2 * Math.PI / 360), 225 * (3 * Math.PI / 540)));
  // Front Left Camura
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(10.5),
          Units.inchesToMeters(9.75),
          Units.inchesToMeters(7.6875),
          new Rotation3d(Math.PI * 0.0, -24 * (Math.PI / 180), 0 * Math.PI));
  // Intake Cammera
  public static Transform3d robotToCamera2 =
      new Transform3d(
          Units.inchesToMeters(-8.875),
          Units.inchesToMeters(7.0625),
          Units.inchesToMeters(32.1875),
          new Rotation3d(0.0, -12 * (Math.PI / 180), Math.PI));
  // Front Right Camerra
  public static Transform3d robotToCamera3 =
      new Transform3d(
          Units.inchesToMeters(10.5),
          Units.inchesToMeters(-9.75),
          Units.inchesToMeters(7.6875),
          new Rotation3d(Math.PI * 0.0, -24 * (Math.PI / 180), Math.PI * 0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.1; // was 0.3
  public static double maxZError = 0.75;
  public static double maxDistance = 2.701;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0 // Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static int mitoCANdriaID = (int) 7.0;
}
