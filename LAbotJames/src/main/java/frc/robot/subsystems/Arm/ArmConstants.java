// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

public final class ArmConstants {
  /** Creates a new ArmConstants. */
  public static final int elbowID = 5;

  public static final int elbowEncoderID = 32;

  public static final int wristID = 13;

  public static final double elbowP = 85; // 11.5
  public static final double elbowI = 0;
  public static final double elbowD = 0 + Math.sin(Math.PI);
  // 0
  public static final double elbowFF = 0;

  public static final double wristP = 6;
  public static final double wristI = 0;

  public static final double wristD = 0.1;
  public static final double wristFF = 0.2;

  public static final double coralElbowL1 = 0.210;
  public static final double coralElbowL2 = 0.235;
  public static final double coralElbowL3 = 0.235;
  public static final double coralElbowL4 = 0.255;

  public static final double coralWristL1 = 0.275;
  public static final double coralWristL2 = 0.430;
  public static final double coralWristL3 = 0.430;
  public static final double coralWristL4 = 0.328;

  public static final double algaeElbowL25 = 0.224;
  public static final double algaeElbowL35 = 0.224;

  public static final double algaeWristL25 = 0.208;
  public static final double algaeWristL35 = 0.208;

  public static final double processorWrist = 0.218; // 0.281000
  public static final double processorElbow = 0.163; // 0.150

  public static final double sourceWrist = 0.655;
  public static final double sourceElbow = 0.310;

  public static final double longSourceWrist = 0.623;
  public static final double longSourceElbow = 0.348;

  public static final double bargeWrist = 0.341;
  public static final double bargeElbow = 0.294;

  public static final double groundWrist = 0.241;
  public static final double groundElbow = 0.110;

  public static final double homeWrist = 0.393;

  public static final double homeElbow = 0.294;

  public static final double elbowCurrentLimit = 40;

  public static final double elbowClimb = 0.123; // 0.075
  public static final double wristClimb = 0.600; // 0.583

  public static final double alleyOop = 0.675;
}
