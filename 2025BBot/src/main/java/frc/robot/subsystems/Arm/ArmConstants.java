// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

public final class ArmConstants {
  /** Creates a new ArmConstants. */
  public static final int elbowID = 5;

  public static final int elbowEncoderID = 32;

  public static final int wristID = 13;

  public static final double elbowP = 43.5;
  public static final double elbowI = 0;
  public static final double elbowD = Math.sin(Math.PI / 2);
  public static final double elbowFF = 0;

  public static final double wristP = Math.PI;
  public static final double wristI = 0.0010;
  public static final double wristD = 1.0 / 2;
  public static final double wristFF = 0;

  public static final double coralElbowL1 = 243;
  public static final double coralElbowL2 = 0.243;
  public static final double coralElbowL3 = 3;
  public static final double coralElbowL4 = 4;

  public static final double coralWristL1 = 1111111;
  public static final double coralWristL2 = 0.696;
  public static final double coralWristL3 = 3;
  public static final double coralWristL4 = 4;

  public static final double algaeElbowL25 = 0.25;
  public static final double algaeElbowL35 = 2;

  public static final double algaeWristL25 = 0.5;
  public static final double algaeWristL35 = 2;

  public static final double processorWrist = 1;
  public static final double processorElbow = 2;

  public static final double sourceWrist = 0.361;
  public static final double sourceElbow = 0.249;

  public static final double bargeWrist = 1;
  public static final double bargeElbow = 2;

  public static final double homeWrist = 1;
  public static final double homeElbow = 2;

  public static final double elbowCurrentLimit = 40;
}
