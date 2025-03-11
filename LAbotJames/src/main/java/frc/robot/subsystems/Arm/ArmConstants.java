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

  public static final double wristP = 2.0001;
  public static final double wristI = 0.000;
  public static final double wristD = 0.1;
  public static final double wristFF = 0;

  public static final double coralElbowL1 = 0.338;
  public static final double coralElbowL2 = 0.338;
  public static final double coralElbowL3 = 0.338;
  public static final double coralElbowL4 = 0.290;

  public static final double coralWristL1 = 0.842;
  public static final double coralWristL2 = 0.842;
  public static final double coralWristL3 = 0.842;
  public static final double coralWristL4 = 0.842;

  public static final double algaeElbowL25 = 0.25;
  public static final double algaeElbowL35 = 0.25;

  public static final double algaeWristL25 = 0.71;
  public static final double algaeWristL35 = 0.71;

  public static final double processorWrist = .8420; // not final
  public static final double processorElbow = .290; // not final

  public static final double sourceWrist = 0.3 + 0.07;
  public static final double sourceElbow = 0.249;

  public static final double bargeWrist = .842; // not final
  public static final double bargeElbow = .290; // not final

  public static final double homeWrist = 0.555;

  public static final double homeElbow = 0.309;

  public static final double elbowCurrentLimit = 40;

  public static final double elbowClimb = 0.114;
  public static final double wristClimb = 0.379;

  public static final double alleyOop = 0.675;
}
