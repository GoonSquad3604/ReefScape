// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  double tx;

  double ty;

  public Limelight() {
    LimelightHelpers.setPipelineIndex("limelight", 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("seesNote", LimelightHelpers.getTV("Limelight"));
    SmartDashboard.putNumber("tx", LimelightHelpers.getTX("Limelight"));
    // if (LimelightHelpers.getTV("limelight")) {
    //   tx = LimelightHelpers.getTX("limelight");
    //   ty =
    // LimelightHelpec:\GoonGit\ReefScape\PheonixSwerveTemplate\src\main\java\frc\robot\subsystems\visionrs.getTY("limelight");
    //   while (Math.abs(tx) > 1) {
    //     if (tx > 1) {}

    //     if (tx < -1) {}
    //   }
    // }
  }
}
