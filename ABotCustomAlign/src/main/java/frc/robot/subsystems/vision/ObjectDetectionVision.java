// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class ObjectDetectionVision extends SubsystemBase {

  private final ObjectDetectionVisionIO[] io;
  private final MitoCANdriaIO mitoIO;
  private final ObjectDetectionVisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private static final PhotonCamera camera = new PhotonCamera("HD_USB_Camera");
  @AutoLogOutput private static Rotation2d tx = new Rotation2d();

  public ObjectDetectionVision(MitoCANdriaIO mitoIO, ObjectDetectionVisionIO... io) {
    this.io = io;
    this.mitoIO = mitoIO;
    // Initialize inputs
    this.inputs = new ObjectDetectionVisionIOInputsAutoLogged[io.length];

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    disconnectedAlerts[4] =
        new Alert("Object Detection camera is disconnected.", AlertType.kWarning);
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public static Rotation2d getTargetX() {
    // var results = camera.getAllUnreadResults();
    // var result = results.get(results.size() - 1);

    // return new Rotation2d(result.getBestTarget().getYaw());
    return tx;
  }

  /**
   * Returns the Y angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetY(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.ty();
  }

  public static boolean hasTarget() {
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        tx = new Rotation2d(result.getBestTarget().getYaw());
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // for (int i = 0; i < io.length; i++) {
    io[4].updateInputs(inputs[4]);
    Logger.processInputs("Vision/Camera" + Integer.toString(4), inputs[4]);
    // }

    // Initialize logging values

    // Loop over cameras

    // Update disconnected alert
    // disconnectedAlerts[].set(!inputs.);
    disconnectedAlerts[4].set(!inputs[4].connected);

    // Initialize logging values

    // Add tag poses

    // Loop over pose observations

    // Log camera datadata

    // Log summary data

  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
