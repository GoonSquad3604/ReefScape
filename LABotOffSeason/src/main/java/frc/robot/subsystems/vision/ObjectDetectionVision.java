// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.vision;

// import java.util.LinkedList;
// import java.util.List;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.Alert;
// import edu.wpi.first.wpilibj.Alert.AlertType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.vision.Vision.VisionConsumer;
// import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

// /** Add your docs here. */
// public class ObjectDetectionVision extends SubsystemBase {

//   private final ObjectDetectionVisionIO io;
//   private final MitoCANdriaIO mitoIO;
//   private final VisionIOInputsAutoLogged[] inputs;
//   private final Alert disconnectedAlert;

//   public Vision(MitoCANdriaIO mitoIO, ObjectDetectionVisionIO io) {
//     this.io = io;
//     this.mitoIO = mitoIO;
//     // Initialize inputs
//     this.inputs = new VisionIOInputsAutoLogged[io.length];
//     for (int i = 0; i < inputs.length; i++) {
//       inputs[i] = new VisionIOInputsAutoLogged();
//     }

//     // Initialize disconnected alerts
//     disconnectedAlert =
//         new Alert(
//             "Object Detection camera is disconnected.", AlertType.kWarning);

//   }

//   /**
//    * Returns the X angle to the best target, which can be used for simple servoing with vision.
//    *
//    * @param cameraIndex The index of the camera to use.
//    */
//   public Rotation2d getTargetX(int cameraIndex) {
//     return inputs[cameraIndex].latestTargetObservation.tx();
//   }

//   /**
//    * Returns the Y angle to the best target, which can be used for simple servoing with vision.
//    *
//    * @param cameraIndex The index of the camera to use.
//    */
//   public Rotation2d getTargetY(int cameraIndex) {
//     return inputs[cameraIndex].latestTargetObservation.ty();
//   }

//   @Override
//   public void periodic() {
//     for (int i = 0; i < io.length; i++) {
//       io[i].updateInputs(inputs[i]);
//       Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
//     }

//     // Initialize logging values

//     // Loop over cameras

//     // Update disconnected alert
//     disconnectedAlert.set(!inputs);

//     // Initialize logging values

//     // Add tag poses

//     // Loop over pose observations

//     // Log camera datadata

//     // Log summary data

//   }

//   @FunctionalInterface
//   public static interface VisionConsumer {
//     public void accept(
//         Pose2d visionRobotPoseMeters,
//         double timestampSeconds,
//         Matrix<N3, N1> visionMeasurementStdDevs);
//   }
// }
