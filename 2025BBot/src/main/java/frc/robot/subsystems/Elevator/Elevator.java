// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {

  /** Creates a new Elevator. */
  private ElevatorIO IO;
  protected final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert leftDisconnected;
  private final Alert rightDisconnected;

  public Elevator(ElevatorIO elevatorIo) {

    // whee elevator go up yippee

    IO = elevatorIo;
    
    //will throw an error if a motor is disconnected
    leftDisconnected = new Alert("Left elevator motor is disconnected!", Alert.AlertType.kWarning);
    rightDisconnected = new Alert("Right elevator motor is disconnected!", Alert.AlertType.kWarning);

      }
    
      public Command elevatorCL1(){
        return run(() -> IO.setPos(ElevatorConstants.l1Pos));
      }

      public Command elevatorCL2(){
        return run(() -> IO.setPos(ElevatorConstants.l2Pos));
      }

      public Command elevatorCL3(){
        return run(() -> IO.setPos(ElevatorConstants.l3Pos));
      }

      public Command elevatorCL4(){
        return run(() -> IO.setPos(ElevatorConstants.l4Pos));
      }

      public Command elevatorAL2(){
        return run(() -> IO.setPos(ElevatorConstants.algaeL2Pos));
      }

      public Command elevatorAL3(){
        return run(() -> IO.setPos(ElevatorConstants.algaeL3Pos));
      }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    IO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    //checks for disconnected motors
    leftDisconnected.set(!inputs.MotorLeftConnected);
    rightDisconnected.set(!inputs.MotorRightConnected);

  }
}
