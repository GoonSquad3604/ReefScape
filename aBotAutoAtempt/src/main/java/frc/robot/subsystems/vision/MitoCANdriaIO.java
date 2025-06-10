package frc.robot.subsystems.vision;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.CouldNotGetException;
import au.grapplerobotics.MitoCANdria;

public class MitoCANdriaIO implements PowerSystemIO {

  private MitoCANdria mito;

  public MitoCANdriaIO() {
    this.mito = new MitoCANdria(VisionConstants.mitoCANdriaID);
    setFiveVoltAEnabled(true);
  }

  @Override
  public void updateInputs(PowerSystemIOInputs inputs) {
    inputs.mitoCANdriaConnected = true;
    try {
      inputs.mitoFiveVoltAEnabled =
          mito.getChannelEnabled(MitoCANdria.MITOCANDRIA_CHANNEL_5VA).getAsInt() == 1
              ? true
              : false;
      inputs.mitoFiveVoltACurrent =
          mito.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_5VA).orElse(0);
    } catch (CouldNotGetException e) {
      inputs.mitoFiveVoltAEnabled = false;
      inputs.mitoFiveVoltACurrent = 0;
    }
    try {
      inputs.mitoUSBOneCurrent =
          mito.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_USB1).orElse(0);
    } catch (CouldNotGetException e) {
      inputs.mitoUSBOneCurrent = 0;
    }
    try {
      inputs.mitoUSBTwoCurrent =
          mito.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_USB2).orElse(0);
    } catch (CouldNotGetException e) {
      inputs.mitoUSBTwoCurrent = 0;
    }
  }

  @Override
  public void setFiveVoltAEnabled(boolean enabled) {
    try {
      mito.setChannelEnabled(MitoCANdria.MITOCANDRIA_CHANNEL_5VA, enabled);
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
    }
  }
}
