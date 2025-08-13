package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;
import java.util.List;

public final class LEDConstants {

  public static final int STRIP_LENGTH = 25;
  public static final double BREATHE_DURATION = 1.0;
  public static final double WAVE_EXPONENT = 0.4;
  public static final int LED_ID = 0;

  public static final List<Color> stripes = new ArrayList<>();

  static {
    stripes.add(Color.kRed);
    stripes.add(Color.kDarkRed);
    stripes.add(Color.kCrimson);
    stripes.add(Color.kOrange);
    stripes.add(Color.kYellow);
    stripes.add(Color.kGreen);
    stripes.add(Color.kBlue);
    stripes.add(Color.kPurple);
  }
}

 // ooh im blinded by the lights - weeknd
