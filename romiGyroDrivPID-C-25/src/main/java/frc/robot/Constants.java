// romiGyroDrivPID - C  25                     CONSTANT.J

package frc.robot;

/* you can statically import this class (or one of its inner     
 * classes), wherever needed, to reduce verbosity.
 * most example values unused by Romi refe
 */
public final class Constants {
  public static final class DriveConstants {
    // public static final int kLeftMotor1Port = 0;
    // public static final int kLeftMotor2Port = 1;
    // public static final int kRightMotor1Port = 2;
    // public static final int kRightMotor2Port = 3;

    // public static final int kEncoderCPR = 1024;
    // public static final double kWheelDiameterInches = 6;
    // public static final double kEncoderDistancePerPulse =
    // // Assumes the encoders are directly mounted on wheel shafts
    // (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

    // public static final boolean kGyroReversed = false;

    // public static final double kStabilizP = 0.05;
    // public static final double kStabilizI = 0.0;
    // public static final double kStabilizD = 0.0;

    //used by 2 PID controllers
    public static final double kTurnP = 0.06;
    public static final double kTurnI = 0.0002;
    public static final double kTurnD = 0.000;

    public static final double kTurnTolerDeg = 2;
    public static final double kTurnRateTolerVeloc = 90;
    // deg per second

// used by profiled controller, not sure how to derive 
// or how they limit turn in practice
    public static final double kMaxTurnDegPerSec = 120;
    public static final double kMaxTurnAccel = 120;

   // from romi refe & test:drive motor kS 0.9. kV 0.16-0.25(V-sec/inch)
   // kA 0.01?

   // to measure these, apply 0.5 power to rot param, count deg.rot/sec
   public static final double kvVoltSecondsPerDegree = 0.02;
   public static final double kaVoltSecondsSquaredPerDegree = 0.001;
  } // end drive constant

  // public static final class OIConstants {
  // public static final int kDriverControllerPort = 0;
} // end class
