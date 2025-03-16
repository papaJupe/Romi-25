// romiGyroDrivPID - D                          CONSTANT.J

package frc.robot;

public final class Constant {   
	  // setpoint = a distance for an auto cmd to drive  
	  public static final double kDistP = 0.07;
	  public static final double kDistI = 0.009;
	  public static final double kDistD = 0.00;

  // worked well to stabiliz auto and teleop straight drive 
      public static final double kStabilP = 0.015;
      public static final double kStabilI = 0.000;
      public static final double kStabilD = 0.0;

  //  still erratic 
  // hot edit not reliable, better if Disconnected ?  
      public static final double kTurnP = 0.0055;
      public static final double kTurnI = 0.0003;
      public static final double kTurnD = 0.000;
  
      // public static final double kMaxTurnVeloc = 20;
      // public static final double kMaxTurnAcceler = 20;
  
      // public static final double kTurnTolerDeg = 2;
      // public static final double kTurnTolerVeloc = 10; 
      // deg per second
      
  // inner class for multiple constant or a single constant
  //  could go inside a subsys class too e.g.
    // public static final class OIConstants {}
    // public static final int kDrivXboxPort = 0;
  
 }  // end constant