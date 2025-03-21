// romiGyroDrivPID - D                          CONSTANT.J

package frc.robot;

public final class Constant {

    // these value work with 10 hz sample period
    public static final double kDistP = 0.5;
    public static final double kDistI = 0.005;
    public static final double kDistD = 0.00;

    // to stabiliz [auto and ?] teleop straight drive
    public static final double kStabilP = 0.15;
    public static final double kStabilI = 0.0;
    public static final double kStabilD = 0.0;

    // hot edit not reliable, better if Disconnected ?
    public static final double kTurnP = 0.12;
    public static final double kTurnI = 0.00;
    public static final double kTurnD = 0.00;

    // pid update freq in sec.
    public static final double kPIDperiod = 0.10;

    // public static final double kMaxTurnVeloc = 20;
    // public static final double kMaxTurnAcceler = 20;

    // public static final double kTurnTolerDeg = 2;
    // public static final double kTurnTolerVeloc = 10;
    // deg per second

    // inner class for multiple constant or a single constant
    // could go inside a subsys class too e.g.
    // public static final class OIConstants {}
    // public static final int kDrivXboxPort = 0;

} // end constant