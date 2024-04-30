package frc.robot;

public class Constants {
    // Robot constants
    public final static double updatePeriod = 0.02;// in seconds
    // Motor IDs TODO motor IDs
    public static int FLDriveM = 1;
    public static int FLAngleM = 2;
    public static int FLAngleEnc = 3;

    public static int FRDriveM = 4;
    public static int FRAngleM = 5;
    public static int FRAngleEnc = 6;

    public static int BLDriveM = 7;
    public static int BLAngleM = 8;
    public static int BLAngleEnc = 9;

    public static int BRDriveM = 10;
    public static int BRAngleM = 11;
    public static int BRAngleEnc = 12;
    // PID/FF values TODO PID values
    public static double driveSwerveP = 0;
    public static double driveSwerveI = 0;
    public static double driveSwerveD = 0;

    public static double angleSwerveP = 0;
    public static double angleSwerveI = 0;
    public static double angleSwerveD = 0;

    public static double driveSwerveSFF = 0;
    public static double driveSwerveVFF = 0;
    public static double driveSwerveAFF = 0;

    public static double angleSwerveSFF = 0;
    public static double angleSwerveVFF = 0;
    public static double angleSwerveAFF = 0;
    // Speeds
    public static double maxSwerveAngleSpeed = 2;
    public static double maxSwerveAngleAccel = 2;

    public final static double kMaxSpeed = 5;
    public final static double kMaxAngularSpeed = 3;

    // Gear Ratios
    public static double driveGearRatio = 1;// TODO change this to gear ratio
    public static double wheelCirc = 12.56637;
}
