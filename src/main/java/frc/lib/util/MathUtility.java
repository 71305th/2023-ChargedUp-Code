package frc.lib.util;

// By Chenry (7130 5th Prog Lead)
public class MathUtility {
    //private double encTpr = 42;
    
    //wheel to motor is 50 to 24 and on the 24 its 48 to 12
    //1 rotation on wheel = 2.083 rotation on mid gear
    //2.083 rotation on mid gear = 8.3 rotaions on motor
    //8.3 rotations = 8.3 * 42 ticks
    

    //private double ticksPerWheelRevolution = encTpr * 50 / 24 * 48 / 12;
    private double ticksPerWheelRevolution = 350;
    private double wheelCircumference = 4 * 2.54 * Math.PI / 100 ;

    public MathUtility(){};

    public double encToMeter(double encoderVal){
        return (encoderVal / ticksPerWheelRevolution * wheelCircumference);
    }   
    public static double degToRad(double degrees){
        return degrees / 180 * Math.PI;
    }
    public static double radToDeg(double radians){
        return radians / Math.PI * 180;
    }
    public static double constrainAngleDegrees(double angle){
    return Math.atan2(Math.sin(angle / 180.0 * 3.14159265358979323846), Math.cos(angle / 180.0 * 3.14159265358979323846)) * 180/3.14159265358979323846;
    }

    public static double clamp(double value, double min, double max){
        return Math.min(Math.max(value, min), max);
    }
    public static int clamp(int value, int min, int max){
        return Math.min(Math.max(value, min), max);
    }
    public static boolean isWithin(double value, double min, double max){
        return Math.max(min, value) == Math.min(value, max);
    }

}

