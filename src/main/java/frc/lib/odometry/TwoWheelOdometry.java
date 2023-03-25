package frc.lib.odometry;

import frc.lib.util.MathUtility;

// By Chenry (7130 5th Prog Lead)
public class TwoWheelOdometry {
    double chassisWidth = 1;
    double lastLeft = 0;
    double lastRight = 0;
    MathUtility mathUtil = new MathUtility();
    double x =0;
    double y = 0;
    double headingRad = 0;

    public TwoWheelOdometry(double ichassisWidth){
        chassisWidth = ichassisWidth;
    }
    public void update(double leftValue, double rightValue) {
    
        double deltaLeft = mathUtil.encToMeter(leftValue - lastLeft);
        double deltaRight = mathUtil.encToMeter(rightValue - lastRight);
    
        lastLeft = leftValue;
        lastRight = rightValue;
    
        double deltaS = (deltaLeft + deltaRight) / 2.0;
        double deltaTheta = (deltaLeft - deltaRight) / chassisWidth;
        
        double r = deltaTheta == 0 ? 0 : deltaS / deltaTheta;
        
        double dx = deltaTheta == 0? deltaS : (r * Math.sin(deltaRight));
        double dy = deltaTheta == 0 ? 0 : (r * Math.cos(deltaTheta) - r);

        double fx = dx * Math.cos(headingRad) - dy * Math.sin(headingRad);
        double fy = dx * Math.sin(headingRad) + dy * Math.cos(headingRad);
        
        x += fx;
        y += fy;
        headingRad += deltaTheta;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getHeadingDegrees(){
        return MathUtility.radToDeg(headingRad);
    }
    public double getHeadingRadians(){
        return headingRad;
    }

    //just pass left enc val, right enc val and then set angle to 0;
    public void initialize(double initLeft, double initRight, double initAngle){
        lastLeft = initLeft;
        lastRight = initRight;
        headingRad = MathUtility.degToRad(initAngle);
    }
}
