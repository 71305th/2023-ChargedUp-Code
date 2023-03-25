package frc.lib.util;

// By Chenry (7130 5th Prog Lead)
public class SettledUtility {
    
    private double atTargetTime = 200;
    private double atTargetError = 10;
    private double atTargetDerivative = 5;
    private double lastError = 0;
    private int atTargetCounter;

    public SettledUtility (double targetTime, double targetError, double targetDerivative){
        atTargetTime = targetTime;
        atTargetError = targetError;
        atTargetDerivative = targetDerivative;
    }

    public boolean isSettled (double error){
        if (Math.abs(error) <= atTargetError && Math.abs(error - lastError) <= atTargetDerivative){
            atTargetCounter++;
        }
        else atTargetCounter = 0;
        lastError = error;
        return (atTargetCounter * 20) > atTargetTime;
    }
};