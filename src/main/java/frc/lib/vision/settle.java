package frc.lib.vision;

import edu.wpi.first.wpilibj.Timer;

public class settle {
    double time;//seconds
    double errorLimit;
    double errorAmount = 0;
    double error;
    Timer clock;

    public boolean OKsettle(double error, double errorLimit){
        this.error = error;
        this.errorLimit = errorLimit;

        if (error <= errorLimit){
            return true;
        }else{
            return false;
    }
    }
}
