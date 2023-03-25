package frc.lib.motion;

// By Chenry (7130 5th Prog Lead)
public class FeedForward {
        private double kA, kV, kP_Pos, kP_Vel, kS;
        
        public FeedForward (double ikV, double ikA, double ikP_Pos, double ikP_Vel, double ikS){
            kV = ikV;
            kA = ikA;
            kP_Pos = ikP_Pos;
            kP_Vel = ikP_Vel;
            kS = ikS;
        }
        
        public double calculate(double vel, double accel, double posError, double velError){
            return (kV * vel + kA * accel + kP_Pos * posError + kP_Vel * velError + kS * Math.signum(vel));
        }
        
        public void setGains(double ikV, double ikA, double ikP_Pos, double ikP_Vel, double ikS){
            kV = ikV;
            kA = ikA;
            kP_Pos = ikP_Pos;
            kP_Vel = ikP_Vel;
            kS = ikS;
        }
}
