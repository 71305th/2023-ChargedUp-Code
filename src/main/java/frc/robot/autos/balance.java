package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.pid.PID;
import frc.lib.pid.SetPointPID;
import frc.lib.util.MathUtility;
import frc.robot.subsystems.Swerve;;

public class balance extends CommandBase{
    Swerve drive;
    double output;
    double balanceSpeed = 0.2;
    double currentFrontPitch, currentBackPitch;
    double last, error;
    PID balancePID;
    double p = 5;
    double i = 0;
    double d = 0;
    double settle = 1.5;//degrees
    double define;

    public balance(Swerve swerve){
        drive = swerve;
        addRequirements(swerve);
    }
    @Override
    public void initialize() {
        last = drive.getBackRoll();
    }

    @Override
    public void execute() {
        currentFrontPitch = drive.getBackRoll();
        error = currentFrontPitch - last;
        define = currentFrontPitch > 0 ? -1:1;
        balancePID = new PID(p, i, d, 0, 1);
        output =MathUtility.clamp(balancePID.calculate(error), -0.5, 0.5);

        drive.drive(new Translation2d(output*define, 0), 0, false, true);

        


        SmartDashboard.putNumber("currentROLL", currentFrontPitch);
        SmartDashboard.putNumber("rollError", error);
        SmartDashboard.putNumber("balanceOutput", output);

        last = currentFrontPitch;


    }

    @Override
    public boolean isFinished() {
      if(error <= settle){
        return true;
      }else{
        return false;
      }
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(0, 0), 0, false, false);
    }
}
