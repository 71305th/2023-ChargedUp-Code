package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.ChenryLib.SettledUtility;
import frc.lib.vision.settle;
import frc.robot.subsystems.Swerve;

public class driveSide extends CommandBase{
    Swerve drive;
    double targetDis;
    double currentDistance;
    double error;
    double p = 1.2;
    double i = 0.001;
    double d = 0.5;
    double output;
    boolean finish;
    PID sidePID;
    SettledUtility settled;
    settle ok = new settle();

    public driveSide(Swerve swerve, double targetDis){
        drive = swerve;
        this.targetDis = targetDis;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        sidePID = new PID(p, i, d, targetDis, 1);//😱😱😱😱😱
    }

    @Override
    public void execute() {
        currentDistance = drive.getPose().getY();
        error = targetDis - currentDistance;
        output = MathUtility.clamp(sidePID.calculate(error), -2.5, 2.5);
        // settled = new SettledUtility(100, error, 0.1);
        // finish = settled.isSettled(error);
        finish = ok.OKsettle(error, 0.01);

        drive.drive(new Translation2d(0, output), 0, false, true);

        SmartDashboard.putNumber("autoSideY", currentDistance);
        SmartDashboard.putNumber("autoSideOutput", output);
        SmartDashboard.putNumber("autoSideError", error);
    }

    @Override
    public boolean isFinished() {
        if(finish){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(0, 0), 0, false, false);
        drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0, 0)));
    }


}
