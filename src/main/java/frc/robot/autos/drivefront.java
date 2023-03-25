package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.pid.SetPointPID;
import frc.lib.util.MathUtility;
import frc.lib.util.SettledUtility;
import frc.robot.subsystems.Swerve;
import frc.robot.vision.settle;

public class drivefront extends CommandBase{
    Swerve drive;
    double targetDis;
    double currentDistance;
    double error;
    double p = 1.55;
    double i = 0.8;
    double d = 0;
    double output;
    double startY;
    double startGYRO;
    SetPointPID frontPID;
    SettledUtility settled;
    settle ok = new settle();
    boolean finish;

    public drivefront(Swerve swerve, double targetDis){
        drive = swerve;
        this.targetDis = targetDis;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        frontPID = new SetPointPID(p, i, d, targetDis, 1);//
        startY = drive.getPose().getY();
        startGYRO = drive.getYaw().getRotations();
    }

    @Override
    public void execute() {
        currentDistance = drive.getPose().getX();
        error = targetDis - currentDistance;
        // settled = new SettledUtility(100, error, 0.1);
        output = MathUtility.clamp(frontPID.calculate(error), -2.5, 2.5) ;
        // finish = settled.isSettled(error);
        finish = ok.OKsettle(error, 0.05);

        drive.drive(new Translation2d(output, 0), 0, false, true);

        SmartDashboard.putNumber("autoY", drive.getPose().getY());
        SmartDashboard.putNumber("Yerror", drive.getPose().getY() - startY);
        SmartDashboard.putNumber("autoGYRO", drive.getYaw().getRotations());
        SmartDashboard.putNumber("GYROerror", startGYRO - drive.getYaw().getRotations());

        SmartDashboard.putNumber("autoFrontX", currentDistance);
        SmartDashboard.putNumber("autoFrontOutput", output);
        SmartDashboard.putNumber("autoFrontError", error);
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
        //drive.reset();
    }


}
