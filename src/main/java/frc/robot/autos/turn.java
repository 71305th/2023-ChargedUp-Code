package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.ChenryLib.SetPointPID;
import frc.ChenryLib.SettledUtility;
import frc.robot.subsystems.Swerve;

public class turn extends CommandBase {
    Swerve drive;
    double rotateTarget;
    double currentRotation;
    double error;
    double output;
    double p = 0.001;
    double i = 0;
    double d = 0;
    SetPointPID turnPID;
    SettledUtility settled;
    boolean finish;
    

    public turn(Swerve swerve, double targetDegrees){
        rotateTarget = targetDegrees;
        drive = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        currentRotation = drive.getYaw().getRotations();
    }

    @Override
    public void execute() {
        currentRotation = drive.getYaw().getDegrees();
        error = rotateTarget - currentRotation;
        double mappedError = MathUtility.constrainAngleDegrees(error);
        turnPID = new SetPointPID(p, i, d, 15, 1);//😱😱😱
        settled = new SettledUtility(100, 5, 10);
        output = MathUtility.clamp(turnPID.calculate(mappedError), -3, 3) ;
        finish = settled.isSettled(mappedError);
        
        drive.drive(new Translation2d(0,0), output, false, true);
        
        SmartDashboard.putNumber("autoTurnYaw", currentRotation);
        SmartDashboard.putNumber("autoTurnOutput", output);
        SmartDashboard.putNumber("autoTurnError",error);
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
        drive.drive(new Translation2d(0, 0), 0, false, true);
        drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0, 0)));
    }
}
