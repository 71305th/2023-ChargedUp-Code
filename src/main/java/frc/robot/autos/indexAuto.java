package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Index.indexStates;
import frc.robot.subsystems.Index.tilterPos;
import frc.robot.vision.settle;

public class indexAuto extends CommandBase{
    Index index;
    double desired;
    double current;
    double error;
    boolean finish;
    Timer clock = new Timer();
    double executeTime;
    double timeError, startime;
    indexStates states;
    settle ok = new settle();

    public indexAuto(Index index, indexStates states, double time){
        this.index = index;
        this.states = states;
        executeTime = time;
        addRequirements(index);
    }

    @Override
    public void initialize() {
        startime = Timer.getFPGATimestamp();
        
    }

    @Override
    public void execute() {
        index.setState(states);
        // current = index.getTilterPos();

        // switch(states) {
        //     case Indexing:
        //         desired = tilterPos.downlimit;
        //         break;
        //     case AimTop:
        //         desired = tilterPos.inCom;
        //         break;
        //     case Standby:
        //         desired = tilterPos.upLimit;
        //         break;
        //     default:
        //         break;
        // }

        // error = desired - current;
        timeError = Timer.getFPGATimestamp() - startime;
    
    }

    @Override
    public boolean isFinished() {
        // if(finish){
        //     return true;
        // }else{
        //     return false;
        // }
        if(timeError <= executeTime){
            return true;
        }else{
            return false;
        }
    }
    @Override
    public void end(boolean interrupted) {
        index.setRollers(0);
        index.setState(indexStates.Standby);
    }
}
