package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
//import the motor once it's been made maybe
import frc.robot.utility.LoggedTunableNumber;

public class Indexer {
    private final int index;
    public boolean holdingNote = false;
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final Intake intake;
    
    public Indexer (IndexerIO io, int index, Intake intake) {
        this.io = io;
        this.index = index;
        this.intake = intake;
    }

    public void periodic () {
        if(holdingNote){
            //tell the intake to get a life
            intake.stopIntake();
        } else {
            //tell the intake to get back to work
            intake.startIntake();
        }
    }

    public void moveNote() {
        io.setRollerVoltage(IndexerConstants.VOLTAGE_LIMIT);//update this with the right voltage (zap)
    }
    
    public void stopIndexer(){
        io.setRollerVoltage(0.0);
    }
}  