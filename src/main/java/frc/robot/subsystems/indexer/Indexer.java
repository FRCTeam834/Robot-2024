package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
//import the motor once it's been made maybe
import frc.robot.utility.LoggedTunableNumber;

public class Indexer {
    private boolean holdingNote;
    private boolean movingNote;
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final Intake intake;
    
    public Indexer (IndexerIO io, Intake intake) {
        this.io = io;
        this.intake = intake;
    }

    public void periodic () { 
        //does nothing
    }

    public void startIndexer() {
        io.setRollerVoltage(IndexerConstants.VOLTAGE_LIMIT);//update this with the right voltage (zap)
        movingNote=true;
    }
    
    public void slowIndexer(){
        io.setRollerVoltage(IndexerConstants.VOLTAGE_LIMIT/2);//I assume this isn't too slow
    }
    
    public void stopIndexer(){
        io.setRollerVoltage(0.0);
    }
}  