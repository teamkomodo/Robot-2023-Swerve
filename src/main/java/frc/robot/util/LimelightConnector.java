package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightConnector {
    public static class LimelightResult {
        public double tx;
        public double ty;
        public double ta;
    }
    
    private final NetworkTable table;
    public LimelightConnector(String table) {
        this.table = NetworkTableInstance.getDefault().getTable(table);
    }
}