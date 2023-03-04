package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightConnector {
    public static class LimelightResult {
        public double tx;
        public double ty;
        public double ta;
    }

    private final double getGenericDouble(String name, double def) {
        DoubleSubscriber p = this.table.getDoubleTopic(name).subscribe(def);
        double ret = p.get();
        p.close();
        return ret;
    }
    
    private final NetworkTable table;
    public LimelightConnector(String table) {
        this.table = NetworkTableInstance.getDefault().getTable(table);
        this.setLEDs(false);
    }
    public void setLEDs(boolean state) {
        DoublePublisher p = this.table.getDoubleTopic("ledMode").publish();
        p.set(state ? 3.0 : 1.0);
        p.close();
    }
    public void setPipeline(int pipeline) {
        DoublePublisher p = this.table.getDoubleTopic("pipeline").publish();
        p.set((double)pipeline);
        p.close();
    }
    public LimelightResult get() {
        LimelightResult result = new LimelightResult();
        if (getGenericDouble("tv", 0.0) < 0.5) {
            return null;
        }
        result.tx = getGenericDouble("tx", 0.0);
        result.ty = getGenericDouble("ty", 0.0);
        result.ta = getGenericDouble("ta", 0.0);
        return result;
    }
}