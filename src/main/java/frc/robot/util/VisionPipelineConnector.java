package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionPipelineConnector {
    public static class Target {
        public Target(double[] data) {
            centerX = data[0];
            centerY = data[1];
            width = data[2];
            height = data[3];
        }

        public double centerX;
        public double centerY;
        public double width;
        public double height;
    }

    private final NetworkTable table;
    private final NetworkTableInstance inst;

    public VisionPipelineConnector(String dataTable) {
        this.inst = NetworkTableInstance.getDefault();
        this.table = inst.getTable(dataTable);
    }

    public void setEnabled(boolean state) {
        BooleanPublisher p = this.table.getBooleanTopic("enabled").publish();
        p.set(state);
        p.close();
    }

    public void setOneshot(boolean state) {
        BooleanPublisher p = this.table.getBooleanTopic("oneshot").publish();
        p.set(state);
        p.close();
    }

    public void triggerOneshot() {
        BooleanPublisher p = this.table.getBooleanTopic("oneshot_trigger").publish();
        p.set(true);
        p.close();
    }

    public boolean isReady() {
        BooleanSubscriber s = this.table.getBooleanTopic("ready").subscribe(false);
        boolean result = s.get();
        s.close();
        return result;
    }

    public long getNumTargets() {
        IntegerSubscriber s = this.table.getIntegerTopic("num_targets").subscribe(0);
        long result = s.get();
        s.close();
        return result;
    }

    public Target[] getAllTargets() {
        long num_targets = getNumTargets();
        Target[] ret = new Target[(int) num_targets];
        for (long i = 0; i < num_targets; i++) {
            DoubleArraySubscriber s = this.table.getDoubleArrayTopic("TargetData/Target" + i).subscribe(null);
            double[] data = s.get();
            if (data == null) {
                ret[(int) i] = null;
            } else {
                ret[(int) i] = new Target(data);
            }
            s.close();
        }
        return ret;
    }
}
