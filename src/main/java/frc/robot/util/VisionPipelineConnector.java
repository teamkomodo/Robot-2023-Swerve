package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionPipelineConnector {
    public static class VisionPipelineTest extends CommandBase {
        private final VisionPipelineConnector connector;

        public VisionPipelineTest(VisionPipelineConnector connector) {
            this.connector = connector;
        }

        @Override
        public void initialize() {
            connector.setEnabled(true);
            connector.setOneshot(false);
        }

        @Override
        public void execute() {
            Target[] targets = connector.getAllTargets();
            StringBuilder s = new StringBuilder(); // Oooooh...
            if (targets == null) {
                s.append("Device not ready.");
            } else {
                s.append("Got " + targets.length + " targets: [");
                for (Target t : targets) {
                    s.append("{" + t.centerX + ", " + t.centerY + ", " + t.width + ", " + t.height + "}, ");
                }
                s.append("]");
            }
            SmartDashboard.putString("Vision pipeline result", s.toString());
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            connector.setEnabled(false);
        }
    }

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

        public double getArea() {
            return width * height;
        }

        public boolean isValid() {
            if (this.getArea() <= 0) {
                return false;
            } else if (this.centerX < -1.0 || this.centerX > 1.0) {
                return false;
            } else if (this.centerY < -1.0 || this.centerY > 1.0) {
                return false;
            }
            return true;
        }
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
        if (!isReady()) {
            return null;
        }
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

    public Target getLargestTarget() {
        Target[] targets = getAllTargets();
        if (targets == null) {
            return null;
        }
        Target ret = null;
        for (Target t : targets) {
            if (t == null)
                continue;
            if (ret == null || t.getArea() >= ret.getArea()) {
                ret = t;
            }
        }
        return ret;
    }
}
