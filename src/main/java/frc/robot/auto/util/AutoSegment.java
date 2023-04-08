package frc.robot.auto.util;

public class AutoSegment {
    protected AutoTemplate baseTemplate;
    protected AutoSegment onSuccess = null;
    protected AutoSegment onFailure = null;
    public boolean requirementsAdded = false;

    public static AutoSegment generateUnconditionalSequence(AutoTemplate... templates) {
        AutoSegment head = new AutoSegment(templates[0]);
        AutoSegment tail = head;
        for (int i = 1; i < templates.length; i++) {
            AutoSegment newseg = new AutoSegment(templates[i]);
            tail.onSuccessOrFailure(newseg);
            tail = newseg;
        }
        return head;
    }

    public static AutoSegment generateSensitiveSequence(AutoTemplate... templates) {
        AutoSegment head = new AutoSegment(templates[0]);
        AutoSegment tail = head;
        for (int i = 1; i < templates.length; i++) {
            AutoSegment newseg = new AutoSegment(templates[i]);
            tail.onSuccess(newseg);
            tail = newseg;
        }
        return head;
    }

    public AutoSegment(AutoTemplate template) {
        this.baseTemplate = template;
    }

    public AutoSegment onSuccess(AutoSegment segment) {
        this.onSuccess = segment;
        return this;
    }

    public AutoSegment onFailure(AutoSegment segment) {
        this.onFailure = segment;
        return this;
    }

    public AutoSegment onSuccessOrFailure(AutoSegment segment) {
        this.onSuccess = segment;
        this.onFailure = segment;
        return this;
    }
}
