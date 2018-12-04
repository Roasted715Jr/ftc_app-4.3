package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class CloseableVuforiaLocalizer extends VuforiaLocalizerImpl {
    private boolean closed = false;
    private boolean paused = false;

    CloseableVuforiaLocalizer(Parameters parameters) {
        super(parameters);
    }

    @Override
    public void close() {
        if (!closed)
            super.close();
        closed = true;
    }

    @Override
    public void pauseAR() {
        if (!paused)
            super.pauseAR();
        paused = true;
    }

    @Override
    public void resumeAR() {
        if (paused)
            super.resumeAR();
        paused = false;
    }
}
