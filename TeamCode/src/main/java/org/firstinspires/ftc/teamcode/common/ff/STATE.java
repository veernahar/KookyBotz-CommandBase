package org.firstinspires.ftc.teamcode.common.ff;

public enum STATE {
    INTAKE, REST;

    public STATE next() {
        return this == STATE.INTAKE ? REST : INTAKE;
    }
}
