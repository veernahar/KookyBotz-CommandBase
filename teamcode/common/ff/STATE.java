package org.firstinspires.ftc.teamcode.common.ff;

public enum STATE {
    INTAKE, REST, OUTTAKE;

    public STATE next() {
        switch (this) {
            case INTAKE:
                return REST;
            case REST:
                return OUTTAKE;
            default:
                return INTAKE;
        }
    }
}
