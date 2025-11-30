package org.firstinspires.ftc.teamcode.teleOp.util;

public enum Mosaic {
    PPG,
    GPP,
    PGP,
    UNKNOWN;

    public static Mosaic fromId(byte id) {
        switch (id) {
            case 21: return GPP;
            case 22: return PGP;
            case 23: return PPG;
            default: return UNKNOWN;
        }
    }
}
