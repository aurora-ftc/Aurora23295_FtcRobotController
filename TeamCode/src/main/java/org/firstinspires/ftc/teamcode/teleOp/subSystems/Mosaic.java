package org.firstinspires.ftc.teamcode.teleOp.subSystems;

public enum Mosaic {
    PPG,
    GPP,
    PGP,
    UNKNOWN;

    public static Mosaic fromId(byte id) {
        switch (id) {
            case 21:
                return GPP;  // example
            case 22:
                return PPG;  // example
            case 23:
                return PGP;  // example
            default:
                return UNKNOWN;
        }
    }
}
