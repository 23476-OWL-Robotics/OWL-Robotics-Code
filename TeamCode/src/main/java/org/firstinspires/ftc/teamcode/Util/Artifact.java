package org.firstinspires.ftc.teamcode.Util;

public class Artifact {

    ArtifactType type;
    int slot;

    public Artifact(ArtifactType type, int slot) {
        this.type = type;
        this.slot = slot;
    }

    public ArtifactType getType() {
        return type;
    }
    public int getSlot() {
        return slot;
    }

    public int isType(ArtifactType t) {
        int value = 0;
        if (type == t) {
            value += 1;
        }
        return value;
    }
}
