package com.team2383.lib.SLAM;

import java.util.function.BiFunction;

import org.ejml.simple.SimpleMatrix;

public class MotionModelJacobian implements BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> {
    private SimpleMatrix F_x;

    public MotionModelJacobian(SimpleMatrix F_x) {
        this.F_x = F_x;
    }

    @Override
    public SimpleMatrix apply(SimpleMatrix u, SimpleMatrix mu) {
        double dx = u.get(0);
        double dy = u.get(1);
        double dtheta = u.get(2);

        double rw = mu.get(3);
        double rx = mu.get(4);
        double ry = mu.get(5);
        double rz = mu.get(6);

        SimpleMatrix G = new SimpleMatrix(7, 7);

        G.set(0, 0);
        G.set(1, 0);
        G.set(2, 0);
        G.set(3, 2 * (2 * dx * rw * Math.pow(ry, 2) + 2 * dx * rw * Math.pow(rz, 2) + dy * Math.pow(rw, 2) * rz
                - 2 * dy * rw * rx * ry - dy * Math.pow(rx, 2) * rz - dy * Math.pow(ry, 2) * rz - dy * Math.pow(rz, 3))
                / (Math.pow(rw, 4) + 2 * Math.pow(rw, 2) * Math.pow(rx, 2) + 2 * Math.pow(rw, 2) * Math.pow(ry, 2)
                        + 2 * Math.pow(rw, 2) * Math.pow(rz, 2) + Math.pow(rx, 4)
                        + 2 * Math.pow(rx, 2) * Math.pow(ry, 2) + 2 * Math.pow(rx, 2) * Math.pow(rz, 2)
                        + Math.pow(ry, 4) + 2 * Math.pow(ry, 2) * Math.pow(rz, 2) + Math.pow(rz, 4)));
        G.set(4, 2 * (2 * dx * rx * Math.pow(ry, 2) + 2 * dx * rx * Math.pow(rz, 2) + dy * Math.pow(rw, 2) * ry
                + 2 * dy * rw * rx * rz - dy * Math.pow(rx, 2) * ry + dy * Math.pow(ry, 3) + dy * ry * Math.pow(rz, 2))
                / (Math.pow(rw, 4) + 2 * Math.pow(rw, 2) * Math.pow(rx, 2) + 2 * Math.pow(rw, 2) * Math.pow(ry, 2)
                        + 2 * Math.pow(rw, 2) * Math.pow(rz, 2) + Math.pow(rx, 4)
                        + 2 * Math.pow(rx, 2) * Math.pow(ry, 2) + 2 * Math.pow(rx, 2) * Math.pow(rz, 2)
                        + Math.pow(ry, 4) + 2 * Math.pow(ry, 2) * Math.pow(rz, 2) + Math.pow(rz, 4)));
        G.set(5, 2 * (-2 * dx * Math.pow(rw, 2) * ry - 2 * dx * Math.pow(rx, 2) * ry + dy * Math.pow(rw, 2) * rx
                + 2 * dy * rw * ry * rz + dy * Math.pow(rx, 3) - dy * rx * Math.pow(ry, 2) + dy * rx * Math.pow(rz, 2))
                / (Math.pow(rw, 4) + 2 * Math.pow(rw, 2) * Math.pow(rx, 2) + 2 * Math.pow(rw, 2) * Math.pow(ry, 2)
                        + 2 * Math.pow(rw, 2) * Math.pow(rz, 2) + Math.pow(rx, 4)
                        + 2 * Math.pow(rx, 2) * Math.pow(ry, 2) + 2 * Math.pow(rx, 2) * Math.pow(rz, 2)
                        + Math.pow(ry, 4) + 2 * Math.pow(ry, 2) * Math.pow(rz, 2) + Math.pow(rz, 4)));
        G.set(6, 2
                * (-2 * dx * Math.pow(rw, 2) * rz - 2 * dx * Math.pow(rx, 2) * rz - dy * Math.pow(rw, 3)
                        - dy * rw * Math.pow(rx, 2) - dy * rw * Math.pow(ry, 2) + dy * rw * Math.pow(rz, 2)
                        - 2 * dy * rx * ry * rz)
                / (Math.pow(rw, 4) + 2 * Math.pow(rw, 2) * Math.pow(rx, 2) + 2 * Math.pow(rw, 2) * Math.pow(ry, 2)
                        + 2 * Math.pow(rw, 2) * Math.pow(rz, 2) + Math.pow(rx, 4)
                        + 2 * Math.pow(rx, 2) * Math.pow(ry, 2) + 2 * Math.pow(rx, 2) * Math.pow(rz, 2)
                        + Math.pow(ry, 4) + 2 * Math.pow(ry, 2) * Math.pow(rz, 2) + Math.pow(rz, 4)));
        G.set(7, 0);
        G.set(8, 0);
        G.set(9, 0);
        G.set(10,
                2 * (-dx * Math.pow(rw, 2) * rz - 2 * dx * rw * rx * ry + dx * Math.pow(rx, 2) * rz
                        + dx * Math.pow(ry, 2) * rz + dx * Math.pow(rz, 3) + 2 * dy * rw * Math.pow(rx, 2)
                        + 2 * dy * rw * Math.pow(rz, 2))
                        / (Math.pow(rw, 4) + 2 * Math.pow(rw, 2) * Math.pow(rx, 2)
                                + 2 * Math.pow(rw, 2) * Math.pow(ry, 2) + 2 * Math.pow(rw, 2) * Math.pow(rz, 2)
                                + Math.pow(rx, 4) + 2 * Math.pow(rx, 2) * Math.pow(ry, 2)
                                + 2 * Math.pow(rx, 2) * Math.pow(rz, 2) + Math.pow(ry, 4)
                                + 2 * Math.pow(ry, 2) * Math.pow(rz, 2) + Math.pow(rz, 4)));
        G.set(11, 2
                * (dx * Math.pow(rw, 2) * ry - 2 * dx * rw * rx * rz - dx * Math.pow(rx, 2) * ry + dx * Math.pow(ry, 3)
                        + dx * ry * Math.pow(rz, 2) - 2 * dy * Math.pow(rw, 2) * rx - 2 * dy * rx * Math.pow(ry, 2))
                / (Math.pow(rw, 4) + 2 * Math.pow(rw, 2) * Math.pow(rx, 2) + 2 * Math.pow(rw, 2) * Math.pow(ry, 2)
                        + 2 * Math.pow(rw, 2) * Math.pow(rz, 2) + Math.pow(rx, 4)
                        + 2 * Math.pow(rx, 2) * Math.pow(ry, 2) + 2 * Math.pow(rx, 2) * Math.pow(rz, 2)
                        + Math.pow(ry, 4) + 2 * Math.pow(ry, 2) * Math.pow(rz, 2) + Math.pow(rz, 4)));
        G.set(12, 2
                * (dx * Math.pow(rw, 2) * rx - 2 * dx * rw * ry * rz + dx * Math.pow(rx, 3) - dx * rx * Math.pow(ry, 2)
                        + dx * rx * Math.pow(rz, 2) + 2 * dy * Math.pow(rx, 2) * ry + 2 * dy * ry * Math.pow(rz, 2))
                / (Math.pow(rw, 4) + 2 * Math.pow(rw, 2) * Math.pow(rx, 2) + 2 * Math.pow(rw, 2) * Math.pow(ry, 2)
                        + 2 * Math.pow(rw, 2) * Math.pow(rz, 2) + Math.pow(rx, 4)
                        + 2 * Math.pow(rx, 2) * Math.pow(ry, 2) + 2 * Math.pow(rx, 2) * Math.pow(rz, 2)
                        + Math.pow(ry, 4) + 2 * Math.pow(ry, 2) * Math.pow(rz, 2) + Math.pow(rz, 4)));
        G.set(13,
                2 * (dx * Math.pow(rw, 3) + dx * rw * Math.pow(rx, 2) + dx * rw * Math.pow(ry, 2)
                        - dx * rw * Math.pow(rz, 2) - 2 * dx * rx * ry * rz - 2 * dy * Math.pow(rw, 2) * rz
                        - 2 * dy * Math.pow(ry, 2) * rz)
                        / (Math.pow(rw, 4) + 2 * Math.pow(rw, 2) * Math.pow(rx, 2)
                                + 2 * Math.pow(rw, 2) * Math.pow(ry, 2) + 2 * Math.pow(rw, 2) * Math.pow(rz, 2)
                                + Math.pow(rx, 4) + 2 * Math.pow(rx, 2) * Math.pow(ry, 2)
                                + 2 * Math.pow(rx, 2) * Math.pow(rz, 2) + Math.pow(ry, 4)
                                + 2 * Math.pow(ry, 2) * Math.pow(rz, 2) + Math.pow(rz, 4)));
        G.set(14, 0);
        G.set(15, 0);
        G.set(16, 0);
        G.set(17,
                (rw * (2 * rw * (dx * ry - dy * rx) - rx * (dx * rz + dy * rw) + ry * (dx * rw - dy * rz)
                        - 2 * rz * (dx * rx + dy * ry))
                        - rx * (-dy * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                                + rw * (dx * rz + dy * rw))
                        - ry * (dx * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                                + rw * (-dx * rw + dy * rz))
                        + (-dx * ry + dy * rx)
                                * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        G.set(18,
                (rw * (dy * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                        + rx * (dx * ry - dy * rx))
                        + rx * (rw * (dx * ry - dy * rx) - 2 * rx * (dx * rz + dy * rw) + 2 * ry * (dx * rw - dy * rz)
                                - rz * (dx * rx + dy * ry))
                        - rz * (-dx * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                                + rx * (dx * rx + dy * ry))
                        + (dx * rz + dy * rw) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        G.set(19,
                (-rw * (dx * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                        + ry * (-dx * ry + dy * rx))
                        + ry * (rw * (dx * ry - dy * rx) - 2 * rx * (dx * rz + dy * rw) + 2 * ry * (dx * rw - dy * rz)
                                - rz * (dx * rx + dy * ry))
                        - rz * (-dy * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                                + ry * (dx * rx + dy * ry))
                        + (-dx * rw + dy * rz)
                                * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        G.set(20,
                (-rx * (-dx * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                        + rz * (dx * rz + dy * rw))
                        + ry * (dy * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                                + rz * (dx * rw - dy * rz))
                        + rz * (2 * rw * (dx * ry - dy * rx) - rx * (dx * rz + dy * rw) + ry * (dx * rw - dy * rz)
                                - 2 * rz * (dx * rx + dy * ry))
                        + (dx * rx + dy * ry) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        G.set(21, 0);
        G.set(22, 0);
        G.set(23, 0);
        G.set(24, (-Math.pow(rw, 4) - 2 * Math.pow(rw, 2) * Math.pow(rz, 2)
                + rw * rz * Math.sqrt(Math.pow(rw, 2) + Math.pow(rz, 2)) * Math.sin((1.0 / 2.0) * dtheta)
                - Math.pow(rz, 4)
                + Math.pow(rz, 2) * Math.sqrt(Math.pow(rw, 2) + Math.pow(rz, 2)) * Math.cos((1.0 / 2.0) * dtheta))
                / Math.pow(Math.pow(rw, 2) + Math.pow(rz, 2), 2));
        G.set(25, 0);
        G.set(26, 0);
        G.set(27, -rw * (rw * Math.sin((1.0 / 2.0) * dtheta) + rz * Math.cos((1.0 / 2.0) * dtheta))
                / Math.pow(Math.pow(rw, 2) + Math.pow(rz, 2), 3.0 / 2.0));
        G.set(28, 0);
        G.set(29, 0);
        G.set(30, 0);
        G.set(31, 0);
        G.set(32, -1);
        G.set(33, 0);
        G.set(34, 0);
        G.set(35, 0);
        G.set(36, 0);
        G.set(37, 0);
        G.set(38, 0);
        G.set(39, 0);
        G.set(40, -1);
        G.set(41, 0);
        G.set(42, 0);
        G.set(43, 0);
        G.set(44, 0);
        G.set(45, rz * (-rw * Math.cos((1.0 / 2.0) * dtheta) + rz * Math.sin((1.0 / 2.0) * dtheta))
                / Math.pow(Math.pow(rw, 2) + Math.pow(rz, 2), 3.0 / 2.0));
        G.set(46, 0);
        G.set(47, 0);
        G.set(48, (-Math.pow(rw, 4) - 2 * Math.pow(rw, 2) * Math.pow(rz, 2)
                + Math.pow(rw, 2) * Math.sqrt(Math.pow(rw, 2) + Math.pow(rz, 2)) * Math.cos((1.0 / 2.0) * dtheta)
                - rw * rz * Math.sqrt(Math.pow(rw, 2) + Math.pow(rz, 2)) * Math.sin((1.0 / 2.0) * dtheta)
                - Math.pow(rz, 4)) / Math.pow(Math.pow(rw, 2) + Math.pow(rz, 2), 2));

        return F_x.transpose().mult(G).mult(F_x);
    }
}
