import ntcore
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("SLAM")
    xSub = table.getDoubleArrayTopic("sigma").subscribe([0] * (63 * 63))
    inst.startClient4("example client")
    inst.setServer("127.0.0.1", 0)

    fig, ax = plt.subplots()

    while True:
        val = xSub.get()
        val = np.matrix(val).reshape((63, 63))
        plot = ax.matshow(val, cmap=plt.cm.Blues)
        cb = fig.colorbar(plot)
        fig.canvas.draw()
        plt.pause(0.01)
        fig.canvas.flush_events()
        cb.remove()
