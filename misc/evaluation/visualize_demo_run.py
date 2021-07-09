import sys
import os
import imageio
import matplotlib
import matplotlib.pyplot as plt
import gc
import time

fname = sys.argv[1]
framecount = None if len(sys.argv) < 3 else int(sys.argv[2])
FPS = 20 if fname != "testchamber" else 1
if len(sys.argv) >= 5:
    FPS = int(sys.argv[4])
sleepTime = 1/float(FPS)

with open(fname) as f:
    alltext = f.read()

frametexts = alltext.split("\033[1;1H\033[2J")
frames = []
performances = []
framesexisting = 0
for i, frame in enumerate(frametexts):
    if "ratio=" not in frame:
        continue
    framesexisting += 1
    if "JustASCII" in sys.argv:
        frames.append(frame)
    else:
        if fname == "testchamber":
            frame = frame.split("(same command)\n\n")[1]
        frames.append(frame.split("Input:")[0].split("position")[0].split("shots")[0].split("\n\nCommand:")[0].split(", ratio=")[0])
    performances.append(float(frame.split("ratio=")[1].split(" ")[0].split(",")[0]))

filenames = []
for i,frame in enumerate(frames):
    if not framecount is None and i>framecount:
        break
    if "JustASCII" not in sys.argv:
        fig, axs = plt.subplots(2)
        axs[0].set_title(fname)
        v_padding = 1
        h_padding = 0.33
        axs[0].axis([0, 1, 0, 10])
        fontsize = 4
        if fname == "alien":
            fontsize = 10
        elif fname == "cartpole":
            fontsize = 12
        elif fname == "testchamber":
            fontsize = 9
        
        axs[0].text(h_padding, v_padding, frame, fontfamily = "monospace", fontsize=fontsize, bbox=dict(facecolor='gray', alpha=0.15))
        axs[0].set_yticklabels([])
        axs[0].set_xticklabels([])
        axs[1].axis([0, framesexisting, 0, 1])
        axs[1].set_xlabel('time')
        axs[1].set_ylabel('Success ratio')
        axs[1].set_title("Performance")
        axs[1].plot(performances[:i])
        filename = fname + "_" + str(i) + ".png"
        plt.savefig(filename)
        filenames.append(filename)
        plt.cla() 
        plt.clf()
        plt.close('all')   
        plt.close(fig)
        gc.collect()
    else:
        time.sleep(sleepTime)
    print("\033[1;1H\033[2J")
    print(frame)
    print("\nRatio: " + str(performances[i]))
    print("Progress: " + str(100.0*float(i)/float(framecount if not framecount is None else framesexisting)) + "%")

# build gif
with imageio.get_writer(fname + '.gif', mode='I', fps=FPS) as writer:
    for filename in filenames:
        image = imageio.imread(filename)
        writer.append_data(image)
        
# Remove files
for filename in set(filenames):
    os.remove(filename)
