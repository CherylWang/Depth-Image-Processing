import glob
import math
import subprocess

dataset = glob.glob('D:/Dropbox/Research/Datasets/Sung/CAD-120/Subject 1/*.h5');
dataset.extend(glob.glob('D:/Dropbox/Research/Datasets/Sung/CAD-120/Subject 2/*.h5'));
dataset.extend(glob.glob('D:/Dropbox/Research/Datasets/Sung/CAD-120/Subject 3/*.h5'));
dataset.extend(glob.glob('D:/Dropbox/Research/Datasets/Sung/CAD-120/Subject 4/*.h5'));

for sequence in dataset:
  for masking in range(0, 2):
    output = open(sequence[sequence.rfind('\\') + 1:] + '-' + str(masking) +
                  '.txt', 'w');
    subprocess.call(['build/applications/face_detection/Release/face_detection',
                     str(masking), sequence], stdout = output);
