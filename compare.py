import glob
import math
import subprocess

dataset = glob.glob('D:/Dropbox/Research/Datasets/Sung/CAD-120/Subject 1/*.h5');
dataset.extend(glob.glob('D:/Dropbox/Research/Datasets/Sung/CAD-120/Subject 2/*.h5'));
dataset.extend(glob.glob('D:/Dropbox/Research/Datasets/Sung/CAD-120/Subject 3/*.h5'));
dataset.extend(glob.glob('D:/Dropbox/Research/Datasets/Sung/CAD-120/Subject 4/*.h5'));
dataset.remove('D:/Dropbox/Research/Datasets/Sung/CAD-120/Subject 4\\0504232829.h5');

precision = 0.0; recall = 0.0; time = 0.0;
masking_precision = 0.0; masking_recall = 0.0; masking_time = 0.0;
for sequence in dataset:
  results = sequence[sequence.rfind('\\') + 1:] + '-0.txt';
  masking_results = sequence[sequence.rfind('\\') + 1:] + '-1.txt';

  proc = subprocess.Popen(['build/applications/face_experiment/Release/face_experiment',
                           results, masking_results], stdout=subprocess.PIPE);
  output = proc.communicate()[0][:sequence.find('\r\n') - 1].split(',');

  precision += float(output[0]);
  recall += float(output[1]);
  time += float(output[2]);
  masking_precision += float(output[3]);
  masking_recall += float(output[4]);
  masking_time += float(output[5]);

precision /= len(dataset);
recall /= len(dataset);
time /= len(dataset);
masking_precision /= len(dataset);
masking_recall /= len(dataset);
masking_time /= len(dataset);

print 'Standard';
print 'Precision: ' + str(precision) + \
      ' Recall: ' + str(recall) + \
      ' Time: ' + str(time);
print 'Masking';
print 'Precision: ' + str(masking_precision) + \
      ' Recall: ' + str(masking_recall) + \
      ' Time: ' + str(masking_time);
print '';