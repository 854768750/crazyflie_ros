import csv
import tf
with open('test.csv','rb') as myfile:
	reader=csv.reader(myfile)
	lines = [line for line in reader]
print int(lines[2][1])+int(lines[2][2])
quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
print quaternion
