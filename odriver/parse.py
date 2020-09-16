import re
import csv

with open('current_draw.txt', "r") as f:
	data=f.read()
	split_data = re.split('time: | current: ', data)
	for x in range(10):
		print(split_data[x])
	with open('current_draw.csv', "w", newline='') as out_csv:
		writer = csv.writer(out_csv)
		writer.writerow(["time", "current"])

		i = 1
		while (i > 0 and i < (len(split_data) - 1)):
			writer.writerow([split_data[i], split_data[i + 1]])
			print([split_data[i], split_data[i + 1]])
			i += 2

	

