import re
import csv

with open('current_draw.txt', "r") as f:
	data=f.read()
	split_data = re.split('time: | current: ', data)

	with open('current_draw.csv', "w", newline='') as out_csv:
		writer = csv.writer(out_csv)
		writer.writerow(["time", "current"])
		for i in range(1, len(split_data) - 1, 2):
			writer.writerow([split_data[i], split_data[i + 1]])
			