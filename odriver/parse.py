import re
import csv

with open('current_draw.txt', "r") as f:
	data=f.read()
	split_data = re.split('time: |current: ', data)

	with open('current_draw.csv', "w", newline='') as out_csv:
		writer = csv.writer(out_csv)
		writer.writerow(["time", "current_draw"])

		for i in range(0, len(split_data) - 1):
			writer.writerow([split_data[i], split_data[i + 1]])

	

