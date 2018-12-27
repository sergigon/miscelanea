import csv
# Tutorial: https://pythonprogramming.net/reading-csv-files-python-3/

# Input: Weather Input, Output: X Json Request
# Constants
weather_input = 'weather_input'
weather_type_common = 'common'
# Variables
weather_type = 'current' # Weather Type: 'forecast' or 'current'
json_request_source = 'apixu' # Json Request Source: 'apixu', ...
info_required = 'avgtemp_c' # Info Required: 'date', 'temp_c', ...
info_output = ''

with open('weather_list.csv') as csvfile:
	csv_reader = csv.reader(csvfile, delimiter=',')
	
	############ Find csv information ############
	weather_input_col = -1 # Weather Input column
	json_request_source_col = -1 #  Json Request Source column
	weather_type_row = -1 # Weather Type column
	weather_type_common_row = -1 # Weather Common Type column
	row_len = -1 # Number of rows

	row_n = 0
	for row in csv_reader:
		col_n = 0
		for cell in row:
			# Get weather input and output columns
			if(cell == weather_input): # Weather Input column
				weather_input_col = col_n
			if(cell == json_request_source): # Json Request Source column
				json_request_source_col = col_n
			# Get weather types rows
			if(cell == weather_type): # Weather Type row
				weather_type_row = row_n
			if(cell == weather_type_common): # Weather Common Type row
				weather_type_common_row = row_n

			col_n = col_n+1
		row_n = row_n+1
	# Get number of rows
	row_len = row_n

	################# Find output ################
	csvfile.seek(0) # Restart the csv iterator

	row_n = 0
	list_search = '' # Actual seraching weather type list
	search = False # Indicates if search in the weather type list

	for row in csv_reader:
		# Search the specified or common list
		if(row_n == weather_type_row or row_n == weather_type_common_row): # List found
			if(row_n == weather_type_row): # Weather type list found
				list_search = weather_type
			if(row_n == weather_type_common_row): # Common weather type list found
				list_search = weather_type_common
			search = True
			print('Looking \'' + info_required + '\' in \'' + list_search + '\'')

		# Search info in the list
		if(search == True):
			if(row[weather_input_col] == info_required): # Info found
				info_output = row[json_request_source_col]
				print('>> ' + '\'' + row[weather_input_col] + '\' found')
				break # Break the loop
			elif(row[weather_input_col] == ''): # End of list
				print('>> ' + '\'' + info_required + '\' not found in \'' + weather_type + '\'')
				search = False # Stop the searching
			elif(row_n == row_len-1):
				print('>> ' + 'End of file. \'' + info_required + '\' not found')

		row_n = row_n+1
