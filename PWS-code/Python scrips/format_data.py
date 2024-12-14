import re

data = """ 

"""


# List of words or phrases to add a newline before
keywords = ["|pid find speed", "Waiting for pid", "Testing", "keep speed"]

# Replace every "Waiting for PID" and "Testing" with a newline followed by that string
formatted_data = re.sub(r'(Waiting for PID|Testing|pid find speed|keep speed|s:1 command received|s:2 command received|s:3 command received|s:4 command received)', r'\n\1', data)

# Print the final formatted data
print(formatted_data)
print("format done")
