import numpy as np

# Sample numpy array of x, y coordinates
coordinates = np.array([[1.0, 2.1], [3, 4], [5, 6]])

# Convert numpy array to string format
string_format = "$;" + ";$;".join(";".join(map(str, coord)) for coord in coordinates)+";"
print(string_format)

