# Variant.msg
# This message defines a single variant data entry.
# key: The name of the variant. e.g. observation.state, observation.image.top, etc.
# type: The type of the variant. Matches the name of the field containing the data.
#       e.g. if type == "bool_array", then the data is in the bool_array field.
#       e.g. if type == "int_32_array", then the data is in the int_32_array field.

string key
string type

bool[] bool_array
string[] string_array
std_msgs/Int32MultiArray int_32_array
std_msgs/Int64MultiArray int_64_array
std_msgs/Float32MultiArray float_32_array
std_msgs/Float64MultiArray float_64_array
