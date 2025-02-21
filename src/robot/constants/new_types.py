from typing import NewType

# This is for type hinting. You can use these types to make your code more readable and maintainable.
# There will be a warning (but not an error!) if you try to assign a value of the wrong type to a variable
inches_per_second = NewType("inches_per_second", float)
degrees_per_second = NewType("degrees_per_second", float)
percentage = NewType("percentage", float)
