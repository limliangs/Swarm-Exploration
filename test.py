import concurrent.futures

class MyObject:
    def __init__(self, name):
        self.name = name
        self.value = 1

    def update_value(self, objects):
        # Calculate a new value based on other objects' values
        new_value = self.value  # Start with the current value
        for obj in objects:
            if obj != self:
                new_value += obj.value
        return new_value
    
    def update(self, value):
        # Update the object's value with the new one
        self.value = value

# Create a list of objects
objects = [MyObject(f'Object_{i}') for i in range(2)]

# Display initial values
for obj in objects:
    print(f'{obj.name} initial value: {obj.value}')

# Update objects in parallel
loops = 0
while loops < 5:

    with concurrent.futures.ThreadPoolExecutor() as executor:
        # Submit update_value tasks to the executor
        futures = [executor.submit(obj.update_value, objects) for obj in objects]
        concurrent.futures.wait(futures)
        # Assign the returned values to each object using the update method
        for obj, future in zip(objects, futures):
            obj.update(future.result())

    # Check updated values
    for obj in objects:
        print(f'{obj.name} updated value: {obj.value}')
    
    loops += 1