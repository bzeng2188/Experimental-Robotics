from datetime import datetime

print("Hello World, I'm pitabrot!")

now = datetime.now()
current_time = now.strftime("%H:%M:%S.%f")
print("The local time is ", current_time)