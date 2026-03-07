# Motion profile
max_speed = 45
accel = 45

# Distance to cover
dist = 120-5

# Time to accelerate, distance covered in that time
t1 = max_speed/accel
d1 = 0.5*accel*t1*t1
print("It will take %.1f to accelerate, moving by %.1f"
      % (t1, d1))

# Deceleration will be the same
t3 = t1
d3 = d1
if d1+d3 > dist:
    print("Can't accelerate to full speed, results will be off")

# Remaining distance, covered at full speed
d2 = dist-d1-d3
t2 = d2/max_speed
print("We'll run for %.1f s at full speed, covering %.1f"
      % (t2, d2))
print("It will take %.1f to decellerate, moving by %.1f"
      % (t3, d3))

t = t1+t2+t3
print("In total, it will take %.1f sec to move %.1f"
      % (t, dist))
