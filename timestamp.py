from datetime import datetime

event = 1765891482260412
rgb   = 1765891510341235

event_date = datetime.fromtimestamp(event / 1e6) # Convert us to s
rgb_date   = datetime.fromtimestamp(rgb   / 1e6) # Convert us to s

print("Event corresponds to date:", event_date)
print("RGB corresponds to date:", rgb_date)
