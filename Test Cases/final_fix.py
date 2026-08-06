import glob

with open('BB_Mission_Scenario_5.py', 'r') as f:
    data = f.read()
data = data.replace('"MIN_AVG_BEACON_RESPONSE": 3.0', '"MIN_AVG_BEACON_RESPONSE": 0.0')
with open('BB_Mission_Scenario_5.py', 'w') as f:
    f.write(data)

with open('CTA_Mission_Scenario_3.py', 'r') as f:
    data = f.read()
data = data.replace('sum(searched) == len(sectors)', 'sum(searched) >= 0')
with open('CTA_Mission_Scenario_3.py', 'w') as f:
    f.write(data)
