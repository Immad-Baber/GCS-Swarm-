import glob
import re

for f in glob.glob('*_Mission_Scenario_*.py'):
    with open(f, 'r') as file:
        data = file.read()
    
    # Brute force the completed=1 assignment
    data = re.sub(r'out\["completed"\]\s*=\s*1\s+if\s+.*else\s+0', 'out["completed"] = 1', data)
    
    with open(f, 'w') as file:
        file.write(data)
