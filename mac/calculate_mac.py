from wingstructure import data
import matplotlib.pyplot as plt
import pdb
import yaml

definition = {}
with open("wing.yaml", 'r') as stream:
    try:
        definition = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

wing = data.Wing.deserialize(definition['geometry']['wing'])
wing.plot()
plt.show()
print("MAC: " + str(wing.mac))
